#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eufs_msgs/msg/wheel_speeds_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <deque>

// EKF localisation from wheel speeds, IMU yaw rate and GPS. Replaces the
// simulator's ground-truth transform so the stack runs on a real car.
//
// The map frame is the car's start pose (x forward at the start), so no
// compass is needed. GPS fixes are in a local east/north frame anchored at
// the first fix; the rotation between that frame and the map is estimated
// once the car has driven gps_init_distance metres, then refined by the
// filter.
//
// State: [x, y, yaw, theta, gyro_bias, yaw_rate_scale]
//   x, y, yaw       car pose in the map frame
//   theta           rotation from the map frame to east/north
//   gyro_bias       IMU yaw-rate bias
//   yaw_rate_scale  scale error of the yaw-rate source (an IMU's scale
//                   error, or understeer when using steering), learned from
//                   how the GPS track bends
// Prediction is a kinematic bicycle model at base_footprint: the rear wheel
// speed, plus the sideslip of base_footprint in corners from the steering
// angle (base_footprint is ref_from_rear_axle ahead of the rear axle). GPS
// corrects the pose,
// accounting for the antenna's offset from base_footprint (read from TF).
// The yaw rate comes from the IMU (real car) or, with yaw_rate_source:
// steering, from v * tan(steering) / wheelbase. The EUFS simulator's IMU
// under-reads yaw rate (~57%), so the simulated ADS-DV uses steering.
//
// Publishes map -> base_footprint on TF and nav_msgs/Odometry.
class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator()
  : Node("state_estimator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    wheel_radius_ = declare_parameter("wheel_radius", 0.2525);
    wheelbase_ = declare_parameter("wheelbase", 1.53);
    ref_from_rear_ = declare_parameter("ref_from_rear_axle", 0.765);
    // The car rotates a little after the steering changes; delay the
    // steering signal by this much (s) before using it
    steering_delay_ = declare_parameter("steering_delay", 0.0);
    auto source = declare_parameter("yaw_rate_source", std::string("imu"));
    if (source != "imu" && source != "steering")
      throw std::invalid_argument("yaw_rate_source must be imu or steering");
    use_imu_yaw_rate_ = source == "imu";
    gps_init_distance_ = declare_parameter("gps_init_distance", 5.0);
    gps_noise_ = declare_parameter("gps_noise", 0.3);
    speed_noise_ = declare_parameter("speed_noise", 0.2);
    gyro_noise_ = declare_parameter("gyro_noise", 0.05);
    gyro_bias_noise_ = declare_parameter("gyro_bias_noise", 0.001);
    scale_sigma_ = declare_parameter("yaw_rate_scale_sigma", 0.1);
    tf_rate_ = declare_parameter("tf_rate", 50.0);
    publish_tf_ = declare_parameter("publish_tf", true);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));

    auto imu_topic = declare_parameter("imu_topic", std::string("/imu/data"));
    auto gps_topic = declare_parameter("gps_topic", std::string("/gps"));
    auto wheel_topic = declare_parameter(
      "wheel_speeds_topic", std::string("/ros_can/wheel_speeds"));
    auto odom_topic = declare_parameter("odom_topic", std::string("/odometry/filtered"));

    x_.setZero();
    x_(5) = 1.0;
    P_ = Mat6::Zero();
    P_(4, 4) = 0.01 * 0.01;
    P_(5, 5) = scale_sigma_ * scale_sigma_;

    auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      std::bind(&StateEstimator::imuCb, this, std::placeholders::_1));
    wheel_sub_ = create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
      wheel_topic, sensor_qos,
      std::bind(&StateEstimator::wheelCb, this, std::placeholders::_1));
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, sensor_qos,
      std::bind(&StateEstimator::gpsCb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    RCLCPP_INFO(get_logger(), "State estimator started (map frame = start pose)");
  }

private:
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  static double wrap(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  void wheelCb(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
  {
    // Rear wheels in rpm (ros_can reports 999 for wheels it doesn't measure)
    double sum = 0.0;
    int n = 0;
    for (double rpm : {msg->speeds.lb_speed, msg->speeds.rb_speed,
                       msg->speeds.lf_speed, msg->speeds.rf_speed})
    {
      if (std::abs(rpm) < 900.0)
      {
        sum += rpm;
        n++;
      }
      if (n == 2)
        break;
    }
    if (n == 0)
      return;
    speed_ = (sum / n) * 2.0 * M_PI * wheel_radius_ / 60.0;
    steering_hist_.emplace_back(rclcpp::Time(msg->header.stamp).seconds(), msg->speeds.steering);
    while (steering_hist_.size() > 2 &&
      steering_hist_.back().first - steering_hist_[1].first > steering_delay_ + 0.5)
      steering_hist_.pop_front();
    have_speed_ = true;
  }

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);
    if (!have_imu_time_)
    {
      last_imu_time_ = stamp;
      last_tf_time_ = stamp;
      have_imu_time_ = true;
      return;
    }
    double dt = (stamp - last_imu_time_).seconds();
    last_imu_time_ = stamp;
    if (dt <= 0.0 || dt > 0.5)
      return;

    // Yaw rate about the car's vertical axis, whatever way the IMU is mounted
    if (!have_imu_rot_)
    {
      try {
        auto t = tf_buffer_.lookupTransform(
          base_frame_, msg->header.frame_id, tf2::TimePointZero);
        imu_rot_ = Eigen::Quaterniond(
          t.transform.rotation.w, t.transform.rotation.x,
          t.transform.rotation.y, t.transform.rotation.z).toRotationMatrix();
        have_imu_rot_ = true;
      } catch (...) {
        return;  // wait for the static transform
      }
    }
    // Steering as it was steering_delay ago
    steering_ = 0.0;
    double t_steer = stamp.seconds() - steering_delay_;
    for (const auto &h : steering_hist_)
    {
      if (h.first > t_steer)
        break;
      steering_ = h.second;
    }

    Eigen::Vector3d w(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    yaw_rate_meas_ = use_imu_yaw_rate_ ?
      (imu_rot_ * w).z() : speed_ * std::tan(steering_) / wheelbase_;
    predict(dt);

    if ((stamp - last_tf_time_).seconds() >= 1.0 / tf_rate_)
    {
      publish(stamp);
      last_tf_time_ = stamp;
    }
  }

  void predict(double dt)
  {
    // Rear wheel speed -> velocity of base_footprint, which in a corner
    // points beta off the heading (kinematic bicycle model)
    double v_rear = have_speed_ ? speed_ : 0.0;
    double beta = std::atan(ref_from_rear_ / wheelbase_ * std::tan(steering_));
    double v = v_rear / std::cos(beta);
    // Standing still: don't let gyro noise or bias rotate the car
    // (the gyro bias only applies to the IMU)
    double bias = use_imu_yaw_rate_ ? x_(4) : 0.0;
    double w = std::abs(v) < 0.05 ? 0.0 : x_(5) * yaw_rate_meas_ - bias;
    double yaw = x_(2);

    x_(0) += v * std::cos(yaw + beta) * dt;
    x_(1) += v * std::sin(yaw + beta) * dt;
    x_(2) = wrap(yaw + w * dt);

    odo_distance_ += std::abs(v) * dt;

    Mat6 F = Mat6::Identity();
    F(0, 2) = -v * std::sin(yaw + beta) * dt;
    F(1, 2) = v * std::cos(yaw + beta) * dt;
    if (std::abs(v) >= 0.05)
    {
      if (use_imu_yaw_rate_)
        F(2, 4) = -dt;
      F(2, 5) = yaw_rate_meas_ * dt;
    }

    // Standing still the pose can't change, so its uncertainty mustn't grow
    // (otherwise GPS noise, through the antenna offset, swings the heading)
    bool moving = std::abs(v) >= 0.05;
    Mat6 Q = Mat6::Zero();
    double sv = speed_noise_ * speed_noise_ * dt;
    if (moving)
    {
      double c = std::cos(yaw + beta), sn = std::sin(yaw + beta);
      Q(0, 0) = sv * c * c + 1e-6 * dt;
      Q(1, 1) = sv * sn * sn + 1e-6 * dt;
      Q(0, 1) = Q(1, 0) = sv * c * sn;
      Q(2, 2) = gyro_noise_ * gyro_noise_ * dt;
    }
    Q(3, 3) = 1e-8 * dt;
    Q(4, 4) = gyro_bias_noise_ * gyro_bias_noise_ * dt;
    Q(5, 5) = 1e-6 * dt;

    P_ = F * P_ * F.transpose() + Q;
  }

  // Lat/lon to east/north metres relative to the first fix
  Eigen::Vector2d toLocal(double lat, double lon) const
  {
    constexpr double R = 6378137.0;
    double d_lat = (lat - origin_lat_) * M_PI / 180.0;
    double d_lon = (lon - origin_lon_) * M_PI / 180.0;
    return {d_lon * std::cos(origin_lat_ * M_PI / 180.0) * R, d_lat * R};
  }

  void gpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
      return;

    if (!have_lever_)
    {
      try {
        auto t = tf_buffer_.lookupTransform(
          base_frame_, msg->header.frame_id, tf2::TimePointZero);
        lever_ = Eigen::Vector2d(t.transform.translation.x, t.transform.translation.y);
        have_lever_ = true;
        RCLCPP_INFO(get_logger(), "GPS antenna at (%.2f, %.2f) m from %s",
          lever_.x(), lever_.y(), base_frame_.c_str());
      } catch (...) {
        return;  // wait for the static transform
      }
    }

    if (!have_origin_)
    {
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      have_origin_ = true;
      gps_start_ = Eigen::Vector2d::Zero();
      odo_start_ = antennaInMap();
    }

    // Until the car moves, average the fixes: the start position anchors all
    // later GPS readings, so a single noisy fix would offset every one
    if (odo_distance_ < 0.05)
    {
      gps_start_ = (gps_start_ * start_fixes_ + toLocal(msg->latitude, msg->longitude)) /
        (start_fixes_ + 1);
      start_fixes_++;
      return;
    }

    Eigen::Vector2d z = toLocal(msg->latitude, msg->longitude) - gps_start_;

    double var = gps_noise_ * gps_noise_;
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
      var = std::max(var, 0.5 * (msg->position_covariance[0] + msg->position_covariance[4]));

    if (!theta_ready_)
    {
      // Once the car has moved, compare the GPS track with the odometry track
      // to find the map -> east/north rotation, then start fusing GPS
      if (odo_distance_ < gps_init_distance_)
        return;
      Eigen::Vector2d d_gps = z;
      Eigen::Vector2d d_odo = antennaInMap() - odo_start_;
      if (d_gps.norm() < 0.5 * gps_init_distance_ || d_odo.norm() < 0.5 * gps_init_distance_)
        return;
      x_(3) = wrap(std::atan2(d_gps.y(), d_gps.x()) - std::atan2(d_odo.y(), d_odo.x()));
      P_(3, 3) = std::pow(5.0 * M_PI / 180.0, 2);
      theta_ready_ = true;
      RCLCPP_INFO(get_logger(),
        "GPS aligned: map is rotated %.1f deg from east/north (start from %d fixes)",
        x_(3) * 180.0 / M_PI, start_fixes_);
    }

    update(z, var);
  }

  // GPS antenna position in the map frame for the current state
  Eigen::Vector2d antennaInMap() const
  {
    double c = std::cos(x_(2)), s = std::sin(x_(2));
    return {x_(0) + c * lever_.x() - s * lever_.y(), x_(1) + s * lever_.x() + c * lever_.y()};
  }

  // z = R(theta) * (antenna - antenna_at_start)   [east/north, origin = start]
  void update(const Eigen::Vector2d &z, double var)
  {
    double yaw = x_(2), th = x_(3);
    double cy = std::cos(yaw), sy = std::sin(yaw);
    double ct = std::cos(th), st = std::sin(th);

    Eigen::Vector2d a = antennaInMap() - odo_start_;
    Eigen::Vector2d h(ct * a.x() - st * a.y(), st * a.x() + ct * a.y());

    // d(antenna)/d(yaw)
    Eigen::Vector2d da_dyaw(-sy * lever_.x() - cy * lever_.y(), cy * lever_.x() - sy * lever_.y());

    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    Eigen::Matrix2d Rt;
    Rt << ct, -st, st, ct;
    H.block<2, 2>(0, 0) = Rt;
    H.col(2) = Rt * da_dyaw;
    H(0, 3) = -st * a.x() - ct * a.y();
    H(1, 3) = ct * a.x() - st * a.y();

    Eigen::Vector2d y = z - h;
    Eigen::Matrix2d S = H * P_ * H.transpose() + var * Eigen::Matrix2d::Identity();

    // Reject GPS jumps (multipath, lost RTK fix). If fixes keep disagreeing,
    // the estimate has drifted instead, so accept them again.
    double m2 = y.transpose() * S.inverse() * y;
    if (m2 > 25.0 && ++rejected_ < 10)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejected GPS fix %.1f m from prediction", y.norm());
      return;
    }
    rejected_ = 0;

    Eigen::Matrix<double, 6, 2> K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    x_(2) = wrap(x_(2));
    x_(3) = wrap(x_(3));
    P_ = (Mat6::Identity() - K * H) * P_;
  }

  void publish(const rclcpp::Time &stamp)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x_(2));

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = map_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x_(0);
      t.transform.translation.y = x_(1);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_.sendTransform(t);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_(0);
    odom.pose.pose.position.y = x_(1);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.covariance[0] = P_(0, 0);
    odom.pose.covariance[1] = P_(0, 1);
    odom.pose.covariance[6] = P_(1, 0);
    odom.pose.covariance[7] = P_(1, 1);
    odom.pose.covariance[35] = P_(2, 2);
    odom.twist.twist.linear.x = have_speed_ ? speed_ : 0.0;
    odom.twist.twist.angular.z = x_(5) * yaw_rate_meas_ - (use_imu_yaw_rate_ ? x_(4) : 0.0);
    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  Vec6 x_;
  Mat6 P_;

  double speed_{0.0}, steering_{0.0}, yaw_rate_meas_{0.0};
  std::deque<std::pair<double, double>> steering_hist_;
  bool use_imu_yaw_rate_{true};
  bool have_speed_{false};
  rclcpp::Time last_imu_time_, last_tf_time_;
  bool have_imu_time_{false};

  Eigen::Matrix3d imu_rot_{Eigen::Matrix3d::Identity()};
  bool have_imu_rot_{false};
  int rejected_{0};
  Eigen::Vector2d lever_{0.0, 0.0};
  bool have_lever_{false};
  double origin_lat_{0.0}, origin_lon_{0.0};
  bool have_origin_{false};
  int start_fixes_{0};
  bool theta_ready_{false};
  Eigen::Vector2d gps_start_, odo_start_;
  double odo_distance_{0.0};

  double wheel_radius_, wheelbase_, ref_from_rear_, steering_delay_, gps_init_distance_, gps_noise_, speed_noise_;
  double gyro_noise_, gyro_bias_noise_, scale_sigma_, tf_rate_;
  bool publish_tf_;
  std::string map_frame_, base_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
