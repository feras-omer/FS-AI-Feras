#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

// Pure pursuit with two modes:
//  - explore (lap 1): follow the local /map/centerline at a capped speed
//  - race (lap 2+):   follow the closed /global_path with a speed profile
//                     limited by lateral acceleration and braking
class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController()
  : Node("pure_pursuit_controller"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    wheelbase_ = declare_parameter("wheelbase", 1.58);
    max_steering_ = declare_parameter("max_steering", 0.52);
    explore_speed_ = declare_parameter("explore_speed", 4.0);
    curvature_gain_ = declare_parameter("curvature_gain", 7.0);
    max_speed_ = declare_parameter("max_speed", 8.0);
    min_speed_ = declare_parameter("min_speed", 1.0);
    max_lat_acc_ = declare_parameter("max_lat_acc", 5.0);
    max_acc_ = declare_parameter("max_acc", 2.5);
    max_brake_ = declare_parameter("max_brake", 4.0);
    base_ld_ = declare_parameter("base_lookahead", 2.5);
    min_ld_ = declare_parameter("min_lookahead", 2.0);
    max_ld_ = declare_parameter("max_lookahead", 8.0);
    speed_gain_ = declare_parameter("lookahead_speed_gain", 0.4);

    rclcpp::QoS qos(10);
    qos.reliable();

    centerline_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/map/centerline", qos,
      std::bind(&PurePursuitController::centerlineCb, this, std::placeholders::_1));

    rclcpp::QoS latched(rclcpp::KeepLast(1));
    latched.reliable().transient_local();
    global_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", latched,
      std::bind(&PurePursuitController::globalPathCb, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PurePursuitController::controlLoop, this));
  }

private:
  void centerlineCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    local_path_.clear();
    for (const auto &p : msg->poses)
      local_path_.emplace_back(p.position.x, p.position.y);
  }

  void globalPathCb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // The planner republishes the same path every second; only rebuild on change
    if (!global_path_.empty() && msg->poses.size() == global_path_.size())
      return;

    global_path_.clear();
    for (const auto &p : msg->poses)
      global_path_.emplace_back(p.pose.position.x, p.pose.position.y);
    buildSpeedProfile();
    closest_ = -1;
    RCLCPP_INFO(get_logger(),
      "Global path received (%zu points), switching to race mode", global_path_.size());
  }

  // v = sqrt(a_lat / |k|), then limited by braking (backward pass) and
  // acceleration (forward pass) around the closed loop
  void buildSpeedProfile()
  {
    size_t n = global_path_.size();
    profile_.assign(n, max_speed_);
    if (n < 3)
      return;

    double ds = 0.0;
    for (size_t i = 0; i < n; ++i)
      ds += (global_path_[(i + 1) % n] - global_path_[i]).norm();
    ds /= n;
    // Measure curvature over ~1.5 m so it isn't dominated by sampling noise
    size_t k = std::max<size_t>(1, static_cast<size_t>(std::round(1.5 / ds)));

    for (size_t i = 0; i < n; ++i)
    {
      const auto &a = global_path_[(i + n - k) % n];
      const auto &b = global_path_[i];
      const auto &c = global_path_[(i + k) % n];
      double cross = (b - a).x() * (c - a).y() - (b - a).y() * (c - a).x();
      double denom = (b - a).norm() * (c - b).norm() * (c - a).norm();
      double curv = denom > 1e-9 ? 2.0 * std::abs(cross) / denom : 0.0;
      double v = curv > 1e-6 ? std::sqrt(max_lat_acc_ / curv) : max_speed_;
      profile_[i] = std::clamp(v, min_speed_, max_speed_);
    }

    for (int pass = 0; pass < 2; ++pass)
    {
      for (size_t j = 0; j < n; ++j)
      {
        size_t i = n - 1 - j;
        size_t nx = (i + 1) % n;
        double d = (global_path_[nx] - global_path_[i]).norm();
        profile_[i] = std::min(profile_[i],
          std::sqrt(profile_[nx] * profile_[nx] + 2.0 * max_brake_ * d));
      }
      for (size_t i = 0; i < n; ++i)
      {
        size_t nx = (i + 1) % n;
        double d = (global_path_[nx] - global_path_[i]).norm();
        profile_[nx] = std::min(profile_[nx],
          std::sqrt(profile_[i] * profile_[i] + 2.0 * max_acc_ * d));
      }
    }
  }

  // Speed from successive car poses (the sim's odometry is not used)
  void updateSpeed(const geometry_msgs::msg::TransformStamped &car)
  {
    Eigen::Vector2d pos(car.transform.translation.x, car.transform.translation.y);
    rclcpp::Time stamp(car.header.stamp);
    if (have_last_pose_)
    {
      double dt = (stamp - last_stamp_).seconds();
      if (dt < 1e-3)
        return;
      double v = (pos - last_pos_).norm() / dt;
      speed_ = 0.7 * speed_ + 0.3 * v;
    }
    last_pos_ = pos;
    last_stamp_ = stamp;
    have_last_pose_ = true;
  }

  void controlLoop()
  {
    geometry_msgs::msg::TransformStamped car;
    try {
      car = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (...) {
      return;
    }
    updateSpeed(car);

    Eigen::Vector2d pos(car.transform.translation.x, car.transform.translation.y);
    double yaw = tf2::getYaw(car.transform.rotation);
    double Ld = std::clamp(base_ld_ + speed_gain_ * speed_, min_ld_, max_ld_);

    Eigen::Vector2d target;
    double speed;

    if (!global_path_.empty())
    {
      size_t idx = closestIndex(pos);
      target = lookaheadOnLoop(idx, Ld);
      speed = profile_[idx];
    }
    else
    {
      if (local_path_.empty())
        return;
      target = lookaheadOnLocal(pos, Ld);
      speed = -1.0;  // set below from the pursuit curvature
    }

    // Target in the car frame
    Eigen::Vector2d d = target - pos;
    double tx = std::cos(yaw) * d.x() + std::sin(yaw) * d.y();
    double ty = -std::sin(yaw) * d.x() + std::cos(yaw) * d.y();

    double ld = std::hypot(tx, ty);
    double alpha = std::atan2(ty, tx);
    double curvature = (ld > 0.01) ? 2.0 * std::sin(alpha) / ld : 0.0;
    double steering = std::clamp(
      std::atan(wheelbase_ * curvature), -max_steering_, max_steering_);

    if (speed < 0.0)
    {
      speed = explore_speed_ / (1.0 + curvature_gain_ * std::abs(curvature));
      speed = std::max(speed, 0.3);
    }

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.drive.steering_angle = steering;
    cmd.drive.speed = speed;

    drive_pub_->publish(cmd);
  }

  // Closest point on the loop, searched near the previous one so the car
  // can't jump to another part of the track where it passes close by
  size_t closestIndex(const Eigen::Vector2d &pos)
  {
    size_t n = global_path_.size();
    size_t best = 0;
    double best_d = 1e18;

    if (closest_ < 0)
    {
      for (size_t i = 0; i < n; ++i)
      {
        double d = (global_path_[i] - pos).squaredNorm();
        if (d < best_d) { best_d = d; best = i; }
      }
    }
    else
    {
      for (int off = -20; off <= 80; ++off)
      {
        size_t i = (static_cast<size_t>(closest_) + n + off) % n;
        double d = (global_path_[i] - pos).squaredNorm();
        if (d < best_d) { best_d = d; best = i; }
      }
    }
    closest_ = static_cast<int>(best);
    return best;
  }

  Eigen::Vector2d lookaheadOnLoop(size_t idx, double Ld) const
  {
    size_t n = global_path_.size();
    double acc = 0.0;
    size_t i = idx;
    for (size_t step = 0; step < n; ++step)
    {
      size_t nx = (i + 1) % n;
      acc += (global_path_[nx] - global_path_[i]).norm();
      i = nx;
      if (acc >= Ld)
        break;
    }
    return global_path_[i];
  }

  // The local centerline is ordered forward from the car
  Eigen::Vector2d lookaheadOnLocal(const Eigen::Vector2d &pos, double Ld) const
  {
    double acc = 0.0;
    Eigen::Vector2d prev = pos;
    for (const auto &p : local_path_)
    {
      acc += (p - prev).norm();
      prev = p;
      if (acc >= Ld)
        return p;
    }
    return local_path_.back();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr centerline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<Eigen::Vector2d> local_path_;
  std::vector<Eigen::Vector2d> global_path_;
  std::vector<double> profile_;
  int closest_{-1};

  double speed_{0.0};
  bool have_last_pose_{false};
  Eigen::Vector2d last_pos_;
  rclcpp::Time last_stamp_;

  double wheelbase_, max_steering_, explore_speed_, curvature_gain_;
  double max_speed_, min_speed_, max_lat_acc_, max_acc_, max_brake_;
  double base_ld_, min_ld_, max_ld_, speed_gain_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
