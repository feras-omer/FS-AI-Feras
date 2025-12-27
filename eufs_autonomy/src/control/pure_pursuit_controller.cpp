#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <vector>

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController()
  : Node("pure_pursuit_controller"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    rclcpp::QoS qos(10);
    qos.reliable();


    centerline_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/map/centerline", qos,
      std::bind(&PurePursuitController::centerlineCb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/gps/odom", 10,
      std::bind(&PurePursuitController::odomCb, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PurePursuitController::controlLoop, this));

    wheelbase_ = 1.6;
    max_speed_ = 7.0;
    base_ld_ = 2.5;
    min_ld_ = 2.0;
    max_ld_ = 10.0;
    speed_gain_ = 0.5;
    curvature_gain_ = 7.0;
  }

private:
  void centerlineCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    centerline_ = msg;
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    speed_ = std::hypot(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y);
  }

  void controlLoop()
  {
    if (!centerline_ || centerline_->poses.empty())
      return;

    std::vector<geometry_msgs::msg::PoseStamped> pts;

    for (const auto &p : centerline_->poses)
    {
      geometry_msgs::msg::PoseStamped in, out;
      in.header = centerline_->header;
      in.pose = p;

      try {
        tf2::doTransform(
          in, out,
          tf_buffer_.lookupTransform(
            "base_footprint",
            in.header.frame_id,
            tf2::TimePointZero));

        if (out.pose.position.x > 0.0)
          pts.push_back(out);
      } catch (...) {
        return;
      }
    }

    if (pts.empty()) return;

    std::sort(pts.begin(), pts.end(),
      [](auto &a, auto &b){
        return a.pose.position.x < b.pose.position.x;
      });

    double Ld = std::clamp(
      base_ld_ + speed_gain_ * speed_,
      min_ld_, max_ld_);

    double acc = 0.0;
    double tx = pts.back().pose.position.x;
    double ty = pts.back().pose.position.y;

    for (size_t i = 1; i < pts.size(); ++i)
    {
      auto &p0 = pts[i-1].pose.position;
      auto &p1 = pts[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc >= Ld)
      {
        tx = p1.x;
        ty = p1.y;
        break;
      }
    }

    double ld = std::hypot(tx, ty);
    double alpha = std::atan2(ty, tx);
    double curvature = (ld > 0.01) ? 2.0 * std::sin(alpha) / ld : 0.0;
    double steering = std::atan(wheelbase_ * curvature);

    double speed =
      max_speed_ / (1.0 + curvature_gain_ * std::abs(curvature));

    speed = std::max(speed, 0.3);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.drive.steering_angle = steering;
    cmd.drive.speed = speed;

    drive_pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr centerline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseArray::SharedPtr centerline_;
  double speed_{0.0};

  double wheelbase_, max_speed_;
  double base_ld_, min_ld_, max_ld_, speed_gain_, curvature_gain_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
