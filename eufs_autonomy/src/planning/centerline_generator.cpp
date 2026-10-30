#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <vector>

#include "eufs_autonomy/track_geometry.hpp"

// Local centerline used while exploring (lap 1): midpoints of blue/yellow
// cone pairs, ordered forward from the car. Recomputed at 10 Hz from the
// latest cone map and car pose, so a perception hiccup doesn't starve the
// controller (the map persists).
class CenterlineGenerator : public rclcpp::Node
{
public:
  CenterlineGenerator()
  : Node("centerline_generator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    max_width_ = declare_parameter("max_track_width", 7.0);
    max_step_ = declare_parameter("max_step", 6.0);
    max_gap_ = declare_parameter("max_gap", 12.0);
    max_turn_ = declare_parameter("max_turn_deg", 70.0) * M_PI / 180.0;
    max_length_ = declare_parameter("max_length", 30.0);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    cone_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/map/cones", qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) { cones_ = msg; });

    centerline_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/map/centerline", qos);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CenterlineGenerator::update, this));
  }

private:
  void update()
  {
    if (!cones_)
      return;
    const auto &msg = cones_;

    track_geometry::Points left, right;

    for (const auto &p : msg->poses)
    {
      if (static_cast<int>(p.orientation.z) == 1)       // blue = left
        left.emplace_back(p.position.x, p.position.y);
      else if (static_cast<int>(p.orientation.z) == 2)  // yellow = right
        right.emplace_back(p.position.x, p.position.y);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Centerline: left=%zu right=%zu", left.size(), right.size());

    if (left.empty() || right.empty())
      return;

    geometry_msgs::msg::TransformStamped car;
    try {
      car = tf_buffer_.lookupTransform(
        msg->header.frame_id, "base_footprint", tf2::TimePointZero);
    } catch (...) {
      return;
    }

    Eigen::Vector2d start(car.transform.translation.x, car.transform.translation.y);
    double yaw = tf2::getYaw(car.transform.rotation);

    auto mids = track_geometry::pairMidpoints(left, right, max_width_);
    auto path = track_geometry::orderPath(
      mids, start, yaw, max_step_, max_turn_, max_gap_, max_length_);

    if (path.empty())
      return;

    geometry_msgs::msg::PoseArray centerline;
    centerline.header.frame_id = msg->header.frame_id;
    centerline.header.stamp = car.header.stamp;

    for (const auto &m : path)
    {
      geometry_msgs::msg::Pose p;
      p.position.x = m.x();
      p.position.y = m.y();
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      centerline.poses.push_back(p);
    }

    centerline_pub_->publish(centerline);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centerline_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseArray::SharedPtr cones_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double max_width_, max_step_, max_gap_, max_turn_, max_length_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineGenerator>());
  rclcpp::shutdown();
  return 0;
}
