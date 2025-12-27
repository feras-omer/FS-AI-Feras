#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>

struct MapCone
{
  double x;
  double y;
  int observations;
  int missed;
  int color; // 1=blue, 2=yellow
};

class ConeMapper : public rclcpp::Node
{
public:
  ConeMapper()
  : Node("cone_mapper"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    rclcpp::QoS reliable_qos(rclcpp::KeepLast(10));
reliable_qos.reliable();

cone_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/cones/verified",
    reliable_qos,
    std::bind(&ConeMapper::coneCallback, this, std::placeholders::_1)
  );

map_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
  "/map/cones", reliable_qos);


    RCLCPP_INFO(this->get_logger(), "Cone mapper started");
  }

private:
  static constexpr double ASSOC_RADIUS = 0.75;
  static constexpr int MIN_OBSERVATIONS = 1;
  static constexpr int MAX_MISSES = 5;

  void coneCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Mark all cones unseen this frame
    for (auto &c : map_cones_)
      c.missed++;

    for (const auto &pose : msg->poses)
    {
      geometry_msgs::msg::PoseStamped in, out;
      in.header = msg->header;
      in.pose = pose;

      try {
        out = tf_buffer_.transform(in, "map", tf2::durationFromSec(0.1));
      } catch (...) {
        continue;
      }

      associateCone(
        out.pose.position.x,
        out.pose.position.y,
        static_cast<int>(pose.orientation.z)
      );
    }

    // Remove stale cones
    map_cones_.erase(
      std::remove_if(map_cones_.begin(), map_cones_.end(),
        [](const MapCone &c){ return c.missed > MAX_MISSES; }),
      map_cones_.end()
    );

    publishMap(msg->header.stamp);
  }

  void associateCone(double x, double y, int color)
  {
    for (auto &c : map_cones_)
    {
      if (std::hypot(c.x - x, c.y - y) < ASSOC_RADIUS &&
          c.color == color)
      {
        c.x = (c.x * c.observations + x) / (c.observations + 1);
        c.y = (c.y * c.observations + y) / (c.observations + 1);
        c.observations++;
        c.missed = 0;
        return;
      }
    }

    map_cones_.push_back({x, y, 1, 0, color});
    
    RCLCPP_DEBUG(this->get_logger(), "Assoc cone (color=%d) at (%.2f,%.2f)",
    color, x, y);

  }

  void publishMap(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseArray out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";

    for (const auto &c : map_cones_)
    {
      if (c.observations < MIN_OBSERVATIONS)
        continue;

      geometry_msgs::msg::Pose p;
      p.position.x = c.x;
      p.position.y = c.y;
      p.position.z = 0.0;
      p.orientation.z = c.color;
      p.orientation.w = 1.0;

      out.poses.push_back(p);
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  "Publishing %zu map cones", out.poses.size());


    map_pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr map_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<MapCone> map_cones_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeMapper>());
  rclcpp::shutdown();
  return 0;
}

