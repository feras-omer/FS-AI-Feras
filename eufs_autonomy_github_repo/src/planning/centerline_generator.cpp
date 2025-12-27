#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

class CenterlineGenerator : public rclcpp::Node
{
public:
  CenterlineGenerator() : Node("centerline_generator")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();


    cone_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/map/cones", qos,
      std::bind(&CenterlineGenerator::conesCb, this, std::placeholders::_1));

    centerline_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/map/centerline", qos);
  }

private:
  void conesCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::vector<Eigen::Vector2d> left, right;

    for (const auto &p : msg->poses)
    {
      if (static_cast<int>(p.orientation.z) == 2)
        left.emplace_back(p.position.x, p.position.y);
      else if (static_cast<int>(p.orientation.z) == 1)
        right.emplace_back(p.position.x, p.position.y);
    }
    
    RCLCPP_INFO(this->get_logger(),
  "Centerline: left=%zu right=%zu", left.size(), right.size());
  
    if (left.size() < 2 || right.size() < 2)
      return;

    auto sort_fwd = [](auto &a, auto &b){ return a.x() < b.x(); };
    std::sort(left.begin(), left.end(), sort_fwd);
    std::sort(right.begin(), right.end(), sort_fwd);

    geometry_msgs::msg::PoseArray centerline;
    centerline.header = msg->header;

    size_t N = std::min(left.size(), right.size());
    for (size_t i = 0; i < N; ++i)
    {
      Eigen::Vector2d mid = 0.5 * (left[i] + right[i]);

      geometry_msgs::msg::Pose p;
      p.position.x = mid.x();
      p.position.y = mid.y();
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      centerline.poses.push_back(p);
    }

    centerline_pub_->publish(centerline);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centerline_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineGenerator>());
  rclcpp::shutdown();
  return 0;
}
