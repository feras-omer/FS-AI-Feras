#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "eufs_autonomy/track_geometry.hpp"

// Builds a closed, smooth global path once the first lap is complete.
//
// 1. Tracks laps from the car pose: a lap ends when the car is back within
//    lap_radius of its start position after driving min_lap_distance.
// 2. Pairs blue/yellow cones from the persistent cone map into midpoints and
//    orders them around the loop from the start pose.
// 3. Resamples and lightly smooths the loop, then fits piecewise cubic
//    Bezier segments (Catmull-Rom control points, C1 continuous).
// 4. Publishes /global_path (nav_msgs/Path, transient local so it persists
//    for late subscribers such as RViz and the controller), and republishes
//    it once a second for subscribers using volatile durability.
class GlobalPathPlanner : public rclcpp::Node
{
public:
  GlobalPathPlanner()
  : Node("global_path_planner"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    min_lap_distance_ = declare_parameter("min_lap_distance", 40.0);
    lap_radius_ = declare_parameter("lap_radius", 4.0);
    max_width_ = declare_parameter("max_track_width", 7.0);
    max_step_ = declare_parameter("max_step", 6.0);
    max_first_step_ = declare_parameter("max_first_step", 12.0);
    max_turn_ = declare_parameter("max_turn_deg", 70.0) * M_PI / 180.0;
    resample_spacing_ = declare_parameter("resample_spacing", 2.0);
    sample_spacing_ = declare_parameter("sample_spacing", 0.25);
    smoothing_passes_ = declare_parameter("smoothing_passes", 2);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    cone_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/map/cones", qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) { cones_ = msg; });

    rclcpp::QoS latched(rclcpp::KeepLast(1));
    latched.reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", latched);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GlobalPathPlanner::updateLap, this));

    RCLCPP_INFO(get_logger(), "Global path planner started, waiting for first lap");
  }

private:
  using Points = track_geometry::Points;

  void updateLap()
  {
    geometry_msgs::msg::TransformStamped car;
    try {
      car = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (...) {
      return;
    }

    Eigen::Vector2d pos(car.transform.translation.x, car.transform.translation.y);

    // Republish once a second so RViz displays left on the default
    // (volatile) durability also pick the path up
    if (built_ && ++ticks_since_publish_ >= 10)
    {
      path_msg_.header.stamp = now();
      path_pub_->publish(path_msg_);
      ticks_since_publish_ = 0;
    }

    if (!started_)
    {
      start_ = pos;
      start_yaw_ = tf2::getYaw(car.transform.rotation);
      last_ = pos;
      lap_start_ = now();
      started_ = true;
      return;
    }

    lap_distance_ += (pos - last_).norm();
    last_ = pos;

    if (lap_distance_ < min_lap_distance_ || (pos - start_).norm() > lap_radius_)
      return;

    laps_++;
    RCLCPP_INFO(get_logger(), "Lap %d complete: %.1f m in %.1f s",
      laps_, lap_distance_, (now() - lap_start_).seconds());
    lap_distance_ = 0.0;
    lap_start_ = now();

    if (!built_)
      built_ = buildPath();
  }

  bool buildPath()
  {
    if (!cones_)
      return false;

    Points left, right;
    for (const auto &p : cones_->poses)
    {
      if (static_cast<int>(p.orientation.z) == 1)       // blue = left
        left.emplace_back(p.position.x, p.position.y);
      else if (static_cast<int>(p.orientation.z) == 2)  // yellow = right
        right.emplace_back(p.position.x, p.position.y);
    }

    auto mids = track_geometry::pairMidpoints(left, right, max_width_);
    auto loop = track_geometry::orderPath(
      mids, start_, start_yaw_, max_step_, max_turn_, max_first_step_);

    if (loop.size() < 10 || (loop.back() - loop.front()).norm() > max_step_)
    {
      RCLCPP_WARN(get_logger(),
        "Could not close the loop (%zu of %zu midpoints, gap %.1f m); retrying next lap",
        loop.size(), mids.size(),
        loop.empty() ? 0.0 : (loop.back() - loop.front()).norm());
      return false;
    }

    loop = resampleClosed(loop, resample_spacing_);
    for (int i = 0; i < smoothing_passes_; ++i)
      loop = smoothClosed(loop);
    Points path = bezierClosed(loop, sample_spacing_);

    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();

    for (size_t i = 0; i < path.size(); ++i)
    {
      const auto &p = path[i];
      const auto &next = path[(i + 1) % path.size()];

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, std::atan2(next.y() - p.y(), next.x() - p.x()));

      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      msg.poses.push_back(ps);
    }

    path_pub_->publish(msg);
    path_msg_ = msg;
    RCLCPP_INFO(get_logger(),
      "Global path published: %zu points, %.1f m (from %zu cones, %zu midpoints)",
      path.size(), loopLength(path), left.size() + right.size(), mids.size());
    return true;
  }

  static double loopLength(const Points &pts)
  {
    double len = 0.0;
    for (size_t i = 0; i < pts.size(); ++i)
      len += (pts[(i + 1) % pts.size()] - pts[i]).norm();
    return len;
  }

  // Evenly spaced points along a closed polyline
  static Points resampleClosed(const Points &pts, double spacing)
  {
    double total = loopLength(pts);
    int n = std::max(4, static_cast<int>(std::round(total / spacing)));
    double step = total / n;

    Points out;
    size_t seg = 0;
    double seg_start = 0.0;
    for (int k = 0; k < n; ++k)
    {
      double s = k * step;
      double seg_len = (pts[(seg + 1) % pts.size()] - pts[seg]).norm();
      while (s > seg_start + seg_len && seg + 1 < pts.size())
      {
        seg_start += seg_len;
        seg++;
        seg_len = (pts[(seg + 1) % pts.size()] - pts[seg]).norm();
      }
      double t = seg_len > 1e-9 ? (s - seg_start) / seg_len : 0.0;
      out.push_back(pts[seg] + t * (pts[(seg + 1) % pts.size()] - pts[seg]));
    }
    return out;
  }

  // One pass of [1 2 1]/4 averaging around the loop
  static Points smoothClosed(const Points &pts)
  {
    size_t n = pts.size();
    Points out(n);
    for (size_t i = 0; i < n; ++i)
      out[i] = 0.25 * pts[(i + n - 1) % n] + 0.5 * pts[i] + 0.25 * pts[(i + 1) % n];
    return out;
  }

  // Piecewise cubic Bezier through every point of a closed loop. Control
  // points come from Catmull-Rom tangents, so segments join smoothly:
  //   B0 = P1, B1 = P1 + (P2 - P0) / 6, B2 = P2 - (P3 - P1) / 6, B3 = P2
  static Points bezierClosed(const Points &pts, double spacing)
  {
    size_t n = pts.size();
    Points out;
    for (size_t i = 0; i < n; ++i)
    {
      const auto &p0 = pts[(i + n - 1) % n];
      const auto &p1 = pts[i];
      const auto &p2 = pts[(i + 1) % n];
      const auto &p3 = pts[(i + 2) % n];

      Eigen::Vector2d b0 = p1;
      Eigen::Vector2d b1 = p1 + (p2 - p0) / 6.0;
      Eigen::Vector2d b2 = p2 - (p3 - p1) / 6.0;
      Eigen::Vector2d b3 = p2;

      int m = std::max(1, static_cast<int>(std::ceil((p2 - p1).norm() / spacing)));
      for (int j = 0; j < m; ++j)
      {
        double t = static_cast<double>(j) / m;
        double u = 1.0 - t;
        out.push_back(
          u * u * u * b0 + 3 * u * u * t * b1 + 3 * u * t * t * b2 + t * t * t * b3);
      }
    }
    return out;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseArray::SharedPtr cones_;
  nav_msgs::msg::Path path_msg_;
  int ticks_since_publish_{0};

  bool started_{false};
  bool built_{false};
  int laps_{0};
  Eigen::Vector2d start_, last_;
  double start_yaw_{0.0};
  double lap_distance_{0.0};
  rclcpp::Time lap_start_;

  double min_lap_distance_, lap_radius_, max_width_, max_step_, max_first_step_, max_turn_;
  double resample_spacing_, sample_spacing_;
  int smoothing_passes_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPathPlanner>());
  rclcpp::shutdown();
  return 0;
}
