
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Geometry>
#include <cmath>

// Detects cones in the LiDAR point cloud. The cloud is transformed into the
// car frame (base_footprint, z = 0 on the ground) so the height filter works
// for any sensor mounting: the EUFS car's roof LiDAR, the ADS-DV's nose
// LiDAR, or a real one. Clusters are kept if their size matches a cone.
class LidarConeDetector : public rclcpp::Node
{
public:
  LidarConeDetector()
  : Node("lidar_cone_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
    min_range_ = declare_parameter("min_range", 1.0);
    max_range_ = declare_parameter("max_range", 12.0);
    min_height_ = declare_parameter("min_height", 0.03);
    max_height_ = declare_parameter("max_height", 0.5);
    cluster_tolerance_ = declare_parameter("cluster_tolerance", 0.3);
    max_cluster_points_ = declare_parameter("max_cluster_points", 2000);
    max_cone_width_ = declare_parameter("max_cone_width", 0.5);
    auto cloud_topic = declare_parameter("pointcloud_topic", std::string("/velodyne_points"));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarConeDetector::cloudCallback, this, std::placeholders::_1)
    );

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    cone_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/lidar/cones", qos
    );

    RCLCPP_INFO(this->get_logger(), "LiDAR cone detector started");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Sensor -> car frame. The mounting is fixed, so the latest transform is fine.
    Eigen::Affine3f sensor_to_base;
    try {
      auto t = tf_buffer_.lookupTransform(
        base_frame_, msg->header.frame_id, tf2::TimePointZero);
      const auto &tr = t.transform.translation;
      const auto &r = t.transform.rotation;
      sensor_to_base = Eigen::Translation3f(tr.x, tr.y, tr.z) *
        Eigen::Quaternionf(r.w, r.x, r.y, r.z);
    } catch (...) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No transform %s -> %s yet", msg->header.frame_id.c_str(), base_frame_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, sensor_to_base);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);


    // Donut + height filtering (car frame: z = height above ground)

    for (const auto &pt : cloud->points)
    {
      if (!std::isfinite(pt.x))
        continue;

      double r = std::hypot(pt.x, pt.y);

      // Remove car body + far noise
      if (r < min_range_ || r > max_range_)
        continue;

      // Keep only cone-height points (drops the ground and anything tall)
      if (pt.z < min_height_ || pt.z > max_height_)
        continue;

      filtered->points.push_back(pt);
    }

    //  Debug: see if cones survive filtering
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Filtered points: %zu", filtered->points.size());

    if (filtered->points.empty())
      return;


    // Euclidean clustering

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // cone size
    ec.setMinClusterSize(1);                     // far cones = single hit
    ec.setMaxClusterSize(max_cluster_points_);   // close cones on a real VLP-16
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    geometry_msgs::msg::PoseArray cones;
    cones.header.stamp = msg->header.stamp;
    cones.header.frame_id = base_frame_;


    // Compute centroids of cone-sized clusters

    for (const auto &indices : cluster_indices)
    {
      double cx = 0.0, cy = 0.0, cz = 0.0;
      float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;

      for (int idx : indices.indices)
      {
        const auto &p = filtered->points[idx];
        cx += p.x;
        cy += p.y;
        cz += p.z;
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
      }

      // Wider than a cone: a wall, barrier or another car
      if (max_x - min_x > max_cone_width_ || max_y - min_y > max_cone_width_)
        continue;

      double inv = 1.0 / indices.indices.size();
      cx *= inv;
      cy *= inv;
      cz *= inv;

      geometry_msgs::msg::Pose p;
      p.position.x = cx;
      p.position.y = cy;
      p.position.z = cz;
      p.orientation.w = 1.0;

      cones.poses.push_back(p);
    }

    cone_pub_->publish(cones);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cone_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_frame_;
  double min_range_, max_range_, min_height_, max_height_;
  double cluster_tolerance_, max_cone_width_;
  int max_cluster_points_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarConeDetector>());
  rclcpp::shutdown();
  return 0;
}
