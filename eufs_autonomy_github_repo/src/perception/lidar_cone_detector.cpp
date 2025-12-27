#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cmath>

class LidarConeDetector : public rclcpp::Node
{
public:
  LidarConeDetector()
  : Node("lidar_cone_detector")
  {
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points",
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);

    
    // Donut + height filtering
    
    for (const auto &pt : cloud->points)
    {
      double r = std::hypot(pt.x, pt.y);

      // Remove car body + far noise
      if (r < 1.0 || r > 10)
        continue;

      // Keep only cone-height points
      /*if (pt.z > 0.0 || pt.z < -0.15)
        continue;*/
      if (pt.z > 0.0 )
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
    ec.setClusterTolerance(0.30);  // cone size
    ec.setMinClusterSize(1);       // far cones = single hit
    ec.setMaxClusterSize(20);      // close cones
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    geometry_msgs::msg::PoseArray cones;
    cones.header.stamp = msg->header.stamp;
    cones.header.frame_id = msg->header.frame_id; // velodyne

    
    // Compute centroids
    
    for (const auto &indices : cluster_indices)
    {
      double cx = 0.0, cy = 0.0, cz = 0.0;

      for (int idx : indices.indices)
      {
        cx += filtered->points[idx].x;
        cy += filtered->points[idx].y;
        cz += filtered->points[idx].z;
      }

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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarConeDetector>());
  rclcpp::shutdown();
  return 0;
}

