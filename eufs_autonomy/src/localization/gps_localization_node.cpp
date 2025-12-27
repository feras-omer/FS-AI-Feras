#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <cmath>

class GPSLocalizationNode : public rclcpp::Node
{
public:
  GPSLocalizationNode()
  : Node("gps_localization_node"),
    origin_set_(false)
  {
    // Match GPS QoS (BEST_EFFORT)
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", qos,
        std::bind(&GPSLocalizationNode::gpsCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/gps/odom", 10);

    RCLCPP_INFO(this->get_logger(), "GPS Localization Node started");
  }

private:
  static constexpr size_t FILTER_WINDOW = 10;

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "No valid GPS fix");
      return;
    }

    // Initialize local ENU origin
    if (!origin_set_)
    {
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      origin_alt_ = msg->altitude;
      origin_set_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "GPS origin set: lat=%.8f lon=%.8f alt=%.2f",
                  origin_lat_, origin_lon_, origin_alt_);
      return;
    }

    double x, y, z;
    llaToENU(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    //  Moving average filter 
    x_buffer_.push_back(x);
    y_buffer_.push_back(y);
    z_buffer_.push_back(z);

    if (x_buffer_.size() > FILTER_WINDOW)
    {
      x_buffer_.pop_front();
      y_buffer_.pop_front();
      z_buffer_.pop_front();
    }

    double x_avg = 0.0, y_avg = 0.0, z_avg = 0.0;
    for (size_t i = 0; i < x_buffer_.size(); ++i)
    {
      x_avg += x_buffer_[i];
      y_avg += y_buffer_[i];
      z_avg += z_buffer_[i];
    }

    x_avg /= x_buffer_.size();
    y_avg /= y_buffer_.size();
    z_avg /= z_buffer_.size();

    publishOdom(x_avg, y_avg, z_avg, msg->header.stamp);
  }

  // Convert latitude / longitude / altitude to local ENU
  void llaToENU(double lat, double lon, double alt,
                double &x, double &y, double &z)
  {
    constexpr double R = 6378137.0;  // Earth radius (meters)

    double d_lat = (lat - origin_lat_) * M_PI / 180.0;
    double d_lon = (lon - origin_lon_) * M_PI / 180.0;
    double lat_rad = origin_lat_ * M_PI / 180.0;

    x = d_lon * std::cos(lat_rad) * R;
    y = d_lat * R;
    z = alt - origin_alt_;
  }

  void publishOdom(double x, double y, double z, rclcpp::Time stamp)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    // IMPORTANT: GPS is a sensor, NOT the robot base
    odom.child_frame_id = "gps";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;

    // GPS does not provide orientation
    odom.pose.pose.orientation.w = 1.0;

    // Reasonable GPS covariance
    odom.pose.covariance[0]  = 5.0;   // x
    odom.pose.covariance[7]  = 5.0;   // y
    odom.pose.covariance[14] = 10.0;  // z

    odom_pub_->publish(odom);
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // GPS origin
  bool origin_set_;
  double origin_lat_, origin_lon_, origin_alt_;

  // Moving average buffers
  std::deque<double> x_buffer_;
  std::deque<double> y_buffer_;
  std::deque<double> z_buffer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}


