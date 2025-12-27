#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

enum ConeColor { UNKNOWN = 0, BLUE = 1, YELLOW = 2 };

class CameraConeVerifier : public rclcpp::Node
{
public:
  CameraConeVerifier()
  : Node("camera_cone_verifier"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    auto qos = rclcpp::SensorDataQoS();

    cones_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/lidar/cones", qos,
      std::bind(&CameraConeVerifier::conesCb, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/zed/image_raw", qos,
      std::bind(&CameraConeVerifier::imageCb, this, std::placeholders::_1));

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/zed/camera_info", qos,
      std::bind(&CameraConeVerifier::camInfoCb, this, std::placeholders::_1));

    rclcpp::QoS reliable(rclcpp::KeepLast(10));
    reliable.reliable();

    verified_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/cones/verified", reliable);

    blue_mask_pub_   = create_publisher<sensor_msgs::msg::Image>("/debug/blue_mask", reliable);
    yellow_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("/debug/yellow_mask", reliable);
    debug_img_pub_   = create_publisher<sensor_msgs::msg::Image>("/debug/projection", reliable);

    RCLCPP_INFO(get_logger(), "Camera cone verifier started");
  }

private:
  
  // Image & Camera Info
  

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      last_image_.release();
    }
  }

  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    cam_ready_ = true;
  }

  
  // Main Callback
  

  void conesCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!cam_ready_ || last_image_.empty())
      return;

    cv::Mat hsv;
    cv::cvtColor(last_image_, hsv, cv::COLOR_BGR2HSV);

    cv::Mat blue_mask, yellow_mask;
    cv::inRange(
      hsv,
      cv::Scalar(90, 80, 50),
      cv::Scalar(130, 255, 255),
      blue_mask
    );

    cv::inRange(
      hsv,
      cv::Scalar(20, 80, 50),
      cv::Scalar(40, 255, 255),
      yellow_mask
    );

    publishMask(blue_mask, blue_mask_pub_);
    publishMask(yellow_mask, yellow_mask_pub_);

    geometry_msgs::msg::PoseArray out;
    out.header = msg->header;

    cv::Mat debug = last_image_.clone();
    bool any_verified = false;

    const int BOX_SIZE = 15; // box half-width for debug

    for (const auto &p : msg->poses)
    {
      geometry_msgs::msg::PoseStamped in, cam;
      in.header = msg->header;
      in.pose = p;

      try {
        cam = tf_buffer_.transform(
          in, "zed_left_camera_optical_frame",
          tf2::durationFromSec(0.05));
      } catch (...) {
        continue;
      }

      
      // Axis remap (robot→optical)
      
      double Xr = cam.pose.position.x;
      double Yr = cam.pose.position.y;
      double Zr = cam.pose.position.z;

      double X = -Yr;   // right
      double Y = -Zr;   // down
      double Z =  Xr;   // forward (depth)

      if (Z <= 0.5)
        continue;

      int u = static_cast<int>(fx_ * X / Z + cx_);
      int v = static_cast<int>(fy_ * Y / Z + cy_);

      if (u < 0 || v < 0 ||
          u >= debug.cols || v >= debug.rows)
        continue;

      int color = UNKNOWN;
      if (hitMask(blue_mask, u, v, 5))   color = BLUE;
      if (hitMask(yellow_mask, u, v, 5)) color = YELLOW;

      
      // Draw visible colored boxes
      
      cv::Scalar box_color;
      if (color == BLUE)      box_color = cv::Scalar(255, 0, 0);   // Blue box
      else if (color == YELLOW) box_color = cv::Scalar(0, 255, 255); // Yellow box
      else                    box_color = cv::Scalar(0, 0, 255);   // Red for unknown

      // top-left and bottom-right
      cv::Point tl(u - BOX_SIZE, v - BOX_SIZE);
      cv::Point br(u + BOX_SIZE, v + BOX_SIZE);

      cv::rectangle(debug, tl, br, box_color, 2);

      if (color != UNKNOWN)
      {
        geometry_msgs::msg::Pose pose = p;
        pose.orientation.z = color;
        pose.orientation.w = 1.0;
        out.poses.push_back(pose);
        any_verified = true;
      }
    }

    publishDebug(debug);

    if (!any_verified)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No verified cones found — passing LiDAR cones through");
      out = *msg;
    }

    verified_pub_->publish(out);
  }

  
  // Helpers
  

  bool hitMask(const cv::Mat &mask, int u, int v, int r)
  {
    for (int du = -r; du <= r; ++du)
      for (int dv = -r; dv <= r; ++dv)
      {
        int xx = u + du;
        int yy = v + dv;
        if (xx < 0 || yy < 0 || xx >= mask.cols || yy >= mask.rows)
          continue;
        if (mask.at<uint8_t>(yy, xx) > 0)
          return true;
      }
    return false;
  }

  void publishMask(const cv::Mat &m,
                   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p)
  {
    p->publish(*cv_bridge::CvImage({}, "mono8", m).toImageMsg());
  }

  void publishDebug(const cv::Mat &img)
  {
    debug_img_pub_->publish(*cv_bridge::CvImage({}, "bgr8", img).toImageMsg());
  }

  
  // ROS
  

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cones_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr verified_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr blue_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yellow_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::Mat last_image_;
  bool cam_ready_{false};
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraConeVerifier>());
  rclcpp::shutdown();
  return 0;
}

