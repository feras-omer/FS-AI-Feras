#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

enum ConeColor { UNKNOWN = 0, BLUE = 1, YELLOW = 2 };

struct YoloDet
{
  cv::Rect box;
  int cls;
};

class CameraConeVerifier : public rclcpp::Node
{
public:
  CameraConeVerifier()
  : Node("camera_cone_verifier"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    auto qos = rclcpp::SensorDataQoS();

    cones_sub_.subscribe(this, "/lidar/cones", qos.get_rmw_qos_profile());
    yolo_sub_.subscribe(this, "/zed/cones_detections", qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(Sync(10), cones_sub_, yolo_sub_);
    sync_->registerCallback(
      std::bind(&CameraConeVerifier::syncedCb, this, std::placeholders::_1, std::placeholders::_2)
    );

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/zed/image_raw", qos,
      std::bind(&CameraConeVerifier::imageCb, this, std::placeholders::_1));

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/zed/camera_info", qos,
      std::bind(&CameraConeVerifier::camInfoCb, this, std::placeholders::_1));

    rclcpp::QoS reliable(10);
    reliable.reliable();

    verified_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/cones/verified", reliable);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/debug/projection", reliable);

    RCLCPP_INFO(get_logger(), "Camera cone verifier started and synced with YOLO detections");
  }

private:
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<
      geometry_msgs::msg::PoseArray,
      vision_msgs::msg::Detection2DArray>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (...)
    {
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

  void syncedCb(
    const std::shared_ptr<const geometry_msgs::msg::PoseArray>& cones,
    const std::shared_ptr<const vision_msgs::msg::Detection2DArray>& yolo)
  {
    if (!cam_ready_ || last_image_.empty())
      return;

    std::vector<YoloDet> dets;
    dets.reserve(yolo->detections.size());

    for (const auto &d : yolo->detections)
    {
      if (d.results.empty())
        continue;

      int cls = std::stoi(d.results[0].hypothesis.class_id);

      const auto &b = d.bbox;
      int x = static_cast<int>(b.center.x - b.size_x * 0.5);
      int y = static_cast<int>(b.center.y - b.size_y * 0.5);
      int w = static_cast<int>(b.size_x);
      int h = static_cast<int>(b.size_y);

      dets.push_back({cv::Rect(x, y, w, h), cls});
    }

    geometry_msgs::msg::PoseArray out;
    out.header = cones->header;

    cv::Mat debug = last_image_.clone();
    bool any = false;
    const int BOX = 15;

    for (const auto &p : cones->poses)
    {
      geometry_msgs::msg::PoseStamped in, cam;
      in.header = cones->header;
      in.pose = p;

      try
      {
        cam = tf_buffer_.transform(
          in,
          "zed_left_camera_optical_frame",
          tf2::durationFromSec(0.05));
      }
      catch (...)
      {
        continue;
      }

      double X = -cam.pose.position.y;
      double Y = -cam.pose.position.z;
      double Z =  cam.pose.position.x;

      if (Z <= 0.5)
        continue;

      int u = static_cast<int>(fx_ * X / Z + cx_);
      int v = static_cast<int>(fy_ * Y / Z + cy_);

      if (u < 0 || v < 0 || u >= debug.cols || v >= debug.rows)
        continue;

      int color = UNKNOWN;

      for (const auto &d : dets)
      {
        if (d.box.contains(cv::Point(u, v)))
        {
          if (d.cls == 2)
            color = BLUE;
          else if (d.cls == 0)
            color = YELLOW;
          else
            color = UNKNOWN;
          break;
        }
      }

      cv::Scalar c;
      if (color == BLUE)
        c = cv::Scalar(255, 0, 0);
      else if (color == YELLOW)
        c = cv::Scalar(0, 255, 255);
      else
        c = cv::Scalar(0, 0, 255);

      cv::rectangle(
        debug,
        cv::Point(u - BOX, v - BOX),
        cv::Point(u + BOX, v + BOX),
        c,
        2);

      if (color != UNKNOWN)
      {
        auto pose = p;
        pose.orientation.z = static_cast<double>(color);
        pose.orientation.w = 1.0;
        out.poses.push_back(pose);
        any = true;
      }
    }

    debug_pub_->publish(*cv_bridge::CvImage({}, "bgr8", debug).toImageMsg());

    if (!any)
      out = *cones;

    verified_pub_->publish(out);
  }

  message_filters::Subscriber<geometry_msgs::msg::PoseArray> cones_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> yolo_sub_;
  std::shared_ptr<Sync> sync_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr verified_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

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
