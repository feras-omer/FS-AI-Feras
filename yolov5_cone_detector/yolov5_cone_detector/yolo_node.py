#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from cv_bridge import CvBridge
import torch
import cv2


class YOLOv5ConeNode(Node):
    def __init__(self):
        super().__init__('yolov5_cone_node')

        self.get_logger().info("Loading YOLOv5 model...")

        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom',
            path='.../FS-AI-Feras/best.pt'
        )

        self.model.conf = 0.5
        self.model.iou = 0.45
        self.model.eval()

        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("YOLOv5 running on GPU")

        self.bridge = CvBridge()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            Image,
            '/zed/image_raw',
            self.image_callback,
            qos
        )

        self.pub = self.create_publisher(
            Detection2DArray,
            '/zed/cones_detections',
            qos
        )

        self.get_logger().info("YOLOv5 node ready")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        results = self.model(cv_image, size=640)
        detections = results.xyxy[0].cpu().numpy()

        out = Detection2DArray()
        out.header = msg.header   # CRITICAL: preserve timestamp

        for x1, y1, x2, y2, conf, cls in detections:
            d = Detection2D()
            d.bbox.center.x = float((x1 + x2) * 0.5)
            d.bbox.center.y = float((y1 + y2) * 0.5)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)

            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(int(cls))
            h.hypothesis.score = float(conf)

            d.results.append(h)
            out.detections.append(d)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YOLOv5ConeNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

