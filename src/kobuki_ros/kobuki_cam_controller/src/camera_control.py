#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
import supervision as sv

class OpenCVNode(Node):
    def __init__(self):
        super().__init__("OpenCVNode")
        
        self.publisher_ = self.create_publisher(Image, "/depth_camera/color/image_raw", 10)
        self.br = CvBridge()
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(self.cfg)
        
        self.model = YOLO('/home/kobuki/Desktop/kobuki_ws/src/kobuki_ros/kobuki_cam_controller/src/best.pt')
        self.bounding_box_annotator = sv.BoxAnnotator()

        # Timer to call the ImageCallback function at a regular interval
        self.create_timer(0.1, self.ImageCallback)  # Call every 0.1 seconds

    def ImageCallback(self):
        try:
            frames = self.pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)
            color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)

            results = self.model(color_image)[0]
            detections = sv.Detections.from_ultralytics(results)

            color_image = self.bounding_box_annotator.annotate(scene=color_image, detections=detections)

            # cv2.imshow('rgb', color_image)

            # Convert to ROS Image message
            msg = self.br.cv2_to_imgmsg(color_image, "bgr8")
            self.publisher_.publish(msg)

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in ImageCallback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()