#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class FrameIDChanger(Node):
    def __init__(self):
        super().__init__('frame_id_changer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/depth/points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/depth_camera/depth/points_fixed', 10)

    def listener_callback(self, msg):
        msg.header.frame_id = 'camera_link'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    frame_id_changer = FrameIDChanger()
    rclpy.spin(frame_id_changer)
    frame_id_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
