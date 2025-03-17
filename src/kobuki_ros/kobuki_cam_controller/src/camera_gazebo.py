#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import supervision as sv

class OpenCVNode(Node):
    def __init__(self):
        super().__init__("OpenCVNode")
        
        self.subscriber_dep_ = self.create_subscription(Image, "/depth_camera/color/image_raw", self.DepthImageCallback, 10)
        self.subscriber_rgb_ = self.create_subscription(Image, "/rgb_camera/color/image_raw", self.RGBImageCallback, 10)
        self.br = CvBridge()
        
        self.model = YOLO('/home/nilton/Desktop/Ros2_Projects/Kobuki/src/kobuki_ros/kobuki_cam_controller/src/best.pt')
        self.bounding_box_annotator = sv.BoxAnnotator()

        self.fy = 381.36
        self.fz = 381.36
        self.cy = 320
        self.cz = 240
    def RGBImageCallback(self, msg):
        try:
            frames = self.br.imgmsg_to_cv2(msg, "bgr8")
            # gray_frame = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)

            results = self.model(frames)[0]
            detections = sv.Detections.from_ultralytics(results)
            image = self.bounding_box_annotator.annotate(scene=frames, detections=detections)
            try:
                for detection in detections:

                    center = [int((detection[0][1] + detection[0][3])/2), int((detection[0][0] + detection[0][2])/2)]
                    print(center)
                    d = float(depth_image[center[0]][center[1]])
                    Y = -1*d*(center[1] - self.cy)/self.fy
                    Z = 0.13938 - d*(center[0] - self.cz)/self.fz - 0.05199
                    X = d + 0.195
                    text = "("+str(round(X,3))+","+str(round(Y,3))+","+str(round(Z,3))+")"

                    cv2.putText(
                            image,
                            text,
                            (center[1]-25,center[0]-18),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.35,
                            (255, 255, 255),
                            1,
                    )
                    print(f"X: {X}, Y: {Y}")
            except Exception as e:
                print(e)
            

            # Convert to ROS Image message
            cv2.imshow("RGB_Camera", image)
            cv2.waitKey(10)

        except Exception as e:
            self.get_logger().error(f'Error in ImageCallback: {e}')

    def DepthImageCallback(self, msg):
        global depth_image
        try:
            depth_image = self.br.imgmsg_to_cv2(msg, "32FC1")

            # cv2.imshow('rgb', color_image)

            # Convert to ROS Image message
            # cv2.imshow("Depth_Camera", depth_image)
            # cv2.waitKey(10)

        except Exception as e:
            self.get_logger().error(f'Error in ImageCallback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import numpy as np

# class PurePursuitController(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_controller')
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
#         self.path_points = [(1, 1), (2, 2), (3, 3)]  # Define aquí la trayectoria deseada
#         self.L_d = 0.5
#         self.v_max = 0.2
#         self.current_position = None
#         self.current_orientation = None

#     def odom_callback(self, msg):
#         self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
#         self.current_orientation = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

#         self.get_logger().error(f'Current Position: {self.current_position}')
#         self.get_logger().error(f'Current Orientation: {self.current_orientation}')

#         if self.current_position and self.current_orientation:
#             v, omega = self.pure_pursuit()
#             self.send_velocity_command(v, omega)

#     def get_yaw_from_quaternion(self, orientation):
#         siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
#         cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
#         return np.arctan2(siny_cosp, cosy_cosp)

#     def pure_pursuit(self):
#         x, y = self.current_position
#         theta = self.current_orientation
        
#         target_point = None
#         for point in self.path_points:
#             dist = np.sqrt((point[0] - x)**2 + (point[1] - y)**2)
#             if dist >= self.L_d:
#                 target_point = point
#                 break
#         if target_point is None:
#             return 0.0, 0.0

#         alpha = np.arctan2(target_point[1] - y, target_point[0] - x) - theta
#         v = self.v_max
#         omega = (2 * v * np.sin(alpha)) / self.L_d
#         return v, omega

#     def send_velocity_command(self, v, omega):
#         msg = Twist()
#         msg.linear.x = v
#         msg.angular.z = omega
#         self.publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     pure_pursuit_controller = PurePursuitController()
#     rclpy.spin(pure_pursuit_controller)
#     pure_pursuit_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
