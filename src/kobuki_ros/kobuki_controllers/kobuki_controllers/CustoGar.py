#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory

from .path import Trajectory
from .tools import draw_data, save_data

import numpy as np
import matlab.engine
from time import sleep, time
import math

class LMIsNode(Node):
    def __init__(self ,dt = 0.08, trajectory = "infinite"):
        super().__init__("LMIsNode")

        # Initializing Matlab from Python
        self.eng = matlab.engine.start_matlab()
        package_route = get_package_share_directory('kobuki_controllers')
        self.eng.addpath(self.eng.genpath(package_route+'/controllers_matlab'), nargout=0)
        
        self.dt = dt
        self.trajectory = trajectory
        self.path = Trajectory(self.dt, self.trajectory)
        self.xref, self.yref, self.thref, self.vref, self.wref = self.path.get_path()
        self.B = np.array([
                    [-1, 0],
                    [0, 0],
                    [0, 1]
                ])
        self.length_ = len(self.xref)
        self.x = np.zeros(self.length_)
        self.y = np.zeros(self.length_)
        self.th = np.zeros(self.length_)
        self.vel = np.zeros(self.length_)
        self.wel = np.zeros(self.length_)
        self.X = np.array([0.0, 0.0, 0.0])
        self.mae = [0, 0, 0]
        self.K = np.zeros((2, 3))

        # Auxiliar parameteres
        self.k  = 0
        self.aux = 0

        # LMIs matrices and parameters
        self.vmax = 0.15
        self.wmax = 2.0
        self.Q = np.zeros((3, 3))
        self.Y = np.zeros((2, 3))
        self.name  = "/home/nilton/Desktop/Kobuki/TestesRealSlow/"+"LMIsCG_5-5-5-0-0-1.5-1.5.png"
        self.route = "/home/nilton/Desktop/Kobuki/TestesRealSlow/LMIsCG_5-5-5-0-0-1.5-1.5"

         # To draw the path on Rviz
        self.publ_path_ = self.create_publisher(Path, "/plan", 10)
        self.publish_path()

        # self.publisher_ = self.create_publisher(Twist, "/diff_drive_base_controller/cmd_vel", 10) # /cmd_vel or /commands/velocity
        self.publisher_ = self.create_publisher(TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)
        self.subscriber = self.create_subscription(Odometry, "/diff_drive_base_controller/odom", self.LMIsCallback, 10)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, th in zip(self.xref, self.yref, self.thref):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            qz, qw = np.sin(th / 2.0), np.cos(th / 2.0)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.publ_path_.publish(path_msg)
        self.get_logger().info('Publishing Path to /plan')

    def LMIsCallback(self, msg):
        try:
            t_init = time()
            ####################################################### MATRIZ A #######################################################
            A = np.array([
                [0, self.wref[self.k], 0],
                [-self.wref[self.k], 0, self.vref[self.k]],
                [0, 0, 0]
                ])
    
            ####################################################### ACTUAL POSITION/ORIENTATION #######################################################
            x_actual  = msg.pose.pose.position.x
            y_actual  = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            th_actual = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

            if self.thref[self.k] - th_actual > np.pi:
                th_actual = th_actual + 2*np.pi

            self.x[self.k] = x_actual
            self.y[self.k] = y_actual
            self.th[self.k] = th_actual

            self.X = np.array([x_actual, y_actual, th_actual])

            ####################################################### ERROR #######################################################
            ref_point = np.array([self.xref[self.k], self.yref[self.k], self.thref[self.k]])
        
            R_theta = np.array([
                [np.cos(self.th[self.k]), np.sin(self.th[self.k]), 0],
                [-np.sin(self.th[self.k]), np.cos(self.th[self.k]), 0],
                [0, 0, 1]
            ])

            e = R_theta @ (ref_point - self.X)

            ####################################################### GANHOS #######################################################
            A_matlab = matlab.double(A.tolist())
            B_matlab = matlab.double(self.B.tolist())
            x_matlab = matlab.double(self.X)
            self.Y = matlab.double(self.Y)
            self.Q = matlab.double(self.Q)

            # self.K = np.array(self.eng.LMIsEs(A_matlab, B_matlab))
            K, Y, Q = self.eng.LMIsCustoGarantido(A_matlab, B_matlab, x_matlab, self.Q, self.Y, nargout=3)
            # K, Y, Q = self.eng.LMIsRest(A_matlab, B_matlab, x_matlab, 0.15, 1, self.Q, self.Y, nargout=3)
            
            self.K = np.array(K)
            self.Y = np.array(Y)
            self.Q = np.array(Q)

            t_end = time() - t_init

            # To make sure the sampling time is "self.dt" miliseconds
            sleep_ = max(0.0, self.dt - t_end)
            sleep(sleep_)   

            ####################################################### CONTROLE #######################################################
            u = -self.K @ e

            if (self.aux > 10):
                v = u[0]
                w = -u[1]

                # v = np.clip(v, -1.5, 1.5)
                # w = np.clip(w, -6.6, 6.6)
            else:
                v = 0.0
                w = 0.0

            ####################################################### PUBLISHING VELOCITIES #######################################################
            # In case Twist is being used uncomment
            # twist = Twist()
            # twist.linear.x = v
            # twist.angular.z = w      

            twist = TwistStamped()
            twist.twist.linear.x = v
            twist.twist.angular.z = w

            self.vel[self.k] = v
            self.wel[self.k] = w

            self.publisher_.publish(twist)   

            ####################################################### ? #######################################################
            if (self.aux > 10):

                x_error = np.absolute(self.x[self.k] - self.xref[self.k])
                y_error = np.absolute(self.y[self.k] - self.yref[self.k])
                th_error = np.absolute(self.th[self.k] - self.thref[self.k])

                self.k += 1
                self.mae[0] += np.power(x_error, 1)/self.length_
                self.mae[1] += np.power(y_error, 1)/self.length_
                self.mae[2] += np.power(th_error, 1)/self.length_
            else:
                self.aux += 1

        except Exception as e:
            # Shutting down Matlab
            self.eng.quit()

            # Stopping the robot
            twist = TwistStamped()
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0
            self.publisher_.publish(twist)

            save_data(self.xref, self.yref, self.thref, self.vref, self.wref,
                      self.x, self.y, self.th, self.vel, self.wel,
                      self.route)
            
            draw_data(self.xref, self.yref, self.thref, self.vref, self.wref,
                      self.x, self.y, self.th, self.vel, self.wel,
                      self.name)

            self.get_logger().error(f'Error in LMIsCallback: {e}')
            rclpy.shutdown()

def main(args=None):

    rclpy.init(args=args)
    node = LMIsNode()
    rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == "__main__":
    main()