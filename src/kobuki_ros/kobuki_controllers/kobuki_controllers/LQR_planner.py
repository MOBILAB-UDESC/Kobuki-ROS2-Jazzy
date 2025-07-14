#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory
import tf2_ros
from kobuki_msgs.msg import PoseVel, PoseVelVector

from .path import Trajectory
from .tools import draw_data, save_data, get_MAE, get_IAU_IADU

import numpy as np
from time import sleep, time
import math
import yaml
import os
import control

class LQRNode(Node):

    def __init__(self):
        super().__init__("LQRNode")
        self.yaml_file = get_package_share_directory("kobuki_controllers")+"/config/controllers_param.yaml"
        self.path_following = False
        self.dt = 0.1

        # Aux
        self.k       = -1
        self.aux     = 0
        self.aux_max = 10
        self.t_init  = 0.0

        #
        self.B = np.array([
                    [-1, 0],
                    [0, 0],
                    [0, 1]
                ])
        
        #
        self.istwist = False

        #
        self.vmax = 0.7
        self.wmax = 1.91
        self.current_K = np.zeros((2, 3))
        self.Q = np.diag([3.0, 3.0, 3.0])
        self.R = np.diag([1.0, 1.0])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Draw the path on Rviz
        self.subscriber = self.create_subscription(PoseVelVector, "/posevelvector", self.PathFromTopic, 10)

        # Publishers and subscribers
        if self.path_following:
            callback = self.FollowCallback
        else:
            callback = self.TrackCallback
        if not self.istwist:

            self._publisher = self.create_publisher(TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)
            self.twist = TwistStamped()

            self.subscriber = self.create_subscription(Odometry, "/diff_drive_base_controller/odom", callback, 10)
        else:
            self._publisher = self.create_publisher(Twist, "/commands/velocity", 10)
            self.twist = Twist()

            self.subscriber = self.create_subscription(Odometry, "/odom", callback, 10)

    def PathFromTopic(self, msg):

        self.length_ = msg.num_points

        self.xref = np.array([pt.x for pt in msg.points])
        self.yref = np.array([pt.y for pt in msg.points])
        self.thref = np.array([pt.theta for pt in msg.points])
        self.vref = np.array([pt.v for pt in msg.points])
        self.wref = np.array([pt.w for pt in msg.points])

        self.x = np.zeros(self.length_)
        self.y = np.zeros(self.length_)
        self.th = np.zeros(self.length_)
        self.vel = np.zeros(self.length_)
        self.wel = np.zeros(self.length_)
        self.X = np.array([0.0, 0.0, 0.0])

        self.k = 0

    def apply_control(self, state, k):
        """Finds the error between the ref and the most recent state gotten from odometry callback"""
        
        if self.thref[k] - state[2] > np.pi:
            state[2] = state[2] + 2*np.pi

        # Store trajectory data
        self.x[k] = state[0]
        self.y[k] = state[1]
        self.th[k] = state[2]

        ref_point = np.array([self.xref[k], self.yref[k], self.thref[k]])

        # self.get_logger().info(f"State ref k: x:{self.xref[k]}, y:{self.xref[k]}, th:{self.xref[k]}")
    
        R_theta = np.array([
            [np.cos(state[2]), np.sin(state[2]), 0],
            [-np.sin(state[2]), np.cos(state[2]), 0],
            [0, 0, 1]
        ])

        e = R_theta @ (ref_point - state)
        u = -self.K @ e

        v = np.clip(u[0], -self.vmax, self.vmax)
        w = np.clip(-u[1], -self.wmax, self.wmax)

        if not self.istwist:
            self.twist.twist.linear.x = v
            self.twist.twist.angular.z = w
        else:
            self.twist.linear.x = v
            self.twist.angular.z = w

        self.vel[k] = v
        self.wel[k] = w

        self._publisher.publish(self.twist)

        # self.get_logger().info(f"Control applied: v={v:.3f}, w={w:.3f}, error=[{x_error:.3f}, {y_error:.3f}, {th_error:.3f}]")

    def pred_state(self, state_lqr):
        n = 200
        for i in range(n):
            state_lqr[0]  += self.vel[self.k-1] * np.cos(state_lqr[2]) * self.dt / n
            state_lqr[1]  += self.vel[self.k-1] * np.sin(state_lqr[2]) * self.dt / n
            state_lqr[2]  += self.wel[self.k-1] * self.dt / n

        return state_lqr

    def TrackCallback(self, msg):

        if self.k < 0:
            return

        ####################################################### ACTUAL POSITION/ORIENTATION #######################################################
        x_actual  = msg.pose.pose.position.x
        y_actual  = msg.pose.pose.position.y
        qx        = msg.pose.pose.orientation.x
        qy        = msg.pose.pose.orientation.y
        qz        = msg.pose.pose.orientation.z
        qw        = msg.pose.pose.orientation.w
        th_actual = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        # self.get_logger().info(f"State k: x:{x_actual}, y:{y_actual}, th:{th_actual}")
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time())
            x_tf = trans.transform.translation.x
            y_tf = trans.transform.translation.y
            q_tf = trans.transform.rotation
            siny_cosp = 2 * (q_tf.w * q_tf.z + q_tf.x * q_tf.y)
            cosy_cosp = 1 - 2 * (q_tf.y * q_tf.y + q_tf.z * q_tf.z)
            th_tf = math.atan2(siny_cosp, cosy_cosp)

            x_map = x_tf + x_actual * np.cos(th_tf) - y_actual * np.sin(th_tf)
            y_map = y_tf + x_actual * np.sin(th_tf) + y_actual * np.cos(th_tf)
            th_map = th_tf + th_actual

            x_actual = x_map
            y_actual = y_map
            th_actual = th_map
        except Exception as e:
            self.get_logger().warn(f"No tf map->odom: {e}")

        # self.get_logger().info(f"State transform k: x:{x_actual}, y:{y_actual}, th:{th_actual}")

        # self.get_logger().info(f"Control callback period {(time() - self.t_init):.2f} s!")

        self.t_init = time()

        print(f"k = {self.k}")

        if self.k > 0 and self.k <= self.length_:
            self.apply_control(np.array([x_actual, y_actual, th_actual]), self.k-1)

        if self.k >= self.length_:
            sleep(self.dt)
            # if self.k > self.length_:
            #     self.x  = np.append(self.x[1:], x_actual)
            #     self.y  = np.append(self.y[1:], y_actual)
            #     self.th = np.append(self.th[1:], th_actual)
            #     self.shutdown_sequence()
            self.k += 1
            return
        
        ####################################################### MATRIZ A #######################################################
        A = np.array([
            [0, self.wref[self.k], 0],
            [-self.wref[self.k], 0, self.vref[self.k]],
            [0, 0, 0]
            ])

        ####################################################### LQR solver #######################################################
        self.K, S, E = control.lqr(A, self.B, self.Q, self.R)

        elapsed_time = time() - self.t_init
        
        if elapsed_time > self.dt or self.aux < self.aux_max:
            self.get_logger().warn(f"Control callback exceeded period by {(elapsed_time - self.dt):.2f} s!")
            self.aux += 1
        else:
            sleep_ = self.dt - elapsed_time
            sleep(sleep_)
            self.k += 1

    def FollowCallback(self, msg):

        if self.k < 0 or self.k >= self.length_:
            return

        print(f"k = {self.k}")

        ####################################################### ACTUAL POSITION/ORIENTATION #######################################################
        x_actual  = msg.pose.pose.position.x
        y_actual  = msg.pose.pose.position.y
        qx        = msg.pose.pose.orientation.x
        qy        = msg.pose.pose.orientation.y
        qz        = msg.pose.pose.orientation.z
        qw        = msg.pose.pose.orientation.w
        th_actual = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        try:
            tr = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time())
            x_tf = tr.transform.translation.x
            y_tf = tr.transform.translation.y
            q_tf = tr.transform.rotation
            siny_cosp = 2 * (q_tf.w * q_tf.z + q_tf.x * q_tf.y)
            cosy_cosp = 1 - 2 * (q_tf.y * q_tf.y + q_tf.z * q_tf.z)
            th_tf = math.atan2(siny_cosp, cosy_cosp)

            x_map = x_tf + x_actual * np.cos(th_tf) - y_actual * np.sin(th_tf)
            y_map = y_tf + x_actual * np.sin(th_tf) + y_actual * np.cos(th_tf)
            th_map = th_tf + th_actual

            x_actual = x_map
            y_actual = y_map
            th_actual = th_map
        except Exception as e:
            self.get_logger().warn(f"No tf map->odom: {e}")

        if self.k < self.length_:
            ref = np.array([self.xref[self.k], self.yref[self.k], self.thref[self.k]])
            state = np.array([x_actual, y_actual, th_actual])
            dist = np.linalg.norm(ref[:2] - state[:2])
            angle_error = abs((ref[2] - state[2] + np.pi) % (2 * np.pi) - np.pi)

            pos_tol = 0.05
            ang_tol = 0.10

            if dist < pos_tol and angle_error < ang_tol:
                self.k += 1

        # self.get_logger().info(f"State k: x:{x_actual}, y:{y_actual}, th:{th_actual}")

        # self.get_logger().info(f"Control callback period {(time() - self.t_init):.2f} s!")

        ####################################################### MATRIZ A #######################################################
        A = np.array([
            [0, self.wref[self.k], 0],
            [-self.wref[self.k], 0, self.vref[self.k]],
            [0, 0, 0]
            ])

        ####################################################### LQR solver #######################################################
        self.K, S, E = control.lqr(A, self.B, self.Q, self.R)

        if self.k < self.length_:
            self.apply_control(np.array([x_actual, y_actual, th_actual]), self.k)
            sleep(0.020)

        

    def shutdown_sequence(self):
        """Clean shutdown sequence"""
        # Stop the robot
        if not self.istwist:
            self.twist.twist.linear.x = 0.0
            self.twist.twist.angular.z = 0.0
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LQRNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()