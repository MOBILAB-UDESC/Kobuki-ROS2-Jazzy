#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory

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
        self.read_yaml()
        self.read_path()

        # Aux
        self.k       = 0
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

        # Draw the path on Rviz
        self._pub_path = self.create_publisher(Path, "/plan", 10)
        self.publish_path()

        # Publishers and subscribers
        if not self.istwist:

            self._publisher = self.create_publisher(TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)
            self.twist = TwistStamped()

            self.subscriber = self.create_subscription(Odometry, "/diff_drive_base_controller/odom", self.LQRCallback, 10)
        else:
            self._publisher = self.create_publisher(Twist, "/commands/velocity", 10)
            self.twist = Twist()

            self.subscriber = self.create_subscription(Odometry, "/odom", self.LQRCallback, 10)

    def read_yaml(self):
        with open(self.yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        self.dt   = data["dt"]
        traj      = data["type-traj"]
        eta       = data["eta-traj"]
        cycles    = data["cycl-traj"]
        type_test = data["type-test"]
        alpha     = data[type_test][0]["alpha-traj"]
        center    = np.array(data["cent-traj"])

        self.path = Trajectory(_dt = self.dt, _eta = eta, _alpha = alpha, _cycles = cycles, _center = center, _type = traj)

        self.Q_matrix = np.array([row for row in data["LQR"][0]['Q']])
        self.R_matrix = np.array([row for row in data["LQR"][1]['R']])
        self.name      = f"LQR_Q{self.Q_matrix[:3,:3].diagonal().tolist()}_R{self.R_matrix[:2,:2].diagonal().tolist()}"

        self.route  = f"{data["route"]}/{type_test}/LQR/{self.name}"
        os.makedirs(self.route, exist_ok=True)

    def read_path(self):
        self.xref, self.yref, self.thref, self.vref, self.wref = self.path.get_path()

        self._length = len(self.xref)
        self.x       = np.zeros(self._length)
        self.y       = np.zeros(self._length)
        self.th      = np.zeros(self._length)
        self.vel     = np.zeros(self._length)
        self.wel     = np.zeros(self._length)
        self.X       = np.array([0.0, 0.0, 0.0])
        self.mae     = [0, 0, 0]
        self.K       = np.zeros((2, 3))

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

        self._pub_path.publish(path_msg)
        self.get_logger().info('Path published ...')

    def apply_control(self, state, k):
        """Finds the error between the ref and the most recent state gotten from odometry callback"""
        
        if self.thref[k] - state[2] > np.pi:
            state[2] = state[2] + 2*np.pi

        # Store trajectory data
        self.x[k] = state[0]
        self.y[k] = state[1]
        self.th[k] = state[2]

        self.get_logger().info(f"State k({k}): x:{state[0]}, y:{state[1]}, th:{state[2]}")

        ref_point = np.array([self.xref[k], self.yref[k], self.thref[k]])
    
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

        # Update MAE
        x_error = abs(state[0] - self.xref[k])
        y_error = abs(state[1] - self.yref[k])
        th_error = abs(state[2] - self.thref[k])
        self.mae[0] += x_error
        self.mae[1] += y_error
        self.mae[2] += th_error

        # self.get_logger().info(f"Control applied: v={v:.3f}, w={w:.3f}, error=[{x_error:.3f}, {y_error:.3f}, {th_error:.3f}]")

    def pred_state(self, state_lqr):
        n = 200
        for i in range(n):
            state_lqr[0]  += self.vel[self.k-1] * np.cos(state_lqr[2]) * self.dt / n
            state_lqr[1]  += self.vel[self.k-1] * np.sin(state_lqr[2]) * self.dt / n
            state_lqr[2]  += self.wel[self.k-1] * self.dt / n

        return state_lqr

    def LQRCallback(self, msg):

        ####################################################### ACTUAL POSITION/ORIENTATION #######################################################
        x_actual  = msg.pose.pose.position.x
        y_actual  = msg.pose.pose.position.y
        qx        = msg.pose.pose.orientation.x
        qy        = msg.pose.pose.orientation.y
        qz        = msg.pose.pose.orientation.z
        qw        = msg.pose.pose.orientation.w
        th_actual = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        # self.get_logger().info(f"Control callback period {(time() - self.t_init):.2f} s!")

        self.t_init = time()

        print(f"k = {self.k}")

        if self.k > 0 and self.k <= self._length:
            self.apply_control(np.array([x_actual, y_actual, th_actual]), self.k-1)

        if self.k >= self._length:
            sleep(self.dt)
            if self.k > self._length:
                self.x  = np.append(self.x[1:], x_actual)
                self.y  = np.append(self.y[1:], y_actual)
                self.th = np.append(self.th[1:], th_actual)
                self.shutdown_sequence()
            self.k += 1
            return

        ####################################################### MATRIZ A #######################################################
        A = np.array([
            [0, self.wref[self.k], 0],
            [-self.wref[self.k], 0, self.vref[self.k]],
            [0, 0, 0]
            ])

        ####################################################### LQR solver #######################################################
        self.K, S, E = control.lqr(A, self.B, self.Q_matrix, self.R_matrix)

        elapsed_time = time() - self.t_init
        if elapsed_time > self.dt or self.aux < self.aux_max:
            self.get_logger().warn(f"Control callback exceeded period by {(elapsed_time - self.dt):.2f} s!")
            self.aux += 1
        else:
            sleep_ = self.dt - elapsed_time
            sleep(sleep_)
            self.k += 1

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

        # Save and draw data
        save_data(self.xref, self.yref, self.thref, self.vref, self.wref,
                  self.x, self.y, self.th, self.vel, self.wel,
                  self.route)
        
        draw_data(self.xref, self.yref, self.thref, self.vref, self.wref,
                  self.x, self.y, self.th, self.vel, self.wel,
                  self.route, self.name)
        
        mae = get_MAE(self.xref, self.yref, self.thref,
                self.x, self.y, self.th,
                self.route)
        
        metrics = get_IAU_IADU(self.vel, self.wel, self.dt, self.route)

        self.get_logger().info(f"MAE: x={mae[0]}, y={mae[1]}, theta={mae[2]} m!")
        self.get_logger().info(f"IAU: v={metrics[0]} m/s, w={metrics[1]} rad/s!")
        self.get_logger().info(f"IADU: v={metrics[2]}, w={metrics[3]}")

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LQRNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()