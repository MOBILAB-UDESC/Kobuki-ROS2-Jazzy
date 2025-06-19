#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from kobuki_msgs.msg import PoseVel, PoseVelVector

from .tools import draw_data, save_data

import numpy as np
import control
from time import sleep, time
import math
from scipy.interpolate import splprep, splev
class LQRNode(Node):
    def __init__(self ,dt = 0.1, trajectory = "infinite"):
        super().__init__("LQRNode")

        self.dt = dt

        self.xref, self.yref, self.thref, self.vref, self.wref = [], [], [], [], []

        self.t_init = 0

        self.k  = -1

        self.K = np.zeros((2, 3))
        self.B = np.array([
                    [-1, 0],
                    [0, 0],
                    [0, 1]
                ])
        
        self.Q = np.diag([5, 5, 5])
        self.R = np.diag([2.0, 1.0])

        self.publisher_ = self.create_publisher(TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)
        self.subscriber = self.create_subscription(PoseVelVector, "/posevelvector", self.PathFromTopic, 10)
        self.t_init = time()
        # self.timer = self.create_timer(0.1, self.LQRCallback)
        self.subscriber = self.create_subscription(Odometry, "/diff_drive_base_controller/odom", self.ControlCallback, 10)

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

    def ControlCallback(self, msg):
                
        if self.k<0 or self.k >= self.length_:
            return 

        self.t_init = time()
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
        self.K, S, E = control.lqr(A, self.B, self.Q, self.R)

        ####################################################### CONTROLE #######################################################
        u = -self.K @ e

        v = u[0]
        w = -u[1]
        
        twist = TwistStamped()
        twist.twist.linear.x  = v
        twist.twist.angular.z = w

        self.publisher_.publish(twist)   

        sleep_ = self.dt - (time() - self.t_init)
        sleep(max(sleep_, 0))

        x_error = np.absolute(self.x[self.k] - self.xref[self.k])
        y_error = np.absolute(self.y[self.k] - self.yref[self.k])
        th_error = np.absolute(self.th[self.k] - self.thref[self.k])

        if x_error > 0.01 and y_error > 0.01 and th_error > 0.1:
            return 

        print(f"Time: {sleep_} s")
        print(f"vr: {self.vref[self.k]} m/s, wr: {self.wref[self.k]} rad/s")
        print(f"v: {v} m/s, w: {w} rad/s")

        self.k += 1

    def LQRCallback(self):
        
        if self.k<0 or self.k >= self.length_:
            self.t_init = time()
            return 
        
        print(f"Time: {time()-self.t_init} s")
        print(f"v: {self.vref[self.k]} m/s, w: {self.wref[self.k]} rad/s")

        twist = TwistStamped()
        twist.twist.linear.x  = self.vref[self.k]
        twist.twist.angular.z = self.wref[self.k]
        # twist.twist.linear.x  = 0.5
        # if self.k>(self.length_/2):
        #     twist.twist.angular.z = 2.0
        # else:
        #     twist.twist.angular.z = -2.0
        self.publisher_.publish(twist)   

        self.k += 1

def main(args=None):

    rclpy.init(args=args)
    node = LQRNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
