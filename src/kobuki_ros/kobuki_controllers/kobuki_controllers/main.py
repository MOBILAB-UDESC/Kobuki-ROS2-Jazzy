# Own packages
import MPC
from .path import Trajectory
from .tools import draw_data, save_data, get_MAE, get_IAU_IADU

# ROS2 packages
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory

# General packages
import numpy as np
from time import sleep, time
import math
import yaml
import os

class MPCNode(Node):
    def __init__(self):
        super().__init__("MPCNode")
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
        rho     = data[type_test][0]["rho-traj"]
        center    = np.array(data["cent-traj"])

        self.path = Trajectory(_dt = self.dt, _eta = eta, _rho = rho, _cycles = cycles, _center = center, _type = traj)

        self.Q_matrix = np.array([row for row in data["LQR"][0]['Q']])
        self.R_matrix = np.array([row for row in data["LQR"][1]['R']])
        self.name      = f"LQR_Q{self.Q_matrix[:3,:3].diagonal().tolist()}_R{self.R_matrix[:2,:2].diagonal().tolist()}"

        self.route  = f"{data["route"]}/{type_test}/LQR/{self.name}"
        os.makedirs(self.route, exist_ok=True)

def main():
    mpc = MPC.MPC()
    mpc.setup()
    mpc.solve()

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)