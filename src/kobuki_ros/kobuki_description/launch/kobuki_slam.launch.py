from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    slam_launch_path = PathJoinSubstitution([get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'])

    slam_params_file = PathJoinSubstitution([get_package_share_directory('kobuki_description'), 'config', 'slam_parameters.yaml'])

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    args = [
        DeclareLaunchArgument('slam_params', default_value=slam_params_file, description='Path to the SLAM parameters file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    ]

    return LaunchDescription([
        *args,
        slam_toolbox,
    ])