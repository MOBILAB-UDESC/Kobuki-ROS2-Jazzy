from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    packageName  = "kobuki_description"
    bridgeparams = "bridge_parameters.yaml"
    packagePath = get_package_share_directory(packageName)

    # Launch world
    world_path = PathJoinSubstitution([packagePath, 'worlds', 'empty.sdf'])

    gazebo_world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments=[('gz_args', ['-r ', LaunchConfiguration('world_path')])]
    )

    # Spawn the robot gotten from the robot_description topic
    gazebo_spawn_node = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'kobuki',
                   '-allow_renaming', 'true',
                   '-z', '-0.289810']
    )

    # Enable the bridge between ROS 2 and Gazebo
    gazebo_bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', PathJoinSubstitution([packagePath, 'config', bridgeparams])]
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Declare launch arguments
    args = [
        DeclareLaunchArgument('world_path', default_value=world_path, description='Path to the Gazebo world file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
    ]

    return LaunchDescription([
        *args,
        gazebo_world_node,
        gazebo_spawn_node,
        gazebo_bridge_node
    ])