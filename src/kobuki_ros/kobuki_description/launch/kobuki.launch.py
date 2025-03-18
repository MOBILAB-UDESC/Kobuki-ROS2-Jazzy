from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Launch Arguments
    use_sim_time   = LaunchConfiguration('use_sim_time', default=True)
    namePackage    = "kobuki_description"
    modelFile      = 'kobuki_sim.xacro'
    controllerFile = 'kobuki_control.yaml'
    worldFile      = 'playground.sdf'
    rvizFile       = 'rviz_kobuki.rviz'

    # **************************** ROBOT DESCRIPTION ****************************
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare(namePackage),
                'model',
                modelFile
            ]),
        ]
    )

    # **************************** ROBOT STATE ****************************
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters = [{
        'robot_description': robot_description_content,
        'use_sim_time'     : use_sim_time,
    }]
    )

    # **************************** CONTROLLERS ****************************

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(namePackage),
            'config',
            controllerFile,
        ]
    )

    # **************************** GAZEBO ****************************

    pathWorldFile   = os.path.join(get_package_share_directory(namePackage), 'worlds', worldFile)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # **************************** GZ BRIDGE ****************************

    bridge_params = os.path.join(get_package_share_directory(namePackage), 'config', 'bridge_parameters.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments  = [
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )


    # **************************** RVIZ2 ****************************

    pathRvizConfig     = os.path.join(get_package_share_directory(namePackage), 'config', rvizFile)

    spawnModelNodeRviz = Node(
        package    = 'rviz2',
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["-d", pathRvizConfig],
    )

    twist_remap = Node(
        package='kobuki_description',
        executable='twist_to_twiststamped',
        name='TwistToTwistStamped',
        output='screen',
    )

    ld = LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            # launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
            launch_arguments=[('gz_args', [f'-r {pathWorldFile}'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        node_robot_state_publisher,
        spawnModelNodeRviz,
        bridge,
        gz_spawn_entity,
        twist_remap,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
    return ld
