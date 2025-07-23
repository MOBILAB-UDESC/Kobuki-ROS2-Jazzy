from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def slam_launch(context, *args, **kwargs):
    '''
        Function to include the appropriate SLAM launch file based on the slam_type argument.
        args[0] is the path to the package containing the SLAM launch files.
        args[1] is the use_sim_time argument.
    '''
    slam_type = LaunchConfiguration('slam_type').perform(context)

    if slam_type == 'slam':
        slam_launch_file = 'kobuki_slam.launch.py'
    elif slam_type == 'vslam':
        slam_launch_file = 'kobuki_vslam.launch.py'
    else:
        return []

    slam_launch_path = PathJoinSubstitution([args[0], 'launch', slam_launch_file])

    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch_path),
                                     launch_arguments={'use_sim_time': args[1], 'slam_params': slam_launch_path}.items())]

def generate_launch_description():
    
    namePackage   = "kobuki_description"
    nameRviz2File = "rviz_kobuki.rviz"
    packagePath   = get_package_share_directory(namePackage)

    use_sim_time = LaunchConfiguration('use_sim_time')

    ######################################################### Robot State Node #########################################################
    robot_description = Command(['xacro ', 
                                 PathJoinSubstitution([packagePath, 'model', LaunchConfiguration('robot_model')]), '_sim.xacro'])

    robot_state_node = Node(
                    package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                    parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    ########################################################### Joint State Node ###########################################################
    joint_state_publisher = Node(
                    package='joint_state_publisher', executable='joint_state_publisher', output='screen', name='joint_state_publisher',
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=IfCondition(PythonExpression(["'", use_sim_time, "' == 'false'"]))
    )

    ############################################################ RViz2 Node ############################################################
    rviz2_path = PathJoinSubstitution([packagePath, 'config', nameRviz2File])

    rviz2_node = Node(
                    package="rviz2", executable="rviz2", name="kobuki_rviz2", output="screen",
                    condition=IfCondition(LaunchConfiguration('launch_rviz2')),
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', LaunchConfiguration('rviz2_config')]
    )

    ########################################################### Gazebo Node ###########################################################
    gazebo_path = PathJoinSubstitution([packagePath, 'worlds', LaunchConfiguration('world_name')])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([packagePath, 'launch', 'kobuki_gazebo_launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={'world_path': gazebo_path, 'use_sim_time': use_sim_time}.items()
    )

    ######################################################## ROS2 Control Node ########################################################

    robot_controllers = PathJoinSubstitution([ packagePath, 'config', PythonExpression(['"', LaunchConfiguration('robot_model'), '_control.yaml"']) ])

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([packagePath, 'launch', 'kobuki_ros2_control_launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={'use_sim_time': use_sim_time, 'ros2_control_params': robot_controllers}.items()
    )

    ########################################################### EKF Launch ###########################################################
    ekf_file = PathJoinSubstitution([packagePath, 'config', 'ekf_parameters.yaml'])

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, ekf_file]
    )

    ######################################################## Twist Remap Node ########################################################
    twist_remap_node = Node(
                    package='kobuki_description', executable='twist_to_twiststamped', output='screen', name='twist_remap',
                    parameters=[{'use_sim_time': use_sim_time}]
    )

    ############################################################# LAUNCH #############################################################

    args = [
        DeclareLaunchArgument('robot_model', default_value='kobuki', choices=['kobuki', 'kobuki_d1'], description='Robot model to use'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time (If true, launch gazebo)'),
        DeclareLaunchArgument('launch_rviz2', default_value='true', choices=['true', 'false'], description='Launch RViz2'),
        DeclareLaunchArgument('rviz2_config', default_value=rviz2_path, description='RViz2 configuration file'),
        DeclareLaunchArgument('world_name', default_value='empty.sdf', choices=['empty.sdf', 'mobilab.sdf', 'playground.sdf'], description='Gazebo world file'),
        DeclareLaunchArgument('slam_type', default_value='noslam', choices=['slam', 'vslam', 'noslam'], description='Slam type to use'),
    ]

    return LaunchDescription([*args,
                             robot_state_node,
                             joint_state_publisher,
                             rviz2_node,
                             gazebo_launch,
                             TimerAction(period=5.0, actions=[ros2_control_launch]),
                             ekf_node,
                             twist_remap_node,
                             OpaqueFunction(function=slam_launch, args=[packagePath, use_sim_time]),
    ])
