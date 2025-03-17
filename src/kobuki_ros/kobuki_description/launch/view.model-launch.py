import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    namePackage           = 'kobuki_description'
    modelFileRelativePath = 'model/kobuki_sim.xacro'
    rvizFileRelativePath  = 'rviz/rviz_config.rviz'

    pathModelFile         = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    robotDescription      = xacro.process_file(pathModelFile).toxml()
    pathRvizConfig        = os.path.join(get_package_share_directory(namePackage), rvizFileRelativePath)


                                                                    # --------------- RVIZ CONFIGURATION --------------- 

    spawnModelNodeRviz = Node(
        package    = 'rviz2',
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["-d", pathRvizConfig],
    )

    nodeRobotStatePublisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{
            'robot_description': robotDescription,
            'use_sim_time'     : False,
        }]
    )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
    )

    launchDescriptionObject = LaunchDescription()
    
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(joint_state_publisher)
    launchDescriptionObject.add_action(spawnModelNodeRviz)

    return launchDescriptionObject