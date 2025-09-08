from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    use_sim_time   = LaunchConfiguration('use_sim_time')
    
    rtabmap_parameters = {
        'map_frame_id':'map',
        'frame_id':'base_footprint',
        'odom_frame_id':'odom',
        'odom_tf_linear_variance':0.001,
        'odom_tf_angular_variance':0.001,
        'subscribe_rgbd': True,
        'subscribe_scan': False,
        'approx_sync': True,
        'sync_queue_size': 10,
        'publish_tf': True,
        'tf_tolerance': 0.1,
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace':     'true',
        'RGBD/ProximityByTime':      'false',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Reg/Strategy':              '1', 
        'Vis/MinInliers':            '12',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Grid/FromDepth': 'false',
        'Grid/MaxObstacleHeight': '2.0',
        'publish_last_pose': 'true',
        'RGBD/OptimizeMaxError':     '4',
        'Reg/Force3DoF':             'true',
        'Grid/FromDepth':            'true',
        'Mem/STMSize':               '30',
        'RGBD/LocalRadius':          '5',
        'Icp/CorrespondenceRatio':   '0.2',
        'Icp/PM':                    'false',
        'Icp/PointToPlane':          'false',
        'Icp/MaxCorrespondenceDistance': '0.15',
        'Icp/VoxelSize':             '0.05',
        'use_sim_time': use_sim_time,
    }

    localization = LaunchConfiguration('localization')

    config_rviz = PathJoinSubstitution([
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    ])

    remappings=[
        ('rgb/image',       '/rgb/image'),
        ('depth/image',     '/depth/image'),
        ('rgb/camera_info', '/rgb/camera_info'),
        ('scan',            '/scan'),
    ]

    rtabmap_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[rtabmap_parameters, 
            {'rgb_image_transport':'raw',
            'depth_image_transport':'raw',
            'approx_sync_max_interval': 0.1}
        ],
        remappings=remappings,
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_parameters,
            # {
            #     'use_sim_time': use_sim_time,
            #     'tf_timeout': 1.0,
            #     'wait_for_transform_duration': 0.2,
            #     # 'publish_tf': 'true',
            #     'publish_null_when_lost': 'false',
            # }
        ],
        remappings=remappings,
        arguments=['-d']
    )

    rtabmap_localization = Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings,
    )

    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_parameters],
            remappings=remappings,
    )
    
    rtabmap_rviz = Node(
        package='rviz2', executable='rviz2', name="rviz2", output='screen',
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('slam_params', default_value='', description='Path to the SLAM parameters file'),
        
        rtabmap_rgbd_sync,
        rtabmap_node,
        rtabmap_localization,
        rtabmap_viz,
        rtabmap_rviz,
    ])
    return ld
