import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file
    package_name = 'my_bot'  # <--- CHANGE ME

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Add the RPLIDAR Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # Add the Laser Scan Matcher Node
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,  
            'base_frame': 'base_link',
            'odom_frame': 'odom_matcher',
            'laser_frame': 'laser_frame',
            'publish_odom': '/odom_matcher',
            'publish_tf': True
        }]
    )

    # Add the SLAM Toolbox Node
    slam_toolbox_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'  
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params, {'use_sim_time': False}] 
    )
    
    # Launch all nodes
    return LaunchDescription([
        rsp,
        rplidar_node,
        laser_scan_matcher_node,
        slam_toolbox_node,  
    ])
