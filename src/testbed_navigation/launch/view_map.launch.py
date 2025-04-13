from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('testbed_navigation')

    # Include map server launch
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'map_loader.launch.py'])
        ])
    )

    # Start RViz with our configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'nav2_default_view.rviz'])],
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        map_server_launch,
        rviz_node
    ]) 