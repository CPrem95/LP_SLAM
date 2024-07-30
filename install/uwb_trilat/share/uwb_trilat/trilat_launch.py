from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    r1_topic = '/UWBradar2/readings'
    r2_topic = '/UWBradar4/readings'
    r3_topic = '/UWBradar1/readings'
    r4_topic = '/UWBradar3/readings'

    return LaunchDescription([
        DeclareLaunchArgument(
            'r1',
            default_value=r1_topic,
            description='Specify the observation topic of radar #1'
        ),

        DeclareLaunchArgument(
            'r2',
            default_value=r2_topic,
            description='Specify the observation topic of radar #2'
        ),

        DeclareLaunchArgument(
            'r3',
            default_value=r3_topic,
            description='Specify the observation topic of radar #3'
        ),

        DeclareLaunchArgument(
            'r4',
            default_value=r4_topic,
            description='Specify the observation topic of radar #4'
        ),

        # Left trilaterator
        Node(
            package='uwb_trilat',
            namespace='LeftObs',
            executable='leftTrilaterator',
            name='rTh',
            arguments=['-a', LaunchConfiguration('r1'), '-b', LaunchConfiguration('r2')],
            output='screen'
        ),

        # Right trilaterator
        Node(
            package='uwb_trilat',
            namespace='RightObs',
            executable='rightTrilaterator',
            name='rTh',
            arguments=['-c', LaunchConfiguration('r3'), '-d', LaunchConfiguration('r4')],
            output='screen'
        ),
    ])
