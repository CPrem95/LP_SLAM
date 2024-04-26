from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dev = '/dev/ttyACM0'

    anc_1 = '102'
    anc_2 = '103'
    anc_3 = '105'

    # creates the path to the YAML file
    config = os.path.join(
        get_package_share_directory('uwb_ranger'),
        'config',
        'params.yaml'
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'd',
            default_value=dev,
            description='Specify the UWB device address'
        ),
        DeclareLaunchArgument(
            'a1',
            default_value=anc_1,
            description='Specify the anchor_1 ID'
        ),
        DeclareLaunchArgument(
            'a2',
            default_value=anc_2,
            description='Specify the anchor_2 ID'
        ),
        DeclareLaunchArgument(
            'a3',
            default_value=anc_3,
            description='Specify the anchor_3 ID'
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value=config,
            description='Path to the YAML configuration file'
        ),


        # Parameters node
        # Node(
        #     package='python_parameters',
        # #     namespace='myParams',
        #     executable='param_node',
        # #     name='vals',
        #     parameters=[LaunchConfiguration('config_file')],
        # ),

        # Main node
        Node(
            package='uwb_ranger',
            namespace='Range',
            executable='ranger',
            name='fromBeacons',
            arguments=['-u', LaunchConfiguration('d')],
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),

        
        # Plotting nodes for each radar
        # # Node(
        # #     package='novelda_x4m300',
        #     namespace='UWBplot1',
        # #     executable='uwbListener',
        #     name='plt',
        # #     arguments=['-t', LaunchConfiguration('t1')],
        #     # parameters=[LaunchConfiguration('config_file')],
        # #     output='screen'
        # # )
    ])
