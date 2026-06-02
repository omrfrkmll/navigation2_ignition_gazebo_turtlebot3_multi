import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rmf_dir = os.path.join(get_package_share_directory('tb3_rmf'), 'config')
    fleet_config = os.path.join(rmf_dir, 'fleet_config.yaml')
    nav_graph = os.path.join(rmf_dir, '0.yaml')

    fleet_adapter_node = Node(
        package='rmf_fleet_adapter',
        executable='full_control',
        name='turtlebot3_fleet_adapter',
        output='screen',
        parameters=[{
            'fleet_config_file': fleet_config,
            'nav_graph_file': nav_graph,
            'use_sim_time': True,
            'control_manager': 'fleet_manager' # This would usually point to a fleet manager API
        }]
    )

    return LaunchDescription([
        fleet_adapter_node
    ])
