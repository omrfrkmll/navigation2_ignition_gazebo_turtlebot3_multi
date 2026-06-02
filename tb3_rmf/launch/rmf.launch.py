import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_tb3_rmf = get_package_share_directory('tb3_rmf')
    rmf_config_dir = os.path.join(pkg_tb3_rmf, 'config')
    building_file = os.path.join(rmf_config_dir, 'tb3_world.building.yaml')
    
    # 1. Building Map Server
    building_map_server = Node(
        package='rmf_building_map_tools',
        executable='building_map_server',
        arguments=[building_file],
        output='screen'
    )
    
    # 2. RMF Traffic Schedule
    traffic_schedule = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 3. Nav2 Fleet Adapter (Custom Python script)
    fleet_adapter = Node(
        package='tb3_rmf',
        executable='nav2_fleet_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Visualization Schedule (Bridge to RViz)
    visualization_schedule = Node(
        package='rmf_visualization_schedule',
        executable='schedule_visualizer_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        building_map_server,
        traffic_schedule,
        fleet_adapter,
        visualization_schedule
    ])
