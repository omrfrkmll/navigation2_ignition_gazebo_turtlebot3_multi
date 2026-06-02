import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, 
                            TimerAction, GroupAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tb3_multi_robot = get_package_share_directory('tb3_multi_robot')
    pkg_tb3_rmf = get_package_share_directory('tb3_rmf')
    rmf_config_dir = os.path.join(pkg_tb3_rmf, 'config')
    
    # Paths to sub-launch files
    simulation_launch_path = os.path.join(pkg_tb3_multi_robot, 'launch', 'tb3_simulation.launch.py')
    
    # Launch simulation (Gazebo + Nav2)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # RMF Group (Core nodes)
    rmf_core_group = GroupAction([
        # 1. Building Map Server
        Node(
            package='rmf_building_map_tools',
            executable='building_map_server',
            arguments=[os.path.join(rmf_config_dir, 'tb3_world.building.yaml')],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # 2. RMF Traffic Schedule
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # 3. RMF Task Dispatcher
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # 4. RMF Schedule Visualizer (publishes schedule markers to /schedule_markers)
        Node(
            package='rmf_visualization_schedule',
            executable='schedule_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'rate': 10.0,
                'initial_map_name': 'L1',
                'wait_secs': 10,
            }]
        ),
        # 5. RMF Fleet States Visualizer (publishes fleet shapes to /fleet_markers)
        Node(
            package='rmf_visualization_fleet_states',
            executable='fleetstates_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'fleet_state_nose_scale': 0.5,
                'tb3_radius': 0.3,
            }]
        ),
        # 6. RMF Nav Graph Visualizer (publishes lanes and waypoints to /map_markers)
        Node(
            package='rmf_visualization_navgraphs',
            executable='navgraph_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'initial_map_name': 'L1',
                'lane_width': 0.5,
                'waypoint_scale': 1.3,
                'text_scale': 0.7,
                'lane_transparency': 0.6,
            }]
        ),
        # 7. RMF Marker Transformer (transforms RMF markers to ROS/Gazebo coordinates)
        Node(
            package='tb3_rmf',
            executable='rmf_marker_transformer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

    # RMF Fleet Adapter (The most sensitive part, needs Nav2 to be fully up)
    rmf_adapter_group = GroupAction([
        Node(
            package='tb3_rmf',
            executable='nav2_fleet_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

    # Delay RMF Core by 5s
    delayed_rmf_core = TimerAction(
        period=5.0,
        actions=[rmf_core_group]
    )

    # Delay RMF Adapter by 20s
    delayed_rmf_adapter = TimerAction(
        period=20.0,
        actions=[rmf_adapter_group]
    )

    return LaunchDescription([
        simulation,
        delayed_rmf_core,
        delayed_rmf_adapter
    ])
