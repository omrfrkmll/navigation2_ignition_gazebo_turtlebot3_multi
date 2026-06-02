import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths
    tb3_multi_dir = get_package_share_directory('tb3_multi_robot')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')

    # Declare arguments
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(tb3_multi_dir, 'map', 'map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to start camera bridges'
    )

    # 1. Include World & Robot Spawning
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_multi_dir, 'launch', 'tb3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': LaunchConfiguration('rviz'),
            'use_camera': LaunchConfiguration('use_camera')
        }.items()
    )

    # 2. Include Navigation (AMCL or SLAM)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_multi_dir, 'launch', 'tb3_nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'slam': slam,
            'rviz': LaunchConfiguration('rviz')
        }.items()
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(declare_sim_time_arg)
    ld.add_action(declare_map_arg)
    ld.add_action(declare_slam_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_use_camera_arg)
    ld.add_action(world_launch)
    ld.add_action(nav_launch)

    return ld
