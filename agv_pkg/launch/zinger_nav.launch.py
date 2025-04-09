import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use sim time if true'
    ),
]

def generate_launch_description():
    package_name='agv_pkg'

    use_sim_time = LaunchConfiguration('use_sim_time')

    map_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'home_map.yaml'
    )

    localisation_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'localisation_amcl.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2.yaml'
    )

    # Path to the Slam Toolbox launch file
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': localisation_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': navigation_params_path,
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)

    return ld