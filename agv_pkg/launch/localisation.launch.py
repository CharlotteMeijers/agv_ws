import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    package_name='agv_pkg' 

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld = LaunchDescription(ARGUMENTS)

    map_location = os.path.join(get_package_share_directory(package_name),'config','map_save.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
        map_location,
            {'use_sim_time': use_sim_time}
        ],
    )
    ld.add_action(map_server)

    amcl_server = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )
    ld.add_action(amcl_server)

    # Activate map_server and amcl node
    activate_nodes = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'node_names': 'map_server'}])
    ld.add_action(activate_nodes)

    return ld