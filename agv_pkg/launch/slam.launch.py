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
#     DeclareLaunchArgument('sync', default_value='true',
#                           choices=['true', 'false'],
#                           description='Use synchronous SLAM')
]


def generate_launch_description():
    package_name='agv_pkg' 

    use_sim_time = LaunchConfiguration('use_sim_time')
    # sync = LaunchConfiguration('sync')

    ld = LaunchDescription(ARGUMENTS)

    slam_params = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    print(slam_params)

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[
        slam_params,
            {'use_sim_time': use_sim_time}
        ],
    )
    ld.add_action(slam_toolbox)

    rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'view_robot_in_map.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )
    ld.add_action(rviz)

    return ld