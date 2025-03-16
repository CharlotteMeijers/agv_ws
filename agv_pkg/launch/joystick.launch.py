from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    ),
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld = LaunchDescription(ARGUMENTS)

    joy_params = os.path.join(get_package_share_directory('agv_pkg'),'config','joystick.yaml')

    # Launch joystick controller node
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )
    ld.add_action(joy_node)

    # Republish joy messages as scaled geometry_msgs/Twist messages
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
    ld.add_action(teleop_node)

    return ld