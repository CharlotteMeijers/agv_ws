import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    ld = LaunchDescription(ARGUMENTS)

    simulation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','zinger_sim.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ld.add_action(simulation)

    heartbeat_node = Node(
        package='agv_pkg',
        executable='heartbeat',  
        name='heartbeat',
        output='screen'
    )
    ld.add_action(heartbeat_node)

    motor_control_node = Node(
        package='agv_pkg',
        executable='motor_control',  
        name='motor_control',
        output='screen'
    )
    ld.add_action(motor_control_node)

    return ld

