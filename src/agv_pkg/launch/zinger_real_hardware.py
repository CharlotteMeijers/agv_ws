import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
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
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    ld = LaunchDescription(ARGUMENTS)

    # Publish robot description topic
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','zinger_rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control':'true'}.items()
    )
    ld.add_action(rsp)

    # Launch the joystick launch file to be able to use the jointstick
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ld.add_action(joystick)

    # Combine multiple cmd_inputs and arrange the priority
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_bool}, {'use_stamped': False}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )
    ld.add_action(twist_mux)

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    ld.add_action(delayed_controller_manager)

    # Spawn the robot in Gazebo
    controllers = TimerAction(
        period=8.0,
        actions= [
        #     GroupAction(
        # actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["drive_module_velocity_controller"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["drive_module_steering_angle_controller"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        # ])
    ])
    ld.add_action(controllers)

    # Launch the swerve controller
    swerve_controller = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('swerve_controller'), 'launch', 'swerve_controller.launch.py')]),
                    launch_arguments={'use_sim_time':use_sim_time}.items(),
                    )
    ld.add_action(swerve_controller)

    # heartbeat_node = Node(
    #     package='agv_pkg',
    #     executable='heartbeat',  
    #     name='heartbeat',
    #     output='screen'
    # )
    # ld.add_action(heartbeat_node)

    # motor_control_node = Node(
    #     package='agv_pkg',
    #     executable='motor_control',  
    #     name='motor_control',
    #     output='screen'
    # )
    # ld.add_action(motor_control_node)

    return ld

