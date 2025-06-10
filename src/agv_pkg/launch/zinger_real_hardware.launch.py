import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

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
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)    
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    lifecycle_nodes = ['ldlidar_node',
                       'heartbeat_node',
                       'motor_control_node'
    ]
    
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
            parameters=[twist_mux_params, {'use_sim_time': False}, {'use_stamped': False}],
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

    # Declare parameters
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ldlidar_node'),
                'launch',
                'ldlidar_bringup.launch.py'
                )]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
),

            Node(
                package='agv_pkg',
                executable='heartbeat',
                name='heartbeat_node'
            ),
            Node(
                package='agv_pkg',
                executable='motor_control',
                name='motor_control_node',
                ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)


    return ld

