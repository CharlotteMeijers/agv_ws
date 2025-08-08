import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use sim time if true'
    ),
    DeclareLaunchArgument(
        'make_new_map',
        default_value='false',
        choices=['true', 'false'],
        description='Make a new map if true'
    ),
]

def generate_launch_description():
    package_name='agv_pkg' 

    use_sim_time = LaunchConfiguration('use_sim_time')
    make_new_map = LaunchConfiguration('make_new_map')
    
    # map_file_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'home_map.yaml'
    # )

    # localisation_params_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'localisation_amcl.yaml'
    # )
    
    if use_sim_time == 'true':
        use_sim_bool = True
    else:
        use_sim_bool = False
    ld = LaunchDescription(ARGUMENTS)

    # Publish robot description topic
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'true'}.items()
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
    
    # Use the world
    empty_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'home.sdf'
        )    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=empty_world,
        )
    ld.add_action(world_arg)

    #Launch Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
             )
    ld.add_action(gazebo)

    # Spawn the robot in Gazebo
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')
    ld.add_action(spawn_entity)

    controllers = TimerAction(
        period=10.0,
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
    
    #Launch the ROS-GZ bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ld.add_action(ros_gz_bridge)

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera_left/image_raw", "/camera_right/image_raw"]
    )
    ld.add_action(ros_gz_image_bridge)

    # Launch the slam launch if there is a new map needed
    slam = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','slam.launch.py'
                    )]), 
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                    condition=IfCondition(make_new_map)
    )
    ld.add_action(slam)

    # Launch them all!
    return ld
