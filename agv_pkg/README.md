## Robot description
To publish the robot state:

        ros2 launch agv_pkg rsp.launch.py 

To tell in which state the wheels are:

        ros2 run joint_state_publisher_gui joint_state_publisher_gui 

## RVIZ
To view robot in rviz:

        rviz2 -d src/agv_pkg/config/view_bot.rviz

## Gazebo
To view robot in Gazebo:

        ros2 launch agv_pkg sim.launch.py use_sim_time:=true 

To drive in Gazebo also launch the swerve controller (use the left joystick to move forward and backwards and the right joystick to turn):

        ros2 launch zinger_swerve_controller swerve_controller.launch.py use_sim_time:=true

To drive using the keyboard:
        
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
