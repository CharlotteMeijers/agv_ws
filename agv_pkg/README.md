## Needed packages:
For Gazebo:
 - ros-jazzy-tf-transformations
 - ros-jazzy-ros-gz
 - ros-jazzy-gz-ros2-control
 - ros-jazzy-ros2-control
 - ros-jazzy-ros2-controllers
 - ros-jazzy-twist-mux

For navigation:
 - ros-jazzy-navigation2 
 - ros-jazzy-nav2-bringup
 - ros-jazzy-slam-toolbox


## Robot description
To publish the robot state:

        ros2 launch agv_pkg rsp.launch.py 

To tell in which state the wheels are:

        ros2 run joint_state_publisher_gui joint_state_publisher_gui 

## RVIZ
To view robot in rviz:

        rviz2 -d src/agv_pkg/config/view_bot.rviz

To view robot with the obstacle map in rviz:

        rviz2 -d src/agv_pkg/config/view_bot_in_map.rviz

## Gazebo
To drive robot in Gazebo (use the left joystick to move forward and backwards and the right joystick to turn):

        ros2 launch agv_pkg sim.launch.py use_sim_time:=true 

To drive using the keyboard also launch the teleop_twist_keyboard:
        
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Navigation
To create a map:
        
        ros2 launch agv_pkg sim.launch.py use_sim_time:=true make_new_map:=true
        
