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
        
        ros2 launch slam_toolbox online_async_launch.py params_file:=/home/zwier/agv_ws/src/agv_pkg/config/mapper_params_online_async.yaml use_sim_time:=true
        
Use rviz slam toolbox panel to save in different formats (save and serialized)

To publish the saved map to the /map topic first initialize the node:
        ros2 run nav2_nav_server map_server --ros-args -p yaml_filename:=my_map_save.yaml -p use_sim_time:=true

and then activate the node:
        ros2 run nav2_util lifecycle_bringup map_server

Change RViz durability to transient local

initialize amcl:
        ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true

and then activate the node:
        ros2 run nav2_util lifecycle_bringup amcl

click the 2D Pose estimate to give the amcl an initial pose (with orientation)

For navigation:
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

add new map and set the map to the costmap


These changes are needed in the swerve_controller.py under def get_drive_modules(self) -> List[DriveModule]: 

        robot_length = 0.5111 #511.1mm
        robot_width = 0.3111 #311.1mm

        steering_radius = 0.0 #0mm

        wheel_radius = 0.0378 #75.6mm diameter, 37.8mm
        wheel_width = 0.0254 #25.4mm