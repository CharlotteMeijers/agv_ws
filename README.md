# Needed packages:
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
 - ros-jazzy-joint-state-publisher

For the real hardware:
 - ros-jazzy-ros2-socketcan

# Run commands
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

To view robot with the obstacle map, costmap and planned trajectory in rviz:

        rviz2 -d src/agv_pkg/config/view_bot_planning.rviz

## Gazebo
To drive robot in Gazebo (use the left joystick to move forward and backwards and the right joystick to turn):

        ros2 launch agv_pkg sim.launch.py use_sim_time:=true 

To drive using the keyboard also launch the teleop_twist_keyboard:
        
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Navigation
### Create map with SLAM
Next to the Gazebo launch:
        
        ros2 launch agv_pkg sim.launch.py use_sim_time:=true make_new_map:=true

or two seperate terminals with:

        ros2 launch agv_pkg sim.launch.py use_sim_time:=true 
        ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=agv_pkg/config/mapper_params_online_async.yaml


        
Use rviz slam toolbox panel to save in different formats (save and serialized) or 

        ros2 run nav2_map_server map_saver_cli -f ~/map

Online asynchronized SLAM is used. Online means it runs live and not on recorded logs. Asynchronized means that not every scan needs to be processed, but only the last one. This avoids lagging but means that scans can be skipped.

### Localise only tested for Zinger: 
Next to the zinger Gazebo launch:
        
        ros2 launch agv_pkg zinger_sim.launch.py use_sim_time:=true 

Run the localisation file:

        ros2 launch agv_pkg zinger_loc.launch.py use_sim_time:=true

Click on the 2D Pose estimate to give an initial pose (with orientation) for amcl (RVIZ)

### Navigate to point only tested for Zinger: 
Next to the zinger Gazebo launch:
        
        ros2 launch agv_pkg zinger_sim.launch.py use_sim_time:=true 

Run the localisation file:

        ros2 launch agv_pkg zinger_nav.launch.py use_sim_time:=true

Click on the 2D Pose estimate to give an initial pose (with orientation) for amcl and use the 2D goal pose to let the zinger robot drive to that pose (RVIZ)

# Update current pose (what the Jetson Nano should sent)
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0} }}}'

Use docker for the communication between jetson and raspberry
update initial pose on the jetson, send that to the raspberry

# Home.sdf
To be able to use the home.sdf, add the following to the ~/.bashrc

        export GZ_SIM_RESOURCE_PATH=~/agv_ws/src/agv_pkg/worlds/gazebo_models

Don't forget to source the ~/.bashrc before running again
