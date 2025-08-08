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
 - ros-jazzy-controller-manager
 - ros-jazzy-teleop-twist-joy
 - ros-jazzy-joy
 - ros-jazzy-ros2-socketcan
 - libudev-dev
 - git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git (added as submodule in this workspace)

 - nav2-util
 - python3-serial
 - python3-can
 - python3-rosdep

To complete the rosdep installation:
    
    sudo rosdep init
    rosdep update

Don't forget to build and source the workspace before start running commands:
       cd ~/agv_ws
       colcon build
       source install/setup.bash
 - git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git (added as submodule in this workspace)

 - nav2-util
 - python3-serial
 - python3-can
 - python3-rosdep

To complete the rosdep installation:
    
    sudo rosdep init
    rosdep update

Don't forget to build and source the workspace before start running commands:
       cd ~/agv_ws
       colcon build
       source install/setup.bash

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

### Home.sdf
To be able to use the home.sdf, add the following to the ~/.bashrc

        export GZ_SIM_RESOURCE_PATH=~/agv_ws/src/agv_pkg/worlds/gazebo_models

Don't forget to source the ~/.bashrc before running again

### Localise only tested for Zinger: 
Next to the zinger Gazebo launch:
        
        ros2 launch agv_pkg zinger_sim.launch.py use_sim_time:=true 

Run the localisation file:

        ros2 launch agv_pkg zinger_loc.launch.py use_sim_time:=true

Click on the 2D Pose estimate to give an initial pose (with orientation) for amcl (RVIZ)

### Navigate to point (only tested for Zinger): 
Next to the zinger Gazebo launch:
        
        ros2 launch agv_pkg zinger_sim.launch.py use_sim_time:=true 

Run the localisation file:

        ros2 launch agv_pkg zinger_nav.launch.py use_sim_time:=true

Click on the 2D Pose estimate to give an initial pose (with orientation) for amcl and use the 2D goal pose to let the zinger robot drive to that pose (RVIZ)

### Update the current pose (what the Jetson Nano should sent)
The Jetson Nano schould get the pose of the calibration point out of the qr-code. This pose should be published on the initial pose topic, which can be done with:

        ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0} }}}'
### Update the current pose (what the Jetson Nano should sent)
The Jetson Nano schould get the pose of the calibration point out of the qr-code. This pose should be published on the initial pose topic, which can be done with:

        ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0} }}}'


# Hardware for the physical robot
Adjust the start-up script to be able to use the CANHAT:

        sudo nano /boot/firmware/config.txt 

and add:

        dtparam=spi=on
        dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000

Increase the buffer:
       
        sudo ifconfig can0 txqueuelen 65536

After rebooting, the CANHAT will work.

## Lidar
For the lidar, first install the udev rules:

        cd ~/agv_ws/src/ldrobot-lidar-ros2/scripts
        ./create_udev_rules.sh

Build the packages in the workspace again.

Then update the environment variables:

        echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
        source ~/.bashrc

The lidar can be launched:

        ros2 launch ldlidar_node ldlidar_bringup.launch.py 
        ros2 lifecycle set /ldlidar_node configure
        ros2 lifecycle set /ldlidar_node activate

Also start the lidar filter such that the robot frame will not be visible in the \scan topic:

        python3 src/agv_pkg/hardware/lidar_filter.py 

## Motors
To be able to use the real hardware on the physical robot, the following lifecycles should be launched, configured and activated:
       python3 heartbeat.py
       python3 control_motor.py
       ros2 lifecycle set /heartbeat_node configure
       ros2 lifecycle set /heartbeat_node activate
       ros2 lifecycle set /motor_control_node configure
       ros2 lifecycle set /heartbeat_node activate

To test the motors without a joystick, the following commands can be used. A data value between 0 and 1 will turn the motor on. 

       ros2 topic pub /drive_module_steering_angle_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0]}"
       ros2 topic pub /drive_module_velocity_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0]}"

To drive using the joystick, start the navigation.launch.py:
       ros2 launch agv_pkg navigation.launch.py use_sim_time:=false

# Split navigation and motor control
## On both devices
Make a file:  
        
        nano ~/cyclonedds.xml

With:
```xml
```xml
<CycloneDDS>
 <Domain>
  <General>
   <NetworkInterfaceAddress>XXX.XXX.XXX.*</NetworkInterfaceAddress>
  </General>
 </Domain>
</CycloneDDS>
```
```

in which XXX.XXX.XXX. is the address of the sub network.

Adjust bashrc:

        nano ~/.bashrc

By adding:
export ROS_DOMAIN_ID=8
export CYCLONEDDS_URI=file:///home/zwier/cyclonedds.xml

Disable the firewall:
        
        sudo ufw diable

Restart daemon

        ros2 daemon stop
        ros2 daemon start

## Test connection
On one device:

        ros2 topic pub /test std_msgs/Header "{stamp: {sec: $(date +%s)}, frame_id: 'ping'}" -r 1 --qos-reliability best_effort --qos-durability volatile

The other device:

        ros2 topic echo /test


