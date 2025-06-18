#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor
import can
import struct
import subprocess
import math
from sensor_msgs.msg import JointState
import time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import threading

duty_cycle_mode = 0x2050080
position_mode = 0x2050C80

drive_ids = [11, 21, 31, 41]
steer_ids = [12, 22, 32, 42]
encoder_reset = [0.75, 0.0, 0.25, 0.5]
gear_ratio = 5.5
steering_ratio = 9424 / 203
wheel_diameter = 0.072
tolerance = 5

qos_joint_state = QoSProfile(
depth = 10,
durability = DurabilityPolicy.VOLATILE,
reliability = ReliabilityPolicy.RELIABLE
)

class MotorControlNode(LifecycleNode):
  def __init__(self):
    super().__init__("motor_control_node")
    self.get_logger().info("IN constructor")
    self.bus = None
    self.drive_sub = None
    self.steer_sub = None
    self.active = False
    self.steer_active =  [False] * len(steer_ids) #Messages received
    self.steer_sending =  [0] * len(steer_ids) #Control frame 0=nothing send, 1=CW, 2=CCW

    self.current_steer_position = [0.0] * len(drive_ids)
    self.desired_steer_position = [0.0] * len(drive_ids)
    self.current_steer_velocity = [0.0] * len(steer_ids)
    self.current_drive_position = [0.0] * len(drive_ids)
    self.current_drive_velocity = [0.0] * len(drive_ids)
    self.desired_drive_velocity = [0.0] * len(drive_ids)
    self.joint_state_angle = [0.0] * len(steer_ids)
    self.diff = [0.0] * len(steer_ids)

    self.steering_target = [None] * len(steer_ids) #Variable to store the target position for the steering motors
    self.last_steer_msg_time = time.time()  #Variable to save the last steering callback time

    self.can_thread = threading.Thread(target=self.read_can_loop, daemon=True) #Read encoder messages
    self.can_thread.start()
    self.motor_control_thread = threading.Thread(target=self.motor_control_loop, daemon=True)
    self.motor_control_thread.start()
    self.diff_loop_thread = threading.Thread(target=self.check_diff_loop, daemon=True)
    self.diff_loop_thread.start()

    self.create_timer(0.1, self.check_steer)
    #self.create_timer(0.1, self.check_diff)
    self.create_timer(0.1, self.update_joint_states) #Update the joint state based on the encoder messages, 10Hz

  def on_configure(self, state: LifecycleState):
    self.get_logger().info("IN on_configure")

    self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    # self.send_control_frame(steer_ids[i], duty_cycle_mode, 0.0)
    self.drive_sub = self.create_subscription(Float64MultiArray, '/drive_module_velocity_controller/commands', self.drive_callback, 10)
    self.steer_sub = self.create_subscription(Float64MultiArray, '/drive_module_steering_angle_controller/commands', self.steer_callback, 10)
    self.joint_state_pub = self.create_publisher(JointState, '/current_joint_states', qos_joint_state)
    self.active = False

    try:
      subprocess.run(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"], check=True)
      self.get_logger().info("CAN interface can0 is up.")
    except subprocess.CalledProcessError as e:
      self.get_logger().error(f"Failed to bring up CAN interface: {e}")

    return TransitionCallbackReturn.SUCCESS

  def on_cleanup(self, state: LifecycleState):
    self.get_logger().info("IN on_cleanup")
    self.bus.shutdown()
    self.destroy_subscription(self.drive_sub)
    self.destroy_subscription(self.steer_sub)
    self.destroy_publisher(self.joint_state_pub)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], check=True)
    self.get_logger().info("CAN interface can0 is down.")

    self.active = False
    return TransitionCallbackReturn.SUCCESS

  def on_activate(self, state: LifecycleState):
    self.get_logger().info("IN on_activate")
    self.active = True
    # self.read_steering_messages() 
    return super().on_activate(state)

  def on_deactivate(self, state: LifecycleState):
    self.get_logger().info("IN on_deactivate")
    self.active = False
    self.desired_steer_position = [0.0] * len(drive_ids)
    self.desired_drive_velocity = [0.0] * len(drive_ids)
    return super().on_deactivate(state)

  def on_shutdown(self, state: LifecycleState):
    self.get_logger().info("IN on_shutdown") 
    self.bus.shutdown()    
    self.destroy_subscription(self.drive_sub)
    self.destroy_subscription(self.steer_sub)
    self.destroy_publisher(self.joint_state_pub)
    self.active = False
    return TransitionCallbackReturn.SUCCESS

  def drive_callback(self, msg: Float64MultiArray):
    if self.active:
      # self.get_logger().info(f'Received velocity commands: {msg.data}')
      for i, value in enumerate(msg.data):
        if i < len(drive_ids):
            self.desired_drive_velocity[i] = value / 100
          # self.get_logger().info(f'Motor: {drive_ids[i]} gets the command: {float(value)}')         
          # self.send_control_frame(drive_ids[i], duty_cycle_mode, float(value/100))    

  def steer_callback(self, msg: Float64MultiArray):
    if self.active:
      self.last_steer_msg_time = time.time()
  #    self.get_logger().info(f'In steer callback')
      for i, value in enumerate(msg.data):
        if i < len(steer_ids):
           desired_angle_rad = (value + math.pi) 
           desired_angle_degree = desired_angle_rad * (180 / math.pi)
           self.desired_steer_position[i] = desired_angle_degree
           #self.steer_active[i] = True
      #self.get_logger().info(f'Desired position motor 1: {self.desired_steer_position[0]}, Desired position motor 2: {self.desired_steer_position[1]}, Desired position motor 3: {self.desired_steer_position[2]}, Desired position motor 4: {self.desired_steer_position[3]}')

  def check_steer(self):
     if not self.active:
        return
     #if self.steer_active[i] == True:
     for i, value in enumerate(self.desired_steer_position):
                self.diff[i] = self.desired_steer_position[i] - self.current_steer_position[i]
     #self.get_logger().info(f'difference: {self.diff[0]}, desired: {self.desired_steer_position[0]}, encoder: {self.current_steer_position[0]}')
     #self.get_logger().info(f'desired: {self.desired_steer_position}')
     #self.get_logger().info(f'encoder: {self.current_steer_position}')
                
  def check_diff(self):
     if not self.active:
        return
        
     for i, value in enumerate(self.diff):
            if self.diff[i] > tolerance or self.diff[i] < - tolerance:
              if self.diff[i] > 180 and self.steer_sending[i] != 1:
                          self.send_control_frame(steer_ids[i], duty_cycle_mode, 0.03)
                          self.steer_sending[i] = 1 
                          #self.get_logger().info(f'Motor {steer_ids[i]} CW, desired position: {self.desired_steer_position[i]}, current position: {self.current_steer_position[i]}, difference: {self.diff[i]}')
              elif self.diff[i] < 180 and self.steer_sending[i] != 2:
                          self.send_control_frame(steer_ids[i], duty_cycle_mode, -0.03)
                          self.steer_sending[i] = 2
                          #self.get_logger().info(f'Motor {steer_ids[i]} CCW')
            elif self.steer_sending[i] != 0:
                        self.send_control_frame(steer_ids[i], duty_cycle_mode, 0.0)
                        self.steer_sending[i] = 0
                        #self.get_logger().info(f'Motor {steer_ids[i]} reached the desired position')
                        #self.steer_active[i] = False
     else:
            return
    

  def send_control_frame(self, device_id: int, control_mode: int, setpoint: float):
  #  self.get_logger().info(f'Motor: {device_id} gets the command: {setpoint}') 
    can_id = control_mode + device_id 
    data = struct.pack('<f', setpoint) + bytes([0]*4) # Convert the float data into bytes based on little-endian (lowest byte first)  
    msg = can.Message(
      arbitration_id = can_id, data = data, is_extended_id = True
    )
    try:
      self.bus.send(msg)
      # self.get_logger().info("Motor control frame sent")
    except can.CanError:
      self.get_logger().info("")
      #  self.get_logger().info("Motor control frame could not be send")

  def read_can_loop(self):
    while rclpy.ok():
        if not self.active:
            time.sleep(0.01)
            continue
        try:
            msg = self.bus.recv(timeout=0.01)
            if msg is not None:
                self.process_can_message(msg)
        except can.CanError as e:
            self.get_logger().error(f"CAN error: {e}")

  def motor_control_loop(self):
    while rclpy.ok():
        if not self.active:
            time.sleep(0.01)
            continue
        for i, value in enumerate(self.desired_drive_velocity):
            if value is not 0:
              self.send_control_frame(drive_ids[i], duty_cycle_mode, value)
        time.sleep(0.05) 

  def check_diff_loop(self):
    while rclpy.ok():
        if not self.active:
            time.sleep(0.01)
            continue
        self.check_diff()
        time.sleep(0.02)  # 50 Hz


  def process_can_message(self, msg):
    # if not self.active:
    #   return
    
    # msg = self.bus.recv(timeout=0.001)
    #self.get_logger().info(f"Received message: {msg}")
    
    # if msg is not None:       
      can_id = msg.arbitration_id
 
      device_id = can_id & 0x3F
      api_index = (can_id & 0x3C0) >> 6
      api_class = (can_id & 0x1C00) >> 10
      manufacturer = (can_id & 0xFF0000) >> 16
      device_type = (can_id & 0x1F000000) >> 24
      
      #self.get_logger().info(f"Motor {device_id}, api_index: {api_index}, api_class: {api_class}")
      
      if api_class == 6 and api_index == 2:
        motor_id = device_id
        if device_id % 10 == 1: #What is left if you divide by 10
          
          # Convert bytes to float based on little-endian (lowest byte first)
          raw_rpm = struct.unpack('<f', msg.data[0:4])[0]
          raw_position = struct.unpack('<f', msg.data[4:8])[0] 

          rpm = raw_rpm / gear_ratio
          velocity = math.pi * wheel_diameter * (rpm/60) 

          accumulated_position = raw_position / gear_ratio
#          self.get_logger().info(f"Motor {motor_id}, velocity: {velocity}, accumulated_position: {accumulated_position}")

          if motor_id == drive_ids[0]:
                  self.current_drive_position[0] = accumulated_position
                  self.current_drive_velocity[0] = velocity
          elif motor_id == drive_ids[1]:
                  self.current_drive_position[1] = accumulated_position
                  self.current_drive_velocity[1] = velocity
          elif motor_id == drive_ids[2]:
                  self.current_drive_position[2] = accumulated_position
                  self.current_drive_velocity[2] = velocity
          elif motor_id == drive_ids[3]:
                  self.current_drive_position[3] = accumulated_position
                  self.current_drive_velocity[3] = velocity
                
        elif device_id % 10 == 2: 
          # self.get_logger().info(f"Received message: {msg}")
          # Convert bytes to float based on little-endian (lowest byte first)
          raw_rpm = struct.unpack('<f', msg.data[0:4])[0]
          raw_position = struct.unpack('<f', msg.data[4:8])[0] 

          rpm = (raw_rpm / steering_ratio) 
          turning_angle = abs(rpm) / 60 * 360 # In degrees per seconds
          accumulated_angle = raw_position/steering_ratio *360 #in degrees

          angle_degrees = accumulated_angle % 360 # Remove full rounds
          # self.get_logger().info(f"raw_rpm: {raw_rpm}, raw_position: {raw_position}, rpm: {rpm}, turning_angle: {turning_angle}, accumulated_angle: {accumulated_angle}, angle_degrees: {angle_degrees}")
        #   if angle_degrees < 0:
        #       angle_degrees += 360
          
          # norm_angle = angle_degrees / 360 # The normalised angle 
        #   self.get_logger().info(f"Motor: {motor_id} has a normalised angle of: {norm_angle}")
          
        #  self.get_logger().info(f"Motor {motor_id}, rpm: {rpm}, accumulated_angle: {accumulated_angle}")
          if motor_id == steer_ids[0]:
                  self.current_steer_position[0] = angle_degrees
                  self.current_steer_velocity[0] = rpm
                  self.joint_state_angle[0] = accumulated_angle / 10
                  # self.get_logger().info(f"current_steer_position: {angle_degrees}, current_steer_velocity: {rpm}")
          elif motor_id == steer_ids[1]:
                  self.current_steer_position[1] = angle_degrees
                  self.current_steer_velocity[1] = rpm
                  self.joint_state_angle[1] = accumulated_angle / 10
          elif motor_id == steer_ids[2]:
                  self.current_steer_position[2] = angle_degrees
                  self.current_steer_velocity[2] = rpm
                  self.joint_state_angle[2] = accumulated_angle / 10
          elif motor_id == steer_ids[3]:
                  self.current_steer_position[3] = angle_degrees
                  self.current_steer_velocity[3] = rpm
                  self.joint_state_angle[3] = accumulated_angle / 10
                
        # self.current_position[i] = norm_angle
        #self.get_logger().info(f"Raw position: {raw_position}, rpm: {rpm}, turning angle per second {turning_angle}, accumalted angle {accumulated_angle}, normalised angle {norm_angle}")
            
  def update_steering_targets(self):
    if not self.active:
      return
    for i, target_angle in enumerate(self.steering_target):
      if target_angle is not None:
         self.read_steering_messages(i)
         if self.has_reached_position(self.current_steer_position[i], target_angle):
            self.send_control_frame(steer_ids[i], duty_cycle_mode, 0.001)
#            self.get_logger().info(f"Tying to reach the desired angle of the motor {steer_ids[i]}")            
         else:
            self.send_control_frame(steer_ids[i], duty_cycle_mode, 0.0)
#            self.get_logger().info(f"Steering motor {steer_ids[i]} reached target position {target_angle}")
            self.steering_target[i] = None

  def update_joint_states(self):
    if not self.active:
        return
    
#    self.get_logger().info(f"Publish the joint states")  
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = [
        "joint_steering_to_wheel_left_front",   
        "joint_steering_to_wheel_left_rear",    
        "joint_steering_to_wheel_right_rear",  
        "joint_steering_to_wheel_right_front",   
        "joint_chassis_to_steering_left_front",
        "joint_chassis_to_steering_left_rear", 
        "joint_chassis_to_steering_right_rear", 
        "joint_chassis_to_steering_right_front"
    ]
    msg.position = [
        self.joint_state_angle[0],
        self.joint_state_angle[1],
        self.joint_state_angle[2],
        self.joint_state_angle[3],
        self.current_drive_position[0],
        self.current_drive_position[1],
        self.current_drive_position[2],
        self.current_drive_position[3]
    ]
    msg.velocity = [
        self.current_steer_velocity[0],
        self.current_steer_velocity[1],
        self.current_steer_velocity[2],
        self.current_steer_velocity[3],
        self.current_drive_velocity[0],
        self.current_drive_velocity[1],
        self.current_drive_velocity[2],
        self.current_drive_velocity[3]
        ]
    msg.effort = []
    
 #   self.get_logger().info(f"msg: {msg}") 

    self.joint_state_pub.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  lifecycle_node = MotorControlNode()

  executor = MultiThreadedExecutor(num_threads = 6)
  executor.add_node(lifecycle_node)

  try:
     executor.spin()
  finally:
     lifecycle_node.destroy_node()
     rclpy.shutdown()

if __name__ == "__main__":
  main()
