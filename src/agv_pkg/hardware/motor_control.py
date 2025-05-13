#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray
import can
import struct
import subprocess

duty_cycle_mode = 0x2050080
position_mode = 0x2050C80

drive_ids = [1, 21, 31, 41]
steer_ids = [2, 22, 32, 42]
encoder_reset = [0.0, 0.0, 0.0, 0.0]


class MotorControlNode(LifecycleNode):
  def __init__(self):
    super().__init__("motor_control_node")
    self.get_logger().info("IN constructor")
    self.bus = None
    self.drive_sub = None
    self.steer_sub = None
    self.active = False
    self.encoder_angles = encoder_reset

  def on_configure(self, state: LifecycleState):
    self.get_logger().info("IN on_configure")

    self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

    self.drive_sub = self.create_subscription(Float64MultiArray, '/drive_module_velocity_controller/commands', self.drive_callback, 10)
    self.steer_sub = self.create_subscription(Float64MultiArray, '/drive_module_steer_angle_controller/commands', self.steer_callback, 10)
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
    return super().on_deactivate(state)

  def on_shutdown(self, state: LifecycleState):
    self.get_logger().info("IN on_shutdown") 
    self.bus.shutdown()    
    self.destroy_subscription(self.drive_sub)
    self.destroy_subscription(self.steer_sub)
    self.active = False
    return TransitionCallbackReturn.SUCCESS

  def drive_callback(self, msg: Float64MultiArray):
    if self.active:
      self.get_logger().info(f'Received velocity commands: {msg.data}')
      for i, value in enumerate(msg.data):
        if i < len(drive_ids):
          self.get_logger().info(f'Motor: {drive_ids[i]} gets the command: {float(value)}')         
          self.send_control_frame(drive_ids[i], duty_cycle_mode, float(value))    

  def steer_callback(self, msg: Float64MultiArray):
    if self.active:
      self.get_logger().info(f'Received steer commands: {msg.data}')
      for i, value in enumerate(msg.data):
        if i < len(steer_ids):
          self.get_logger().info(f"Setting steering motor {steer_ids[i]} to desired position: {float(value)}")

          while not self.has_reached_position(self.read_steering_messages(steer_ids[i]), float(value)):
            self.send_control_frame(steer_ids[i], duty_cycle_mode, 0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
          self.get_logger().info(f"Steering motor {i} reached desired position: {float(value)}")

  def send_control_frame(self, device_id: int, control_mode: int, setpoint: float):
    can_id = control_mode + device_id 
    data = struct.pack('<f', setpoint) + bytes([0]*4) # Convert the float data into bytes based on little-endian (lowest byte first)  
    msg = can.Message(
      arbitration_id = can_id, data = data, is_extended_id = True
    )
    try:
      self.bus.send(msg)
      self.get_logger().info("Motor control frame sent")
    except can.CanError:
        self.get_logger().info("Motor control frame could not be send")

  def read_steering_messages(self, motor_id: int):
    while True: # Make sure it keeps reading till it finds a message that fits the requirements
      msg = self.bus.recv()
      if msg is None:
        continue
        
      can_id = msg.arbitration_id

      device_id = can_id & 0x3F
      api_index = (can_id & 0x3C0) >> 6
      api_class = (can_id & 0x1C00) >> 10
      manufacturer = (can_id & 0xFF0000) >> 16
      device_type = (can_id & 0x1F000000) >> 24

      self.get_logger().info(f"[CAN ID] Device ID: {device_id}, API Class: {api_class}, API Index: {api_index}, Manufacturer: {manufacturer}, Device Type: {device_type}")
      
      if device_id == motor_id and api_class == 6 and api_index == 2 :
        steering_ratio = 9424 / 203

        #Convert bytes to float based on little-endian (lowest byte first)
        raw_rpm = struct.unpack('<f', msg.data[0:4])[0]
        raw_position = struct.unpack('<f', msg.data[4:8])[0] 

        rpm = (raw_rpm / steering_ratio) 
        turning_angle = rpm / 60 * 360 # In degrees per seconds
        accumulated_angle = raw_position/steering_ratio *360 #in degrees

        angle_degrees = accumulated_angle % 360 # Remove full rounds
        if angle_degrees < 0:
            angle_degrees += 360
        norm_angle = angle_degrees / 360 # The normalised angle 


        self.get_logger().info(f"Raw position: {raw_position}, rpm: {rpm}, turning angle per second {turning_angle}, accumalted angle {accumulated_angle}, normalised angle {norm_angle}")
        return norm_angle


  # def read_steering_motor_position(self):
  #   try:
  #     msg = self.bus.recv()  
  #     for i, steer_id in enumerate(steer_ids):
  #       if msg.arbitration_id == steer_id:
  #         self.encoder_angles[i] = struct.unpack('f', bytes(msg.data[:4]))[0]
  #         self.get_logger().info(f"Steering motor {i} position: {self.encoder_angles[i]}")
  #   except can.CanError as e:
  #         self.get_logger().error(f"CAN Error: {e}")

  def has_reached_position(self, current_position: float, target_angle: float) -> bool:
    tolerance = 0.1  
        
    if abs(current_position - target_angle) <= tolerance:
      return True
    else:
      return False

def main(args=None):
  rclpy.init(args=args)
  lifecycle_node = MotorControlNode()
  rclpy.spin(lifecycle_node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
