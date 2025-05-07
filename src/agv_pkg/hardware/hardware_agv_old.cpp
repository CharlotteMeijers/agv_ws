// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "agv_pkg/hardware_agv.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace agv_pkg
{
// Set all parameters
hardware_interface::CallbackReturn AgvSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  heartbeat_frame.can_id = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = HEARTBEAT_SIZE;
  pack_data(heartbeat_frame, HEARTBEAT_DATA, HEARTBEAT_SIZE);

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  // hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Set up the communication of the hardware
hardware_interface::CallbackReturn AgvSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  try
  {
    // Make a can service that is binded to the callback function of ros2can
    ros2can_service_ = this->create_service<can_msgs::srv::CanRequest>("ros2can", std::bind(&AgvSystemHardware::ros2can_srv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    // Make subscription for the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&AgvSystemHardware::cmdVelCallback, this, std::placeholders::_1));
    // Make publisher for the cmd_vel topic
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&AgvSystemHardware::cmdVelPublisher, this, std::placeholders::_1));
    
    try
    {
      // Make heartbeat node
      auto node = std::make_shared<rclcpp::Node>("heartbeat_node");   
      
      heartbeat_frame.can_id = HEARTBEAT_ID | CAN_EFF_FLAG;
      heartbeat_frame.can_dlc = HEARTBEAT_SIZE;
      pack_data(heartbeat_frame, HEARTBEAT_DATA, HEARTBEAT_SIZE);
      
      // Make a timer in the node that sends the heartbeat every ..ms
      heartbeat_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(200), 
        std::bind(&AgvSystemHardware::send_heartbeat_frames, this)
      );

      // Let the heartbeat spin in a seperate thread
      std::thread spin_thread([node]() {
        rclcpp::spin(node);  // Laat de node draaien in een aparte thread
      });
  
      spin_thread.detach();  // Deatach the heartbeat such that it keeps spinning on the background

      RCLCPP_INFO(get_logger(), "Heartbeat node started!");
    
  }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(".."), "Failed to configure: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Close the communication of the hardware
hardware_interface::CallbackReturn AgvSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Clean up ...please wait...");

  try
  {
    ros2can_service_ = nullptr;
    cmd_vel_sub_ = nullptr;
    cmd_vel_pub_ = nullptr;
    canbus_.stop();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(".."), "Failed to clean up: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate the hardware, "power" is enabled
hardware_interface::CallbackReturn AgvSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  
  heartbeat_timer_ = rclcpp::Node::create()->create_wall_timer(
    std::chrono::milliseconds(200), 
    std::bind(&AgvSystemHardware::send_heartbeat_frames, this, std::placeholders::_1),
  );
  

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivate the hardware, remove the "power" of the hardware
hardware_interface::CallbackReturn AgvSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  // Make sure the motors will not start spinning when they get a command
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
    heartbeat_timer_.reset();
  }
  
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Getting states from the hardware and store them in the defined variables of export_state_interfaces
hardware_interface::return_type AgvSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  canbus_.CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream);

  return hardware_interface::return_type::OK;
}

// Send commands to the hardware based on the values stored in the defined variables of export_command_interfaces
hardware_interface::return_type agv_pkg ::AgvSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  canbus_.CanSend();

  return hardware_interface::return_type::OK;
}

// Callback for the cmd topic
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
  
}


void packData(can_frame &frame, uint8_t *data, int size){
  for (int i=0; i< size; i++){
    frame.data[i] = data[i];
  }
}

void createData(void *data, byte *frame_data, uint8_t data_size, uint8_t total_size){
  // Copy data to frame_data
  const byte *data_arr = static_cast<const byte *>(data);
  for (int i = 0; i < data_size; i++) {
    frame_data[i] = data_arr[i];
  }

  // Fill remaining space with zeros
  for (int i = data_size; i < total_size; i++) {
    frame_data[i] = 0;
  }
}


// Send command as float to the desired motor
void sendCanFrame(uint8_t id, float value){
  
}

void send_heartbeat_frames()
{
    // Copy heartbeat data to frame
    std::memcpy(heartbeat_frame.data, HEARTBEAT_DATA, HEARTBEAT_SIZE);

    // Send heartbeat frame over CAN
    canbus_->write(heartbeat_frame); 

  RCLCPP_INFO(get_logger(), "Heartbeat frame verzonden.");
}

}  // namespace agv_pkg

PLUGINLIB_EXPORT_CLASS(
  agv_pkg::AgvSystemHardware, hardware_interface::SystemInterface)
