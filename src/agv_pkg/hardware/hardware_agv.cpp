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

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Set up the communication of the hardware
hardware_interface::CallbackReturn AgvSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

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
  
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate the hardware, "power" is enabled
hardware_interface::CallbackReturn AgvSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivate the hardware, remove the "power" of the hardware
hardware_interface::CallbackReturn AgvSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Getting states from the hardware and store them in the defined variables of export_state_interfaces
hardware_interface::return_type AgvSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  return hardware_interface::return_type::OK;
}

// Send commands to the hardware based on the values stored in the defined variables of export_command_interfaces
hardware_interface::return_type agv_pkg ::AgvSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  return hardware_interface::return_type::OK;
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
