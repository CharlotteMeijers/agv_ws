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

#ifndef AGV_PKG__HARDWARE_AGV_HPP_
#define AGV_PKG__HARDWARE_AGV_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2socketcan/ros2socketcan.hpp"

#include "agv_pkg/wheel_agv.hpp"

namespace agv_pkg
{
  enum class ControlMode : uint32_t {
    Duty_Cycle_Set = 0x2050080,
    Speed_Set = 0x2050480,
    Smart_Velocity_Set = 0x20504C0,
    Position_Set = 0x2050C80,
    Voltage_Set = 0x2051080,
    Current_Set = 0x20510C0,
    Smart_Motion_Set = 0x2051480
  };
  
  enum status_frame_id{
    status_0 = 0x2051800;
    status_1 = 0x2051840;
    status_2 = 0x2051880;
    status_3 = 0x20518C0;
    status_4 = 0x2051900;
  }

  
class AgvSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AgvSystemHardware)

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_cleanup(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::CallbackReturn write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    
private:
    std::shared_ptr<drivers::socketcan::SocketCanBus> canbus_; // The socket can interface
    
    // std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    Wheel wheel_left_front("left_front", 1, 12);
    Wheel wheel_left_rear("left_rear", 2, 22);
    Wheel wheel_right_rear("right_rear", 3, 32);
    Wheel wheel_right_front("right_front", 4, 42);
    
    // Heartbeat variables 
    struct can_frame heartbeat_frame;
    uint8_t HEARTBEAT_SIZE = 8;
    uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
    std::uint32_t HEARTBEAT_ID = 0x2052C80;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_ = 200;

    // Control variables 
    struct can_frame control_frame;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // Reads from the cmd topic
    
    // Callback for the cmd topic
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void packData(can_frame &frame, uint8_t *data, int size)

    void createData(void *data, byte *frame_data, uint8_t data_size, uint8_t total_size);

    // Send the canframe to the desired wheel
    void sendCanFrame(Wheel &wheel_position);
};

}  // namespace agv_pkg

#endif  // AGV_PKG__HARDWARE_AGV_HPP_
