#ifndef SWERVE_WHEEL_AGV_HPP
#define SWERVE_WHEEL_AGV_HPP

#include <string>
#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>
#include <cstring>
#include <array>

namespace agv_pkg
{
  
class Wheel
{
public:
  Wheel() = default;

  Wheel(const std::string &name, uint8_t drive_id, uint8_t steer_id)
  {
    setup(name, drive_id, steer_id);
  }

  void setup(const std::string &wheel_name, uint8_t drive_id, uint8_t steer_id)
  {
    name = wheel_name;
    can_drive_id = drive_id;
    can_steer_id = steer_id;
  }

  // Getters
  const std::string &get_name() const { return name; }
  uint8_t get_drive_id() const { return can_drive_id; }
  uint8_t get_steer_id() const { return can_steer_id; }

  double get_current_position() const { return current_position; }
  double get_current_velocity() const { return current_velocity; }
  
  double get_gear_ratio() const { return gear_ratio; }
  double get_steering_ratio() const { return steering_ratio; }
  double get_wheel_diameter() const { return wheel_diameter; }

  ControlMode get_control_mode() const { return control_mode; }
 

  // // Setters
  // void set_control_mode(ControlMode mode) { control_mode = mode; }
  // void set_desired_position() const {desired_position = desired_position; }
  // void set_desired_velocity() const {desired_velocity = desired_velocity; }

  // CAN feedback verwerking
  void update_from_can(double pos_data, double vel_data)
  {
    position = pos_data / gear_ratio;
    velocity = vel_data / gear_ratio;
  }

private:
  std::string name = "";
  uint8_t can_drive_id = 0;
  uint8_t can_steer_id = 0;

  double current_position = 0.0;
  double current_velocity = 0.0;

  double desired_position = 0.0;
  double desired_velocity = 0.0;

  double gear_ratio = 5.5;
  double steering_ratio = 9424.0/203.0;
  double wheel_diameter = 0.072;  // meters

  ControlMode control_mode = ControlMode::Duty_Cycle_Set;
};

}  // namespace agv_pkg

#endif  // SWERVE_WHEEL_AGV_HPP
