#ifndef PARKER_CONTROLLER_INTERFACE__PARKER_CONTROLLER_INTERFACE_HPP_
#define PARKER_CONTROLLER_INTERFACE__PARKER_CONTROLLER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "parker_controller_interface/parker_core.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace parker_controller_interface
{
class ParkerControllerInterface : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ParkerControllerInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parker motor driver
  std::unique_ptr<ParkerCore> parker_;

  // Hardware communication parameters
  std::string host_;
  int port_;
  
  // Linear actuator state
  double hw_position_state_;
  double hw_velocity_state_;
  double hw_effort_state_;
  
  // Linear actuator commands
  double hw_position_command_;
  double hw_velocity_command_;
  double hw_effort_command_;
  
  // Physical limits
  double min_position_;
  double max_position_;
  double max_velocity_;
  double max_effort_;

  // Communication status
  bool is_connected_;

  // Joint name
  std::string joint_name_;

  // Track last commanded position to avoid redundant commands
  double last_commanded_position_;
};

}  // namespace parker_controller_interface

#endif  // PARKER_CONTROLLER_INTERFACE__PARKER_CONTROLLER_INTERFACE_HPP_