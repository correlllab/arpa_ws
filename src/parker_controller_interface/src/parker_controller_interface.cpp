#include "parker_controller_interface/parker_controller_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace parker_controller_interface
{
hardware_interface::CallbackReturn ParkerControllerInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF (info_ is set by parent on_init)
  host_ = info_.hardware_parameters.count("host") ?
          info_.hardware_parameters.at("host") : DEFAULT_HOST;

  port_ = info_.hardware_parameters.count("port") ?
          std::stoi(info_.hardware_parameters.at("port")) : DEFAULT_PORT;

  // Create Parker driver instance
  parker_ = std::make_unique<ParkerCore>(host_, port_);

  // Get physical limits
  min_position_ = std::stod(info_.hardware_parameters["min_position"]);
  max_position_ = std::stod(info_.hardware_parameters["max_position"]);
  max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
  max_effort_ = std::stod(info_.hardware_parameters["max_effort"]);

  // Initialize state and command variables
  hw_position_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_velocity_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_effort_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_effort_command_ = std::numeric_limits<double>::quiet_NaN();

  // Validate configuration - expect exactly one joint
  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerHardwareInterface"),
      "Expected 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];
  joint_name_ = joint.name;

  // Validate joint has position command interface
  if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerHardwareInterface"),
      "Joint '%s' must have exactly one position command interface", joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate joint has position state interface
  if (joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerHardwareInterface"),
      "Joint '%s' must have exactly one position state interface", joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state
  hw_position_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerHardwareInterface"),
    "Initialized with host=%s, port=%d, joint=%s",
    host_.c_str(), port_, joint_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerHardwareInterface"),
    "Configuring Parker hardware interface...");

  // Create Parker driver instance
  parker_ = std::make_unique<ParkerCore>(host_, port_);

  // Connect to hardware
  if (!parker_->connect()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerHardwareInterface"),
      "Failed to connect to Parker controller at %s:%d", host_.c_str(), port_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerHardwareInterface"),
    "Connected to Parker controller");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ParkerControllerInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export position, velocity, and effort state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_state_));
  
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_));
  
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ParkerControllerInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export position, velocity, and effort command interfaces
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_command_));
  
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //   info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_command_));
  
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //   info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Activating parker controller interface...");

  // Initialize motor
  parker_->init_motor();

  // Start position monitoring
  parker_->start_monitoring();

  // Read initial position
  hw_position_state_ = parker_->get_position();
  hw_position_command_ = hw_position_state_;

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerHardwareInterface"),
    "Activated. Initial position: %.4f mm", hw_position_state_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerHardwareInterface"),
    "Deactivating Parker hardware interface...");

  if (parker_) {
    parker_->stop_monitoring();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ParkerControllerInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get position from monitoring thread (non-blocking)
  double position = parker_->get_last_position();

  if (!std::isnan(position)) {
    hw_position_state_ = position;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ParkerControllerInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Only send command if it has changed significantly
  if (!std::isnan(hw_position_command_)) {
    double position_error = std::abs(hw_position_command_ - hw_position_state_);

    // Only command if difference is significant (avoid jitter)
    if (position_error > 0.1) {  // 0.1mm threshold
      parker_->goto_pose(hw_position_command_);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace parker_controller_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  parker_controller_interface::ParkerControllerInterface, hardware_interface::ActuatorInterface)