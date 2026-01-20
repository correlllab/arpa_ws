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

  // Get parameters from URDF
  device_port_ = info_.hardware_parameters["device_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  use_simulation_ = info_.hardware_parameters["use_simulation"] == "true";
  
  // Get physical limits
  min_position_ = std::stod(info_.hardware_parameters["min_position"]);
  max_position_ = std::stod(info_.hardware_parameters["max_position"]);
  max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
  max_effort_ = std::stod(info_.hardware_parameters["max_effort"]);

  // Initialize state and command variables
  hw_position_ = std::numeric_limits<double>::quiet_NaN();
  hw_velocity_ = std::numeric_limits<double>::quiet_NaN();
  hw_effort_ = std::numeric_limits<double>::quiet_NaN();
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_effort_command_ = std::numeric_limits<double>::quiet_NaN();

  // Verify joint configuration
  if (info_.joints.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("LinearActuatorHardware"),
      "Linear actuator expects exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const hardware_interface::ComponentInfo & joint = info_.joints[0];
  
  // Verify state interfaces
  if (joint.state_interfaces.size() != 3)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("LinearActuatorHardware"),
      "Joint '%s' needs 3 state interfaces (position, velocity, effort)",
      joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verify command interfaces
  if (joint.command_interfaces.size() != 3)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("LinearActuatorHardware"),
      "Joint '%s' needs 3 command interfaces (position, velocity, effort)",
      joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("LinearActuatorHardware"),
    "Successfully initialized linear actuator hardware for joint '%s'",
    joint.name.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Configuring parker controller interface...");

  // Initialize hardware communication if not using simulation
  if (!use_simulation_)
  {
    if (!connect_to_hardware())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ParkerControllerInterface"),
        "Failed to connect to hardware on port %s", device_port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    RCLCPP_INFO(
      rclcpp::get_logger("ParkerControllerInterface"),
      "Running in simulation mode");
    is_connected_ = true;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ParkerControllerInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export position, velocity, and effort state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ParkerControllerInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export position, velocity, and effort command interfaces
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_command_));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_command_));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Activating parker controller interface...");

  // Set initial position from hardware or use center position for simulation
  if (use_simulation_)
  {
    hw_position_ = (min_position_ + max_position_) / 2.0;
    hw_velocity_ = 0.0;
    hw_effort_ = 0.0;
  }
  else
  {
    if (!read_state_from_hardware(hw_position_, hw_velocity_, hw_effort_))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ParkerControllerInterface"),
        "Failed to read initial state from hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize commands to current state
  hw_position_command_ = hw_position_;
  hw_velocity_command_ = 0.0;
  hw_effort_command_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Parker controller interface activated at position: %.3f m", hw_position_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Deactivating parker controller interface...");

  // Send zero velocity/effort command before deactivating
  if (!use_simulation_)
  {
    send_command_to_hardware(hw_position_, 0.0, 0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ParkerControllerInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_simulation_)
  {
    // Simple simulation: move towards commanded position
    double position_error = hw_position_command_ - hw_position_;
    const double max_step = 0.001;  // 1mm per read cycle
    
    if (std::abs(position_error) > max_step)
    {
      hw_position_ += (position_error > 0 ? max_step : -max_step);
    }
    else
    {
      hw_position_ = hw_position_command_;
    }
    
    // Clamp to physical limits
    hw_position_ = std::max(min_position_, std::min(max_position_, hw_position_));
    
    // Simulate velocity based on position change
    hw_velocity_ = hw_velocity_command_;
    
    // Simulate effort
    hw_effort_ = hw_effort_command_;
  }
  else
  {
    // Read actual state from hardware
    if (!read_state_from_hardware(hw_position_, hw_velocity_, hw_effort_))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ParkerControllerInterface"),
        "Failed to read state from hardware");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ParkerControllerInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Validate and clamp commands
  if (!std::isnan(hw_position_command_))
  {
    hw_position_command_ = std::max(min_position_, 
                                    std::min(max_position_, hw_position_command_));
  }
  
  if (!std::isnan(hw_velocity_command_))
  {
    hw_velocity_command_ = std::max(-max_velocity_, 
                                    std::min(max_velocity_, hw_velocity_command_));
  }
  
  if (!std::isnan(hw_effort_command_))
  {
    hw_effort_command_ = std::max(-max_effort_, 
                                  std::min(max_effort_, hw_effort_command_));
  }

  if (!use_simulation_)
  {
    // Send commands to actual hardware
    if (!send_command_to_hardware(hw_position_command_, hw_velocity_command_, hw_effort_command_))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ParkerControllerInterface"),
        "Failed to write commands to hardware");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

// Private helper methods
bool ParkerControllerInterface::connect_to_hardware()
{
  // TODO: Implement actual hardware connection logic
  // This could involve:
  // - Opening serial port communication
  // - Initializing CAN bus
  // - Establishing TCP/IP connection
  // - Validating device responses
  
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Connecting to hardware on port %s at %d baud", 
    device_port_.c_str(), baud_rate_);
  
  is_connected_ = true;
  return true;
}

void ParkerControllerInterface::disconnect_from_hardware()
{
  // TODO: Implement hardware disconnection logic
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Disconnecting from hardware");
  
  is_connected_ = false;
}

bool ParkerControllerInterface::send_command_to_hardware(
  double position, double velocity, double effort)
{
  if (!is_connected_)
  {
    return false;
  }

  // TODO: Implement actual hardware command transmission
  // Example:
  // - Format command packet
  // - Send via serial/CAN/network
  // - Wait for acknowledgment
  
  return true;
}

bool ParkerControllerInterface::read_state_from_hardware(
  double & position, double & velocity, double & effort)
{
  if (!is_connected_)
  {
    return false;
  }

  // TODO: Implement actual hardware state reading
  // Example:
  // - Request state from device
  // - Parse response packet
  // - Update position, velocity, effort values
  
  return true;
}

}  // namespace parker_controller_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  parker_controller_interface::ParkerControllerInterface, hardware_interface::ActuatorInterface)