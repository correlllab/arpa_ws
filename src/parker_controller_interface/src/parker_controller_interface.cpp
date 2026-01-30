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
  rclcpp::Clock steady_clock{RCL_STEADY_TIME};

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

  // Get controller type: "POS" for position-only, "VEL" for velocity control
  // controller_type_ = info_.hardware_parameters.count("controller_type") ?
  //                    info_.hardware_parameters.at("controller_type") : "POS";
  controller_type_ = "POS";
  // Initialize state and command variables
  hw_position_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_velocity_state_ = std::numeric_limits<double>::quiet_NaN();
  // hw_effort_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
  // hw_effort_command_ = std::numeric_limits<double>::quiet_NaN();

  // Validate configuration - expect exactly one joint
  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerControllerInterface"),
      "Expected 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];
  joint_name_ = joint.name;

  // Validate joint has position and velocity command interfaces
  if (joint.command_interfaces.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerControllerInterface"),
      "Joint '%s' must have exactly two command interfaces (position and velocity)", joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate joint has position and velocity state interfaces
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerControllerInterface"),
      "Joint '%s' must have exactly two state interfaces (position and velocity)", joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Initialized with host=%s, port=%d, joint=%s",
    host_.c_str(), port_, joint_name_.c_str());



  last_write_time_ = std::chrono::steady_clock::now();
  magic_five_counter_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Configuring Parker hardware interface...");

  // Create Parker driver instance
  parker_ = std::make_unique<ParkerCore>(host_, port_);

  // Connect to hardware
  if (!parker_->connect()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ParkerControllerInterface"),
      "Failed to connect to Parker controller at %s:%d", host_.c_str(), port_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Connected to Parker controller");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ParkerControllerInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export position and velocity state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_state_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_));

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
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_command_));
  
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
  last_commanded_position_ = hw_position_state_;

  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Activated. Initial position: %.4f m", hw_position_state_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ParkerControllerInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ParkerControllerInterface"),
    "Deactivating Parker hardware interface...");

  if (parker_) {
    parker_->stop_monitoring();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ParkerControllerInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get position and velocity from monitoring thread (non-blocking)
  double position = parker_->get_last_position();
  double velocity = parker_->get_last_velocity();

  if (!std::isnan(position)) {
    hw_position_state_ = position;
  }

  if (!std::isnan(velocity)) {
    hw_velocity_state_ = velocity;
  }

  // ROS LOG FOR READ POSITION AND VELOCITY
  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("ParkerControllerInterface"), steady_clock, 250,
    "Read position: %.4f m, velocity: %.4f m/s", position, velocity);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ParkerControllerInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Only send at a fixed rate
  if (std::chrono::steady_clock::now() - last_write_time_ <
      std::chrono::milliseconds(200))
  {
    return hardware_interface::return_type::OK;
  }
  last_write_time_ = std::chrono::steady_clock::now();

  if (std::isnan(hw_position_command_)) {
    return hardware_interface::return_type::OK;
  }

  if (controller_type_ == "VEL") {
    // Velocity control with position feedback (P-controller)
    if (!std::isnan(hw_position_state_)) {
      double position_error = hw_position_command_ - hw_position_state_;

      // Feedforward velocity from trajectory + proportional correction
      double Kp = 2.0;
      double feedforward_velocity = std::isnan(hw_velocity_command_) ? 0.0 : hw_velocity_command_;
      double commanded_velocity = feedforward_velocity + (Kp * position_error);

      // Clamp to velocity limits (m/s)
      double max_vel = 0.5;
      commanded_velocity = std::max(-max_vel, std::min(commanded_velocity, max_vel));

      // Convert to mm/s for Parker
      double velocity_mm_s = std::abs(commanded_velocity) * 1000.0;

      // Deadband to prevent jitter (5mm/s minimum)
      constexpr double deadband_mm_s = 5.0;

      if (velocity_mm_s < deadband_mm_s) {
        parker_->jog_off();
      } else if (commanded_velocity > 0) {
        parker_->jog_reverse(velocity_mm_s);
      } else {
        parker_->jog_forward(velocity_mm_s);
      }

      RCLCPP_INFO(
        rclcpp::get_logger("ParkerControllerInterface"),
        "[VEL] Pos cmd: %.4f, State: %.4f, Err: %.4f, Vel: %.4f m/s",
        hw_position_command_, hw_position_state_, position_error, commanded_velocity);
    }
  } else {
      if(hw_position_command_ == last_commanded_position_)
      {
        if(magic_five_counter_ >= 6)
        {
          return hardware_interface::return_type::OK;
        }
        magic_five_counter_++; 
        if(magic_five_counter_ > 5)
        {
          if(magic_five_counter_ == 6)
          {
            // send true stop
            parker_->halt_motion();
            RCLCPP_INFO(rclcpp::get_logger("ParkerControllerInterface"), "Sending HALT command");
          }
          return hardware_interface::return_type::OK;
        }
      } else {
          magic_five_counter_ = 0;
      }

      parker_->set_velocity(hw_velocity_command_);
      parker_->goto_pose(hw_position_command_);
      last_commanded_position_ = hw_position_command_;

      RCLCPP_INFO(
        rclcpp::get_logger("ParkerControllerInterface"),
        "[POS] Sending position: %.4f m, velocity: %.4f m/s",
        hw_position_command_, hw_velocity_command_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace parker_controller_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  parker_controller_interface::ParkerControllerInterface, hardware_interface::ActuatorInterface)