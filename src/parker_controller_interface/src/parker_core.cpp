#include "parker_controller_interface/parker_core.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <sstream>
#include <chrono>
#include <thread>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

namespace parker_controller_interface
{

ParkerCore::ParkerCore(const std::string& host, int port, int timeout_sec)
: host_(host),
  port_(port),
  timeout_sec_(timeout_sec),
  main_sock_(-1),
  monitor_sock_(-1),
  zero_pose_(ENCODER_0_READING / ENCODER_PPU),
  is_moving_(false),
  monitor_running_(false),
  last_position_(std::nan("")),
  last_velocity_(std::nan("")),
  last_command_time_(std::chrono::steady_clock::now()),
  last_commanded_position_(std::nan(""))
{
}

ParkerCore::~ParkerCore()
{
  close();
}

bool ParkerCore::connect()
{
  // Create main socket
  main_sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (main_sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Failed to create main socket");
    return false;
  }

  // Set socket timeout
  struct timeval tv;
  tv.tv_sec = timeout_sec_;
  tv.tv_usec = 0;
  setsockopt(main_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(main_sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  // Connect main socket
  struct sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);

  if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Invalid address: %s", host_.c_str());
    ::close(main_sock_);
    main_sock_ = -1;
    return false;
  }

  if (::connect(main_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Failed to connect main socket to %s:%d", host_.c_str(), port_);
    ::close(main_sock_);
    main_sock_ = -1;
    return false;
  }

  // Create monitor socket
  monitor_sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (monitor_sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Failed to create monitor socket");
    ::close(main_sock_);
    main_sock_ = -1;
    return false;
  }

  setsockopt(monitor_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(monitor_sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  if (::connect(monitor_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Failed to connect monitor socket");
    ::close(main_sock_);
    ::close(monitor_sock_);
    main_sock_ = -1;
    monitor_sock_ = -1;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "Zero pose set to %f user units.", zero_pose_);
  return true;
}

void ParkerCore::close()
{
  stop_monitoring();

  if (main_sock_ >= 0) {
    ::close(main_sock_);
    main_sock_ = -1;
  }
  if (monitor_sock_ >= 0) {
    ::close(monitor_sock_);
    monitor_sock_ = -1;
  }
}

bool ParkerCore::is_connected() const
{
  return main_sock_ >= 0;
}

std::vector<std::string> ParkerCore::send_telnet(int sock_fd, const std::string& message, bool blocking)
{
  std::string msg = message;
  if (msg.size() < 2 || msg.substr(msg.size() - 2) != "\r\n") {
    msg += "\r\n";
  }

  // Send message
  ssize_t sent = send(sock_fd, msg.c_str(), msg.size(), 0);
  if (sent < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Failed to send message: %s", message.c_str());
    return {};
  }
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("ParkerCore"), "[Sent] " << message);
  // If non-blocking, return immediately without waiting for response
  if (!blocking) {
    return {};
  }

  // Read response
  std::string response;
  char buffer[4096];
  bool finished_reading = false;

  while (!finished_reading) {
    ssize_t bytes_read = recv(sock_fd, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read <= 0) {
      if (bytes_read == 0) {
        finished_reading = true;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        RCLCPP_WARN(rclcpp::get_logger("ParkerCore"), "Socket read timeout reached message=%s", message.c_str());
        finished_reading = true;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "Socket read error");
        finished_reading = true;
      }
    } else {
      buffer[bytes_read] = '\0';
      response += buffer;
      // Check if we received a prompt indicating completion
      if (response.size() > 0 && response.back() == '>') {
        finished_reading = true;
      }
    }
  }

  // Split response into lines
  std::vector<std::string> response_list;
  std::istringstream stream(response);
  std::string line;

  while (std::getline(stream, line)) {
    // Remove trailing \r if present
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      response_list.push_back(line);
    }
  }

  // Remove last line if it ends with '>'
  if (!response_list.empty() && !response_list.back().empty() &&
      response_list.back().back() == '>')
  {
    response_list.pop_back();
  }

  return response_list;
}

void ParkerCore::init_motor()
{
  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] Sending PROG0...");
  auto prog0_response = send_telnet(main_sock_, "PROG0");
  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] PROG0 response lines: %zu", prog0_response.size());
  for (const auto& line : prog0_response) {
    RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] PROG0: %s", line.c_str());
  }

  // Wait 500ms between PROG0 and DRIVE ON X to let drive stabilize
  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] Waiting 500ms before DRIVE ON...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto initial_move = goto_pose(get_position());
  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] Sending DRIVE ON X...");
  auto drive_response = send_telnet(main_sock_, "DRIVE ON X");
  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] DRIVE ON X response lines: %zu", drive_response.size());
  for (const auto& line : drive_response) {
    RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[Init motor] DRIVE ON X: %s", line.c_str());
  }
}

std::vector<std::string> ParkerCore::goto_pose(double position_m)
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  // Input position_m is in meters (range 0.1 to 2.0)
  // Convert to mm: 0.1m -> 100mm, 0.5m -> 500mm, 2.0m -> 2000mm
  double position_mm = position_m * 1000.0;

  // Clamp to valid range [100, 2000] mm
  position_mm = std::max(MIN_POSITION_MM, std::min(position_mm, MAX_POSITION_MM));
  double clamped_position_m = position_mm / 1000.0;

  // Check if position change is significant enough to warrant a command
  // if (!std::isnan(last_commanded_position_)) {
  //   double position_delta = std::abs(clamped_position_m - last_commanded_position_);
  //   if (position_delta < POSITION_COMMAND_THRESHOLD) {
  //     // Position change too small, skip command
  //     return {};
  //   }
  // }

  // Rate limiting: check time since last command
  auto now = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_).count();

  if (elapsed_ms < MIN_COMMAND_INTERVAL_MS) {
    // Too soon since last command, skip to avoid overwhelming the drive
    RCLCPP_WARN(rclcpp::get_logger("ParkerCore"),
                 "[goto_pose] Rate limited: %ld ms since last command (min: %f ms)",
                 elapsed_ms, MIN_COMMAND_INTERVAL_MS);
    return {};
  }

  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[goto_pose] Requested: %f m -> %f mm", position_m, position_mm);

  // Match Python: target_user_units = self.zero_pose - user_units
  double target_user_units = zero_pose_ - position_mm;

  std::ostringstream cmd_stream;
  cmd_stream << "MOV X " << target_user_units;
  std::string cmd = cmd_stream.str();

  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[goto_pose] Sending command: %s", cmd.c_str());
  
  // Send command non-blocking to avoid delays in the control loop
  auto response = send_telnet(main_sock_, cmd, true);
  
  RCLCPP_INFO(rclcpp::get_logger("ParkerCore"), "[goto_pose] Command response lines: %zu", response.size());
  // Update rate limiting state
  last_command_time_ = now;
  last_commanded_position_ = clamped_position_m;

  return response;
}

double ParkerCore::get_position()
{
  return get_position_from_socket(main_sock_);
}

double ParkerCore::get_position_from_socket(int sock_fd)
{
  auto response = send_telnet(sock_fd, "PRINT(P12290/P12375)");

  if (response.size() > 1) {
    try {
      double user_units = std::stod(response[1]);
      // Python: location = -1*(user_units - self.zero_pose) returns mm
      double location_mm = -1.0 * (user_units - zero_pose_);
      // Convert from mm to meters (positive, range 0.1-2.0)
      double location_m = location_mm / 1000.0;
      return location_m;
    } catch (const std::exception& e) {
      return std::nan("");
    }
  }
  return std::nan("");
}

double ParkerCore::get_velocity_from_socket(int sock_fd)
{
  auto response = send_telnet(sock_fd, "PRINT(P28741*60)");

  if (response.size() > 1) {
    try {
      double velocity_mm_per_min = std::stod(response[1]);
      // Convert from mm/min to m/s
      double velocity_m_per_s = velocity_mm_per_min / 1000.0 / 60.0;
      return velocity_m_per_s;
    } catch (const std::exception& e) {
      return std::nan("");
    }
  }
  return std::nan("");
}

void ParkerCore::start_monitoring()
{
  if (monitor_running_) {
    return;
  }

  monitor_running_ = true;
  monitor_thread_ = std::thread(&ParkerCore::monitor_position, this);
}

void ParkerCore::stop_monitoring()
{
  monitor_running_ = false;
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
}

bool ParkerCore::is_moving() const
{
  return is_moving_;
}

double ParkerCore::get_last_position() const
{
  return last_position_;
}

double ParkerCore::get_last_velocity() const
{
  return last_velocity_;
}

void ParkerCore::monitor_position()
{
  double last_position = -std::numeric_limits<double>::infinity();
  int stationary_count = 0;
  int cycle_count = 0;

  while (monitor_running_) {
    try {
      double current_position;
      {
        std::lock_guard<std::mutex> lock(monitor_sock_mutex_);
        current_position = get_position_from_socket(monitor_sock_);
      }
      last_position_ = current_position;

      // Only query velocity every 5th cycle to reduce communication load
      if (cycle_count % 5 == 0) {
        double current_velocity;
        {
          std::lock_guard<std::mutex> lock(monitor_sock_mutex_);
          current_velocity = get_velocity_from_socket(monitor_sock_);
        }
        last_velocity_ = current_velocity;
      }
      cycle_count++;

      if (!std::isnan(current_position) && !std::isinf(last_position)) {
        double position_delta = std::abs(current_position - last_position);
        if (position_delta > MOVEMENT_THRESHOLD) {
          is_moving_ = true;
          stationary_count = 0;
        } else {
          stationary_count++;
          if (stationary_count >= STATIONARY_THRESHOLD) {
            is_moving_ = false;
          }
        }
      }

      last_position = current_position;

      // Sleep for the configured interval (now 500ms instead of 100ms)
      // std::this_thread::sleep_for(
      //   std::chrono::milliseconds(static_cast<int>(POSITION_CHECK_INTERVAL * 1000)));

    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ParkerCore"), "[Monitor thread] Error: %s", e.what());
      // std::this_thread::sleep_for(
      //   std::chrono::milliseconds(static_cast<int>(POSITION_CHECK_INTERVAL * 1000)));
    }
  }
}

}  // namespace parker_controller_interface
