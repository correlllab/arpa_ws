#include "parker_controller_interface/parker_core.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <sstream>
#include <iostream>
#include <chrono>
#include <algorithm>

namespace parker_controller_interface
{

ParkerCore::ParkerCore(const std::string& host, int port, int timeout_sec)
: host_(host),
  port_(port),
  timeout_sec_(timeout_sec),
  main_sock_(-1),
  monitor_sock_(-1),
  zero_pose_(ENCODER_0_READING / ENCODER_PPU),
  monitor_running_(false),
  last_position_(std::nan("")),
  last_velocity_(std::nan(""))
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
    std::cerr << "Failed to create main socket" << std::endl;
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
    std::cerr << "Invalid address: " << host_ << std::endl;
    ::close(main_sock_);
    main_sock_ = -1;
    return false;
  }

  if (::connect(main_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Failed to connect main socket to " << host_ << ":" << port_ << std::endl;
    ::close(main_sock_);
    main_sock_ = -1;
    return false;
  }

  // Create monitor socket
  monitor_sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (monitor_sock_ < 0) {
    std::cerr << "Failed to create monitor socket" << std::endl;
    ::close(main_sock_);
    main_sock_ = -1;
    return false;
  }

  setsockopt(monitor_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(monitor_sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  if (::connect(monitor_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Failed to connect monitor socket" << std::endl;
    ::close(main_sock_);
    ::close(monitor_sock_);
    main_sock_ = -1;
    monitor_sock_ = -1;
    return false;
  }

  std::cout << "Zero pose set to " << zero_pose_ << " user units." << std::endl;
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
    std::cerr << "Failed to send message: " << message << std::endl;
    return {};
  }
  // std::cout << "[Sent] " << message << std::endl;
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
        std::cout << "Socket read timeout reached message=" << message << std::endl;
        finished_reading = true;
      } else {
        std::cerr << "Socket read error" << std::endl;
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
  std::cout << "[Init motor] Sending PROG0..." << std::endl;
  auto prog0_response = send_telnet(main_sock_, "PROG0");
  std::cout << "[Init motor] PROG0 response lines: " << prog0_response.size() << std::endl;
  for (const auto& line : prog0_response) {
    std::cout << "[Init motor] PROG0: " << line << std::endl;
  }

  auto drive_response = send_telnet(main_sock_, "DRIVE ON X");
  std::cout << "[Init motor] DRIVE ON X response lines: " << drive_response.size() << std::endl;
  for (const auto& line : drive_response) {
    std::cout << "[Init motor] DRIVE ON X: " << line << std::endl;
  }

  auto stp_response = send_telnet(main_sock_, "STP 0");
  std::cout << "[Init motor] STP 0 response lines: " << stp_response.size() << std::endl;
  for (const auto& line : stp_response) {
    std::cout << "[Init motor] STP 0: " << line << std::endl;
  }

}

std::vector<std::string> ParkerCore::set_velocity(double velocity_m_per_s)
{
  double velocity_mm_per_sec = std::abs(velocity_m_per_s) * 1000.0;

  std::string cmd = "VEL " + std::to_string(velocity_mm_per_sec);

  // Print Sent Command
  // std::cout << "[set_velocity] Sending command: " << cmd << std::endl;

  auto response = send_telnet(main_sock_, cmd, false);
  return response;
}

std::vector<std::string> ParkerCore::goto_pose(double position_m)
{
  // Input position_m is in meters (range 0.1 to 2.0)
  // Convert to mm: 0.1m -> 100mm, 0.5m -> 500mm, 2.0m -> 2000mm
  // Python equivalent: user_units = min(MAX, max(user_units, MIN))
  double position_mm = position_m * 1000.0;

  // Clamp to valid range [100, 2000] mm
  position_mm = std::max(MIN_POSITION_MM, std::min(position_mm, MAX_POSITION_MM));

  // std::cout << "[goto_pose] Requested: " << position_m << " m -> " << position_mm << " mm" << std::endl;

  // Match Python: target_user_units = self.zero_pose - user_units
  double target_user_units = zero_pose_ - position_mm;

  std::ostringstream cmd_stream;
  cmd_stream << "MOV X " << target_user_units;
  std::string cmd = cmd_stream.str();

  // Print Sent Command
  // std::cout << "[goto_pose] Sending command: " << cmd << std::endl;

  auto response = send_telnet(main_sock_, cmd, false);
  return response;
}

void ParkerCore::jog_forward(double velocity_mm_per_s)
{
  // Set JOG velocity and start forward jog
  std::string vel_cmd = "JOG VEL X " + std::to_string(velocity_mm_per_s);
  send_telnet(main_sock_, vel_cmd, false);
  send_telnet(main_sock_, "JOG FWD X", false);
}

void ParkerCore::jog_reverse(double velocity_mm_per_s)
{
  // Set JOG velocity and start reverse jog
  std::string vel_cmd = "JOG VEL X " + std::to_string(velocity_mm_per_s);
  send_telnet(main_sock_, vel_cmd, false);
  send_telnet(main_sock_, "JOG REV X", false);
}

void ParkerCore::jog_off()
{
  send_telnet(main_sock_, "JOG OFF X", false);
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

  while (monitor_running_) {
    try {
      double current_position;
      double current_velocity;
      {
        std::lock_guard<std::mutex> lock(monitor_sock_mutex_);
        current_position = get_position_from_socket(monitor_sock_);
        current_velocity = get_velocity_from_socket(monitor_sock_);
      }
      last_position_ = current_position;
      last_velocity_ = current_velocity;

      // std::cout << "[Monitor] Pos: " << current_position << std::endl;

      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(POSITION_CHECK_INTERVAL_MS)));

    } catch (const std::exception& e) {
      std::cerr << "[Monitor thread] Error: " << e.what() << std::endl;
    }
  }
}

void ParkerCore::halt_motion()
{
  send_telnet(main_sock_, "HALT", false);
}  
}  // namespace parker_controller_interface