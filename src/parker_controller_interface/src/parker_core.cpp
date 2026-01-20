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

namespace arpa_ethernet_motor
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
  last_position_(std::nan(""))
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

std::vector<std::string> ParkerCore::send_telnet(int sock_fd, const std::string& message)
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
  auto prog0_response = send_telnet(main_sock_, "PROG0");
  auto drive_response = send_telnet(main_sock_, "DRIVE ON X");
}

std::vector<std::string> ParkerCore::goto_pose(double user_units)
{
  // Clamp to valid range
  user_units = std::max(MIN_POSITION_MM, std::min(user_units, MAX_POSITION_MM));
  double target_user_units = zero_pose_ - user_units;

  std::ostringstream cmd_stream;
  cmd_stream << "MOV X " << target_user_units;
  std::string cmd = cmd_stream.str();

  std::cout << "[Goto pose] Sending command: " << cmd << std::endl;

  std::vector<std::string> response;
  do {
    response = send_telnet(main_sock_, cmd);
    std::cout << "[Goto pose] Response size: " << response.size() << std::endl;
    for (const auto& line : response) {
      std::cout << "[Goto pose] " << line << std::endl;
    }
    if (response.size() > 1) {
      init_motor();
      std::cout << "[Goto pose] Re-sending command after re-init: " << cmd << std::endl;
    }
  } while (response.empty() || response.size() > 1);

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
      double location = -1.0 * (user_units - zero_pose_);
      return location;
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

void ParkerCore::monitor_position()
{
  double last_position = -std::numeric_limits<double>::infinity();
  int stationary_count = 0;

  while (monitor_running_) {
    try {
      double current_position;
      {
        std::lock_guard<std::mutex> lock(monitor_sock_mutex_);
        current_position = get_position_from_socket(monitor_sock_);
      }
      last_position_ = current_position;

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

      if (is_moving_) {
        std::cout << "[Monitor] Pos: " << current_position << ", Moving: "
                  << (is_moving_ ? "true" : "false") << std::endl;
      }

      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(POSITION_CHECK_INTERVAL * 1000)));

    } catch (const std::exception& e) {
      std::cerr << "[Monitor thread] Error: " << e.what() << std::endl;
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(POSITION_CHECK_INTERVAL * 1000)));
    }
  }
}

}  // namespace arpa_ethernet_motor
