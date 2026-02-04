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
  estop_sock_(-1),
  zero_pose_(ENCODER_0_READING / ENCODER_PPU),
  monitor_running_(false),
  last_position_(std::nan("")),
  last_velocity_(std::nan("")),
  command_worker_running_(false)
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

  //connect estop socket
  estop_sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (estop_sock_ < 0) {
    std::cerr << "Failed to create estop socket" << std::endl;
    ::close(main_sock_);
    ::close(monitor_sock_);
    main_sock_ = -1;
    monitor_sock_ = -1;
    return false;
  }
  setsockopt(estop_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(estop_sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  if (::connect(estop_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Failed to connect estop socket" << std::endl;
    ::close(main_sock_);
    ::close(monitor_sock_);
    ::close(estop_sock_);
    main_sock_ = -1;
    monitor_sock_ = -1;
    estop_sock_ = -1;
    return false;
  }

  std::cout << "Zero pose set to " << zero_pose_ << " user units." << std::endl;

  // Start command queue worker thread
  command_worker_running_ = true;
  command_worker_thread_ = std::thread(&ParkerCore::process_command_queue, this);
  std::cout << "[ParkerCore] Command queue worker thread started." << std::endl;

  return true;
}

void ParkerCore::close()
{
  stop_monitoring();

  // Stop command queue worker thread
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_worker_running_ = false;
  }
  command_queue_cv_.notify_all();
  if (command_worker_thread_.joinable()) {
    command_worker_thread_.join();
  }

  if (main_sock_ >= 0) {
    ::close(main_sock_);
    main_sock_ = -1;
  }
  if (monitor_sock_ >= 0) {
    ::close(monitor_sock_);
    monitor_sock_ = -1;
  }
  if (estop_sock_ >= 0) {
    ::close(estop_sock_);
    estop_sock_ = -1;
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


  // send_telnet(main_sock_, "MBUF ON");
  // send_telnet(main_sock_, "DIM MBUF (10)");
  // send_telnet(main_sock_, "LOOK ON");


}

void ParkerCore::set_stp(double stp){
  std::string cmd = "STP " + std::to_string(stp);

  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_queue_.push({cmd, false});
  }
  command_queue_cv_.notify_one();
  std::cout << "[Set STP] Queued: " << cmd << std::endl;
  // send_telnet(main_sock_, cmd, false);
}


void ParkerCore::set_velocity(double velocity_m_per_s)
{
  double velocity_mm_per_sec = std::abs(velocity_m_per_s) * 1000.0;

  std::string cmd = "VEL " + std::to_string(velocity_mm_per_sec);
  // std::string cmd = "FOV " + std::to_string(velocity_mm_per_sec);


  // Enqueue command for async processing (non-blocking send, queue provides ordering)
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_queue_.push({cmd, false});
  }
  command_queue_cv_.notify_one();
  // send_telnet(main_sock_, cmd, false);

}

void ParkerCore::set_inmotion_params()
{
  set_stp(0);
}

void ParkerCore::set_final_motion_params()
{
  set_stp(100);
  set_velocity(0.5);
}


void ParkerCore::goto_pose(double position_m)
{
  // Input position_m is in meters (range 0.1 to 2.0)
  // Convert to mm: 0.1m -> 100mm, 0.5m -> 500mm, 2.0m -> 2000mm
  // Python equivalent: user_units = min(MAX, max(user_units, MIN))
  double position_mm = position_m * 1000.0;

  // Clamp to valid range [100, 2000] mm
  position_mm = std::max(MIN_POSITION_MM, std::min(position_mm, MAX_POSITION_MM));

  // Match Python: target_user_units = self.zero_pose - user_units
  double target_user_units = zero_pose_ - position_mm;

  std::ostringstream cmd_stream;
  cmd_stream << "MOV X " << target_user_units;
  std::string cmd = cmd_stream.str();

  // Enqueue command for async processing (non-blocking send, queue provides ordering)
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_queue_.push({cmd, false});
  }
  command_queue_cv_.notify_one();

}

void ParkerCore::jog_forward(double velocity_mm_per_s)
{
  // Set JOG velocity and start forward jog
  std::string vel_cmd = "JOG VEL X " + std::to_string(velocity_mm_per_s);
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_queue_.push({vel_cmd, false});
    command_queue_.push({"JOG FWD X", false});
  }
  command_queue_cv_.notify_one();
}

void ParkerCore::jog_reverse(double velocity_mm_per_s)
{
  // Set JOG velocity and start reverse jog
  std::string vel_cmd = "JOG VEL X " + std::to_string(velocity_mm_per_s);
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_queue_.push({vel_cmd, false});
    command_queue_.push({"JOG REV X", false});
  }
  command_queue_cv_.notify_one();
}

void ParkerCore::jog_off()
{
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    command_queue_.push({"JOG OFF X", false});
  }
  command_queue_cv_.notify_one();
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

void ParkerCore::process_command_queue()
{
  while (true) {
    Command cmd;
    {
      std::unique_lock<std::mutex> lock(command_queue_mutex_);
      command_queue_cv_.wait(lock, [this] {
        std::cout << "[Command queue] Waiting for commands..." << std::endl;
        return !command_queue_.empty() || !command_worker_running_;
      });

      if (!command_worker_running_ && command_queue_.empty()) {
        std::cout << "[Command queue] Worker thread stopping (no more commands and not running)." << std::endl;

        break;
      }

      if (!command_queue_.empty()) {
        cmd = command_queue_.front();
        command_queue_.pop();
      } else {
        continue;
      }
    }

    // Process command outside lock - send with blocking to ensure delivery
    try {
      std::cout << "[Command queue] Processing command: " << cmd.cmd << std::endl;
      // send_telnet(main_sock_, cmd.cmd, cmd.blocking);
      // send_telnet(main_sock_, cmd.cmd, true);
      // Use estop socket for CLEAR STREAM, SET, and CLR commands
      if (cmd.cmd.find("CLEAR STREAM") != std::string::npos ||
          cmd.cmd.find("SET") != std::string::npos ||
          cmd.cmd.find("CLR") != std::string::npos) {
        send_telnet(estop_sock_, cmd.cmd, true);
      } else {
        send_telnet(main_sock_, cmd.cmd, true);
      }


    } catch (const std::exception& e) {
      std::cerr << "[Command queue] Error sending command: " << e.what() << std::endl;
    }
  }
  std::cout << "[ParkerCore] Command queue worker thread stopped." << std::endl;
}


void ParkerCore::set_force_stop()
{
  // Clear the command queue to prevent pending commands from executing
  size_t queue_size = 0;
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    queue_size = command_queue_.size();
    std::queue<Command> empty;
    std::swap(command_queue_, empty);
    // command_queue_.push({"CLEAR STREAM", false});

    //set the kill all moves bit
    // command_queue_.push({"SET 522", false});
    command_queue_.push({"SET 8467", false});

  }
  command_queue_cv_.notify_one();
  std::cout << "[Force Stop] Cleared " << queue_size << " commands from queue. Queued SET." << std::endl;
}

void ParkerCore::clear_force_stop()
{
  // Clear the kill all moves bit
  {
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    // command_queue_.push({"CLEAR STREAM", false});
    command_queue_.push({"CLR 522", false});
    command_queue_.push({"CLR 8467", false});

    
  }
  command_queue_cv_.notify_one();
  std::cout << "[Clear Force Stop] Queued CLR." << std::endl;
}

void ParkerCore::quick_stop()
{
  set_force_stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  clear_force_stop();
}


}    // namespace parker_controller_interface