#ifndef ARPA_ETHERNET_MOTOR__PARKER_CORE_HPP_
#define ARPA_ETHERNET_MOTOR__PARKER_CORE_HPP_

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

namespace arpa_ethernet_motor
{

// Default settings
constexpr const char* DEFAULT_HOST = "192.168.100.1";
constexpr int DEFAULT_PORT = 5002;
constexpr int TIMEOUT_SEC = 5;
constexpr double DELAY_BETWEEN_CMDS = 0.2;
constexpr double POSITION_CHECK_INTERVAL = 0.1;
constexpr double MOVEMENT_THRESHOLD = 0.001;
constexpr double ENCODER_0_READING = -517891070.0;
constexpr double ENCODER_PPU = 26214.4;
constexpr int STATIONARY_THRESHOLD = 5;
constexpr double MIN_POSITION_MM = 100.0;
constexpr double MAX_POSITION_MM = 2000.0;

class ParkerCore
{
public:
  ParkerCore(
    const std::string& host = DEFAULT_HOST,
    int port = DEFAULT_PORT,
    int timeout_sec = TIMEOUT_SEC);

  ~ParkerCore();

  // Disable copy
  ParkerCore(const ParkerCore&) = delete;
  ParkerCore& operator=(const ParkerCore&) = delete;

  // Connection management
  bool connect();
  void close();
  bool is_connected() const;

  // Motor control
  void init_motor();
  std::vector<std::string> goto_pose(double user_units);
  double get_position();

  // Position monitoring
  void start_monitoring();
  void stop_monitoring();
  bool is_moving() const;
  double get_last_position() const;

private:
  // Socket communication
  std::vector<std::string> send_telnet(int sock_fd, const std::string& message);
  double get_position_from_socket(int sock_fd);

  // Monitoring thread function
  void monitor_position();

  // Connection parameters
  std::string host_;
  int port_;
  int timeout_sec_;

  // Socket file descriptors
  int main_sock_;
  int monitor_sock_;

  // Zero pose reference
  double zero_pose_;

  // Monitoring state
  std::atomic<bool> is_moving_;
  std::atomic<bool> monitor_running_;
  std::atomic<double> last_position_;
  std::thread monitor_thread_;
  std::mutex monitor_sock_mutex_;
};

}  // namespace arpa_ethernet_motor

#endif  // ARPA_ETHERNET_MOTOR__PARKER_CORE_HPP_
