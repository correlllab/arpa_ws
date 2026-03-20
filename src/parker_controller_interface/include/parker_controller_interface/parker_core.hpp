#ifndef PARKER_CONTROLLER_INTERFACE__PARKER_CORE_HPP_
#define PARKER_CONTROLLER_INTERFACE__PARKER_CORE_HPP_

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <cmath>

namespace parker_controller_interface
{

// Default settings
constexpr const char* DEFAULT_HOST = "192.168.100.1";
constexpr int DEFAULT_PORT = 5002;
constexpr int TIMEOUT_SEC = 5;
constexpr double DELAY_BETWEEN_CMDS = 0.2;
constexpr double MOVEMENT_THRESHOLD = 0.0005;
constexpr double ENCODER_0_READING = -518027935.0; //PRINT(P12290)
constexpr double ENCODER_PPU = 26214.4;
constexpr int STATIONARY_THRESHOLD = 3;
constexpr double MIN_POSITION_MM = 100.0;
constexpr double MAX_POSITION_MM = 2000.0;
constexpr double POSITION_CHECK_INTERVAL_MS = 20;

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
  void set_velocity(double velocity_m_per_s);
  void goto_pose(double position_m);
  void set_stp(double stp);

  // JOG mode control (for continuous velocity control)
  void jog_forward(double velocity_mm_per_s);
  void jog_reverse(double velocity_mm_per_s);
  void jog_off();

  // Position monitoring
  void start_monitoring();
  void stop_monitoring();
  double get_last_position() const;
  double get_last_velocity() const;


  void set_force_stop();
  void clear_force_stop();
  void quick_stop();
  void set_inmotion_params();
  void set_final_motion_params();


private:
  // Socket communication
  std::vector<std::string> send_telnet(int sock_fd, const std::string& message, bool blocking = true);
  double get_position_from_socket(int sock_fd);
  double get_velocity_from_socket(int sock_fd);

  // Monitoring thread function
  void monitor_position();

  // Command queue worker thread function
  void process_command_queue();

  // Command structure for queuing
  struct Command {
    std::string cmd;
    bool blocking;
  };

  // Connection parameters
  std::string host_;
  int port_;
  int timeout_sec_;

  // Socket file descriptors
  int main_sock_;
  int monitor_sock_;
  int estop_sock_;

  // Zero pose reference
  double zero_pose_;

  // Monitoring state
  std::atomic<bool> monitor_running_;
  std::atomic<double> last_position_;
  std::atomic<double> last_velocity_;
  std::thread monitor_thread_;
  std::mutex monitor_sock_mutex_;

  // Command queue state
  std::queue<Command> command_queue_;
  std::mutex command_queue_mutex_;
  std::condition_variable command_queue_cv_;
  std::atomic<bool> command_worker_running_;
  std::thread command_worker_thread_;
};

}  // namespace parker_controller_interface

#endif  // PARKER_CONTROLLER_INTERFACE__PARKER_CORE_HPP_