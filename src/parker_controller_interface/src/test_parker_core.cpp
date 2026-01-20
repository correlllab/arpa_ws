#include "arpa_ethernet_motor/parker_core.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>

using namespace arpa_ethernet_motor;

int main(int argc, char** argv)
{
std::cout << "Starting ParkerCore interactive test" << std::endl;

ParkerCore parker;  // uses default host/port/timeout

if (!parker.connect()) {
std::cerr << "Failed to connect to Parker controller" << std::endl;
return 1;
}

std::cout << "Connected successfully." << std::endl;
parker.init_motor();
parker.start_monitoring();

bool running = true;
while (running) {
std::cout << "\n=== Parker Core Menu ===\n";
std::cout << "m <pos_mm>  : Move to position (mm)\n";
std::cout << "p           : Print current position\n";
std::cout << "s           : Show moving state\n";
std::cout << "q           : Quit\n";
std::cout << "> ";


std::string cmd;
std::cin >> cmd;

if (!std::cin.good()) {
  std::cin.clear();
  std::cin.ignore(1024, '\n');
  continue;
}

if (cmd == "m") {
  double target;
  std::cin >> target;
  if (std::cin.fail()) {
    std::cin.clear();
    std::cin.ignore(1024, '\n');
    std::cerr << "Invalid position input" << std::endl;
    continue;
  }

  std::cout << "Moving to " << target << " mm" << std::endl;
  parker.goto_pose(target);
}
else if (cmd == "p") {
  double pos = parker.get_position();
  if (std::isnan(pos)) {
    std::cout << "Position: NaN (not available)" << std::endl;
  } else {
    std::cout << "Position: " << pos << " mm" << std::endl;
  }
}
else if (cmd == "s") {
  std::cout << "Moving: "
            << (parker.is_moving() ? "YES" : "NO")
            << ", Last position: "
            << parker.get_last_position()
            << std::endl;
}
else if (cmd == "q") {
  running = false;
}
else {
  std::cout << "Unknown command" << std::endl;
}


}

std::cout << "Shutting down..." << std::endl;
parker.stop_monitoring();
parker.close();

return 0;
}
