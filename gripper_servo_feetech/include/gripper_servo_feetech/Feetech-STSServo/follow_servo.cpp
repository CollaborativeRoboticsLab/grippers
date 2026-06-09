// Make one servo follow another (Linux)
// Leader torque is disabled so it can be moved by hand.
// Purpose: Mirror the leader's position on the follower servo
// Flow:
//  - Parse CLI for serial port and leader/follower IDs
//  - Open serial and initialize driver
//  - Disable torque on the leader, set follower to POSITION mode
//  - Loop: read leader position and write follower target position

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <string>

#include "linux_serial.hpp"
#include "STSServoDriver.hpp"

int main(int argc, char** argv) {
  std::string port = "/dev/ttyUSB0";
  int leader = 1;
  int follower = 2;

  // CLI: --port, --leader, --follower
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (arg == "--leader" && i + 1 < argc) {
      leader = std::stoi(argv[++i]);
    } else if (arg == "--follower" && i + 1 < argc) {
      follower = std::stoi(argv[++i]);
    } else if (arg == "--help") {
      std::cout << "Usage: follow_servo [--port /dev/ttyUSB0] [--leader 1] [--follower 2]\n";
      return 0;
    }
  }

  // Open and configure the Linux serial device
  LinuxSerial serial(port.c_str());
  if (!serial.begin(1000000)) {
    std::cerr << "Failed to open serial port at 1Mbps: " << port << std::endl;
    return 1;
  }
  serial.setTimeout(2);

  STSServoDriver servos;
  if (!servos.init(&serial)) {
    std::cerr << "Servo init failed" << std::endl;
    return 1;
  }

  const byte LEADER = static_cast<byte>(leader);
  const byte FOLLOWER = static_cast<byte>(follower);

  // Disable torque on leader (so it can be moved by hand)
  // and set follower to POSITION mode for tracking
  servos.writeRegister(LEADER, STSRegisters::TORQUE_SWITCH, 0);
  servos.setMode(FOLLOWER, STSMode::POSITION);

  // Follow loop: read the leader and command the follower
  while (true) {
    int pos = servos.getCurrentPosition(LEADER);
    servos.setTargetPosition(FOLLOWER, pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}