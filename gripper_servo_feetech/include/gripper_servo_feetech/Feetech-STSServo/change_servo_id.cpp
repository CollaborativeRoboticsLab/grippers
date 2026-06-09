// Change the ID of a Feetech STS servo (Linux)
// Purpose: Reassign a servo's ID and verify the change
// Flow:
//  - Parse CLI for port, old ID, and new ID
//  - Open serial and initialize driver
//  - Use driver.setId() to safely change ID (handles write lock)
//  - Ping new ID to confirm success and report

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
  int oldId = 1;
  int newId = 2;

  // Simple CLI parsing: --port, --old, --new
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (arg == "--old" && i + 1 < argc) {
      oldId = std::stoi(argv[++i]);
    } else if (arg == "--new" && i + 1 < argc) {
      newId = std::stoi(argv[++i]);
    } else if (arg == "--help") {
      std::cout << "Usage: change_servo_id [--port /dev/ttyUSB0] [--old 1] [--new 2]\n";
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

  // Attempt to change the ID (returns false if old not found or new already taken)
  bool ok = servos.setId(static_cast<byte>(oldId), static_cast<byte>(newId));
  if (!ok) {
    std::cerr << "Failed to change servo ID " << oldId << " -> " << newId
              << " (old may not exist, or new already taken)" << std::endl;
    return 2;
  }

  // Verify by pinging new ID
  if (servos.ping(static_cast<byte>(newId))) {
    std::cout << "Servo ID changed successfully: " << oldId << " -> " << newId << std::endl;
    return 0;
  } else {
    std::cerr << "ID change reported success but new ID did not respond." << std::endl;
    return 3;
  }
}