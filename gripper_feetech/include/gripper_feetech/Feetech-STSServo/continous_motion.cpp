// Purpose: Continuous demonstration of repeated position moves.
// Flow:
//  - Parse CLI for port and servo ID
//  - Open serial and initialize driver
//  - Set POSITION mode (broadcast) and loop through 0/2048/4095
//  - Ping each cycle (optional), command position, and poll movement

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

int main(int argc, char **argv)
{
  std::string port = "/dev/ttyUSB0";
  int servo_id = 2;

  // Parse CLI args: --port, --id
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc)
    {
      port = argv[++i];
    }
    else if (arg == "--id" && i + 1 < argc)
    {
      servo_id = std::stoi(argv[++i]);
    }
    else if (arg == "--help")
    {
      std::cout << "Usage: sweep_motion_with_interface_board [--port /dev/ttyUSB0] [--id 2]\n";
      return 0;
    }
  }

  LinuxSerial serial(port.c_str());
  if (!serial.begin(1000000))
  {
    std::cerr << "Failed to open serial port at 1Mbps: " << port << std::endl;
    return 1;
  }
  serial.setTimeout(2);

  STSServoDriver servos;
  const uint8_t SERVO_ID = static_cast<uint8_t>(servo_id);

  if (servos.init(&serial))
  {
    std::cout << "Servos are ready" << std::endl;
  }
  else
  {
    std::cerr << "Servo init failed" << std::endl;
    return 1;
  }

  servos.setMode(0xFE, STSMode::POSITION);

  while (true)
  {
    // Optional: ping each cycle to check communication
    if (!servos.ping(SERVO_ID))
    {
      std::cerr << "servo-" << int(SERVO_ID) << " response nothing" << std::endl;
    }

    // Move to near 0°
    servos.setTargetPosition(SERVO_ID, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (servos.isMoving(SERVO_ID))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move to near 180°
    servos.setTargetPosition(SERVO_ID, 2048);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (servos.isMoving(SERVO_ID))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move to near 360° with a slower speed
    servos.setTargetPosition(SERVO_ID, 4095, 500);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (servos.isMoving(SERVO_ID))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}