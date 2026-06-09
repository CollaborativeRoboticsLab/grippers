// Operate the STS3215 servo in velocity mode (Linux)
// Continuously spins at two velocities with delays.
// Purpose: Demonstrate VELOCITY mode by alternating speeds
// Flow:
//  - Parse CLI for port and servo ID
//  - Open serial and initialize driver
//  - Set VELOCITY mode for the target servo
//  - Loop: +500 steps/s for 4s, then -2048 steps/s for 2s

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
    int servo_id = 1;

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
            std::cout << "Usage: velocity_mode [--port /dev/ttyUSB0] [--id 1]\n";
            return 0;
        }
    }

    // Open and configure the Linux serial device
    LinuxSerial serial(port.c_str());
    if (!serial.begin(1000000))
    {
        std::cerr << "Failed to open serial port at 1Mbps: " << port << std::endl;
        return 1;
    }
    serial.setTimeout(2);

    STSServoDriver servos;
    const uint8_t SERVO_ID = static_cast<uint8_t>(servo_id);

    if (!servos.init(&serial))
    {
        std::cerr << "Servo init failed" << std::endl;
        return 1;
    }

    // Set the target servo to VELOCITY mode
    servos.setMode(SERVO_ID, STSMode::VELOCITY);

    // Continuous spin: +500 steps/s for 4s, then -2048 steps/s for 2s
    while (true)
    {
        // Spin forward at +500 steps/s
        servos.setTargetVelocity(SERVO_ID, 500);
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        // Spin backward at -2048 steps/s
        servos.setTargetVelocity(SERVO_ID, -2048);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    return 0;
}