// Linux version of the simple motion example (header-only driver)
// Purpose: Home a single servo (move to raw position 0) and exit
// Flow:
//  - Parse CLI for serial port and target servo ID
//  - Open Linux serial at 1 Mbps and initialize driver
//  - Set all servos to POSITION mode (broadcast)
//  - Ping target servo (optional) and send target position 0
//  - Poll movement status until motion completes

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
            std::cout << "Usage: move_to_home [--port /dev/ttyUSB0] [--id 1]\n";
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

    if (!servos.init(&serial))
    {
        std::cerr << "Servo init failed" << std::endl;
        return 1;
    }

    servos.setMode(0xFE, STSMode::POSITION);

    // Helper: wait until the target servo finishes moving
    auto wait_until_done = [&](uint8_t id)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (servos.isMoving(id))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    };

    // Optional: ping and then home the single target
    if (!servos.ping(SERVO_ID))
    {
        std::cerr << "Servo " << int(SERVO_ID) << " not responding" << std::endl;
    }
    // Command home (raw position 0)
    servos.setTargetPosition(SERVO_ID, 0);
    wait_until_done(SERVO_ID);

    return 0;
}