// Linux version of the simple motion example (header-only driver)
// Purpose: Move a single servo through a simple position sequence
// Flow:
//  - Parse CLI for serial port and servo ID
//  - Open Linux serial at 1 Mbps and initialize driver
//  - Set all servos to POSITION mode (broadcast)
//  - Move to 0 -> 2048 -> 4095 (with speed on the last move)
//  - Poll the servo movement status until each move finishes

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
            std::cout << "Usage: simple_motion [--port /dev/ttyUSB0] [--id 1]\n";
            return 0;
        }
    }

    // Open and configure the Linux serial device (/dev/ttyUSB0 by default)
    LinuxSerial serial(port.c_str());
    if (!serial.begin(1000000))
    {
        std::cerr << "Failed to open serial port at 1Mbps: " << port << std::endl;
        return 1;
    }
    serial.setTimeout(2);

    // Instantiate the header-only STSServo driver
    STSServoDriver servos;
    const uint8_t SERVO_ID = static_cast<uint8_t>(servo_id); // change to your servo's ID

    if (!servos.init(&serial))
    {
        std::cerr << "Servo init failed" << std::endl;
        return 1;
    }

    // Broadcast: set all servos to POSITION mode (0xFE is broadcast ID)
    servos.setMode(0xFE, STSMode::POSITION);

    // Simple motion: 0 -> 2048 -> 4095
    // Helper: wait until a given servo finishes moving
    auto wait_until_done = [&](uint8_t id)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (servos.isMoving(id))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    };

    // Optional: ping to confirm communication with target servo
    if (!servos.ping(SERVO_ID))
    {
        std::cerr << "Servo " << int(SERVO_ID) << " not responding" << std::endl;
    }

    // Move to near 0° (raw position 0)
    servos.setTargetPosition(SERVO_ID, 0);
    wait_until_done(SERVO_ID);

    // Move to near 180° (raw position ~2048)
    servos.setTargetPosition(SERVO_ID, 2048);
    wait_until_done(SERVO_ID);

    // Move to near 360° (raw position ~4095) with a speed limit (500)
    servos.setTargetPosition(SERVO_ID, 4095, 500);
    wait_until_done(SERVO_ID);

    return 0;
}