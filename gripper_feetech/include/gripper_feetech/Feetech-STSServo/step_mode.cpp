// Step mode for two servos (Linux)
// Sends synchronous step commands to both motors.
// Purpose: Demonstrate STEP mode by issuing forward/backward steps to two servos
// Flow:
//  - Parse CLI for port and two servo IDs
//  - Open serial and initialize driver
//  - Set both servos to STEP mode
//  - Loop: sync write +500 steps, wait, then -500 steps, wait

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
    int id1 = 1;
    int id2 = 2;

    // Parse CLI args: --port, --id1, --id2
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc)
        {
            port = argv[++i];
        }
        else if (arg == "--id1" && i + 1 < argc)
        {
            id1 = std::stoi(argv[++i]);
        }
        else if (arg == "--id2" && i + 1 < argc)
        {
            id2 = std::stoi(argv[++i]);
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: step_mode [--port /dev/ttyUSB0] [--id1 1] [--id2 2]\n";
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
    const uint8_t ID1 = static_cast<uint8_t>(id1);
    const uint8_t ID2 = static_cast<uint8_t>(id2);

    if (!servos.init(&serial))
    {
        std::cerr << "Servo init failed" << std::endl;
        return 1;
    }

    // Set both servos to STEP mode.
    servos.setMode(ID1, STSMode::STEP);
    servos.setMode(ID2, STSMode::STEP);

    // Convenience: synchronous write arrays for both IDs
    byte ids[2] = {ID1, ID2};
    int steps[2];
    int speeds[2];

    // Helper: wait until a given servo finishes moving
    auto wait_until_done = [&](uint8_t id)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (servos.isMoving(id))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    };

    // Loop: forward steps, then backward steps.
    while (true)
    {
        // Forward: +500 steps at moderate speed
        steps[0] = 500;
        steps[1] = 500;
        speeds[0] = 1200;
        speeds[1] = 1200;
        servos.setTargetPositions(2, ids, steps, speeds);
        wait_until_done(ID1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Backward: -500 steps
        steps[0] = -500;
        steps[1] = -500;
        speeds[0] = 1200;
        speeds[1] = 1200;
        servos.setTargetPositions(2, ids, steps, speeds);
        wait_until_done(ID1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}