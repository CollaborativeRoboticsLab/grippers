// Synchronous motion example for two STS servos (Linux)
// Purpose: Move two servos to the same targets in lockstep
// Flow:
//  - Parse CLI for port and two servo IDs
//  - Open serial and initialize driver
//  - Set all servos to POSITION mode (broadcast)
//  - Send synchronized targets using setTargetPositions()
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

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int id1 = 1;
    int id2 = 2;

    // CLI: --port, --id1, --id2
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--id1" && i + 1 < argc) {
            id1 = std::stoi(argv[++i]);
        } else if (arg == "--id2" && i + 1 < argc) {
            id2 = std::stoi(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "Usage: move_synchronously [--port /dev/ttyUSB0] [--id1 1] [--id2 2]\n";
            return 0;
        }
    }

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

    // Broadcast: set all servos to POSITION mode (0xFE)
    servos.setMode(0xFE, STSMode::POSITION);

    byte ids[2] = { static_cast<byte>(id1), static_cast<byte>(id2) };
    int positions[2];
    int speeds[2] = { 2400, 2400 };

    // Helper: wait until a given servo finishes moving
    auto wait_until_done = [&](uint8_t id){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (servos.isMoving(id)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    };

    // Move both to ~180° (raw ~2048)
    positions[0] = 2048;
    positions[1] = 2048;
    servos.setTargetPositions(2, ids, positions, speeds);
    wait_until_done(static_cast<uint8_t>(id1));

    // Move both to ~360° (raw ~4095)
    positions[0] = 4095;
    positions[1] = 4095;
    servos.setTargetPositions(2, ids, positions, speeds);
    wait_until_done(static_cast<uint8_t>(id1));

    return 0;
}