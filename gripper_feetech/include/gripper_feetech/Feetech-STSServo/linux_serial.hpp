#ifndef LINUX_SERIAL_HPP
#define LINUX_SERIAL_HPP

#include <cstdint>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <thread>

// Minimal Linux serial class compatible with STSServoDriver's HardwareSerial API
class LinuxSerial
{
public:
    explicit LinuxSerial(const char *device) : fd_(-1)
    {
        fd_ = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            perror("open serial port");
        }
    }
    ~LinuxSerial()
    {
        if (fd_ >= 0)
            close(fd_);
    }

    bool begin(long /*baudrate*/)
    {
        if (fd_ < 0)
            return false;
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd_, &tty) != 0)
            return false;
        cfsetospeed(&tty, B1000000);
        cfsetispeed(&tty, B1000000);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2; // 0.2s read timeout
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            return false;
        return true;
    }

    void setTimeout(int ms) { timeout_ = ms; }

    int write(const void *buf, size_t len) { return ::write(fd_, buf, len); }
    int write(uint8_t b) { return ::write(fd_, &b, 1); }

    int read()
    {
        uint8_t b;
        int n = ::read(fd_, &b, 1);
        return n == 1 ? b : -1;
    }

    size_t readBytes(uint8_t *buf, size_t len)
    {
        size_t total = 0;
        auto start = std::chrono::steady_clock::now();
        while (total < len)
        {
            int n = ::read(fd_, buf + total, len - total);
            if (n > 0)
            {
                total += n;
            }
            else
            {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return total;
    }

    operator bool() const { return fd_ >= 0; }

private:
    int fd_;
    int timeout_ = 2;
};

// Alias expected by STSServoDriver.hpp so it can bind to LinuxSerial
#ifndef HardwareSerial
#define HardwareSerial LinuxSerial
#endif

#endif // LINUX_SERIAL_HPP
