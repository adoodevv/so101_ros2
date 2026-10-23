// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include "so101_hardware/serial_port.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

namespace so101_hardware
{

namespace
{
speed_t to_speed(int baud_rate)
{
  switch (baud_rate) {
    case 1000000: return B1000000;
    case 500000: return B500000;
    case 115200: return B115200;
    case 57600: return B57600;
    default: return 0;
  }
}
}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baud_rate, std::string * error)
{
  close();
  const speed_t speed = to_speed(baud_rate);
  if (speed == 0) {
    *error = "unsupported baud rate " + std::to_string(baud_rate);
    return false;
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    *error = "cannot open " + device + ": " + std::strerror(errno) +
      (errno == EACCES ? " (is your user in the 'dialout' group?)" : "");
    return false;
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    *error = "tcgetattr failed on " + device + ": " + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    *error = "tcsetattr failed on " + device + ": " + std::strerror(errno);
    close();
    return false;
  }
  // Ask the USB-serial driver for low latency; not every driver supports it.
  serial_struct serial{};
  if (ioctl(fd_, TIOCGSERIAL, &serial) == 0) {
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &serial);
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write_bytes(const std::vector<uint8_t> & bytes)
{
  if (fd_ < 0) {
    return false;
  }
  size_t sent = 0;
  while (sent < bytes.size()) {
    const ssize_t n = ::write(fd_, bytes.data() + sent, bytes.size() - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        pollfd pfd{fd_, POLLOUT, 0};
        poll(&pfd, 1, 10);
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return tcdrain(fd_) == 0;
}

std::vector<uint8_t> SerialPort::read_bytes(size_t max_bytes, std::chrono::microseconds timeout)
{
  std::vector<uint8_t> out;
  if (fd_ < 0 || max_bytes == 0) {
    return out;
  }
  pollfd pfd{fd_, POLLIN, 0};
  const int timeout_ms = static_cast<int>((timeout.count() + 999) / 1000);
  if (poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN)) {
    return out;
  }
  out.resize(max_bytes);
  const ssize_t n = ::read(fd_, out.data(), max_bytes);
  out.resize(n > 0 ? static_cast<size_t>(n) : 0);
  return out;
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

}  // namespace so101_hardware
