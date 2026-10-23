// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SO101_HARDWARE__SERIAL_PORT_HPP_
#define SO101_HARDWARE__SERIAL_PORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "so101_hardware/feetech_bus.hpp"

namespace so101_hardware
{

// Raw 8N1 POSIX serial port (e.g. /dev/ttyACM0 for the SO-101 servo board).
class SerialPort : public Transport
{
public:
  SerialPort() = default;
  ~SerialPort() override;
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Returns false and fills `error` on failure.
  bool open(const std::string & device, int baud_rate, std::string * error);
  void close();
  bool is_open() const {return fd_ >= 0;}

  bool write_bytes(const std::vector<uint8_t> & bytes) override;
  std::vector<uint8_t> read_bytes(size_t max_bytes, std::chrono::microseconds timeout) override;
  void flush_input() override;

private:
  int fd_ = -1;
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__SERIAL_PORT_HPP_
