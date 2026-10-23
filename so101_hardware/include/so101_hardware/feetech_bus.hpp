// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SO101_HARDWARE__FEETECH_BUS_HPP_
#define SO101_HARDWARE__FEETECH_BUS_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "so101_hardware/feetech_protocol.hpp"

namespace so101_hardware
{

// Byte transport the bus talks through: a serial port on the robot, a fake
// in unit tests.
class Transport
{
public:
  virtual ~Transport() = default;
  virtual bool write_bytes(const std::vector<uint8_t> & bytes) = 0;
  // Read up to `max_bytes`, waiting at most `timeout`. Returns what arrived
  // (possibly nothing).
  virtual std::vector<uint8_t> read_bytes(size_t max_bytes, std::chrono::microseconds timeout) = 0;
  virtual void flush_input() = 0;
};

// Request/response transactions on a Feetech bus. Not thread safe; the
// ros2_control read()/write() cycle uses it from a single thread.
class FeetechBus
{
public:
  FeetechBus(Transport & transport, std::chrono::microseconds timeout);

  bool ping(uint8_t id);
  bool read(uint8_t id, feetech::Register reg, uint32_t * value);
  bool write(uint8_t id, feetech::Register reg, uint32_t value);
  // Raw bytes of `reg` for every id, keyed by id. Fails if any servo does not
  // answer correctly.
  bool sync_read(
    feetech::Register reg, const std::vector<uint8_t> & ids,
    std::map<uint8_t, std::vector<uint8_t>> * out);
  // One value per id; no response is expected.
  bool sync_write(
    feetech::Register reg, const std::vector<uint8_t> & ids,
    const std::vector<uint32_t> & values);

  // Why the last failed call failed.
  const std::string & last_error() const {return last_error_;}
  // Error byte reported by the last servo that answered (0 = none).
  uint8_t last_servo_error() const {return last_servo_error_;}

private:
  bool transact(
    const std::vector<uint8_t> & packet, uint8_t expected_id,
    size_t expected_params, feetech::StatusPacket * status);
  bool receive_status(
    uint8_t expected_id, size_t expected_params,
    std::chrono::steady_clock::time_point deadline, feetech::StatusPacket * status);
  bool fail(const std::string & reason);

  Transport & transport_;
  std::chrono::microseconds timeout_;
  std::string last_error_;
  uint8_t last_servo_error_ = 0;
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__FEETECH_BUS_HPP_
