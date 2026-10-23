// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include "so101_hardware/feetech_bus.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace so101_hardware
{

using feetech::Register;
using feetech::StatusPacket;

FeetechBus::FeetechBus(Transport & transport, std::chrono::microseconds timeout)
: transport_(transport), timeout_(timeout)
{
}

bool FeetechBus::fail(const std::string & reason)
{
  last_error_ = reason;
  return false;
}

bool FeetechBus::receive_status(
  uint8_t expected_id, size_t expected_params,
  std::chrono::steady_clock::time_point deadline, StatusPacket * status)
{
  // Status packets are 6 + params bytes. Scan for the 0xFF 0xFF header so that
  // stray bytes on the line are skipped rather than misparsed.
  const size_t total = 6 + expected_params;
  std::vector<uint8_t> buffer;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return fail(
        "timeout waiting for servo " + std::to_string(expected_id) + " (" +
        std::to_string(buffer.size()) + "/" + std::to_string(total) + " bytes)");
    }
    const auto left = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    auto chunk = transport_.read_bytes(total - buffer.size(), left);
    buffer.insert(buffer.end(), chunk.begin(), chunk.end());

    // Drop leading bytes until the buffer starts with a header.
    while (buffer.size() >= 2 && !(buffer[0] == 0xFF && buffer[1] == 0xFF)) {
      buffer.erase(buffer.begin());
    }
    // 0xFF 0xFF 0xFF ... : extra 0xFF before the id.
    while (buffer.size() >= 3 && buffer[2] == 0xFF) {
      buffer.erase(buffer.begin());
    }
    if (buffer.size() < total) {
      continue;
    }

    std::string why;
    auto parsed = feetech::parse_status(
      std::vector<uint8_t>(buffer.begin(), buffer.begin() + static_cast<long>(total)), &why);
    if (!parsed) {
      return fail("bad status packet from servo " + std::to_string(expected_id) + ": " + why);
    }
    if (parsed->id != expected_id) {
      return fail(
        "expected a reply from servo " + std::to_string(expected_id) + ", got servo " +
        std::to_string(parsed->id));
    }
    last_servo_error_ = parsed->error;
    *status = std::move(*parsed);
    return true;
  }
}

bool FeetechBus::transact(
  const std::vector<uint8_t> & packet, uint8_t expected_id,
  size_t expected_params, StatusPacket * status)
{
  transport_.flush_input();
  if (!transport_.write_bytes(packet)) {
    return fail("serial write failed");
  }
  return receive_status(
    expected_id, expected_params, std::chrono::steady_clock::now() + timeout_, status);
}

bool FeetechBus::ping(uint8_t id)
{
  StatusPacket status;
  return transact(feetech::make_ping(id), id, 0, &status);
}

bool FeetechBus::read(uint8_t id, Register reg, uint32_t * value)
{
  StatusPacket status;
  if (!transact(feetech::make_read(id, reg), id, reg.size, &status)) {
    return false;
  }
  *value = feetech::from_bytes(status.params.data(), reg.size);
  return true;
}

bool FeetechBus::write(uint8_t id, Register reg, uint32_t value)
{
  StatusPacket status;
  return transact(
    feetech::make_write(id, reg.address, feetech::to_bytes(value, reg.size)), id, 0, &status);
}

bool FeetechBus::sync_read(
  Register reg, const std::vector<uint8_t> & ids,
  std::map<uint8_t, std::vector<uint8_t>> * out)
{
  transport_.flush_input();
  if (!transport_.write_bytes(feetech::make_sync_read(reg, ids))) {
    return fail("serial write failed");
  }
  // Servos answer one after another, in the order of `ids`. The timeout
  // scales with the number of replies.
  const auto deadline = std::chrono::steady_clock::now() +
    timeout_ * static_cast<int64_t>(std::max<size_t>(1, ids.size()));
  out->clear();
  for (uint8_t id : ids) {
    StatusPacket status;
    if (!receive_status(id, reg.size, deadline, &status)) {
      return false;
    }
    (*out)[id] = std::move(status.params);
  }
  return true;
}

bool FeetechBus::sync_write(
  Register reg, const std::vector<uint8_t> & ids,
  const std::vector<uint32_t> & values)
{
  if (ids.size() != values.size()) {
    return fail("sync write: ids and values differ in length");
  }
  std::vector<uint8_t> data;
  data.reserve(ids.size() * reg.size);
  for (uint32_t value : values) {
    auto bytes = feetech::to_bytes(value, reg.size);
    data.insert(data.end(), bytes.begin(), bytes.end());
  }
  if (!transport_.write_bytes(feetech::make_sync_write(reg, ids, data))) {
    return fail("serial write failed");
  }
  return true;
}

}  // namespace so101_hardware
