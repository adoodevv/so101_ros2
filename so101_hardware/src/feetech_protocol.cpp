// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include "so101_hardware/feetech_protocol.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace so101_hardware
{
namespace feetech
{

uint8_t checksum(
  uint8_t id, uint8_t length, uint8_t instruction_or_error,
  const std::vector<uint8_t> & params)
{
  uint32_t sum = id + length + instruction_or_error;
  for (uint8_t p : params) {
    sum += p;
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

std::vector<uint8_t> make_packet(
  uint8_t id, Instruction instruction,
  const std::vector<uint8_t> & params)
{
  if (params.size() > 250) {
    throw std::invalid_argument("feetech packet too long");
  }
  const auto length = static_cast<uint8_t>(params.size() + 2);
  const auto instr = static_cast<uint8_t>(instruction);
  std::vector<uint8_t> packet{0xFF, 0xFF, id, length, instr};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(checksum(id, length, instr, params));
  return packet;
}

std::vector<uint8_t> make_ping(uint8_t id)
{
  return make_packet(id, Instruction::kPing);
}

std::vector<uint8_t> make_read(uint8_t id, Register reg)
{
  return make_packet(id, Instruction::kRead, {reg.address, reg.size});
}

std::vector<uint8_t> make_write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params{address};
  params.insert(params.end(), data.begin(), data.end());
  return make_packet(id, Instruction::kWrite, params);
}

std::vector<uint8_t> make_sync_read(Register reg, const std::vector<uint8_t> & ids)
{
  std::vector<uint8_t> params{reg.address, reg.size};
  params.insert(params.end(), ids.begin(), ids.end());
  return make_packet(kBroadcastId, Instruction::kSyncRead, params);
}

std::vector<uint8_t> make_sync_write(
  Register reg, const std::vector<uint8_t> & ids,
  const std::vector<uint8_t> & data)
{
  if (data.size() != ids.size() * reg.size) {
    throw std::invalid_argument("sync write data size does not match ids * register size");
  }
  std::vector<uint8_t> params{reg.address, reg.size};
  for (size_t i = 0; i < ids.size(); ++i) {
    params.push_back(ids[i]);
    params.insert(
      params.end(), data.begin() + static_cast<long>(i * reg.size),
      data.begin() + static_cast<long>((i + 1) * reg.size));
  }
  return make_packet(kBroadcastId, Instruction::kSyncWrite, params);
}

std::optional<StatusPacket> parse_status(const std::vector<uint8_t> & bytes, std::string * why)
{
  auto fail = [why](const std::string & reason) -> std::optional<StatusPacket> {
      if (why) {
        *why = reason;
      }
      return std::nullopt;
    };
  if (bytes.size() < 6) {
    return fail("status packet shorter than 6 bytes");
  }
  if (bytes[0] != 0xFF || bytes[1] != 0xFF) {
    return fail("missing 0xFF 0xFF header");
  }
  const uint8_t length = bytes[3];
  if (length < 2 || bytes.size() != static_cast<size_t>(length) + 4) {
    return fail("length field does not match packet size");
  }
  StatusPacket status;
  status.id = bytes[2];
  status.error = bytes[4];
  status.params.assign(bytes.begin() + 5, bytes.end() - 1);
  if (checksum(status.id, length, status.error, status.params) != bytes.back()) {
    return fail("checksum mismatch");
  }
  return status;
}

std::string describe_error(uint8_t error)
{
  static const char * const kBits[] = {
    "input voltage", "angle sensor", "overheat", "overcurrent",
    "bit4", "overload", "bit6", "bit7"};
  std::string out;
  for (int bit = 0; bit < 8; ++bit) {
    if (error & (1u << bit)) {
      if (!out.empty()) {
        out += ", ";
      }
      out += kBits[bit];
    }
  }
  return out;
}

std::vector<uint8_t> to_bytes(uint32_t value, uint8_t size)
{
  std::vector<uint8_t> bytes(size);
  for (uint8_t i = 0; i < size; ++i) {
    bytes[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
  }
  return bytes;
}

uint32_t from_bytes(const uint8_t * data, uint8_t size)
{
  uint32_t value = 0;
  for (uint8_t i = 0; i < size; ++i) {
    value |= static_cast<uint32_t>(data[i]) << (8 * i);
  }
  return value;
}

uint32_t encode_sign_magnitude(int32_t value, int sign_bit)
{
  const uint32_t magnitude = static_cast<uint32_t>(std::abs(value));
  if (magnitude >= (1u << sign_bit)) {
    throw std::out_of_range("value does not fit in sign-magnitude field");
  }
  return value < 0 ? (magnitude | (1u << sign_bit)) : magnitude;
}

int32_t decode_sign_magnitude(uint32_t raw, int sign_bit)
{
  const uint32_t magnitude = raw & ((1u << sign_bit) - 1);
  return (raw & (1u << sign_bit)) ? -static_cast<int32_t>(magnitude) :
         static_cast<int32_t>(magnitude);
}

}  // namespace feetech
}  // namespace so101_hardware
