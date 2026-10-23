// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

// Feetech STS/SMS servo protocol (protocol 0), as used by the STS3215 servos
// of the SO-101. Packets are half-duplex on a single bus:
//
//   instruction: 0xFF 0xFF ID LEN INSTR PARAM... CHECKSUM
//   status:      0xFF 0xFF ID LEN ERROR PARAM... CHECKSUM
//
// LEN = number of params + 2, CHECKSUM = ~(ID + LEN + INSTR/ERROR + params).
// Multi-byte values on STS servos are little-endian; some registers use
// sign-magnitude encoding with a model-specific sign bit.

#ifndef SO101_HARDWARE__FEETECH_PROTOCOL_HPP_
#define SO101_HARDWARE__FEETECH_PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace so101_hardware
{
namespace feetech
{

constexpr uint8_t kBroadcastId = 0xFE;

enum class Instruction : uint8_t
{
  kPing = 0x01,
  kRead = 0x02,
  kWrite = 0x03,
  kSyncRead = 0x82,
  kSyncWrite = 0x83,
};

// STS3215 control table (address, size in bytes).
struct Register
{
  uint8_t address;
  uint8_t size;
};

namespace reg
{
// EEPROM
constexpr Register kModelNumber{3, 2};
constexpr Register kReturnDelayTime{7, 1};
constexpr Register kMinPositionLimit{9, 2};
constexpr Register kMaxPositionLimit{11, 2};
constexpr Register kMaxTorqueLimit{16, 2};
constexpr Register kPhase{18, 1};
constexpr Register kPCoefficient{21, 1};
constexpr Register kDCoefficient{22, 1};
constexpr Register kICoefficient{23, 1};
constexpr Register kProtectionCurrent{28, 2};
constexpr Register kHomingOffset{31, 2};
constexpr Register kOperatingMode{33, 1};
constexpr Register kOverloadTorque{36, 1};
constexpr Register kMaximumAcceleration{85, 1};
// SRAM
constexpr Register kTorqueEnable{40, 1};
constexpr Register kAcceleration{41, 1};
constexpr Register kGoalPosition{42, 2};
constexpr Register kLock{55, 1};
constexpr Register kPresentPosition{56, 2};
// Present_Position .. Present_Temperature, read in one block every cycle.
constexpr Register kFeedbackBlock{56, 8};
}  // namespace reg

// Sign bits of sign-magnitude encoded registers on STS servos.
constexpr int kHomingOffsetSignBit = 11;
constexpr int kPositionSignBit = 15;
constexpr int kVelocitySignBit = 15;
constexpr int kLoadSignBit = 10;

constexpr uint16_t kSts3215ModelNumber = 777;
constexpr int kResolution = 4096;  // encoder ticks per revolution

uint8_t checksum(
  uint8_t id, uint8_t length, uint8_t instruction_or_error,
  const std::vector<uint8_t> & params);

std::vector<uint8_t> make_packet(
  uint8_t id, Instruction instruction,
  const std::vector<uint8_t> & params = {});

std::vector<uint8_t> make_ping(uint8_t id);
std::vector<uint8_t> make_read(uint8_t id, Register reg);
std::vector<uint8_t> make_write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data);
std::vector<uint8_t> make_sync_read(Register reg, const std::vector<uint8_t> & ids);
// `data` holds reg.size bytes per id, in the same order as `ids`.
std::vector<uint8_t> make_sync_write(
  Register reg, const std::vector<uint8_t> & ids,
  const std::vector<uint8_t> & data);

struct StatusPacket
{
  uint8_t id = 0;
  uint8_t error = 0;
  std::vector<uint8_t> params;
};

// Parse one complete status packet (header included). Returns nullopt and
// sets `why` on a malformed packet or checksum mismatch.
std::optional<StatusPacket> parse_status(const std::vector<uint8_t> & bytes, std::string * why);

// Human-readable description of the servo error byte (empty if 0).
std::string describe_error(uint8_t error);

std::vector<uint8_t> to_bytes(uint32_t value, uint8_t size);  // little-endian
uint32_t from_bytes(const uint8_t * data, uint8_t size);  // little-endian

uint32_t encode_sign_magnitude(int32_t value, int sign_bit);
int32_t decode_sign_magnitude(uint32_t raw, int sign_bit);

}  // namespace feetech
}  // namespace so101_hardware

#endif  // SO101_HARDWARE__FEETECH_PROTOCOL_HPP_
