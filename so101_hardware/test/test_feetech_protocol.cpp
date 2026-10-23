// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "so101_hardware/feetech_protocol.hpp"

namespace fp = so101_hardware::feetech;
using Bytes = std::vector<uint8_t>;

// Expected bytes worked out by hand from the Feetech protocol:
// checksum = ~(id + len + instr + params) & 0xFF.
TEST(FeetechProtocol, Ping)
{
  EXPECT_EQ(fp::make_ping(1), (Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(FeetechProtocol, ReadPresentPosition)
{
  // id 1, READ, addr 56 (0x38), 2 bytes: sum = 1+4+2+0x38+2 = 0x41 -> 0xBE
  EXPECT_EQ(fp::make_read(1, fp::reg::kPresentPosition),
    (Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
}

TEST(FeetechProtocol, WriteGoalPositionLittleEndian)
{
  // id 2, WRITE addr 42 value 2048 = 0x0800 -> bytes 00 08
  const auto packet = fp::make_write(2, fp::reg::kGoalPosition.address, fp::to_bytes(2048, 2));
  EXPECT_EQ(packet, (Bytes{0xFF, 0xFF, 0x02, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC3}));
}

TEST(FeetechProtocol, SyncWriteLayout)
{
  const auto packet = fp::make_sync_write(
    fp::reg::kTorqueEnable, {1, 2}, Bytes{1, 0});
  // FF FF FE LEN 83 addr size id1 d1 id2 d2 chk; LEN = 6 params + 2
  ASSERT_EQ(packet.size(), 12u);
  EXPECT_EQ(Bytes(packet.begin(), packet.begin() + 11),
    (Bytes{0xFF, 0xFF, 0xFE, 0x08, 0x83, 40, 1, 1, 1, 2, 0}));
  EXPECT_EQ(packet.back(), fp::checksum(0xFE, 0x08, 0x83, {40, 1, 1, 1, 2, 0}));
  EXPECT_THROW(fp::make_sync_write(fp::reg::kGoalPosition, {1, 2}, Bytes{1, 2, 3}),
    std::invalid_argument);
}

TEST(FeetechProtocol, SyncReadLayout)
{
  const auto packet = fp::make_sync_read(fp::reg::kFeedbackBlock, {1, 2, 3});
  EXPECT_EQ(Bytes(packet.begin(), packet.end() - 1),
    (Bytes{0xFF, 0xFF, 0xFE, 0x07, 0x82, 56, 8, 1, 2, 3}));
}

TEST(FeetechProtocol, ParseStatus)
{
  // Servo 1 answers a 2-byte read with value 2048, no error.
  Bytes reply{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08};
  reply.push_back(fp::checksum(1, 4, 0, {0x00, 0x08}));
  std::string why;
  auto status = fp::parse_status(reply, &why);
  ASSERT_TRUE(status) << why;
  EXPECT_EQ(status->id, 1);
  EXPECT_EQ(status->error, 0);
  EXPECT_EQ(fp::from_bytes(status->params.data(), 2), 2048u);

  reply.back() ^= 0xFF;
  EXPECT_FALSE(fp::parse_status(reply, &why));
  EXPECT_EQ(why, "checksum mismatch");
  EXPECT_FALSE(fp::parse_status(Bytes{0xFF, 0xFF, 0x01}, &why));
}

TEST(FeetechProtocol, SignMagnitude)
{
  EXPECT_EQ(fp::encode_sign_magnitude(1355, fp::kHomingOffsetSignBit), 1355u);
  EXPECT_EQ(fp::encode_sign_magnitude(-1767, fp::kHomingOffsetSignBit), 1767u | (1u << 11));
  EXPECT_EQ(fp::decode_sign_magnitude(1767u | (1u << 11), fp::kHomingOffsetSignBit), -1767);
  EXPECT_EQ(fp::decode_sign_magnitude(0x8000 | 50, fp::kVelocitySignBit), -50);
  EXPECT_EQ(fp::decode_sign_magnitude(1024 | 300, fp::kLoadSignBit), -300);
  EXPECT_THROW(fp::encode_sign_magnitude(2048, fp::kHomingOffsetSignBit), std::out_of_range);
}

TEST(FeetechProtocol, DescribeError)
{
  EXPECT_EQ(fp::describe_error(0), "");
  EXPECT_EQ(fp::describe_error(0x24), "overheat, overload");
}
