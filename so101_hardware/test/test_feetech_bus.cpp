// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <deque>
#include <map>
#include <vector>

#include "so101_hardware/feetech_bus.hpp"

namespace fp = so101_hardware::feetech;
using Bytes = std::vector<uint8_t>;
using so101_hardware::FeetechBus;

namespace
{
// Records what the bus sends and replays scripted reply bytes, delivering
// them in small chunks the way a USB serial port does.
class FakeTransport : public so101_hardware::Transport
{
public:
  bool write_bytes(const Bytes & bytes) override
  {
    sent.push_back(bytes);
    return true;
  }
  Bytes read_bytes(size_t max_bytes, std::chrono::microseconds) override
  {
    Bytes out;
    while (!pending.empty() && out.size() < std::min<size_t>(max_bytes, 3)) {
      out.push_back(pending.front());
      pending.pop_front();
    }
    return out;
  }
  void flush_input() override {++flushes;}

  void reply(uint8_t id, const Bytes & params, uint8_t error = 0)
  {
    Bytes packet{0xFF, 0xFF, id, static_cast<uint8_t>(params.size() + 2), error};
    packet.insert(packet.end(), params.begin(), params.end());
    packet.push_back(fp::checksum(id, packet[3], error, params));
    pending.insert(pending.end(), packet.begin(), packet.end());
  }

  std::vector<Bytes> sent;
  std::deque<uint8_t> pending;
  int flushes = 0;
};
}  // namespace

TEST(FeetechBus, PingAndRead)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(5));
  t.reply(3, {});
  EXPECT_TRUE(bus.ping(3));
  EXPECT_EQ(t.sent.back(), fp::make_ping(3));

  t.reply(3, {0x39, 0x05});  // Homing_Offset raw 0x0539 = 1337
  uint32_t value = 0;
  ASSERT_TRUE(bus.read(3, fp::reg::kHomingOffset, &value));
  EXPECT_EQ(value, 0x0539u);
}

TEST(FeetechBus, TimeoutWhenServoSilent)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(2));
  EXPECT_FALSE(bus.ping(4));
  EXPECT_NE(bus.last_error().find("timeout"), std::string::npos);
}

TEST(FeetechBus, SkipsGarbageBeforeHeader)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(5));
  t.pending = {0x00, 0x12, 0xFF};
  t.reply(1, {});
  EXPECT_TRUE(bus.ping(1)) << bus.last_error();
}

TEST(FeetechBus, RejectsWrongIdAndBadChecksum)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(5));
  t.reply(2, {});
  EXPECT_FALSE(bus.ping(1));

  t.pending.clear();
  t.reply(1, {});
  t.pending.back() ^= 0x01;
  EXPECT_FALSE(bus.ping(1));
  EXPECT_NE(bus.last_error().find("checksum"), std::string::npos);
}

TEST(FeetechBus, SyncReadCollectsEveryServo)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(5));
  t.reply(1, {0x00, 0x08});
  t.reply(2, {0x10, 0x08}, 0x20);
  std::map<uint8_t, Bytes> out;
  ASSERT_TRUE(bus.sync_read(fp::reg::kPresentPosition, {1, 2}, &out)) << bus.last_error();
  EXPECT_EQ(fp::from_bytes(out[1].data(), 2), 2048u);
  EXPECT_EQ(fp::from_bytes(out[2].data(), 2), 2064u);
  EXPECT_EQ(bus.last_servo_error(), 0x20);
  EXPECT_EQ(t.sent.back(), fp::make_sync_read(fp::reg::kPresentPosition, {1, 2}));
}

TEST(FeetechBus, SyncReadFailsIfOneServoMissing)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(2));
  t.reply(1, {0x00, 0x08});
  std::map<uint8_t, Bytes> out;
  EXPECT_FALSE(bus.sync_read(fp::reg::kPresentPosition, {1, 2}, &out));
}

TEST(FeetechBus, SyncWriteSendsOnePacket)
{
  FakeTransport t;
  FeetechBus bus(t, std::chrono::milliseconds(5));
  ASSERT_TRUE(bus.sync_write(fp::reg::kGoalPosition, {1, 2}, {2048, 1000}));
  ASSERT_EQ(t.sent.size(), 1u);
  EXPECT_EQ(t.sent[0], fp::make_sync_write(fp::reg::kGoalPosition, {1, 2},
    Bytes{0x00, 0x08, 0xE8, 0x03}));
}
