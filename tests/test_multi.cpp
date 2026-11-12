/**
 * @file test_multi.cpp
 * @brief Broadcast ping, ID scan, Sync and Bulk read and write, the fast replies, per-device
 * outcomes.
 */
#include <array>
#include <cstdint>

#include "rig.hpp"

using namespace dynamixel;
using namespace rig;

TEST_CASE("Broadcast ping collects every servo in ID order") {
  V2Rig r;
  std::array<PingInfo, 8> found{};
  const auto n = r.drv.BroadcastPing(std::span<PingInfo>{found});
  REQUIRE(n.ok());
  CHECK_EQ(n.value, 3U);
  CHECK_EQ(found[0].id, 1U);
  CHECK_EQ(found[1].id, 2U);
  CHECK_EQ(found[2].id, 3U);
  CHECK_EQ(found[2].model, 1020U);
  CHECK_EQ(r.drv.GetStats().retries, 0U); // not repeated: a broadcast is not idempotent in time
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Broadcast ping stops when the result span is full and on an empty bus returns zero") {
  V2Rig r;
  std::array<PingInfo, 2> two{};
  const auto n = r.drv.BroadcastPing(std::span<PingInfo>{two});
  REQUIRE(n.ok());
  CHECK_EQ(n.value, 2U);
  mock::Bus bus{};
  mock::Port port{bus};
  Driver drv{port};
  REQUIRE(drv.EnsureInitialized().ok());
  std::array<PingInfo, 2> none{};
  const auto z = drv.BroadcastPing(std::span<PingInfo>{none});
  REQUIRE(z.ok());
  CHECK_EQ(z.value, 0U);
}

TEST_CASE("Broadcast ping: empty span refused, protocol 1.0 unsupported") {
  V2Rig r;
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.BroadcastPing(std::span<PingInfo>{}).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  std::array<PingInfo, 2> found{};
  CHECK_EQ(v.drv.BroadcastPing(std::span<PingInfo>{found}).error, Error::Unsupported);
  CHECK_EQ(v.bus.writes, 0);
}

TEST_CASE("Broadcast ping ignores a duplicate ID and stops at a damaged answer") {
  V2Rig r;
  r.bus.knob.corrupt_crc = 0;
  r.bus.Find(2)->silent = true;
  std::array<PingInfo, 8> found{};
  const auto n = r.drv.BroadcastPing(std::span<PingInfo>{found});
  REQUIRE(n.ok());
  CHECK_EQ(n.value, 2U);
  CHECK_EQ(found[1].id, 3U);
}

TEST_CASE("ScanIds pings a range and lists the servos that answer") {
  V2Rig r;
  std::array<PingInfo, 8> found{};
  const auto n = r.drv.ScanIds(1, 5, std::span<PingInfo>{found});
  REQUIRE(n.ok());
  CHECK_EQ(n.value, 3U);
  CHECK_EQ(found[2].id, 3U);
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.ScanIds(5, 1, std::span<PingInfo>{found}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.ScanIds(1, 254, std::span<PingInfo>{found}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.ScanIds(1, 5, std::span<PingInfo>{}).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
}

TEST_CASE("ScanIds stops with Bus when the port fails") {
  V2Rig r;
  std::array<PingInfo, 8> found{};
  r.bus.fail_write = true;
  CHECK_EQ(r.drv.ScanIds(1, 3, std::span<PingInfo>{found}).error, Error::Bus);
}

TEST_CASE("Sync Write sets a goal per servo; one packet, no replies") {
  V2Rig r;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  const std::array<uint8_t, 12> data{0x00, 0x01, 0, 0, 0x00, 0x02, 0, 0, 0x00, 0x03, 0, 0};
  const auto tx = r.drv.GetStats().transactions;
  REQUIRE(
      r.drv.SyncWrite(116, 4, std::span<const uint8_t>{ids}, std::span<const uint8_t>{data}).ok());
  CHECK_EQ(r.drv.GetStats().transactions, tx + 1U);
  CHECK_EQ(r.bus.Find(1)->Dword(116), 256);
  CHECK_EQ(r.bus.Find(2)->Dword(116), 512);
  CHECK_EQ(r.bus.Find(3)->Dword(116), 768);
  CHECK_EQ(r.bus.bad_packets, 0);
  CHECK_EQ(r.bus.Queued(), 0U);
}

TEST_CASE("Sync Write validation sends nothing") {
  V2Rig r;
  const std::array<uint8_t, 2> ids{1, 2};
  const std::array<uint8_t, 7> bad{};
  const std::array<uint8_t, 8> ok{};
  const std::array<uint8_t, 2> broadcast{1, 254};
  const int w = r.bus.writes;
  CHECK_EQ(
      r.drv.SyncWrite(116, 4, std::span<const uint8_t>{ids}, std::span<const uint8_t>{bad}).error,
      Error::InvalidArgument);
  CHECK_EQ(r.drv.SyncWrite(116, 4, std::span<const uint8_t>{}, std::span<const uint8_t>{}).error,
           Error::InvalidArgument);
  CHECK_EQ(r.drv.SyncWrite(116, 0, std::span<const uint8_t>{ids}, std::span<const uint8_t>{}).error,
           Error::InvalidArgument);
  CHECK_EQ(
      r.drv.SyncWrite(116, 4, std::span<const uint8_t>{broadcast}, std::span<const uint8_t>{ok})
          .error,
      Error::InvalidArgument);
  std::array<uint8_t, 100> many{};
  std::array<uint8_t, 100> bulk{};
  CHECK_EQ(
      r.drv.SyncWrite(116, 4, std::span<const uint8_t>{many}, std::span<const uint8_t>{bulk}).error,
      Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
}

TEST_CASE("Sync Write on protocol 1.0") {
  V1Rig r;
  const std::array<uint8_t, 2> ids{1, 2};
  const std::array<uint8_t, 4> data{0x00, 0x01, 0x00, 0x02};
  REQUIRE(
      r.drv.SyncWrite(30, 2, std::span<const uint8_t>{ids}, std::span<const uint8_t>{data}).ok());
  CHECK_EQ(r.bus.Find(1)->Word(30), 256);
  CHECK_EQ(r.bus.Find(2)->Word(30), 512);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Bulk Write writes different registers per servo") {
  V2Rig r;
  const std::array<uint8_t, 4> goal{0x00, 0x04, 0, 0};
  const std::array<uint8_t, 1> led{1};
  const std::array<BulkWriteItem, 2> items{
      {{1, 116, std::span<const uint8_t>{goal}}, {2, 65, std::span<const uint8_t>{led}}}};
  REQUIRE(r.drv.BulkWrite(std::span<const BulkWriteItem>{items}).ok());
  CHECK_EQ(r.bus.Find(1)->Dword(116), 1024);
  CHECK_EQ(r.bus.Find(2)->mem[65], 1U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Bulk Write validation: empty, duplicate ID, empty data, broadcast ID, protocol 1.0") {
  V2Rig r;
  const std::array<uint8_t, 1> d{1};
  const std::array<BulkWriteItem, 2> dup{
      {{1, 65, std::span<const uint8_t>{d}}, {1, 65, std::span<const uint8_t>{d}}}};
  const std::array<BulkWriteItem, 1> empty_data{{{1, 65, std::span<const uint8_t>{}}}};
  const std::array<BulkWriteItem, 1> bc{{{254, 65, std::span<const uint8_t>{d}}}};
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.BulkWrite(std::span<const BulkWriteItem>{}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.BulkWrite(std::span<const BulkWriteItem>{dup}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.BulkWrite(std::span<const BulkWriteItem>{empty_data}).error,
           Error::InvalidArgument);
  CHECK_EQ(r.drv.BulkWrite(std::span<const BulkWriteItem>{bc}).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  CHECK_EQ(v.drv.BulkWrite(std::span<const BulkWriteItem>{dup}).error, Error::Unsupported);
}

TEST_CASE("Sync Read returns each servo's data in the order of the IDs") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 111);
  r.bus.Find(2)->SetDword(132, 222);
  r.bus.Find(3)->SetDword(132, 333);
  const std::array<uint8_t, 3> ids{3, 1, 2};
  std::array<uint8_t, 12> out{};
  std::array<DeviceReply, 3> rep{};
  REQUIRE(r.drv
              .SyncRead(132, 4, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                        std::span<DeviceReply>{rep})
              .ok());
  CHECK_EQ(out[0], 77U); // 333 = 0x14D
  CHECK_EQ(out[1], 1U);
  CHECK_EQ(out[4], 111U);
  CHECK_EQ(out[8], 222U);
  for (const DeviceReply& d : rep) {
    CHECK(d.result == Error::Ok);
  }
  CHECK_EQ(rep[0].id, 3U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Sync Read: a silent servo is reported for that servo, the others still deliver") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 111);
  r.bus.Find(3)->SetDword(132, 333);
  r.bus.Find(2)->silent = true;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 12> out{};
  std::array<DeviceReply, 3> rep{};
  const auto res = r.drv.SyncRead(132, 4, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                                  std::span<DeviceReply>{rep});
  CHECK_EQ(res.error, Error::Timeout);
  CHECK(rep[0].result == Error::Ok);
  CHECK(rep[1].result == Error::Timeout);
  CHECK(rep[2].result == Error::Ok);
  CHECK_EQ(out[0], 111U);
  CHECK_EQ(out[8], 77U);
}

TEST_CASE("Sync Read: a servo's error is that servo's result and the alert flag is per device") {
  V2Rig r;
  r.bus.Find(2)->alert = true;
  r.bus.Find(3)->force_error = 7;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto res = r.drv.SyncRead(146, 1, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                                  std::span<DeviceReply>{rep});
  CHECK_EQ(res.error, Error::DeviceError);
  CHECK(rep[0].result == Error::Ok);
  CHECK(!rep[0].alert);
  CHECK(rep[1].result == Error::Ok);
  CHECK(rep[1].alert);
  CHECK(rep[2].result == Error::DeviceError);
  CHECK_EQ(rep[2].error_byte, 7U);
  CHECK_EQ(out[0], 25U);
}

TEST_CASE("Sync Read validation sends nothing") {
  V2Rig r;
  const std::array<uint8_t, 2> ids{1, 2};
  std::array<uint8_t, 8> out{};
  std::array<DeviceReply, 2> rep{};
  std::array<DeviceReply, 1> rep1{};
  const int w = r.bus.writes;
  CHECK_EQ(r.drv
               .SyncRead(132, 4, std::span<const uint8_t>{ids}, std::span<uint8_t>{out}.first(7),
                         std::span<DeviceReply>{rep})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.drv
               .SyncRead(132, 4, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                         std::span<DeviceReply>{rep1})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.drv
               .SyncRead(132, 0, std::span<const uint8_t>{ids}, std::span<uint8_t>{},
                         std::span<DeviceReply>{rep})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.drv
               .SyncRead(132, 4, std::span<const uint8_t>{}, std::span<uint8_t>{},
                         std::span<DeviceReply>{})
               .error,
           Error::InvalidArgument);
  const std::array<uint8_t, 2> bc{1, 254};
  CHECK_EQ(r.drv
               .SyncRead(132, 4, std::span<const uint8_t>{bc}, std::span<uint8_t>{out},
                         std::span<DeviceReply>{rep})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  CHECK_EQ(v.drv
               .SyncRead(36, 2, std::span<const uint8_t>{ids}, std::span<uint8_t>{out}.first(4),
                         std::span<DeviceReply>{rep})
               .error,
           Error::Unsupported);
  CHECK_EQ(v.bus.writes, 0);
}

TEST_CASE("Sync Read at status return level 0 is refused without sending") {
  Config c{};
  c.status_return = StatusReturn::PingOnly;
  V2Rig r{c};
  const std::array<uint8_t, 1> ids{1};
  std::array<uint8_t, 1> out{};
  std::array<DeviceReply, 1> rep{};
  CHECK_EQ(r.drv
               .SyncRead(146, 1, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                         std::span<DeviceReply>{rep})
               .error,
           Error::Unsupported);
  CHECK_EQ(r.bus.writes, 0);
}

TEST_CASE("Bulk Read returns a different register block per servo") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 1000);
  r.bus.Find(2)->mem[146] = 41;
  const std::array<BulkReadItem, 2> items{{{1, 132, 4}, {2, 146, 1}}};
  std::array<uint8_t, 5> out{};
  std::array<DeviceReply, 2> rep{};
  REQUIRE(r.drv
              .BulkRead(std::span<const BulkReadItem>{items}, std::span<uint8_t>{out},
                        std::span<DeviceReply>{rep})
              .ok());
  CHECK_EQ(out[0], 0xE8U);
  CHECK_EQ(out[1], 0x03U);
  CHECK_EQ(out[4], 41U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Bulk Read on protocol 1.0 uses the 1.0 layout") {
  V1Rig r;
  r.bus.Find(1)->SetWord(36, 700);
  r.bus.Find(2)->mem[43] = 30;
  const std::array<BulkReadItem, 2> items{{{1, 36, 2}, {2, 43, 1}}};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 2> rep{};
  REQUIRE(r.drv
              .BulkRead(std::span<const BulkReadItem>{items}, std::span<uint8_t>{out},
                        std::span<DeviceReply>{rep})
              .ok());
  CHECK_EQ(out[0], 0xBCU);
  CHECK_EQ(out[1], 0x02U);
  CHECK_EQ(out[2], 30U);
}

TEST_CASE("Bulk Read validation: duplicates, zero length, protocol 1.0 address") {
  V2Rig r;
  const std::array<BulkReadItem, 2> dup{{{1, 132, 4}, {1, 146, 1}}};
  const std::array<BulkReadItem, 1> zero{{{1, 132, 0}}};
  std::array<uint8_t, 8> out{};
  std::array<DeviceReply, 2> rep{};
  CHECK_EQ(r.drv
               .BulkRead(std::span<const BulkReadItem>{dup}, std::span<uint8_t>{out}.first(5),
                         std::span<DeviceReply>{rep})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.drv
               .BulkRead(std::span<const BulkReadItem>{zero}, std::span<uint8_t>{},
                         std::span<DeviceReply>{rep}.first(1))
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, 0);
  V1Rig v;
  const std::array<BulkReadItem, 1> far{{{1, 300, 2}}};
  CHECK_EQ(v.drv
               .BulkRead(std::span<const BulkReadItem>{far}, std::span<uint8_t>{out}.first(2),
                         std::span<DeviceReply>{rep}.first(1))
               .error,
           Error::InvalidArgument);
  CHECK_EQ(v.bus.writes, 0);
}

TEST_CASE("Fast Sync Read: one packet carries every servo, each part checked with its own CRC") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 111);
  r.bus.Find(2)->SetDword(132, 222);
  r.bus.Find(3)->SetDword(132, 333);
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 12> out{};
  std::array<DeviceReply, 3> rep{};
  REQUIRE(r.drv
              .FastSyncRead(132, 4, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                            std::span<DeviceReply>{rep})
              .ok());
  CHECK_EQ(out[0], 111U);
  CHECK_EQ(out[4], 222U);
  CHECK_EQ(out[8], 77U);
  for (const DeviceReply& d : rep) {
    CHECK(d.result == Error::Ok);
  }
  CHECK_EQ(r.bus.bad_packets, 0);
  CHECK_EQ(r.drv.GetStats().transactions, 1U);
}

TEST_CASE(
    "Fast Sync Read: a servo that does not answer ends the packet; the earlier parts stay valid") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 111);
  r.bus.Find(2)->fast_read_ok = false;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 12> out{};
  std::array<DeviceReply, 3> rep{};
  const auto res = r.drv.FastSyncRead(132, 4, std::span<const uint8_t>{ids},
                                      std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK(!res.ok());
  CHECK(rep[0].result == Error::Ok);
  CHECK_EQ(out[0], 111U);
  CHECK(rep[1].result != Error::Ok);
  CHECK(rep[2].result == Error::Ok);
}

TEST_CASE("Fast Sync Read: a damaged last part fails the CRC for the parts it covers") {
  V2Rig r;
  const std::array<uint8_t, 2> ids{1, 2};
  std::array<uint8_t, 2> out{};
  std::array<DeviceReply, 2> rep{};
  r.bus.knob.corrupt_crc = 1;
  const auto res = r.drv.FastSyncRead(146, 1, std::span<const uint8_t>{ids},
                                      std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK(!res.ok());
  CHECK(rep[0].result != Error::Ok);
  CHECK(rep[1].result != Error::Ok);
}

TEST_CASE("Fast Sync Read: no answer is a timeout for every device; a truncated packet keeps the "
          "whole parts") {
  V2Rig r;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  r.bus.knob.drop_replies = 1;
  CHECK_EQ(r.drv
               .FastSyncRead(146, 1, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                             std::span<DeviceReply>{rep})
               .error,
           Error::Timeout);
  for (const DeviceReply& d : rep) {
    CHECK(d.result == Error::Timeout);
  }
  // 8 header bytes, part one (4 + 1), then a cut inside part two: part one is intact and trusted.
  r.bus.knob.truncate_to = 8 + 5 + 3;
  const auto res = r.drv.FastSyncRead(146, 1, std::span<const uint8_t>{ids},
                                      std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK(!res.ok());
  CHECK(rep[0].result == Error::Ok);
  CHECK_EQ(out[0], 25U);
  CHECK(rep[1].result == Error::Timeout);
  CHECK(rep[2].result == Error::Timeout);
}

TEST_CASE("Fast Bulk Read: different blocks in one packet") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 1000);
  r.bus.Find(3)->mem[146] = 44;
  const std::array<BulkReadItem, 2> items{{{1, 132, 4}, {3, 146, 1}}};
  std::array<uint8_t, 5> out{};
  std::array<DeviceReply, 2> rep{};
  REQUIRE(r.drv
              .FastBulkRead(std::span<const BulkReadItem>{items}, std::span<uint8_t>{out},
                            std::span<DeviceReply>{rep})
              .ok());
  CHECK_EQ(out[0], 0xE8U);
  CHECK_EQ(out[4], 44U);
}

TEST_CASE("Fast reads are refused on protocol 1.0 and when the packet cannot fit the buffer") {
  V1Rig v;
  const std::array<uint8_t, 1> ids{1};
  std::array<uint8_t, 2> out{};
  std::array<DeviceReply, 1> rep{};
  CHECK_EQ(v.drv
               .FastSyncRead(36, 2, std::span<const uint8_t>{ids}, std::span<uint8_t>{out},
                             std::span<DeviceReply>{rep})
               .error,
           Error::Unsupported);
  V2Rig r;
  const std::array<uint8_t, 3> three{1, 2, 3};
  std::array<uint8_t, 600> big{};
  std::array<DeviceReply, 3> rep3{};
  CHECK_EQ(r.drv
               .FastSyncRead(0, 200, std::span<const uint8_t>{three}, std::span<uint8_t>{big},
                             std::span<DeviceReply>{rep3})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, 0);
}

TEST_CASE(
    "Alert policy Fail turns a hardware alert in a fast part into DeviceFault for that device") {
  Config c{};
  c.alert_policy = AlertPolicy::Fail;
  V2Rig r{c};
  r.bus.Find(2)->alert = true;
  const std::array<uint8_t, 2> ids{1, 2};
  std::array<uint8_t, 2> out{};
  std::array<DeviceReply, 2> rep{};
  const auto res = r.drv.FastSyncRead(146, 1, std::span<const uint8_t>{ids},
                                      std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(res.error, Error::DeviceFault);
  CHECK(rep[0].result == Error::Ok);
  CHECK(rep[1].result == Error::DeviceFault);
  CHECK(rep[1].alert);
}
