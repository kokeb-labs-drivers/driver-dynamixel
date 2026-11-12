/**
 * @file mock_servo_bus.hpp
 * @brief A model of a DYNAMIXEL bus that shares no code with the driver: its own CRC, its own
 * packet parser and its own hand-typed control tables, so a mistake in the driver cannot cancel
 * against the same mistake here.
 *
 * `Bus` holds up to eight servos (protocol 2.0 XM430-style tables, or protocol 1.0 AX-12A-style
 * tables), the bytes queued towards the host, fault-injection knobs and counters. `Port` is the
 * UART port the driver talks to. The model follows the ROBOTIS e-Manual: reply rules by Status
 * Return Level, error byte semantics, byte stuffing, Reg Write/Action, sync and bulk instructions,
 * fast replies.
 */
#ifndef DYNAMIXEL_MOCK_SERVO_BUS_HPP
#define DYNAMIXEL_MOCK_SERVO_BUS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include "dynamixel_types.hpp"
#include "dynamixel_uart_interface.hpp"

namespace mock {

/// CRC-16 with polynomial 0x8005, bit by bit (the e-Manual's definition), as an independent check.
inline uint16_t Crc(const uint8_t* data, std::size_t len, uint16_t crc = 0) {
  for (std::size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint32_t>(data[i]) << 8U));
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((static_cast<uint32_t>(crc) << 1U) ^ 0x8005U);
      } else {
        crc = static_cast<uint16_t>(static_cast<uint32_t>(crc) << 1U);
      }
    }
  }
  return crc;
}

inline uint8_t U8(std::size_t v) {
  return static_cast<uint8_t>(v & 0xFFU);
}

/// Protocol 1.0 checksum: complement of the sum of everything after the header.
inline uint8_t Sum1(const uint8_t* data, std::size_t len) {
  uint32_t sum = 0;
  for (std::size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(~sum & 0xFFU);
}

/// The kind of servo: selects the control table layout.
enum class Kind : uint8_t { Xm430, Ax12 };

/// Control table addresses that the model gives meaning to (e-Manual, typed by hand).
struct Layout {
  uint16_t eeprom_end;    ///< Addresses below this are EEPROM.
  uint16_t id;            ///< ID register.
  uint16_t baud;          ///< Baud Rate register.
  uint16_t status_level;  ///< Status Return Level register.
  uint16_t torque;        ///< Torque Enable register.
  uint16_t goal_position; ///< Goal Position.
  uint16_t present_position;
  uint16_t table_size; ///< Bytes of control table.
};
inline constexpr Layout kXm430{64, 7, 8, 68, 64, 116, 132, 1024};
inline constexpr Layout kAx12{24, 3, 4, 16, 24, 30, 36, 74};

/// One simulated servo.
struct Servo {
  bool present{false};
  Kind kind{Kind::Xm430};
  uint8_t id{1};
  std::array<uint8_t, 1024> mem{};
  bool has_pending{false};
  uint16_t pending_addr{0};
  std::array<uint8_t, 300> pending{};
  uint16_t pending_len{0};
  bool silent{false}; ///< Never answers.
  bool alert{false};  ///< Reports the hardware alert (bit 7, or the V1 overheating flag).
  uint8_t force_error{
      0}; ///< Rejects the next instruction with this error number (V2) or flags (V1).
  bool fast_read_ok{true}; ///< Answers fast sync/bulk read.
  uint32_t reboots{0};
  uint32_t factory_resets{0};
  uint8_t last_reset_option{0};
  uint32_t clears{0};
  uint32_t backups{0};
  uint32_t restores{0};
  uint32_t instructions{0}; ///< Instruction packets addressed to this servo that it accepted.

  [[nodiscard]] const Layout& L() const {
    return kind == Kind::Xm430 ? kXm430 : kAx12;
  }
  [[nodiscard]] uint16_t Model() const {
    return kind == Kind::Xm430 ? 1020U : 12U;
  }
  [[nodiscard]] uint8_t Level() const {
    return mem[L().status_level];
  }
  [[nodiscard]] bool TorqueOn() const {
    return mem[L().torque] != 0U;
  }
  [[nodiscard]] uint16_t Word(uint16_t addr) const {
    return static_cast<uint16_t>(mem[addr] | (static_cast<uint32_t>(mem[addr + 1U]) << 8U));
  }
  void SetWord(uint16_t addr, uint16_t v) {
    mem[addr] = U8(v);
    mem[addr + 1U] = U8(v >> 8U);
  }
  [[nodiscard]] int32_t Dword(uint16_t addr) const {
    uint32_t v = 0;
    for (uint32_t i = 0; i < 4U; ++i) {
      v |= static_cast<uint32_t>(mem[addr + i]) << (8U * i);
    }
    return static_cast<int32_t>(v);
  }
  void SetDword(uint16_t addr, int32_t value) {
    const auto v = static_cast<uint32_t>(value);
    for (uint32_t i = 0; i < 4U; ++i) {
      mem[addr + i] = U8(v >> (8U * i));
    }
  }

  void Reset(Kind k, uint8_t new_id) {
    kind = k;
    id = new_id;
    present = true;
    mem.fill(0);
    has_pending = false;
    silent = false;
    alert = false;
    force_error = 0;
    LoadDefaults(true);
  }

  void LoadDefaults(bool keep_nothing) {
    const uint8_t keep_id = id;
    const uint8_t keep_baud = mem[L().baud];
    mem.fill(0);
    SetWord(0, Model());
    if (kind == Kind::Xm430) {
      mem[6] = 46; // firmware
      mem[7] = keep_nothing ? keep_id : 1;
      mem[8] = keep_nothing ? 1 : keep_baud; // 57 600
      mem[9] = 250;
      mem[68] = 2;
      mem[146] = 25;     // temperature
      SetWord(144, 121); // 12.1 V
    } else {
      mem[2] = 25;
      mem[3] = keep_nothing ? keep_id : 1;
      mem[4] = keep_nothing ? 34 : keep_baud;
      mem[5] = 250;
      mem[16] = 2;
      mem[42] = 120;
      mem[43] = 27;
    }
    id = mem[L().id];
  }

  [[nodiscard]] bool Known(uint16_t addr, std::size_t len) const {
    return (static_cast<std::size_t>(addr) + len) <= L().table_size;
  }
  [[nodiscard]] bool ReadOnlyByte(uint16_t a) const {
    if (kind == Kind::Xm430) {
      return (a <= 6U) || ((a >= 69U) && (a <= 70U)) || ((a >= 120U) && (a <= 146U));
    }
    return (a <= 2U) || ((a >= 36U) && (a <= 43U)) || (a == 46U) || (a == 44U);
  }
  [[nodiscard]] bool Eeprom(uint16_t a) const {
    return a < L().eeprom_end;
  }
};

/// What the bus does to the next replies, for the fault tests.
struct Knobs {
  int drop_replies{0};   ///< Swallow this many replies (no bytes).
  int corrupt_crc{0};    ///< Flip a bit of the check field of this many replies.
  int truncate_to{-1};   ///< Cut the next reply to this many bytes (once); -1 off.
  int garbage_prefix{0}; ///< Put this many junk bytes before the next reply (once).
  int trailing_junk{0};  ///< Put this many bytes after the next reply (once).
  int stuck{0};          ///< 1: answer with 0x00 bytes, 2: with 0xFF bytes (every instruction).
  int stuck_len{12};
  int wrong_id{0};      ///< This many replies carry id+1.
  int split_chunk{0};   ///< When non-zero, `Read` returns at most this many bytes per call.
  bool echo{false};     ///< Loopback: the transmitted bytes also arrive at the receiver.
  int echo_corrupt{0};  ///< Flip a byte of the next echo.
  int echo_drop{0};     ///< Drop the next echo entirely.
  int bad_stuffing{0};  ///< Replace the next reply's parameter FD stuffing with a bare FF FF FD.
  int oversize_read{0}; ///< `Read` claims more bytes than asked (contract breach), once.
  int status_instruction{0}; ///< Reply's instruction byte is wrong (0x56).
  int extra_param{0};        ///< Reply carries one extra parameter byte.
};

/// The whole bus.
struct Bus {
  bool v2{true};
  std::array<Servo, 8> servos{};
  std::array<uint8_t, 8192> rx{};
  std::size_t rx_head{0};
  std::size_t rx_tail{0};
  Knobs knob{};
  // Counters and logs the tests read.
  int writes{0};
  int flushes{0};
  int reads{0};
  int bad_packets{0}; ///< Instruction packets the servo model could not parse (should stay 0).
  uint32_t delay_us{0};
  uint32_t baud{0};
  int baud_calls{0};
  int ensure_calls{0};
  std::array<uint8_t, 1200> last_tx{};
  std::size_t last_tx_len{0};
  int log_lines{0};
  // Port failures.
  bool fail_ensure{false};
  bool fail_write{false};
  bool fail_flush{false};
  int fail_flush_first{0}; ///< The first this many FlushInput() calls fail, then it works.
  bool fail_baud{false};
  int fail_write_after{-1}; ///< Writes after which Write() returns false (counts down).
  int fail_read_after{
      -1}; ///< Reads after which the port reports a bus failure by `Read` returning > max.

  Servo& Add(Kind kind, uint8_t id) {
    for (Servo& s : servos) {
      if (!s.present) {
        s.Reset(kind, id);
        return s;
      }
    }
    return servos[0];
  }
  Servo* Find(uint8_t id) {
    for (Servo& s : servos) {
      if (s.present && (s.id == id)) {
        return &s;
      }
    }
    return nullptr;
  }
  [[nodiscard]] std::size_t Queued() const {
    return rx_tail - rx_head;
  }
  void Push(uint8_t b) {
    if (rx_tail < rx.size()) {
      rx[rx_tail++] = b;
    }
  }
  void PushBytes(const uint8_t* p, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
      Push(p[i]);
    }
  }
  void Clear() {
    rx_head = rx_tail = 0;
  }

  // ---------------------------------------------------------------- reply building
  /// Emit one reply packet with fault injection. `error` is the raw error byte.
  void Reply(const Servo& s, uint8_t error, const uint8_t* params, std::size_t n) {
    if (s.silent) {
      return;
    }
    if (knob.drop_replies > 0) {
      --knob.drop_replies;
      return;
    }
    std::array<uint8_t, 700> pkt{};
    std::size_t len = 0;
    const uint8_t id = U8(s.id + (knob.wrong_id > 0 ? 1U : 0U));
    if (knob.wrong_id > 0) {
      --knob.wrong_id;
    }
    uint8_t instruction = 0x55;
    if (knob.status_instruction > 0) {
      --knob.status_instruction;
      instruction = 0x56;
    }
    if (v2) {
      // Stuff the parameters: an FD after every FF FF FD.
      std::array<uint8_t, 600> body{};
      std::size_t b = 0;
      for (std::size_t i = 0; i < n; ++i) {
        body[b++] = params[i];
        if ((i >= 2U) && (params[i] == 0xFDU) && (params[i - 1U] == 0xFFU) &&
            (params[i - 2U] == 0xFFU)) {
          body[b++] = 0xFD;
        }
      }
      if (knob.bad_stuffing > 0) {
        --knob.bad_stuffing;
        body[b++] = 0xFF;
        body[b++] = 0xFF;
        body[b++] = 0xFD;
      }
      if (knob.extra_param > 0) {
        --knob.extra_param;
        body[b++] = 0x5A;
      }
      const std::size_t l = b + 4U; // instruction, error, CRC
      pkt = {0xFF, 0xFF, 0xFD, 0x00, id, U8(l), U8(l >> 8U), instruction, error};
      len = 9;
      for (std::size_t i = 0; i < b; ++i) {
        pkt[len++] = body[i];
      }
      const uint16_t crc = Crc(pkt.data(), len);
      pkt[len++] = U8(crc);
      pkt[len++] = U8(crc >> 8U);
    } else {
      std::size_t extra = 0;
      if (knob.extra_param > 0) {
        --knob.extra_param;
        extra = 1;
      }
      pkt = {0xFF, 0xFF, id, U8(n + extra + 2U), error};
      len = 5;
      for (std::size_t i = 0; i < n; ++i) {
        pkt[len++] = params[i];
      }
      if (extra != 0U) {
        pkt[len++] = 0x5A;
      }
      pkt[len] = Sum1(pkt.data() + 2, len - 2U);
      ++len;
    }
    Deliver(pkt.data(), len);
  }

  /// Apply the packet-level knobs and queue the bytes.
  void Deliver(uint8_t* pkt, std::size_t len) {
    if (knob.corrupt_crc > 0) {
      --knob.corrupt_crc;
      pkt[len - 1U] = U8(pkt[len - 1U] ^ 0x40U);
    }
    for (int i = 0; i < knob.garbage_prefix; ++i) {
      Push(static_cast<uint8_t>(0x11 + i));
    }
    knob.garbage_prefix = 0;
    std::size_t out = len;
    if (knob.truncate_to >= 0) {
      out = std::min(len, static_cast<std::size_t>(knob.truncate_to));
      knob.truncate_to = -1;
    }
    PushBytes(pkt, out);
    for (int i = 0; i < knob.trailing_junk; ++i) {
      Push(static_cast<uint8_t>(0xA0 + i));
    }
    knob.trailing_junk = 0;
  }

  /// A fast read reply: one packet, one part per servo, each CRC over the packet so far.
  void FastReply(const std::array<const Servo*, 8>& who, const std::array<std::size_t, 8>& want,
                 const std::array<uint16_t, 8>& addr, std::size_t count) {
    std::array<uint8_t, 1100> pkt{};
    std::size_t len = 7;
    std::size_t total = 1;
    for (std::size_t k = 0; k < count; ++k) {
      total += want[k] + 4U;
    }
    pkt = {0xFF, 0xFF, 0xFD, 0x00, who[0]->id, U8(total), U8(total >> 8U), 0x55};
    len = 8;
    for (std::size_t k = 0; k < count; ++k) {
      const Servo& s = *who[k];
      uint8_t err = s.alert ? 0x80U : 0x00U;
      const bool bad = !s.Known(addr[k], want[k]);
      if (bad) {
        err = 0x07;
      }
      pkt[len++] = err;
      pkt[len++] = s.id;
      for (std::size_t i = 0; i < want[k]; ++i) {
        pkt[len++] = bad ? 0U : s.mem[addr[k] + i];
      }
      const uint16_t crc = Crc(pkt.data(), len);
      pkt[len++] = U8(crc);
      pkt[len++] = U8(crc >> 8U);
    }
    if (knob.drop_replies > 0) {
      --knob.drop_replies;
      return;
    }
    Deliver(pkt.data(), len);
  }

  // ---------------------------------------------------------------- servo-side semantics
  /// Error byte for a rejected instruction.
  [[nodiscard]] uint8_t Rejection(const Servo& s, uint16_t number_or_flag) const {
    static_cast<void>(s);
    return U8(number_or_flag);
  }

  /// Write `n` bytes at `addr`; returns the error number (V2) / flag (V1), 0 when carried out.
  uint8_t DoWrite(Servo& s, uint16_t addr, const uint8_t* data, std::size_t n) const {
    const uint8_t access = v2 ? 7U : 0x08U;
    if (!s.Known(addr, n) || (n == 0U)) {
      return v2 ? 7U : 0x08U;
    }
    for (std::size_t i = 0; i < n; ++i) {
      const auto a = static_cast<uint16_t>(addr + i);
      if (s.ReadOnlyByte(a) || (v2 && s.Eeprom(a) && s.TorqueOn() && (a != s.L().torque))) {
        return access;
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      s.mem[addr + i] = data[i];
    }
    if (addr <= s.L().id && (addr + n) > s.L().id) {
      s.id = s.mem[s.L().id];
    }
    return 0;
  }

  [[nodiscard]] bool ReplyAllowed(const Servo& s, uint8_t inst, uint8_t target) const {
    if (target == 0xFE) {
      return false;
    }
    if (inst == 0x01) {
      return true;
    }
    if (inst == 0x02) {
      return s.Level() >= 1U;
    }
    return s.Level() >= 2U;
  }

  /// A servo answers `error` (and data) to an addressed instruction.
  void Answer(Servo& s, uint8_t inst, uint8_t target, uint8_t error, const uint8_t* p,
              std::size_t n) {
    uint8_t e = error;
    if (s.alert) {
      e = U8(e | (v2 ? 0x80U : 0x04U));
    }
    if (!ReplyAllowed(s, inst, target)) {
      return;
    }
    if ((error != 0U) && (!v2 || (error != 0x80U))) {
      Reply(s, e, nullptr, 0);
    } else {
      Reply(s, e, p, n);
    }
  }

  /// Handle one instruction addressed to `target` (a servo ID or 0xFE).
  void Execute(uint8_t target, uint8_t inst, const uint8_t* p, std::size_t n) {
    const bool bc = (target == 0xFE);
    std::array<Servo*, 8> hit{};
    std::size_t hits = 0;
    for (Servo& s : servos) {
      if (s.present && (bc || (s.id == target))) {
        hit[hits++] = &s;
      }
    }
    // Servos answer a broadcast in ID order.
    for (std::size_t i = 1; i < hits; ++i) { // insertion sort: at most 8 entries
      for (std::size_t j = i; (j > 0U) && (hit[j]->id < hit[j - 1U]->id); --j) {
        std::swap(hit[j], hit[j - 1U]);
      }
    }
    for (std::size_t h = 0; h < hits; ++h) {
      Servo& s = *hit[h];
      ++s.instructions;
      uint8_t forced = s.force_error;
      if (forced != 0U) {
        s.force_error = 0;
        if (inst != 0x01) {
          Answer(s, inst, target, forced, nullptr, 0);
          continue;
        }
      }
      switch (inst) {
      case 0x01: { // ping
        if (v2) {
          const std::array<uint8_t, 3> d{U8(s.Model()), U8(s.Model() >> 8U), s.mem[6]};
          if (bc) {
            Reply(s, s.alert ? 0x80U : 0U, d.data(), d.size());
          } else {
            Answer(s, inst, target, 0, d.data(), d.size());
          }
        } else {
          Answer(s, inst, target, 0, nullptr, 0);
        }
        break;
      }
      case 0x02: { // read
        const uint16_t addr = v2 ? U16(p) : p[0];
        const std::size_t len = v2 ? U16(p + 2) : p[1];
        if (!s.Known(addr, len) || (len == 0U)) {
          Answer(s, inst, target, v2 ? 7U : 0x08U, nullptr, 0);
        } else {
          Answer(s, inst, target, 0, s.mem.data() + addr, len);
        }
        break;
      }
      case 0x03: { // write
        const uint16_t addr = v2 ? U16(p) : p[0];
        const std::size_t skip = v2 ? 2U : 1U;
        const uint8_t err = DoWrite(s, addr, p + skip, n - skip);
        Answer(s, inst, target, err, nullptr, 0);
        break;
      }
      case 0x04: { // reg write
        const uint16_t addr = v2 ? U16(p) : p[0];
        const std::size_t skip = v2 ? 2U : 1U;
        s.has_pending = true;
        s.pending_addr = addr;
        s.pending_len = static_cast<uint16_t>(n - skip);
        std::copy(p + skip, p + n, s.pending.begin());
        Answer(s, inst, target, 0, nullptr, 0);
        break;
      }
      case 0x05: { // action
        if (!s.has_pending) {
          Answer(s, inst, target, v2 ? 2U : 0x40U, nullptr, 0);
        } else {
          const uint8_t err = DoWrite(s, s.pending_addr, s.pending.data(), s.pending_len);
          s.has_pending = false;
          Answer(s, inst, target, err, nullptr, 0);
        }
        break;
      }
      case 0x06: { // factory reset
        const uint8_t opt = (v2 && (n >= 1U)) ? p[0] : 0xFF;
        if (v2 && (opt != 0xFF) && (opt != 0x01) && (opt != 0x02)) {
          Answer(s, inst, target, 7, nullptr, 0);
          break;
        }
        Answer(s, inst, target, 0, nullptr, 0);
        ++s.factory_resets;
        s.last_reset_option = opt;
        if (opt == 0xFF) {
          s.id = 1;
        }
        const uint8_t keep_id = s.id;
        const uint8_t keep_baud = s.mem[s.L().baud];
        s.LoadDefaults(opt != 0xFF);
        if (opt == 0x02) {
          s.mem[s.L().baud] = keep_baud;
        }
        if (opt == 0x01 || opt == 0x02) {
          s.mem[s.L().id] = keep_id;
          s.id = keep_id;
        }
        break;
      }
      case 0x08: { // reboot
        Answer(s, inst, target, 0, nullptr, 0);
        ++s.reboots;
        s.mem[s.L().torque] = 0;
        break;
      }
      case 0x10: { // clear
        const std::array<uint8_t, 5> mt{0x01, 0x44, 0x58, 0x4C, 0x22};
        const std::array<uint8_t, 5> er{0x02, 0x45, 0x52, 0x43, 0x4C};
        const bool ok = (n == 5U) && (std::equal(mt.begin(), mt.end(), p) ||
                                      std::equal(er.begin(), er.end(), p));
        if (!ok) {
          Answer(s, inst, target, 5, nullptr, 0);
        } else {
          ++s.clears;
          Answer(s, inst, target, 0, nullptr, 0);
        }
        break;
      }
      case 0x20: { // backup
        const std::array<uint8_t, 4> magic{0x43, 0x54, 0x52, 0x4C};
        const bool ok = (n == 5U) && std::equal(magic.begin(), magic.end(), p + 1) &&
                        ((p[0] == 1U) || (p[0] == 2U));
        if (!ok) {
          Answer(s, inst, target, 5, nullptr, 0);
        } else {
          if (p[0] == 1U) {
            ++s.backups;
          } else {
            ++s.restores;
          }
          Answer(s, inst, target, 0, nullptr, 0);
        }
        break;
      }
      default:
        Answer(s, inst, target, v2 ? 2U : 0x40U, nullptr, 0);
        break;
      }
    }
  }

  static uint16_t U16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint32_t>(p[1]) << 8U));
  }

  /// Sync/bulk instructions are addressed to 0xFE and handled per listed ID.
  void Multi(uint8_t inst, const uint8_t* p, std::size_t n) {
    if (inst == 0x83) { // sync write
      const uint16_t addr = v2 ? U16(p) : p[0];
      const std::size_t len = v2 ? U16(p + 2) : p[1];
      const std::size_t hdr = v2 ? 4U : 2U;
      if ((len == 0U) || (((n - hdr) % (len + 1U)) != 0U)) {
        ++bad_packets;
        return;
      }
      for (std::size_t off = hdr; off < n; off += len + 1U) {
        Servo* s = Find(p[off]);
        if (s != nullptr) {
          ++s->instructions;
          static_cast<void>(DoWrite(*s, addr, p + off + 1U, len));
        }
      }
      return;
    }
    if (inst == 0x93) { // bulk write (V2)
      std::size_t off = 0;
      while ((off + 5U) <= n) {
        const std::size_t len = U16(p + off + 3U);
        Servo* s = Find(p[off]);
        if ((off + 5U + len) > n) {
          ++bad_packets;
          return;
        }
        if (s != nullptr) {
          ++s->instructions;
          static_cast<void>(DoWrite(*s, U16(p + off + 1U), p + off + 5U, len));
        }
        off += 5U + len;
      }
      return;
    }
    std::array<uint8_t, 8> ids{};
    std::array<uint16_t, 8> addr{};
    std::array<std::size_t, 8> len{};
    std::size_t count = 0;
    if ((inst == 0x82) || (inst == 0x8A)) { // sync read, fast sync read
      const uint16_t a = U16(p);
      const std::size_t l = U16(p + 2);
      for (std::size_t i = 4; (i < n) && (count < 8U); ++i) {
        ids[count] = p[i];
        addr[count] = a;
        len[count] = l;
        ++count;
      }
    } else if ((inst == 0x92) || (inst == 0x9A)) { // bulk read
      if (v2) {
        for (std::size_t i = 0; ((i + 5U) <= n) && (count < 8U); i += 5U) {
          ids[count] = p[i];
          addr[count] = U16(p + i + 1U);
          len[count] = U16(p + i + 3U);
          ++count;
        }
      } else {
        for (std::size_t i = 1; ((i + 3U) <= n) && (count < 8U);
             i += 3U) { // 0x00, then (len, id, addr)
          len[count] = p[i];
          ids[count] = p[i + 1U];
          addr[count] = p[i + 2U];
          ++count;
        }
      }
    } else {
      ++bad_packets;
      return;
    }
    if ((inst == 0x8A) || (inst == 0x9A)) {
      std::array<const Servo*, 8> who{};
      std::size_t found = 0;
      std::array<std::size_t, 8> w{};
      std::array<uint16_t, 8> a{};
      for (std::size_t k = 0; k < count; ++k) {
        const Servo* s = Find(ids[k]);
        if ((s != nullptr) && !s->silent && s->fast_read_ok && (s->Level() >= 1U)) {
          who[found] = s;
          w[found] = len[k];
          a[found] = addr[k];
          ++found;
        }
      }
      if (found != 0U) {
        FastReply(who, w, a, found);
      }
      return;
    }
    // Separate replies, in the order listed. Each is a Read.
    for (std::size_t k = 0; k < count; ++k) {
      Servo* s = Find(ids[k]);
      if (s == nullptr) {
        continue;
      }
      ++s->instructions;
      if (s->force_error != 0U) {
        const uint8_t forced = s->force_error;
        s->force_error = 0;
        Answer(*s, 0x02, ids[k], forced, nullptr, 0);
      } else if (!s->Known(addr[k], len[k])) {
        Answer(*s, 0x02, ids[k], v2 ? 7U : 0x08U, nullptr, 0);
      } else {
        Answer(*s, 0x02, ids[k], 0, s->mem.data() + addr[k], len[k]);
      }
    }
  }

  // ---------------------------------------------------------------- host -> bus
  /// The bytes the host transmitted: parse every instruction packet inside.
  void Transmit(const uint8_t* data, std::size_t len) {
    if (knob.echo && (knob.echo_drop == 0)) {
      std::array<uint8_t, 1200> copy{};
      std::copy(data, data + std::min(len, copy.size()), copy.begin());
      if (knob.echo_corrupt > 0) {
        --knob.echo_corrupt;
        copy[len / 2U] = U8(copy[len / 2U] ^ 0x01U);
      }
      PushBytes(copy.data(), std::min(len, copy.size()));
    } else if (knob.echo_drop > 0) {
      --knob.echo_drop;
    }
    std::copy(data, data + std::min(len, last_tx.size()), last_tx.begin());
    last_tx_len = std::min(len, last_tx.size());
    if (knob.stuck != 0) {
      for (int i = 0; i < knob.stuck_len; ++i) {
        Push(knob.stuck == 1 ? 0x00U : 0xFFU);
      }
      return;
    }
    if (v2) {
      ParseV2(data, len);
    } else {
      ParseV1(data, len);
    }
  }

  void ParseV2(const uint8_t* d, std::size_t len) {
    if ((len < 10U) || (d[0] != 0xFF) || (d[1] != 0xFF) || (d[2] != 0xFD) || (d[3] != 0x00)) {
      ++bad_packets;
      return;
    }
    const std::size_t l = U16(d + 5);
    if ((l + 7U) != len || l < 3U) {
      ++bad_packets;
      return;
    }
    const uint16_t crc = Crc(d, len - 2U);
    if (crc != U16(d + len - 2U)) {
      ++bad_packets;
      return;
    }
    // Remove the stuffing: the FD that follows FF FF FD in the parameters is dropped.
    std::array<uint8_t, 1200> out{};
    std::size_t o = 0;
    bool skip = false;
    for (std::size_t i = 8; i < (len - 2U); ++i) {
      if (skip && (d[i] == 0xFDU)) {
        skip = false;
        continue;
      }
      out[o++] = d[i];
      skip =
          (o >= 3U) && (out[o - 3U] == 0xFFU) && (out[o - 2U] == 0xFFU) && (out[o - 1U] == 0xFDU);
    }
    Dispatch(d[4], d[7], out.data(), o);
  }

  void ParseV1(const uint8_t* d, std::size_t len) {
    if ((len < 6U) || (d[0] != 0xFF) || (d[1] != 0xFF) ||
        ((static_cast<std::size_t>(d[3]) + 4U) != len)) {
      ++bad_packets;
      return;
    }
    if (Sum1(d + 2, len - 3U) != d[len - 1U]) {
      ++bad_packets;
      return;
    }
    Dispatch(d[2], d[4], d + 5, len - 6U);
  }

  void Dispatch(uint8_t target, uint8_t inst, const uint8_t* p, std::size_t n) {
    if ((inst == 0x82) || (inst == 0x83) || (inst == 0x8A) || (inst == 0x92) || (inst == 0x93) ||
        (inst == 0x9A)) {
      Multi(inst, p, n);
    } else {
      Execute(target, inst, p, n);
    }
  }
};

/// The UART port the driver uses. `Bus` is shared with the test.
class Port : public dynamixel::UartInterface<Port> {
public:
  explicit Port(Bus& bus) noexcept : bus_(bus) {}
  bool EnsureInitialized() noexcept {
    ++bus_.ensure_calls;
    return !bus_.fail_ensure;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    ++bus_.writes;
    if (bus_.fail_write) {
      return false;
    }
    if (bus_.fail_write_after >= 0) {
      if (bus_.fail_write_after == 0) {
        return false;
      }
      --bus_.fail_write_after;
    }
    bus_.Transmit(data, len);
    return true;
  }
  std::size_t Read(uint8_t* data, std::size_t max, uint32_t /*timeout_us*/) noexcept {
    ++bus_.reads;
    if (bus_.knob.oversize_read > 0) {
      --bus_.knob.oversize_read;
      return max + 1U;
    }
    std::size_t n = std::min(max, bus_.Queued());
    if (bus_.knob.split_chunk > 0) {
      n = std::min(n, static_cast<std::size_t>(bus_.knob.split_chunk));
    }
    for (std::size_t i = 0; i < n; ++i) {
      data[i] = bus_.rx[bus_.rx_head++];
    }
    return n;
  }
  bool FlushInput() noexcept {
    ++bus_.flushes;
    if (bus_.fail_flush_first > 0) {
      --bus_.fail_flush_first;
      return false;
    }
    if (bus_.fail_flush) {
      return false;
    }
    bus_.Clear();
    return true;
  }
  void DelayUs(uint32_t us) noexcept {
    bus_.delay_us += us;
  }
  bool SetBaudRate(uint32_t baud) noexcept {
    ++bus_.baud_calls;
    if (bus_.fail_baud) {
      return false;
    }
    bus_.baud = baud;
    return true;
  }
  void Log(dynamixel::LogLevel /*level*/, const char* /*message*/) noexcept {
    ++bus_.log_lines;
  }

private:
  Bus& bus_;
};

} // namespace mock

#endif // DYNAMIXEL_MOCK_SERVO_BUS_HPP
