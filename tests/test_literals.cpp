/**
 * @file test_literals.cpp
 * @brief The `_u8`, `_u16` and `_u32` literals: every notation, the digit separator and the largest
 * values give the value written, at the type named. A value past the type does not compile
 * (tests/compile_fail/literal_u8_out_of_range.cpp).
 */
#include <cstdint>
#include <type_traits>

#include "dynamixel_detail.hpp"
#include "test_harness.hpp"

namespace {

using namespace dynamixel::literals;

static_assert(std::is_same_v<decltype(1_u8), std::uint8_t>);
static_assert(std::is_same_v<decltype(1_u16), std::uint16_t>);
static_assert(std::is_same_v<decltype(1_u32), std::uint32_t>);

// Decimal, with the single digit 0 and a separator.
static_assert(0_u8 == 0U);
static_assert(7_u8 == 7U);
static_assert(255_u8 == 255U);
static_assert(1'000_u16 == 1000U);
static_assert(65'535_u16 == 65'535U);
static_assert(1'000'000_u32 == 1'000'000U);
static_assert(4'294'967'295_u32 == 4'294'967'295U);
// Hexadecimal, either case of prefix and digits.
static_assert(0xFF_u8 == 255U);
static_assert(0Xa5_u8 == 0xA5U);
static_assert(0x4E19_u16 == 0x4E19U);
static_assert(0xFFFF'FFFF_u32 == 0xFFFF'FFFFU);
static_assert(0x0_u8 == 0U);
// Binary.
static_assert(0b1010'0101_u8 == 0xA5U);
static_assert(0B11_u8 == 3U);
// Octal.
static_assert(010_u8 == 8U);
static_assert(0377_u8 == 255U);
static_assert(00_u8 == 0U);

} // namespace

TEST_CASE("typed literals give the value written at run time too") {
  using namespace dynamixel::literals;
  const volatile std::uint32_t seen = 0x1234'5678_u32;
  CHECK_EQ(seen, 0x1234'5678U);
  CHECK_EQ(0xFE_u8, 254U);
}
