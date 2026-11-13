/**
 * @file literal_u8_out_of_range.cpp
 * @brief Must NOT compile: 0x100 does not fit `uint8_t`, so the `_u8` literal is rejected at build
 * time and not truncated (MISRA C++:2023 Rule 7.0.6).
 *
 * EXPECT: LiteralOutOfRange
 */
#include "dynamixel_detail.hpp"

int main() {
  using namespace dynamixel::literals;
  constexpr auto too_big = 0x100_u8;
  return static_cast<int>(too_big);
}
