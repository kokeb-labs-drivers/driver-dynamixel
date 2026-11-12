/**
 * @file dynamixel_detail.hpp
 * @brief Compile-time helpers of the DYNAMIXEL driver: typed integer literals, exact hook matching,
 * typed register fields and enumeration checks.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * Everything here is `constexpr` or used only in unevaluated operands, so it adds no code to the
 * run-time path (handbook C++20 profile, KB-CPP20-006, -008, -020, -021, -025, -026).
 */
#ifndef DYNAMIXEL_DETAIL_HPP
#define DYNAMIXEL_DETAIL_HPP

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace dynamixel::detail {

/**
 * @brief Reached only when a `_u8`, `_u16` or `_u32` literal does not fit its type: it is never
 * defined and not `constexpr`, so the constant evaluation fails and the build names it.
 */
void LiteralOutOfRange() noexcept;

/// Returned by `LiteralDigit()` for a character that is no digit.
inline constexpr std::uint64_t kNotADigit = 99U;

/**
 * @brief The value of a digit character, 0 to 15 (`0`-`9`, `a`-`f`, `A`-`F`).
 * @details A `switch` on the character, not arithmetic on it (MISRA C++:2023 Rule 7.0.3).
 * @param c The character.
 * @return Its value, or `kNotADigit`.
 */
consteval std::uint64_t LiteralDigit(char c) noexcept {
  switch (c) {
  case '0':
    return 0U;
  case '1':
    return 1U;
  case '2':
    return 2U;
  case '3':
    return 3U;
  case '4':
    return 4U;
  case '5':
    return 5U;
  case '6':
    return 6U;
  case '7':
    return 7U;
  case '8':
    return 8U;
  case '9':
    return 9U;
  case 'a':
  case 'A':
    return 10U;
  case 'b':
  case 'B':
    return 11U;
  case 'c':
  case 'C':
    return 12U;
  case 'd':
  case 'D':
    return 13U;
  case 'e':
  case 'E':
    return 14U;
  case 'f':
  case 'F':
    return 15U;
  default:
    return kNotADigit;
  }
}

/**
 * @brief The base a literal's second character selects after a leading `0`.
 * @param c The character after the `0`.
 * @return 16 for `x` or `X`, 2 for `b` or `B`, 0 for any other character (an octal literal).
 */
consteval std::uint64_t LiteralPrefixBase(char c) noexcept {
  switch (c) {
  case 'x':
  case 'X':
    return 16U;
  case 'b':
  case 'B':
    return 2U;
  default:
    return 0U;
  }
}

/**
 * @brief The value of an integer literal given as its characters, for the raw literal operators
 * below: decimal, `0x` hexadecimal, `0b` binary or `0` octal, with digit separators (`'`).
 * @tparam Digits The characters of the literal, as written.
 * @param maximum The largest value the literal's type holds.
 * @return The value; a literal larger than `maximum` does not compile.
 */
template <char... Digits>
consteval std::uint64_t RawLiteralValue(std::uint64_t maximum) noexcept {
  constexpr std::array<char, sizeof...(Digits)> kText{Digits...};
  std::uint64_t base = 10U;
  std::size_t first = 0U;
  if ((kText.size() > 1U) && (LiteralDigit(kText[0]) == 0U)) {
    base = LiteralPrefixBase(kText[1]);
    first = 2U;
    if (base == 0U) {
      base = 8U; // no prefix: the leading 0 only marks octal
      first = 1U;
    }
  }
  std::uint64_t value = 0U;
  for (std::size_t i = first; i < kText.size(); ++i) {
    const std::uint64_t digit = LiteralDigit(kText[i]);
    if (digit == kNotADigit) {
      continue; // a digit separator: the compiler has accepted the literal, so nothing else is left
    }
    // Checked before the multiplication, so a long literal cannot wrap round to a small value.
    if ((digit >= base) || (value > ((maximum - digit) / base))) {
      LiteralOutOfRange();
    }
    value = (value * base) + digit;
  }
  return value;
}

} // namespace dynamixel::detail

namespace dynamixel::inline literals {
/// The `_u8`, `_u16` and `_u32` literals are visible in the whole driver namespace without
/// using-declarations.

/**
 * @brief An integer constant of type `uint8_t`, for an argument whose parameter is `uint8_t`
 * (MISRA C++:2023 Rule 7.0.6). `0xFF_u8` compiles; `0x100_u8` does not.
 * @tparam Digits The characters of the literal.
 * @return The literal's value as `uint8_t`.
 */
template <char... Digits>
consteval std::uint8_t operator""_u8() noexcept {
  return static_cast<std::uint8_t>(
      detail::RawLiteralValue<Digits...>(std::numeric_limits<std::uint8_t>::max()));
}

/**
 * @brief An integer constant of type `uint16_t` (Rule 7.0.6); out of range does not compile.
 * @tparam Digits The characters of the literal.
 * @return The literal's value as `uint16_t`.
 */
template <char... Digits>
consteval std::uint16_t operator""_u16() noexcept {
  return static_cast<std::uint16_t>(
      detail::RawLiteralValue<Digits...>(std::numeric_limits<std::uint16_t>::max()));
}

/**
 * @brief An integer constant of type `uint32_t` (Rule 7.0.6): on targets where `uint32_t` is
 * `unsigned long`, a `U` literal is still `unsigned int`. Out of range does not compile.
 * @tparam Digits The characters of the literal.
 * @return The literal's value as `uint32_t`.
 */
template <char... Digits>
consteval std::uint32_t operator""_u32() noexcept {
  return static_cast<std::uint32_t>(
      detail::RawLiteralValue<Digits...>(std::numeric_limits<std::uint32_t>::max()));
}

} // namespace dynamixel::inline literals

namespace dynamixel::detail {

/**
 * @brief Matches a port hook against exactly one signature, `R(Args...) noexcept`, as a member
 * function of `Port` (`const` or not) (KB-CPP20-006).
 *
 * `ExactHook<Port, Sig>::Take(&Port::Hook)` is well-formed only when `&Port::Hook` names a member
 * function of exactly that type: the address of an overloaded or template hook resolves to the
 * one that matches, and a hook with a narrower or different parameter, another result type or no
 * `noexcept` has no match. No value is converted. Declared only: it appears in
 * requires-expressions, never in evaluated code.
 *
 * @tparam Port The port class.
 * @tparam Sig The hook's function type, `noexcept`; a `Sig` without `noexcept` has no definition.
 */
template <typename Port, typename Sig>
  requires std::is_class_v<Port> && std::is_function_v<Sig>
struct ExactHook;

/// The `noexcept` signatures: the only ones a hook may have.
template <typename Port, typename R, typename... Args>
  requires std::is_class_v<Port>
struct ExactHook<Port, R(Args...) noexcept> {
  /// Takes the address of a non-`const` hook of exactly this signature.
  static void Take(R (Port::*hook)(Args...) noexcept) noexcept;
  /// Takes the address of a `const` hook of exactly this signature.
  static void Take(R (Port::*hook)(Args...) const noexcept) noexcept;
};

/// A class type: what `kAlwaysFalse` depends on, the port or another class of the branch.
template <typename T>
concept ClassType = std::is_class_v<T>;

/// `false` for every class `T`: the condition of a `static_assert` in a branch that must not be
/// instantiated (KB-CPP20-008).
template <ClassType>
inline constexpr bool kAlwaysFalse = false;

/// A scoped or unscoped enumeration: what the enumeration helpers take (KB-CPP20-005, -026).
template <typename E>
concept Enumeration = std::is_enum_v<E>;

/**
 * @brief The value of an enumerator in its underlying type, our stand-in for C++23
 * `std::to_underlying` (KB-CPP20-002, -026).
 */
template <Enumeration E>
[[nodiscard]] constexpr std::underlying_type_t<E> ToUnderlying(E e) noexcept {
  return static_cast<std::underlying_type_t<E>>(e);
}

/**
 * @brief The last enumerator of a contiguous enumeration that starts at 0; specialised next to each
 * enumeration that `IsEnumerator()` checks (KB-CPP20-026).
 */
template <Enumeration E>
struct EnumLast;

/**
 * @brief Whether `e` holds an enumerator of a contiguous enumeration that starts at 0. A value made
 * by `static_cast` from an integer can be anything its underlying type holds (KB-CPP20-025).
 */
template <Enumeration E>
[[nodiscard]] constexpr bool IsEnumerator(E e) noexcept {
  return static_cast<uint32_t>(ToUnderlying(e)) <=
         static_cast<uint32_t>(ToUnderlying(EnumLast<E>::kValue));
}

/**
 * @brief Whether `value` is between `low` and `high`, both inclusive.
 * @param value The value to test.
 * @param low The smallest accepted value.
 * @param high The largest accepted value.
 * @return true when `low <= value <= high`.
 */
[[nodiscard]] constexpr bool InRange(uint32_t value, uint32_t low, uint32_t high) noexcept {
  return (value >= low) && (value <= high);
}

/**
 * @brief A bit field of a register: `Width` bits from bit `Shift` of a `Reg`. Every shift is a
 * constant below the width and every operand is widened to `uint32_t` once, by construction
 * (KB-CPP20-020, -021).
 */
template <typename Reg, uint32_t Shift, uint32_t Width>
  requires std::unsigned_integral<Reg> && (Width > 0U) && (Width < 32U) &&
           ((Shift + Width) <= static_cast<uint32_t>(std::numeric_limits<Reg>::digits))
struct Field {
  /// The largest value the field holds.
  static constexpr uint32_t kMax = (uint32_t{1} << Width) - 1U;
  /// The field's bits in the register.
  static constexpr Reg kMask = static_cast<Reg>(kMax << Shift);

  /// `reg` with the field set to `value` (masked to the field's width); the other bits unchanged.
  [[nodiscard]] static constexpr Reg Insert(Reg reg, uint32_t value) noexcept {
    return static_cast<Reg>((static_cast<uint32_t>(reg) & ~static_cast<uint32_t>(kMask)) |
                            ((value & kMax) << Shift));
  }
  /// The field's value in `reg`.
  [[nodiscard]] static constexpr uint32_t Extract(Reg reg) noexcept {
    return (static_cast<uint32_t>(reg) >> Shift) & kMax;
  }
};

} // namespace dynamixel::detail

#endif // DYNAMIXEL_DETAIL_HPP
