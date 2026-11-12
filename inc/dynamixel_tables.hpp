/**
 * @file dynamixel_tables.hpp
 * @brief Control tables of the DYNAMIXEL servo families, the model catalogue and the unit scales.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * One logical register name (`Reg`) maps to an address, a size and a decoding in every family that
 * has it (`Lookup()`); a family without it returns `Unsupported` from the typed calls. The tables
 * are data of the ROBOTIS e-Manual "Control Table" pages (docs/datasheets.yml,
 * docs/control-tables.md), one table per family, checked against them in tests/test_model.cpp.
 * Names the e-Manual spells differently per family (`Present Speed`, `Present Velocity`) are one
 * `Reg`; the decoding (`Kind`) and the unit scale (`ModelInfo`, `FamilyInfo`) differ per family.
 * Families the table does not list (the Pro, P, H, L and Y series) are reached with raw addresses
 * (`Family::Generic`).
 */
#ifndef DYNAMIXEL_TABLES_HPP
#define DYNAMIXEL_TABLES_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "dynamixel_detail.hpp"

namespace dynamixel {

/** @brief A servo family: one control table layout (several models share it). */
enum class Family : uint8_t {
  Generic = 0, ///< Unknown or unlisted model: raw addresses only.
  AxRxDx,      ///< AX-12A/12W, RX-10/24F/28/64, DX-113/116/117 (protocol 1.0).
  Mx28,        ///< MX-28 and MX-12W (protocol 1.0 firmware).
  Mx64,        ///< MX-64 (protocol 1.0 firmware).
  Mx106,       ///< MX-106 (protocol 1.0 firmware).
  Mx28v2,      ///< MX-28(2.0) (protocol 2.0 firmware).
  Mx64v2,      ///< MX-64(2.0) and MX-106(2.0).
  Xl320,       ///< XL-320 (protocol 2.0 only, its own table).
  X330,        ///< XL330, XC330, XM335.
  X430Basic,   ///< XL430, XC430, 2XL430, 2XC430 (no current registers).
  X430,        ///< XM430, XH430, XD430.
  X540,        ///< XM540, XH540, XD540 (adds the external ports).
  Xw,          ///< XW430, XW540 (X430 table without LED).
};

/** @brief Whether a register can be written. */
enum class Access : uint8_t {
  ReadOnly = 0,  ///< `R`: a write is an access error.
  ReadWrite = 1, ///< `RW`.
};

/** @brief The memory a register lives in. */
enum class Area : uint8_t {
  Eeprom = 0, ///< Kept at power-off; written only while torque is off.
  Ram = 1,    ///< Reset at power-on.
};

/** @brief How the raw bytes of a register are read as a number. */
enum class Kind : uint8_t {
  Unsigned = 0,   ///< Little-endian unsigned.
  Signed = 1,     ///< Little-endian two's complement of the register width.
  SignMag10 = 2,  ///< Protocol 1.0 speed, load, torque: bits 9..0 magnitude, bit 10 direction (CW).
  Offset2048 = 3, ///< MX-64/106 current: `value - 2048` counts.
};

/** @brief Family-neutral register names. */
enum class Reg : uint8_t {

  ModelNumber = 0,
  FirmwareVersion = 1,
  Id = 2,
  BaudRate = 3,
  ReturnDelayTime = 4,
  CwAngleLimit = 5,
  CcwAngleLimit = 6,
  TemperatureLimit = 7,
  MinVoltageLimit = 8,
  MaxVoltageLimit = 9,
  MaxTorque = 10,
  StatusReturnLevel = 11,
  AlarmLed = 12,
  Shutdown = 13,
  TorqueEnable = 14,
  Led = 15,
  CwComplianceMargin = 16,
  CcwComplianceMargin = 17,
  CwComplianceSlope = 18,
  CcwComplianceSlope = 19,
  GoalPosition = 20,
  MovingSpeed = 21,
  TorqueLimit = 22,
  PresentPosition = 23,
  PresentVelocity = 24,
  PresentLoad = 25,
  PresentVoltage = 26,
  PresentTemperature = 27,
  RegisteredInstruction = 28,
  Moving = 29,
  Lock = 30,
  Punch = 31,
  MultiTurnOffset = 32,
  ResolutionDivider = 33,
  DGain = 34,
  IGain = 35,
  PGain = 36,
  RealtimeTick = 37,
  GoalAcceleration = 38,
  Current = 39,
  TorqueControlModeEnable = 40,
  GoalTorque = 41,
  DriveMode = 42,
  ModelInformation = 43,
  OperatingMode = 44,
  SecondaryId = 45,
  ProtocolType = 46,
  HomingOffset = 47,
  MovingThreshold = 48,
  PwmLimit = 49,
  AccelerationLimit = 50,
  VelocityLimit = 51,
  MaxPositionLimit = 52,
  MinPositionLimit = 53,
  HardwareErrorStatus = 54,
  VelocityIGain = 55,
  VelocityPGain = 56,
  PositionDGain = 57,
  PositionIGain = 58,
  PositionPGain = 59,
  Feedforward2ndGain = 60,
  Feedforward1stGain = 61,
  BusWatchdog = 62,
  GoalPwm = 63,
  GoalVelocity = 64,
  ProfileAcceleration = 65,
  ProfileVelocity = 66,
  MovingStatus = 67,
  PresentPwm = 68,
  VelocityTrajectory = 69,
  PositionTrajectory = 70,
  CurrentLimit = 71,
  GoalCurrent = 72,
  PresentCurrent = 73,
  ControlMode = 74,
  StartupConfiguration = 75,
  PwmSlope = 76,
  BackupReady = 77,
  ExternalPortMode1 = 78,
  ExternalPortMode2 = 79,
  ExternalPortMode3 = 80,
  ExternalPortData1 = 81,
  ExternalPortData2 = 82,
  ExternalPortData3 = 83,
};

/** @brief One register of a family's control table. */
struct RegInfo {
  Reg reg;          ///< Which register.
  uint16_t address; ///< Start address in the control table.
  uint8_t size;     ///< Size in bytes: 1, 2 or 4.
  Access access;    ///< Read-only or read-write.
  Area area;        ///< EEPROM or RAM.
  Kind kind;        ///< Decoding.
};

namespace detail {
inline constexpr Access kR = Access::ReadOnly;
inline constexpr Access kRW = Access::ReadWrite;
inline constexpr Area kE = Area::Eeprom;
inline constexpr Area kM = Area::Ram;
inline constexpr Kind kU = Kind::Unsigned;
inline constexpr Kind kS = Kind::Signed;
inline constexpr Kind kSM = Kind::SignMag10;
inline constexpr Kind kO = Kind::Offset2048;

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 32> kAxRxDxTable{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 2_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 3_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 4_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 5_u16, 1_u8, kRW, kE, kU},
    {Reg::CwAngleLimit, 6_u16, 2_u8, kRW, kE, kU},
    {Reg::CcwAngleLimit, 8_u16, 2_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxTorque, 14_u16, 2_u8, kRW, kE, kU},
    {Reg::StatusReturnLevel, 16_u16, 1_u8, kRW, kE, kU},
    {Reg::AlarmLed, 17_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 18_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 24_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 25_u16, 1_u8, kRW, kM, kU},
    {Reg::CwComplianceMargin, 26_u16, 1_u8, kRW, kM, kU},
    {Reg::CcwComplianceMargin, 27_u16, 1_u8, kRW, kM, kU},
    {Reg::CwComplianceSlope, 28_u16, 1_u8, kRW, kM, kU},
    {Reg::CcwComplianceSlope, 29_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPosition, 30_u16, 2_u8, kRW, kM, kU},
    {Reg::MovingSpeed, 32_u16, 2_u8, kRW, kM, kSM},
    {Reg::TorqueLimit, 34_u16, 2_u8, kRW, kM, kU},
    {Reg::PresentPosition, 36_u16, 2_u8, kR, kM, kU},
    {Reg::PresentVelocity, 38_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentLoad, 40_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentVoltage, 42_u16, 1_u8, kR, kM, kU},
    {Reg::PresentTemperature, 43_u16, 1_u8, kR, kM, kU},
    {Reg::RegisteredInstruction, 44_u16, 1_u8, kR, kM, kU},
    {Reg::Moving, 46_u16, 1_u8, kR, kM, kU},
    {Reg::Lock, 47_u16, 1_u8, kRW, kM, kU},
    {Reg::Punch, 48_u16, 2_u8, kRW, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 35> kMx28Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 2_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 3_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 4_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 5_u16, 1_u8, kRW, kE, kU},
    {Reg::CwAngleLimit, 6_u16, 2_u8, kRW, kE, kU},
    {Reg::CcwAngleLimit, 8_u16, 2_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxTorque, 14_u16, 2_u8, kRW, kE, kU},
    {Reg::StatusReturnLevel, 16_u16, 1_u8, kRW, kE, kU},
    {Reg::AlarmLed, 17_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 18_u16, 1_u8, kRW, kE, kU},
    {Reg::MultiTurnOffset, 20_u16, 2_u8, kRW, kE, kS},
    {Reg::ResolutionDivider, 22_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 24_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 25_u16, 1_u8, kRW, kM, kU},
    {Reg::DGain, 26_u16, 1_u8, kRW, kM, kU},
    {Reg::IGain, 27_u16, 1_u8, kRW, kM, kU},
    {Reg::PGain, 28_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPosition, 30_u16, 2_u8, kRW, kM, kU},
    {Reg::MovingSpeed, 32_u16, 2_u8, kRW, kM, kSM},
    {Reg::TorqueLimit, 34_u16, 2_u8, kRW, kM, kU},
    {Reg::PresentPosition, 36_u16, 2_u8, kR, kM, kU},
    {Reg::PresentVelocity, 38_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentLoad, 40_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentVoltage, 42_u16, 1_u8, kR, kM, kU},
    {Reg::PresentTemperature, 43_u16, 1_u8, kR, kM, kU},
    {Reg::RegisteredInstruction, 44_u16, 1_u8, kR, kM, kU},
    {Reg::Moving, 46_u16, 1_u8, kR, kM, kU},
    {Reg::Lock, 47_u16, 1_u8, kRW, kM, kU},
    {Reg::Punch, 48_u16, 2_u8, kRW, kM, kU},
    {Reg::RealtimeTick, 50_u16, 2_u8, kR, kM, kU},
    {Reg::GoalAcceleration, 73_u16, 1_u8, kRW, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 38> kMx64Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 2_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 3_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 4_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 5_u16, 1_u8, kRW, kE, kU},
    {Reg::CwAngleLimit, 6_u16, 2_u8, kRW, kE, kU},
    {Reg::CcwAngleLimit, 8_u16, 2_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxTorque, 14_u16, 2_u8, kRW, kE, kU},
    {Reg::StatusReturnLevel, 16_u16, 1_u8, kRW, kE, kU},
    {Reg::AlarmLed, 17_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 18_u16, 1_u8, kRW, kE, kU},
    {Reg::MultiTurnOffset, 20_u16, 2_u8, kRW, kE, kS},
    {Reg::ResolutionDivider, 22_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 24_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 25_u16, 1_u8, kRW, kM, kU},
    {Reg::DGain, 26_u16, 1_u8, kRW, kM, kU},
    {Reg::IGain, 27_u16, 1_u8, kRW, kM, kU},
    {Reg::PGain, 28_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPosition, 30_u16, 2_u8, kRW, kM, kU},
    {Reg::MovingSpeed, 32_u16, 2_u8, kRW, kM, kSM},
    {Reg::TorqueLimit, 34_u16, 2_u8, kRW, kM, kU},
    {Reg::PresentPosition, 36_u16, 2_u8, kR, kM, kU},
    {Reg::PresentVelocity, 38_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentLoad, 40_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentVoltage, 42_u16, 1_u8, kR, kM, kU},
    {Reg::PresentTemperature, 43_u16, 1_u8, kR, kM, kU},
    {Reg::RegisteredInstruction, 44_u16, 1_u8, kR, kM, kU},
    {Reg::Moving, 46_u16, 1_u8, kR, kM, kU},
    {Reg::Lock, 47_u16, 1_u8, kRW, kM, kU},
    {Reg::Punch, 48_u16, 2_u8, kRW, kM, kU},
    {Reg::RealtimeTick, 50_u16, 2_u8, kR, kM, kU},
    {Reg::Current, 68_u16, 2_u8, kRW, kM, kO},
    {Reg::TorqueControlModeEnable, 70_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalTorque, 71_u16, 2_u8, kRW, kM, kSM},
    {Reg::GoalAcceleration, 73_u16, 1_u8, kRW, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 39> kMx106Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 2_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 3_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 4_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 5_u16, 1_u8, kRW, kE, kU},
    {Reg::CwAngleLimit, 6_u16, 2_u8, kRW, kE, kU},
    {Reg::CcwAngleLimit, 8_u16, 2_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxTorque, 14_u16, 2_u8, kRW, kE, kU},
    {Reg::StatusReturnLevel, 16_u16, 1_u8, kRW, kE, kU},
    {Reg::AlarmLed, 17_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 18_u16, 1_u8, kRW, kE, kU},
    {Reg::MultiTurnOffset, 20_u16, 2_u8, kRW, kE, kS},
    {Reg::ResolutionDivider, 22_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 24_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 25_u16, 1_u8, kRW, kM, kU},
    {Reg::DGain, 26_u16, 1_u8, kRW, kM, kU},
    {Reg::IGain, 27_u16, 1_u8, kRW, kM, kU},
    {Reg::PGain, 28_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPosition, 30_u16, 2_u8, kRW, kM, kU},
    {Reg::MovingSpeed, 32_u16, 2_u8, kRW, kM, kSM},
    {Reg::TorqueLimit, 34_u16, 2_u8, kRW, kM, kU},
    {Reg::PresentPosition, 36_u16, 2_u8, kR, kM, kU},
    {Reg::PresentVelocity, 38_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentLoad, 40_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentVoltage, 42_u16, 1_u8, kR, kM, kU},
    {Reg::PresentTemperature, 43_u16, 1_u8, kR, kM, kU},
    {Reg::RegisteredInstruction, 44_u16, 1_u8, kR, kM, kU},
    {Reg::Moving, 46_u16, 1_u8, kR, kM, kU},
    {Reg::Lock, 47_u16, 1_u8, kRW, kM, kU},
    {Reg::Punch, 48_u16, 2_u8, kRW, kM, kU},
    {Reg::RealtimeTick, 50_u16, 2_u8, kR, kM, kU},
    {Reg::Current, 68_u16, 2_u8, kRW, kM, kO},
    {Reg::TorqueControlModeEnable, 70_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalTorque, 71_u16, 2_u8, kRW, kM, kSM},
    {Reg::GoalAcceleration, 73_u16, 1_u8, kRW, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 50> kMx28v2Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::AccelerationLimit, 40_u16, 4_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 65_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentLoad, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 52> kMx64v2Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::CurrentLimit, 38_u16, 2_u8, kRW, kE, kU},
    {Reg::AccelerationLimit, 40_u16, 4_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 65_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalCurrent, 102_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentCurrent, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 31> kXl320Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 2_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 3_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 4_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 5_u16, 1_u8, kRW, kE, kU},
    {Reg::CwAngleLimit, 6_u16, 2_u8, kRW, kE, kU},
    {Reg::CcwAngleLimit, 8_u16, 2_u8, kRW, kE, kU},
    {Reg::ControlMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 14_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxTorque, 15_u16, 2_u8, kRW, kE, kU},
    {Reg::StatusReturnLevel, 17_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 18_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 24_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 25_u16, 1_u8, kRW, kM, kU},
    {Reg::DGain, 27_u16, 1_u8, kRW, kM, kU},
    {Reg::IGain, 28_u16, 1_u8, kRW, kM, kU},
    {Reg::PGain, 29_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPosition, 30_u16, 2_u8, kRW, kM, kU},
    {Reg::MovingSpeed, 32_u16, 2_u8, kRW, kM, kSM},
    {Reg::TorqueLimit, 35_u16, 2_u8, kRW, kM, kU},
    {Reg::PresentPosition, 37_u16, 2_u8, kR, kM, kU},
    {Reg::PresentVelocity, 39_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentLoad, 41_u16, 2_u8, kR, kM, kSM},
    {Reg::PresentVoltage, 45_u16, 1_u8, kR, kM, kU},
    {Reg::PresentTemperature, 46_u16, 1_u8, kR, kM, kU},
    {Reg::RegisteredInstruction, 47_u16, 1_u8, kR, kM, kU},
    {Reg::Moving, 49_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 50_u16, 1_u8, kR, kM, kU},
    {Reg::Punch, 51_u16, 2_u8, kRW, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 54> kX330Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::CurrentLimit, 38_u16, 2_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::StartupConfiguration, 60_u16, 1_u8, kRW, kE, kU},
    {Reg::PwmSlope, 62_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 65_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalCurrent, 102_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentCurrent, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
    {Reg::BackupReady, 147_u16, 1_u8, kR, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 51> kX430BasicTable{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::StartupConfiguration, 60_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 65_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentLoad, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
    {Reg::BackupReady, 147_u16, 1_u8, kR, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 53> kX430Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::CurrentLimit, 38_u16, 2_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::StartupConfiguration, 60_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 65_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalCurrent, 102_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentCurrent, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
    {Reg::BackupReady, 147_u16, 1_u8, kR, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 59> kX540Table{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::CurrentLimit, 38_u16, 2_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::ExternalPortMode1, 56_u16, 1_u8, kRW, kE, kU},
    {Reg::ExternalPortMode2, 57_u16, 1_u8, kRW, kE, kU},
    {Reg::ExternalPortMode3, 58_u16, 1_u8, kRW, kE, kU},
    {Reg::StartupConfiguration, 60_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::Led, 65_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalCurrent, 102_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentCurrent, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
    {Reg::BackupReady, 147_u16, 1_u8, kR, kM, kU},
    {Reg::ExternalPortData1, 152_u16, 2_u8, kRW, kM, kU},
    {Reg::ExternalPortData2, 154_u16, 2_u8, kRW, kM, kU},
    {Reg::ExternalPortData3, 156_u16, 2_u8, kRW, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<RegInfo, 52> kXwTable{{
    {Reg::ModelNumber, 0_u16, 2_u8, kR, kE, kU},
    {Reg::ModelInformation, 2_u16, 4_u8, kR, kE, kU},
    {Reg::FirmwareVersion, 6_u16, 1_u8, kR, kE, kU},
    {Reg::Id, 7_u16, 1_u8, kRW, kE, kU},
    {Reg::BaudRate, 8_u16, 1_u8, kRW, kE, kU},
    {Reg::ReturnDelayTime, 9_u16, 1_u8, kRW, kE, kU},
    {Reg::DriveMode, 10_u16, 1_u8, kRW, kE, kU},
    {Reg::OperatingMode, 11_u16, 1_u8, kRW, kE, kU},
    {Reg::SecondaryId, 12_u16, 1_u8, kRW, kE, kU},
    {Reg::ProtocolType, 13_u16, 1_u8, kRW, kE, kU},
    {Reg::HomingOffset, 20_u16, 4_u8, kRW, kE, kS},
    {Reg::MovingThreshold, 24_u16, 4_u8, kRW, kE, kU},
    {Reg::TemperatureLimit, 31_u16, 1_u8, kRW, kE, kU},
    {Reg::MaxVoltageLimit, 32_u16, 2_u8, kRW, kE, kU},
    {Reg::MinVoltageLimit, 34_u16, 2_u8, kRW, kE, kU},
    {Reg::PwmLimit, 36_u16, 2_u8, kRW, kE, kU},
    {Reg::CurrentLimit, 38_u16, 2_u8, kRW, kE, kU},
    {Reg::VelocityLimit, 44_u16, 4_u8, kRW, kE, kU},
    {Reg::MaxPositionLimit, 48_u16, 4_u8, kRW, kE, kU},
    {Reg::MinPositionLimit, 52_u16, 4_u8, kRW, kE, kU},
    {Reg::StartupConfiguration, 60_u16, 1_u8, kRW, kE, kU},
    {Reg::Shutdown, 63_u16, 1_u8, kRW, kE, kU},
    {Reg::TorqueEnable, 64_u16, 1_u8, kRW, kM, kU},
    {Reg::StatusReturnLevel, 68_u16, 1_u8, kRW, kM, kU},
    {Reg::RegisteredInstruction, 69_u16, 1_u8, kR, kM, kU},
    {Reg::HardwareErrorStatus, 70_u16, 1_u8, kR, kM, kU},
    {Reg::VelocityIGain, 76_u16, 2_u8, kRW, kM, kU},
    {Reg::VelocityPGain, 78_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionDGain, 80_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionIGain, 82_u16, 2_u8, kRW, kM, kU},
    {Reg::PositionPGain, 84_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward2ndGain, 88_u16, 2_u8, kRW, kM, kU},
    {Reg::Feedforward1stGain, 90_u16, 2_u8, kRW, kM, kU},
    {Reg::BusWatchdog, 98_u16, 1_u8, kRW, kM, kU},
    {Reg::GoalPwm, 100_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalCurrent, 102_u16, 2_u8, kRW, kM, kS},
    {Reg::GoalVelocity, 104_u16, 4_u8, kRW, kM, kS},
    {Reg::ProfileAcceleration, 108_u16, 4_u8, kRW, kM, kU},
    {Reg::ProfileVelocity, 112_u16, 4_u8, kRW, kM, kU},
    {Reg::GoalPosition, 116_u16, 4_u8, kRW, kM, kS},
    {Reg::RealtimeTick, 120_u16, 2_u8, kR, kM, kU},
    {Reg::Moving, 122_u16, 1_u8, kR, kM, kU},
    {Reg::MovingStatus, 123_u16, 1_u8, kR, kM, kU},
    {Reg::PresentPwm, 124_u16, 2_u8, kR, kM, kS},
    {Reg::PresentCurrent, 126_u16, 2_u8, kR, kM, kS},
    {Reg::PresentVelocity, 128_u16, 4_u8, kR, kM, kS},
    {Reg::PresentPosition, 132_u16, 4_u8, kR, kM, kS},
    {Reg::VelocityTrajectory, 136_u16, 4_u8, kR, kM, kS},
    {Reg::PositionTrajectory, 140_u16, 4_u8, kR, kM, kS},
    {Reg::PresentVoltage, 144_u16, 2_u8, kR, kM, kU},
    {Reg::PresentTemperature, 146_u16, 1_u8, kR, kM, kU},
    {Reg::BackupReady, 147_u16, 1_u8, kR, kM, kU},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on

} // namespace detail

/** @brief How a family encodes its Baud Rate register. */
enum class BaudScheme : uint8_t {
  None = 0,    ///< No baud register in the table (generic).
  Divider = 1, ///< Protocol 1.0 families: `2 000 000 / (value + 1)` bit/s.
  Table4 = 2,  ///< XL-320: values 0..3 for 9 600, 57 600, 115 200 and 1 000 000.
  Table7 = 3,  ///< X330: values 0..6 up to 4 000 000.
  Table8 = 4,  ///< X430, X540, XW, MX(2.0): values 0..7 up to 4 500 000.
};

/** @brief Protocol bits of `FamilyInfo::protocols`. */
inline constexpr uint8_t kProtocolBit1 = 0x01U; ///< The family speaks protocol 1.0.
inline constexpr uint8_t kProtocolBit2 = 0x02U; ///< The family speaks protocol 2.0.

/** @brief Static facts of a family (e-Manual "Control Table" and "Specifications" pages). */
struct FamilyInfo {
  const char* name;         ///< Display name.
  uint8_t protocols;        ///< `kProtocolBit1`, `kProtocolBit2` or both.
  BaudScheme baud_scheme;   ///< Encoding of the Baud Rate register.
  uint32_t position_counts; ///< Position counts per full turn the unit table gives (4096, or 1024
                            ///< for 300 degrees).
  uint32_t
      microdeg_per_count;  ///< Position unit in micro-degrees (0.29, 0.088 or 360/4096 degrees).
  uint32_t mrpm_per_count; ///< Velocity unit in milli-rpm.
  uint8_t indirect_blocks; ///< 0 none, 1 = 28 indirect pairs, 2 = 56 (the second block above
                           ///< address 255).
};

namespace detail {
// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<FamilyInfo, 13> kFamilyInfo{{
    {"Generic",   kProtocolBit1 | kProtocolBit2, BaudScheme::None,    0_u32,    0_u32,     0_u32, 0_u8},
    {"AX/RX/DX",  kProtocolBit1,                 BaudScheme::Divider, 1024_u32, 290000_u32, 111_u32, 0_u8},
    {"MX-28",     kProtocolBit1,                 BaudScheme::Divider, 4096_u32, 88000_u32,  114_u32, 0_u8},
    {"MX-64",     kProtocolBit1,                 BaudScheme::Divider, 4096_u32, 88000_u32,  114_u32, 0_u8},
    {"MX-106",    kProtocolBit1,                 BaudScheme::Divider, 4096_u32, 88000_u32,  114_u32, 0_u8},
    {"MX-28(2.0)", kProtocolBit2,                BaudScheme::Table8,  4096_u32, 87891_u32,  229_u32, 2_u8},
    {"MX-64/106(2.0)", kProtocolBit2,            BaudScheme::Table8,  4096_u32, 87891_u32,  229_u32, 2_u8},
    {"XL-320",    kProtocolBit2,                 BaudScheme::Table4,  1024_u32, 290000_u32, 111_u32, 0_u8},
    {"X330",      kProtocolBit1 | kProtocolBit2, BaudScheme::Table7,  4096_u32, 87891_u32,  229_u32, 1_u8},
    {"XL430/XC430", kProtocolBit1 | kProtocolBit2, BaudScheme::Table8, 4096_u32, 87891_u32, 229_u32, 2_u8},
    {"XM430/XH430/XD430", kProtocolBit1 | kProtocolBit2, BaudScheme::Table8, 4096_u32, 87891_u32, 229_u32, 2_u8},
    {"XM540/XH540/XD540", kProtocolBit1 | kProtocolBit2, BaudScheme::Table8, 4096_u32, 87891_u32, 229_u32, 2_u8},
    {"XW430/XW540", kProtocolBit1 | kProtocolBit2, BaudScheme::Table8, 4096_u32, 87891_u32, 229_u32, 2_u8},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on
} // namespace detail

/**
 * @brief The control table of a family.
 * @param family The family.
 * @return Its registers in address order; empty for `Family::Generic` and for a value that is not a
 * family.
 */
[[nodiscard]] constexpr std::span<const RegInfo> Table(Family family) noexcept {
  switch (family) {
  case Family::AxRxDx:
    return detail::kAxRxDxTable;
  case Family::Mx28:
    return detail::kMx28Table;
  case Family::Mx64:
    return detail::kMx64Table;
  case Family::Mx106:
    return detail::kMx106Table;
  case Family::Mx28v2:
    return detail::kMx28v2Table;
  case Family::Mx64v2:
    return detail::kMx64v2Table;
  case Family::Xl320:
    return detail::kXl320Table;
  case Family::X330:
    return detail::kX330Table;
  case Family::X430Basic:
    return detail::kX430BasicTable;
  case Family::X430:
    return detail::kX430Table;
  case Family::X540:
    return detail::kX540Table;
  case Family::Xw:
    return detail::kXwTable;
  case Family::Generic:
    break;
  default:
    break;
  }
  return {};
}

/**
 * @brief The facts of a family.
 * @param family The family; a value that is not a family gives `Generic`.
 * @return Its `FamilyInfo`.
 */
[[nodiscard]] constexpr const FamilyInfo& Info(Family family) noexcept {
  const auto index = static_cast<std::size_t>(detail::ToUnderlying(family));
  return (index < detail::kFamilyInfo.size()) ? detail::kFamilyInfo[index] : detail::kFamilyInfo[0];
}

/**
 * @brief Find a register of a family.
 * @param family The family.
 * @param reg The register name.
 * @return A pointer into the family's table, or `nullptr` when the family has no such register.
 */
[[nodiscard]] constexpr const RegInfo* Lookup(Family family, Reg reg) noexcept {
  const std::span<const RegInfo> table = Table(family);
  const auto found = std::find_if(table.begin(), table.end(),
                                  [reg](const RegInfo& info) noexcept { return info.reg == reg; });
  return (found != table.end()) ? &*found : nullptr;
}

/** @brief The area an indirect pair lives in. */
struct IndirectPair {
  uint16_t address_register; ///< Address of the indirect address entry.
  uint16_t data_register;    ///< Address of the indirect data byte.
};

/**
 * @brief The registers of indirect pair `index` (1-based): the indirect address (2 bytes) that
 * points at any control-table address, and the data byte that reads or writes through it
 * (X330: 1..28; others 1..56, where 29..56 lie above address 255 and need protocol 2.0).
 * @param family The family.
 * @param index 1 to 28 (X330) or 56.
 * @return The two addresses, or `{0, 0}` when the family has no such pair.
 */
[[nodiscard]] constexpr IndirectPair Indirect(Family family, uint32_t index) noexcept {
  const uint32_t blocks = Info(family).indirect_blocks;
  if ((index < 1U) || (index > (blocks * 28U))) {
    return IndirectPair{.address_register = 0_u16, .data_register = 0_u16};
  }
  if (index <= 28U) {
    return IndirectPair{.address_register = static_cast<uint16_t>(168U + ((index - 1U) * 2U)),
                        .data_register = static_cast<uint16_t>(224U + (index - 1U))};
  }
  return IndirectPair{.address_register = static_cast<uint16_t>(578U + ((index - 29U) * 2U)),
                      .data_register = static_cast<uint16_t>(634U + (index - 29U))};
}

/// Rates of the table baud scheme (X and similar families), indexed by the register value.
inline constexpr std::array<uint32_t, 8> kRates{9'600U,     57'600U,    115'200U,   1'000'000U,
                                                2'000'000U, 3'000'000U, 4'000'000U, 4'500'000U};

/**
 * @brief The Baud Rate register value for a bit rate.
 * @param family The family.
 * @param bits_per_second The wanted rate.
 * @param value Set to the register value on success.
 * @return true when the family's baud scheme offers exactly that rate (protocol 1.0 families: any
 *         `2 000 000 / (n + 1)` that is exact, n 0 to 254).
 */
[[nodiscard]] constexpr bool BaudValueFor(Family family, uint32_t bits_per_second,
                                          uint8_t& value) noexcept {
  std::size_t count = 0;
  switch (Info(family).baud_scheme) {
  case BaudScheme::Divider:
    for (uint32_t n = 0U; n <= 254U; ++n) {
      if (bits_per_second == (2'000'000U / (n + 1U))) {
        value = static_cast<uint8_t>(n);
        return true;
      }
    }
    return false;
  case BaudScheme::Table4:
    count = 4;
    break;
  case BaudScheme::Table7:
    count = 7;
    break;
  case BaudScheme::Table8:
    count = 8;
    break;
  case BaudScheme::None:
    break;
  default:
    break;
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (kRates[i] == bits_per_second) {
      value = static_cast<uint8_t>(i);
      return true;
    }
  }
  return false;
}

/**
 * @brief The bit rate a Baud Rate register value selects.
 * @param family The family.
 * @param value The register value.
 * @return The rate in bit/s, or 0 when the scheme has no such value.
 */
[[nodiscard]] constexpr uint32_t BaudRateOf(Family family, uint8_t value) noexcept {
  std::size_t count = 0;
  switch (Info(family).baud_scheme) {
  case BaudScheme::Divider:
    return (value <= 254U) ? (2'000'000U / (static_cast<uint32_t>(value) + 1U)) : 0U;
  case BaudScheme::Table4:
    count = 4;
    break;
  case BaudScheme::Table7:
    count = 7;
    break;
  case BaudScheme::Table8:
    count = 8;
    break;
  case BaudScheme::None:
    break;
  default:
    break;
  }
  return (static_cast<std::size_t>(value) < count) ? kRates[value] : 0U;
}

/** @brief One known model number: its family, name and the facts that differ per model. */
struct ModelInfo {
  uint16_t number;     ///< Model Number register (address 0, 2 bytes).
  Family family;       ///< Its control table.
  const char* name;    ///< Display name (several variants may share a number).
  uint32_t current_ua; ///< Current unit in micro-amperes per count; 0 when the table has no current
                       ///< register.
  uint8_t fast_min_firmware; ///< First firmware that has Fast Sync/Bulk Read and Control Table
                             ///< Backup; 0 = none documented.
};

namespace detail {
// clang-format off
// NOLINTBEGIN(modernize-use-designated-initializers) positional rows: one register or model per line
inline constexpr std::array<ModelInfo, 47> kModels{{
    {12_u16,   Family::AxRxDx,   "AX-12A",         0_u32,    0_u8},
    {300_u16,  Family::AxRxDx,   "AX-12W",         0_u32,    0_u8},
    {10_u16,   Family::AxRxDx,   "RX-10",          0_u32,    0_u8},
    {24_u16,   Family::AxRxDx,   "RX-24F",         0_u32,    0_u8},
    {28_u16,   Family::AxRxDx,   "RX-28",          0_u32,    0_u8},
    {64_u16,   Family::AxRxDx,   "RX-64",          0_u32,    0_u8},
    {113_u16,  Family::AxRxDx,   "DX-113",         0_u32,    0_u8},
    {116_u16,  Family::AxRxDx,   "DX-116",         0_u32,    0_u8},
    {117_u16,  Family::AxRxDx,   "DX-117",         0_u32,    0_u8},
    {29_u16,   Family::Mx28,     "MX-28",          0_u32,    0_u8},
    {360_u16,  Family::Mx28,     "MX-12W",         0_u32,    0_u8},
    {310_u16,  Family::Mx64,     "MX-64",          4500_u32, 0_u8},
    {320_u16,  Family::Mx106,    "MX-106",         4500_u32, 0_u8},
    {30_u16,   Family::Mx28v2,   "MX-28(2.0)",     0_u32,    0_u8},
    {311_u16,  Family::Mx64v2,   "MX-64(2.0)",     3360_u32, 0_u8},
    {321_u16,  Family::Mx64v2,   "MX-106(2.0)",    3360_u32, 0_u8},
    {350_u16,  Family::Xl320,    "XL-320",         0_u32,    0_u8},
    {1190_u16, Family::X330,     "XL330-M077",     1000_u32, 46_u8},
    {1200_u16, Family::X330,     "XL330-M288",     1000_u32, 46_u8},
    {1210_u16, Family::X330,     "XC330-T181",     1000_u32, 46_u8},
    {1220_u16, Family::X330,     "XC330-T288",     1000_u32, 46_u8},
    {1230_u16, Family::X330,     "XC330-M181",     1000_u32, 46_u8},
    {1240_u16, Family::X330,     "XC330-M288",     1000_u32, 46_u8},
    {1710_u16, Family::X330,     "XM335-T323",     1000_u32, 46_u8},
    {1060_u16, Family::X430Basic, "XL430-W250",    0_u32,    45_u8},
    {1070_u16, Family::X430Basic, "XC430-W150/T150BB", 0_u32, 45_u8},
    {1080_u16, Family::X430Basic, "XC430-W240/T240BB", 0_u32, 45_u8},
    {1090_u16, Family::X430Basic, "2XL430-W250",   0_u32,    0_u8},
    {1160_u16, Family::X430Basic, "2XC430-W250",   0_u32,    0_u8},
    {1000_u16, Family::X430,     "XH430-W350",     2690_u32, 45_u8},
    {1010_u16, Family::X430,     "XH430-W210",     2690_u32, 45_u8},
    {1040_u16, Family::X430,     "XH430-V350",     1340_u32, 45_u8},
    {1050_u16, Family::X430,     "XH430-V210",     1340_u32, 45_u8},
    {1020_u16, Family::X430,     "XM430-W350",     2690_u32, 45_u8},
    {1030_u16, Family::X430,     "XM430-W210",     2690_u32, 45_u8},
    {1001_u16, Family::X430,     "XD430-T350",     2690_u32, 45_u8},
    {1011_u16, Family::X430,     "XD430-T210",     2690_u32, 45_u8},
    {1100_u16, Family::X540,     "XH540-W270",     2690_u32, 45_u8},
    {1110_u16, Family::X540,     "XH540-W150",     2690_u32, 45_u8},
    {1140_u16, Family::X540,     "XH540-V270",     2690_u32, 45_u8},
    {1150_u16, Family::X540,     "XH540-V150",     2690_u32, 45_u8},
    {1120_u16, Family::X540,     "XM540-W270",     2690_u32, 45_u8},
    {1130_u16, Family::X540,     "XM540-W150",     2690_u32, 45_u8},
    {1101_u16, Family::X540,     "XD540-T270",     2690_u32, 45_u8},
    {1270_u16, Family::Xw,       "XW430-T200/T333", 2690_u32, 0_u8},
    {1170_u16, Family::Xw,       "XW540-T260",     2690_u32, 0_u8},
    {1180_u16, Family::Xw,       "XW540-T140",     2690_u32, 0_u8},
}};
// NOLINTEND(modernize-use-designated-initializers)
// clang-format on
} // namespace detail

/**
 * @brief Look a model number up in the catalogue.
 * @param number The Model Number register value (from a ping reply or a read of address 0).
 * @return The catalogue entry, or `nullptr` for a number the catalogue does not list.
 */
[[nodiscard]] constexpr const ModelInfo* FindModel(uint16_t number) noexcept {
  for (const ModelInfo& model : detail::kModels) {
    if (model.number == number) {
      return &model;
    }
  }
  return nullptr;
}

/**
 * @brief The family of a model number.
 * @param number The Model Number register value.
 * @return The family, or `Family::Generic` for an unlisted number.
 */
[[nodiscard]] constexpr Family FamilyOf(uint16_t number) noexcept {
  const ModelInfo* model = FindModel(number);
  return (model != nullptr) ? model->family : Family::Generic;
}

/**
 * @brief Whether the e-Manual lists Fast Sync Read, Fast Bulk Read and Control Table Backup for a
 * model at a firmware version. Advice only: the driver never refuses an instruction because of it.
 * @param number The Model Number register value.
 * @param firmware The Firmware Version register value.
 * @return true when the model is listed and `firmware` reaches the first documented version.
 */
[[nodiscard]] constexpr bool SupportsFastRead(uint16_t number, uint8_t firmware) noexcept {
  const ModelInfo* model = FindModel(number);
  return (model != nullptr) && (model->fast_min_firmware != 0U) &&
         (firmware >= model->fast_min_firmware);
}

namespace detail {
/// The last enumerator of `Kind`; with the others in the file that declares the enumeration
/// (MISRA C++:2023 Rule 6.2.3).
template <>
struct EnumLast<Kind> {
  static constexpr Kind kValue = Kind::Offset2048;
};
/// The last enumerator of `Reg` (KB-CPP20-026).
template <>
struct EnumLast<Reg> {
  static constexpr Reg kValue = Reg::ExternalPortData3;
};
/// The last enumerator of `Family`.
template <>
struct EnumLast<Family> {
  static constexpr Family kValue = Family::Xw;
};
} // namespace detail

} // namespace dynamixel

#endif // DYNAMIXEL_TABLES_HPP
