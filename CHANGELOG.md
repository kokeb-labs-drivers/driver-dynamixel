# Changelog

All notable changes to this repository are listed here. Versions follow [Semantic Versioning](https://semver.org/).
Entries under **Breaking** need source changes in code that used the affected API; any of them calls for a major
version bump (a minor bump before 1.0).

## [Unreleased]

### Added

- DYNAMIXEL bus-master driver: protocol 1.0 and 2.0 over a half-duplex UART, through a UART port you write.
- Ping, Read, Write, Reg Write, Action, Factory Reset, Reboot, Clear, Control Table Backup, Sync Read, Sync Write,
  Fast Sync Read, Bulk Read, Bulk Write and Fast Bulk Read, with one `DeviceReply` per servo.
- Control tables for 47 model numbers in 9 families, typed register access, `ReadFeedback()`, indirect addresses
  and baud schemes.
- Reply search that skips noise, echo and late packets; `LineFault` and `EchoMismatch` for wiring faults.
- A torque-off stop that is never refused, even when the input flush fails, and is sent three times when no answer can
  confirm it; `Reboot()` is sent the same way; destructive instructions behind `Config::allow_destructive`.
- `DisableTorqueAt()`: the same stop for a servo whose family is not in the catalogue.
- Typed calls and the stop refuse a family that does not speak the bus protocol, sending nothing.
- Host example, ESP32 example (UART in RS-485 half-duplex mode, not verified on hardware), two fuzz targets with
  corpora.
