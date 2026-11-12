# Datasheet map

The driver follows the DYNAMIXEL datasheet. This page says which revision, where to get it, and where each part of
the driver is described in it, so you can check any behaviour against the source. Section names and page numbers
are the datasheet's own (the printed page numbers).


<!-- generated: datasheet-map -->
- **Protocol specification: DYNAMIXEL Protocol 2.0 (e-Manual)**, ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0, revision web
  page, no revision mark (2026-11-12). Get it from the [vendor page](https://emanual.robotis.com/docs/en/dxl/protocol2/).
  SHA-256 of the file used: `e0b32dde396a855b001bf3ff265410a0d5e40b7c8c16679834f80f0686d3ca4a`.
- **Protocol specification: DYNAMIXEL Protocol 1.0 (e-Manual)**, ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0, revision web
  page, no revision mark (2026-11-12). Get it from the [vendor page](https://emanual.robotis.com/docs/en/dxl/protocol1/).
  SHA-256 of the file used: `8d65b87322f05e57feac86cfd824c507b99078003d15f5c332c25d6e27b6baeb`.
- **Protocol specification: CRC Calculation (e-Manual)**, ROBOTIS e-Manual, CRC Calculation, revision web page, no
  revision mark (2026-11-12). Get it from the [vendor page](https://emanual.robotis.com/docs/en/dxl/crc/).
  SHA-256 of the file used: `b69cb7d863877004f6dd9f299b7c05673cfc1dceedbfe6a590b527bfdedbaed4`.
- **Servo manual: XM430-W350 (e-Manual): the X series reference**, ROBOTIS e-Manual, XM430-W350, revision web page, no
  revision mark (2026-11-12). Get it from the [vendor page](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/).
  SHA-256 of the file used: `38038f5b32cbbb7e3c26570658855986be33589b06eb6d15e6259db338caafd3`.
- **Servo manual: AX-12A (e-Manual): the protocol 1.0 reference**, ROBOTIS e-Manual, AX-12A, revision web page, no
  revision mark (2026-11-12). Get it from the [vendor page](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/).
  SHA-256 of the file used: `b1314eb58659d1f0a179dcfe23eefb9af429521a947401cb601da091388c2520`.
- **Servo manual: XL-320 (e-Manual)**, ROBOTIS e-Manual, XL-320, revision web page, no revision mark (2026-11-12). Get
  it from the [vendor page](https://emanual.robotis.com/docs/en/dxl/x/xl320/).
  SHA-256 of the file used: `40b018bb1610de0ec1939e51c5011623f9de57adf771eb0be013e2af20dec622`.

| Topic | Datasheet section | Page | In the driver | Explained on |
| --- | --- | --- | --- | --- |
| Half duplex line and Tx/Rx direction | e-Manual, DYNAMIXEL Protocol 2.0: 1.4 Half Duplex; 1.5 Tx, Rx Direction | p. 1 | `UartInterface::Write()`, `Config::echo`, `EchoMode::Loopback` | [Hardware](hardware.md), [Porting](porting.md) |
| Byte to byte time and the packet gap | e-Manual, DYNAMIXEL Protocol 2.0: 1.6 Byte to Byte Time | p. 1 | `kInstructionByteGapMaxUsV2`, `Config::gap_timeout_us` | [Porting](porting.md), [Configuration](configuration.md), [Quickstart](quickstart.md) |
| Instruction packet layout | e-Manual, DYNAMIXEL Protocol 2.0: 2 Instruction Packet | p. 1 | `protocol::FrameWriter`, `DYNAMIXEL::Read()`, `Write()` | [Protocol](protocol.md), [Architecture](architecture.md) |
| Status packet, error byte and alert bit | e-Manual, DYNAMIXEL Protocol 2.0: 3 Status Packet; 3.2 Error | p. 1 | `Protocol2Error`, `Error::DeviceError`, `Error::DeviceFault`, `GetLastReply()` | [API guide](api-guide.md), [Troubleshooting](troubleshooting.md), [Protocol](protocol.md) |
| Response policy and the broadcast ID | e-Manual, DYNAMIXEL Protocol 2.0: 3.4 Response Policy | p. 1 | `Config::status_return`, `StatusReturn`, `BroadcastPing()` | [Configuration](configuration.md), [Protocol](protocol.md) |
| Packet process, header and byte stuffing | e-Manual, DYNAMIXEL Protocol 2.0: 4 Packet Process | p. 1 | `protocol::Scan()`, `protocol::Destuff()`, `DYNAMIXEL::Resync()` | [Protocol](protocol.md), [Architecture](architecture.md), [Contract](contract.md) |
| Ping, Read, Write, Reg Write, Action | e-Manual, DYNAMIXEL Protocol 2.0: 5.1 Ping to 5.5 Action | p. 1 | `Ping()`, `Read()`, `Write()`, `RegWrite()`, `Action()` | [API guide](api-guide.md), [Protocol](protocol.md) |
| Factory Reset, Reboot, Clear, Control Table Backup | e-Manual, DYNAMIXEL Protocol 2.0: 5.6 Factory Reset to 5.9 Control Table Backup | p. 1 | `FactoryReset()`, `Reboot()`, `Clear()`, `Backup()`, `Config::allow_destructive` | [API guide](api-guide.md), [Protocol](protocol.md), [Contract](contract.md) |
| Sync Read, Sync Write, Fast Sync Read | e-Manual, DYNAMIXEL Protocol 2.0: 5.10 Sync Read; 5.11 Sync Write; 5.12 Fast Sync Read | p. 1 | `SyncRead()`, `SyncWrite()`, `FastSyncRead()` | [API guide](api-guide.md), [Protocol](protocol.md) |
| Bulk Read, Bulk Write, Fast Bulk Read | e-Manual, DYNAMIXEL Protocol 2.0: 5.13 Bulk Read; 5.14 Bulk Write; 5.15 Fast Bulk Read | p. 1 | `BulkRead()`, `BulkWrite()`, `FastBulkRead()` | [API guide](api-guide.md), [Protocol](protocol.md) |
| Protocol 1.0 packets and checksum | e-Manual, DYNAMIXEL Protocol 1.0: 2 Instruction Packet; 2.6 Checksum; 3 Status Packet; 3.2 Checksum | p. 1 | `protocol::Checksum1()`, `protocol::FrameWriter`, `Protocol::V1` | [Protocol](protocol.md), [Architecture](architecture.md) |
| Protocol 1.0 instructions | e-Manual, DYNAMIXEL Protocol 1.0: 4 Instruction Details (Ping to Bulk Read) | p. 1 | `Ping()`, `Read()`, `Write()`, `RegWrite()`, `Action()`, `FactoryReset()`, `Reboot()`, `SyncWrite()`, `BulkRead()` | [API guide](api-guide.md), [Protocol](protocol.md) |
| Protocol 1.0 error byte | e-Manual, DYNAMIXEL Protocol 1.0: 3.1 Error | p. 1 | `p1_error`, `Error::DeviceError`, `Error::DeviceFault` | [Troubleshooting](troubleshooting.md), [Protocol](protocol.md) |
| CRC-16 of protocol 2.0 packets | e-Manual, CRC Calculation: CRC Calculation | p. 1 | `protocol::Crc16()`, `Error::Crc` | [Protocol](protocol.md), [Architecture](architecture.md) |
| Control table, X series (addresses, sizes, units) | e-Manual, XM430-W350: 2 Control Table; 2.1 Control Table of EEPROM Area; 2.2 Control Table of RAM Area | p. 1 | `Reg`, `Lookup()`, `Family::X430`, `ReadRegister()`, `WriteRegister()` | [Control tables](control-tables.md), [API guide](api-guide.md), [Quickstart](quickstart.md) |
| Specifications, baud rates and ID range | e-Manual, XM430-W350: 1 Specifications | p. 1 | `kMaxBaudRate`, `kMaxIdV2`, `Info()` | [Hardware](hardware.md), [Control tables](control-tables.md) |
| Connector, power and communication circuit | e-Manual, XM430-W350: 4.5 Connector Information; 4.6 Communication Circuit | p. 1 | `UartInterface`, `EchoMode` | [Hardware](hardware.md), [Platforms](platforms.md) |
| Control table, AX / RX / DX series (protocol 1.0) | e-Manual, AX-12A: 2 Control Table; 2.2 EEPROM Area; 2.3 RAM Area | p. 1 | `Family::AxRxDx`, `Lookup()`, `LegacyBusPreset()` | [Control tables](control-tables.md), [API guide](api-guide.md) |
| Connector and half-duplex circuit, protocol 1.0 servos | e-Manual, AX-12A: 5.2 Connector Information; 5.4 Communication Circuit | p. 1 | `EchoMode::Loopback`, `LegacyBusPreset()` | [Hardware](hardware.md), [Porting](porting.md) |
| Control table, XL-320 | e-Manual, XL-320: 2 Control Table; 2.2 EEPROM Area; 2.3 RAM Area | p. 1 | `Family::Xl320`, `Lookup()` | [Control tables](control-tables.md) |
<!-- /generated -->

## When the datasheet and the driver differ

The driver is written against the revision above. If you have a newer revision, compare its revision history with
the topics above; a change to a pin, a register, a timing or a value the driver uses is a bug report for this
repository. Where the driver deliberately goes beyond the datasheet, the [Contract](contract.md) says so.

The repository never contains the PDF itself: vendors own their documents. Download it from the vendor, and compare
its SHA-256 with the one above to confirm you have the same file.
