# API guide

The driver's API by task, with code that compiles: initialise, find servos, move and read them by register name,
read many at once, stop safely, handle errors and shut down.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    - Status packet, error byte and alert bit: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 3 Status Packet; 3.2 Error, p. 1
    - Ping, Read, Write, Reg Write, Action: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.1 Ping to 5.5 Action, p. 1
    - Factory Reset, Reboot, Clear, Control Table Backup: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.6 Factory Reset to 5.9 Control Table Backup, p. 1
    - Sync Read, Sync Write, Fast Sync Read: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.10 Sync Read; 5.11 Sync Write; 5.12 Fast Sync Read, p. 1
    - Bulk Read, Bulk Write, Fast Bulk Read: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.13 Bulk Read; 5.14 Bulk Write; 5.15 Fast Bulk Read, p. 1
    - Protocol 1.0 instructions: ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0 web page, no revision mark, 4 Instruction Details (Ping to Bulk Read), p. 1
    - Control table, X series (addresses, sizes, units): ROBOTIS e-Manual, XM430-W350 web page, no revision mark, 2 Control Table; 2.1 Control Table of EEPROM Area; 2.2 Control Table of RAM Area, p. 1
    - Control table, AX / RX / DX series (protocol 1.0): ROBOTIS e-Manual, AX-12A web page, no revision mark, 2 Control Table; 2.2 EEPROM Area; 2.3 RAM Area, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

## The shape of the API

| Group | Calls |
| --- | --- |
| Lifecycle | `EnsureInitialized()`, `IsInitialized()`, `Deinitialize()`, `Resync()`, `ApplyConfig()`, `GetConfig()` |
| Discovery | `Ping()`, `Identify()`, `BroadcastPing()`, `ScanIds()` |
| Raw registers | `Read()`, `Write()`, `RegWrite()`, `Action()` |
| Typed registers | `ReadRegister()`, `WriteRegister()`, `ReadFeedback()`, `SetTorque()`, `SetGoalPosition()` |
| Many servos | `SyncRead()`, `FastSyncRead()`, `BulkRead()`, `FastBulkRead()`, `SyncWrite()`, `BulkWrite()` |
| The safe stop | `DisableTorque()`, `DisableTorqueAll()`, `DisableTorqueEach()`, `DisableTorqueAt()` |
| Setup of a servo | `SetId()`, `SetServoBaudRate()`, `ChangeBusBaud()`, `SetStatusReturnLevel()` |
| Destructive | `FactoryReset()`, `Reboot()`, `Clear()`, `Backup()` |
| Diagnostics | `GetLastReply()`, `GetLatchedAlert()`, `ClearLatchedAlert()`, `GetStats()`, `ResetStats()`, `GetLastRequest()` |
| Tables | `Info()`, `FindModel()`, `FamilyOf()`, `Indirect()`, `BaudValueFor()` (free functions) |

Every call that uses the bus returns `Result<T>`: check `ok()` (or the `bool` conversion) and read `value`, or read
`error`. `Result` is `[[nodiscard]]`, so an unchecked call is a compiler warning. Every call blocks until its reply
is complete or its deadlines have passed ([why](architecture.md#why-there-is-no-non-blocking-api)).

## Initialise

```cpp
--8<-- "tests/docs/api_tasks.cpp:lifecycle"
```

The constructor stores the port and the configuration (`Config{}` unless you pass one) and does no I/O.
`EnsureInitialized()` is the one way to start the driver: it checks the configuration, brings the UART up through
your port and applies `Config::baud`. It sends nothing on the bus, so it succeeds with no servo connected.

Once it has succeeded, `EnsureInitialized()` returns `Ok` at once with no I/O, so a caller can call it before every
use. After a failure the driver stays uninitialised and the next call starts again from the beginning. Before it
succeeds every call that uses the bus returns `NotInitialized` and sends nothing; argument and permission errors
(`InvalidArgument`, `Unsupported`, `NotPermitted`) come first, and the torque-off calls are never refused
([Contract](contract.md#promises)).

## Find the servos

```cpp
--8<-- "tests/docs/api_tasks.cpp:discover"
```

`Ping(id)` answers with the model number and firmware; `PingInfo::family` is the control table that model uses
(`Family::Generic` for a model the catalogue does not list). `BroadcastPing()` asks every servo at once in protocol
2.0; the servos answer in ID order, each within `Config::broadcast_timeout_us`. Protocol 1.0 has no broadcast
answers: `ScanIds(first, last, found)` pings each ID in turn and takes a missing one as absent. A protocol 1.0 ping
does not return the model; `Identify(id)` pings and then reads the model number and firmware.

## Move and read by register name

```cpp
--8<-- "tests/docs/api_tasks.cpp:typed"
```

A `ServoRef` is an ID and a `Family`. The typed calls look up the register in the family's table, check that the
value fits and that the register is writable, encode it (little-endian, two's complement or sign-magnitude as the
family does) and send it; reads decode the same way. `ReadFeedback()` returns position, velocity, load or current,
voltage and temperature in one transfer. A family without the register returns `Unsupported`; a value that does not
fit returns `InvalidArgument`. Nothing is sent in either case.

For a register the table does not name, use the raw `Read()` and `Write()` with an address and bytes, on any family
including `Generic`. `RegWrite()` stores the write in the servo and `Action()` executes the stored writes of every
servo together, so a group of servos starts moving on the same packet.

## Read many servos at once

```cpp
--8<-- "tests/docs/api_tasks.cpp:sync-read"
```

`SyncRead()` asks a list of servos for the same registers; they answer one after another. `FastSyncRead()` (protocol
2.0, firmware that lists it: `SupportsFastRead()`) has them answer inside one packet. `BulkRead()` and
`FastBulkRead()` take a different address and length per servo. All four fill the caller's buffer, one slice per
servo in request order, and one `DeviceReply` per servo:

| `DeviceReply` | Meaning |
| --- | --- |
| `result` | `Ok`, or why this servo's data is not valid: `Timeout` (silent, or its part did not arrive), `Crc`, `DeviceIdMismatch`, `DeviceError`, `DeviceFault` |
| `error_byte` | The raw error byte, when a packet arrived |
| `alert` | The hardware alert flag of that servo |

The call's own `Result` says whether the exchange worked; it can be `Ok` while a servo is `Timeout`, and a
damaged packet still delivers the servos whose part was whole. Read the data only of the servos that are `Ok`.

`SyncWrite()` and `BulkWrite()` send one packet to many servos and get no answer; `Ok` means sent.

## Stop safely

```cpp
--8<-- "tests/docs/api_tasks.cpp:stop"
```

`DisableTorque(servo)`, `DisableTorqueAll(family)`, `DisableTorqueEach(servos)` and `DisableTorqueAt(id, address)`
write Torque Enable to 0. They are never refused for the driver's state: no initialisation is needed (they start the
port themselves and set the bus baud rate), no consent flag is checked, and a failing input flush does not stop the
write (the call then reports `Bus` after sending). Each servo gets up to three attempts: one is enough when a trusted
answer confirms it; when no answer can (status return level below 2, or the broadcast form, which no servo answers)
all three are sent, because one lost packet would leave the torque on, and `Ok` then means "sent", so confirm with a
read if the stop matters.

The only refusals, which send nothing, are an argument the bus protocol does not have (an ID out of range, an
address above 255 in protocol 1.0, a family that does not speak the bus protocol) and a port that cannot start.
`DisableTorqueEach()` tries every servo and returns only the first error; call `DisableTorque()` for each servo when
you need to know which one failed. A family with no torque register (`Generic`, a model the catalogue does not list)
is refused by `DisableTorque()` and `DisableTorqueAll()`, because the driver cannot know the address: use
`DisableTorqueAt(id, address)` with the Torque Enable address from that servo's own control table (64 on the X series,
24 on AX/RX/DX); it is the same stop path, so a `Generic` servo has the same safety stop. **Take the address from
the e-Manual's control table for the exact model and firmware.** The call writes one byte, 0, at whatever address you
give, and with ID 254 it does so to every servo on the bus: a wrong address can set every ID to 0, change the baud
rate or silence the servos (on an X-series servo, address 7 is the ID, 8 the baud rate and 68 the status return
level), and 24 is a different register on protocol 2.0 X-series servos than on AX/RX/DX. Safe uses are a torque
address read from the table of the servo in hand, one servo ID at a time, tested once on the bench. `Reboot()` is
sent the same way.

The time a stop can take is bounded, not short: in each attempt the first read waits the reply timeout and every
later read the gap timeout, for at most `4 * FrameBytes + 16` reads, and there are three attempts. A line that gives
one byte per read can therefore hold a stop for up to
`3 * (Config::reply_timeout_us + (4 * FrameBytes + 15) * Config::gap_timeout_us)` (the timeouts are held inside 100
microseconds to 1 second on this path). A test pins this formula. Use short timeouts when the stop must return quickly.

## Alerts

```cpp
--8<-- "tests/docs/api_tasks.cpp:alerts"
```

A servo with a hardware error pending sets the alert flag in its reply (bit 7 of the error byte in protocol 2.0;
the voltage, temperature and overload flags in protocol 1.0). With `AlertPolicy::Report` (the default) the call
succeeds with valid data and the alert is latched until `ClearLatchedAlert()`; with `AlertPolicy::Fail` the call
returns `DeviceFault` and no data. Read `Reg::HardwareErrorStatus` for the cause.

## Handle errors

```cpp
--8<-- "tests/docs/api_tasks.cpp:errors"
```

| Signal | Meaning | What to do |
| --- | --- | --- |
| `Error::Timeout` | No complete reply | The servo's power, ID, rate, protocol, the direction control; raise `reply_timeout_us` |
| `Error::LineFault` | The line is stuck or floating | Power, ground, the half-duplex stage; then `Resync()` |
| `Error::EchoMismatch` | The echo differs from the request | Another talker, or `Config::echo` is wrong |
| `Error::Crc`, `InvalidReply` | A damaged or malformed reply | The rate, the ground, the cable; `Resync()` |
| `Error::DeviceError` | The servo refused the instruction | `GetLastReply().error_byte`; often an EEPROM write with the torque on |
| `Error::DeviceFault` | A hardware alert with `AlertPolicy::Fail` | `Reg::HardwareErrorStatus` |
| `Error::NotPermitted` | A destructive call without consent | `Config::allow_destructive`, on purpose |
| `Error::Bus` | Your port reported a failure | The port and the UART |

Every value, with the calls that return it, is on the [Contract](contract.md#errors) page, and
[Troubleshooting](troubleshooting.md) goes through each. `dynamixel::ToString()` names an error for a log.

## Reset a servo

```cpp
--8<-- "tests/docs/api_tasks.cpp:reset"
```

Factory Reset, Control Table Restore (`Backup()` with `BackupOption::Restore`), `Clear()` and an ID write to every
servo (`SetId()` with the broadcast ID) change or erase what a servo keeps, so they return `NotPermitted` unless
`Config::allow_destructive` is set. A reset can change the ID and baud rate: `ResetOption::KeepIdAndBaud` keeps them.
After a reset or a reboot the driver waits `Config::reboot_wait_us` before the next call. `Backup()` with
`BackupOption::Store` is not behind that flag: it overwrites the servo's backup area but nothing the servo runs on.

`SetStatusReturnLevel()` adopts a lower level when the answer to its write is silent, because the servo stops
answering at the new level. A lost write looks the same, so the driver logs a warning in that case: read the register
back to verify.

## Diagnostics

For bring-up and fault finding the driver keeps what it saw (handbook KB-CPP-119, KB-CPP-129), with no I/O:

| Call | What it gives |
| --- | --- |
| `GetLastRequest()` | The bytes of the last instruction packet, as put on the line |
| `GetLastReply()` | The decoded error byte of the last status packet |
| `GetLatchedAlert()`, `ClearLatchedAlert()` | The first alert since the last clear |
| `GetStats()`, `ResetStats()` | Transactions, retries, timeouts, CRC errors, garbage bytes, line faults |

## Shut down

```cpp
--8<-- "tests/docs/api_tasks.cpp:deinit"
```

After `Deinitialize()` every call that uses the bus returns `NotInitialized` until the next `EnsureInitialized()`.
`Deinitialize()` sends nothing: the servos keep their state, so stop them first when they must not move.

Next: [Configuration](configuration.md).
