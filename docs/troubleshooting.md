# Troubleshooting

What each error means on a servo bus, what to look at first, and how to read what the driver kept.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    - Status packet, error byte and alert bit: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 3 Status Packet; 3.2 Error, p. 1
    - Protocol 1.0 error byte: ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0 web page, no revision mark, 3.1 Error, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

## First, look at what the driver kept

Every call leaves evidence, with no extra I/O:

| Call | What it tells you |
| --- | --- |
| `GetLastRequest()` | The bytes of the last instruction packet, as put on the line (stuffed, with CRC) |
| `GetLastReply()` | The decoded error byte of the last status packet: error number, alert flag, protocol |
| `GetLatchedAlert()` | The first hardware alert seen since `ClearLatchedAlert()` |
| `GetStats()` | Transactions, retries, timeouts, CRC errors, skipped garbage bytes, line faults |

A growing `timeouts` with no `crc_errors` is silence; growing `garbage_bytes` is noise or a wrong rate; growing
`crc_errors` is damaged packets. Compare `GetLastRequest()` with what a logic analyser sees: a request that never
appears on the wire is the port, one that appears but gets no answer is the servo or the bus.

## Ping returns Timeout

Nothing answered, or the answer was not a packet the driver could read. In order of likelihood:

| Cause | Check |
| --- | --- |
| The servo has no power | The servo supply: 10.0 V to 14.8 V for the XM430-W350 |
| Wrong ID | Ping by ID with `ScanIds()` (protocol 1.0) or `BroadcastPing()` (protocol 2.0) |
| Wrong bit rate | The servo's Baud Rate register against `Config::baud`; X series ship at 57 600, AX at 1 000 000 |
| Wrong protocol | Servos of the X330, XL430, XM, XH and XD series can be set to either protocol; AX, RX, DX and MX 1.0 speak 1.0 |
| The line is not turned round | The direction pin or half-duplex mode: the reply must be heard ([Hardware](hardware.md#the-data-line)) |
| A reply deadline that is too short | Raise `Config::reply_timeout_us` ([Configuration](configuration.md)); a USB adapter needs `UsbAdapterPreset()` |
| TX and RX swapped | Cross them (a separate-line transceiver), or check the single-wire stage |

## Calls return LineFault

The line carried only `00` or only `FF` for a whole candidate packet, or did not go quiet within 16 flush rounds. The
wiring is at fault, not the servo: the data line is stuck low (a short, a servo with no power pulling it down, the
transmitter left enabled), stuck high or floating (no pull-up, the transmitter disabled and nothing driving it).
Check the direction control and the pull-up, then call `Resync()`.

## Calls return EchoMismatch

`Config::echo` is `Loopback` and the bytes read back are not the bytes sent: another device is talking at the same
time (two masters, a servo that answers a broadcast it should not), or the wiring does not loop back. If TX and RX
are not tied, set `EchoMode::None`.

## Calls return Crc, or garbage_bytes grows

Packets arrive damaged. Check, in this order: the bit rate (a rate error of more than 2 to 3 percent damages bytes),
the ground between the master and the servos, the length of the cable and its termination at a high rate, and a
port that shifts bytes (a `Read()` that returns more than it was given, a FIFO that overflows). A single `Crc` is
retried for a read; a write is not repeated, so a `Crc` on a write means the reply was damaged, not that the
instruction failed.

## Calls return InvalidReply

The bytes looked like a status packet but broke a rule: the length does not fit, the instruction is not a status
instruction, the parameter count is wrong for the read, or a protocol 2.0 packet has an unstuffed header. A servo of
another protocol on the bus produces this; so does a reply to an older request that arrived late. Call `Resync()` to
empty the line.

## Calls return DeviceError

The servo understood the packet and refused it. Read `GetLastReply()`:

| Protocol 2.0 error number | Meaning | Typical cause |
| --- | --- | --- |
| 1 Result Fail | The instruction could not be processed | Wrong state, a register not allowed now |
| 2 Instruction Error | Undefined instruction, or Action without Reg Write | A call the servo's firmware does not have |
| 3 CRC Error | The servo's CRC check failed | A damaged request |
| 4 Data Range Error | The value is outside the register's range | A goal beyond a limit |
| 5 Data Length Error | The data is shorter than the register | A write of the wrong size |
| 6 Data Limit Error | The value exceeds the register's limit | A goal above the servo's limit |
| 7 Access Error | Write to a read-only register, or a protected one | Writing the EEPROM area while the torque is on |

The most common one: a write to an EEPROM register (ID, baud rate, limits, operating mode) with the torque on. Turn
the torque off first (`SetTorque(servo, false)`).

## Calls return DeviceFault

The servo reported a hardware alert (bit 7 of the error byte in protocol 2.0) and `Config::alert_policy` is `Fail`.
Read `Reg::HardwareErrorStatus` for the cause: input voltage, overheating, motor encoder, electrical shock or
overload. With `AlertPolicy::Report` (the default) the call succeeds and the alert is latched: poll
`GetLatchedAlert()`.

## Calls return NotPermitted or Unsupported

| Error | Cause | Fix |
| --- | --- | --- |
| `NotPermitted` | A destructive instruction without `Config::allow_destructive` | Set it for that call on purpose ([API guide](api-guide.md#reset-a-servo)) |
| `Unsupported` | The instruction does not exist in the configured protocol, the family has no such register, the servo does not answer at the configured `status_return`, or the port has no `SetBaudRate()` | Check [Protocol](protocol.md#instructions) and [Control tables](control-tables.md) |
| `InvalidArgument` | A value does not fit the register, an ID is out of range, or the packet does not fit `FrameBytes` | Check the argument; raise `FrameBytes` for a very long sync read |

## A sync read returns Ok but a device is not Ok

That is by design: `Result` says the exchange worked, `DeviceReply::result` says what happened to each servo. A
servo that is `Timeout` did not answer (wrong ID, no power, or it was shifted out of the packet by an earlier
silent one); `Crc` means its part arrived damaged; `DeviceError` carries the servo's error byte. A packet that ends
early still delivers the parts that arrived whole.

## After a Reboot or Factory Reset nothing answers

The servo restarts and may come back with another ID or bit rate: a Factory Reset with `ResetOption::All` returns
the ID to 1 and the rate to the factory one. Use `ResetOption::KeepIdAndBaud` to keep them, and wait
`Config::reboot_wait_us` before the next call (the driver does).

## The servo does not move

| Check | Why |
| --- | --- |
| Torque is on | `Reg::TorqueEnable` is 0 at power-up; `SetTorque(servo, true)` |
| The operating mode | Position control is the factory mode of most families; a velocity or PWM mode ignores a goal position |
| The limits | A goal outside the min and max position limits is refused or clamped |
| The supply | A servo with a weak supply reboots under load: the voltage in `ReadFeedback()` sags |
