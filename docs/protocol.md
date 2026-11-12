# Protocol

The two DYNAMIXEL packet protocols as the driver implements them: the packet layout, the byte stuffing, the CRC,
how a reply is found in the receive bytes and which instruction each protocol has.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    - Instruction packet layout: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 2 Instruction Packet, p. 1
    - Status packet, error byte and alert bit: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 3 Status Packet; 3.2 Error, p. 1
    - Response policy and the broadcast ID: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 3.4 Response Policy, p. 1
    - Packet process, header and byte stuffing: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 4 Packet Process, p. 1
    - Ping, Read, Write, Reg Write, Action: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.1 Ping to 5.5 Action, p. 1
    - Factory Reset, Reboot, Clear, Control Table Backup: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.6 Factory Reset to 5.9 Control Table Backup, p. 1
    - Sync Read, Sync Write, Fast Sync Read: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.10 Sync Read; 5.11 Sync Write; 5.12 Fast Sync Read, p. 1
    - Bulk Read, Bulk Write, Fast Bulk Read: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 5.13 Bulk Read; 5.14 Bulk Write; 5.15 Fast Bulk Read, p. 1
    - Protocol 1.0 packets and checksum: ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0 web page, no revision mark, 2 Instruction Packet; 2.6 Checksum; 3 Status Packet; 3.2 Checksum, p. 1
    - Protocol 1.0 instructions: ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0 web page, no revision mark, 4 Instruction Details (Ping to Bulk Read), p. 1
    - Protocol 1.0 error byte: ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0 web page, no revision mark, 3.1 Error, p. 1
    - CRC-16 of protocol 2.0 packets: ROBOTIS e-Manual, CRC Calculation web page, no revision mark, CRC Calculation, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

## One bus, one protocol

Every servo on a bus speaks the same protocol, chosen by `Config::protocol`. Protocol 2.0 is the one of the X
series, the MX(2.0) servos and the XL-320; protocol 1.0 is the one of the AX, RX, DX and MX (1.0 firmware)
servos. The X330, XL430, XM, XH, XD and XW servos can be set to either (their Protocol Type register). The driver
does not detect the protocol: a wrong choice shows as `Error::Timeout` on every call, because a servo drops a packet
it cannot parse.

## Protocol 2.0 packets

| Field | Bytes | Value |
| --- | --- | --- |
| Header | 4 | `FF FF FD 00` |
| ID | 1 | 0 to 252; 254 is the broadcast ID |
| Length | 2 | The bytes after this field: instruction, parameters, CRC |
| Instruction | 1 | See the table below; `55` in a status packet |
| Parameters | n | Stuffed |
| CRC | 2 | CRC-16, polynomial `0x8005`, initial value 0, low byte first, over everything before it |

A status packet carries the error byte and then the parameters. Bit 7 of the error byte is the alert (the servo has
a hardware error pending, see its Hardware Error Status register); bits 6 to 0 are the error number
(`Protocol2Error`).

**Byte stuffing.** The header must not occur inside a packet, so the sender inserts one `FD` after every `FF FF FD`
in the parameters, and the Length counts the inserted byte. `protocol::FrameWriter` stuffs while it builds a packet
and `protocol::Destuff()` removes it from a reply; an `FF FF FD` not followed by `FD` is a bad packet
(`Error::InvalidReply`). The driver sizes its buffer for the worst case: every third parameter byte stuffed.

## Protocol 1.0 packets

| Field | Bytes | Value |
| --- | --- | --- |
| Header | 2 | `FF FF` |
| ID | 1 | 0 to 253; 254 is the broadcast ID |
| Length | 1 | Parameters plus 2 |
| Instruction | 1 | In a status packet, the error byte |
| Parameters | n | 0 to 253 |
| Checksum | 1 | The complement of the low byte of the sum from the ID to the last parameter |

The error byte is a set of flags (`p1_error`). The driver splits them in two: the command faults (angle limit,
range, checksum, instruction) mean the servo did not carry out the instruction and give `Error::DeviceError`; the
conditions (input voltage, overheating, overload) report the servo's state next to valid data and are handled by
`Config::alert_policy`.

## Instructions

| Instruction | Code | Protocol | Driver call |
| --- | --- | --- | --- |
| Ping | `01` | 1.0, 2.0 | `Ping()`, `Identify()`, `BroadcastPing()`, `ScanIds()` |
| Read | `02` | 1.0, 2.0 | `Read()`, `ReadRegister()`, `ReadFeedback()` |
| Write | `03` | 1.0, 2.0 | `Write()`, `WriteRegister()`, `SetTorque()`, `SetGoalPosition()` |
| Reg Write | `04` | 1.0, 2.0 | `RegWrite()` |
| Action | `05` | 1.0, 2.0 | `Action()` |
| Factory Reset | `06` | 1.0, 2.0 | `FactoryReset()` |
| Reboot | `08` | 1.0, 2.0 | `Reboot()` |
| Clear | `10` | 2.0 | `Clear()` |
| Control Table Backup | `20` | 2.0 | `Backup()` |
| Sync Read | `82` | 2.0 | `SyncRead()` |
| Sync Write | `83` | 1.0, 2.0 | `SyncWrite()` |
| Fast Sync Read | `8A` | 2.0 | `FastSyncRead()` |
| Bulk Read | `92` | 1.0 (MX and X servos), 2.0 | `BulkRead()` |
| Bulk Write | `93` | 2.0 | `BulkWrite()` |
| Fast Bulk Read | `9A` | 2.0 | `FastBulkRead()` |

A call for an instruction the configured protocol does not have returns `Error::Unsupported` before anything is sent.

## Who answers

A servo answers according to its Status Return Level register (`StatusReturn`): a ping always, a read at level 1 and
2, every other instruction at level 2 only. `Config::status_return` is what the driver expects of the whole bus; a
call that needs an answer the level does not give returns `Unsupported` instead of waiting for a timeout. The
broadcast ID is answered only for a ping, a sync read and a bulk read (and by each servo in turn: the answers come in
ID order, one reply timeout apart for a broadcast ping, `Config::broadcast_timeout_us`). A write or an action on
the broadcast ID is sent and `Ok` means "sent".

## Finding the reply

The receive bytes can hold garbage, the echo of the request, or a late reply. `protocol::Scan()` looks for the first
valid status packet:

1. It skips bytes until a header. A partial header at the end is kept, because the next read may complete it.
2. A candidate needs a plausible length (it must fit the receive buffer) and, in protocol 2.0, the status
   instruction `55`.
3. A candidate whose CRC or checksum does not match is dropped by one byte, and the search goes on inside it, so a
   real packet that starts within damaged bytes is still found. Each such drop is counted in `Stats::crc_errors`.
4. A packet shorter than its length says is waited for, bounded by `Config::gap_timeout_us` per read.

A line that carries only `00` or only `FF` for a whole candidate is a stuck or floating line: `Error::LineFault`
(counted in `Stats::line_faults`), not a timeout, so the wiring is blamed and not the servo.

## Fast Sync Read and Fast Bulk Read

The servos answer back to back inside one status packet: the first one sends the header, its own part and a CRC,
and each following servo continues the same packet with its part (error byte, ID, data, CRC). The driver parses the
parts one by one, checks each CRC, and fills `DeviceReply` per ID. A packet that ends early still delivers the parts
that arrived whole and checked; the rest are `Timeout`. The e-Manual gives the layout of the parts, not what the
first servo puts in the packet header's ID field or the length when a listed servo is silent; the driver accepts
the header of any ID and re-reads the parts it can verify (see [Troubleshooting](troubleshooting.md) and the
open points in the [Contract](contract.md)). Use the plain `SyncRead()` where the servos' firmware does not list Fast
Sync Read (`SupportsFastRead()` is advice built from the e-Manual's firmware notes).
