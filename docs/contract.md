# Contract

What the DYNAMIXEL driver promises, what it needs, and the rules a caller follows.

The driver inherits the [Kokeb Labs driver contract](https://github.com/kokeb-labs/.github/blob/main/standards/driver.md);
this page states only what is specific to it. A change to anything here is a breaking change, listed under
**Breaking** in the [changelog](changelog.md).

## Promises

- The constructor is `noexcept`, stores the configuration, does no I/O and cannot fail.
- `EnsureInitialized()` is the only way to start the driver. Its first call checks the stored configuration
  (`InvalidArgument`, nothing sent), brings the port up and applies `Config::baud` through `SetBaudRate()` when the
  port has it. It sends nothing on the bus. On any failure the driver stays uninitialised and the next call starts
  again from the beginning; once it has succeeded, later calls return `Ok` with no I/O.
- Before `EnsureInitialized()` succeeds, every call that would use the bus fails and sends nothing. The error is
  `NotInitialized`, except that an argument or permission problem is reported first: `InvalidArgument`,
  `Unsupported` (the protocol or family has no such call) and `NotPermitted` (a destructive call without
  `Config::allow_destructive`) do not depend on the state of the bus, so they are returned whether or not the
  driver is up. In all of these cases nothing is sent.
- The torque-off calls `DisableTorque()`, `DisableTorqueAll()`, `DisableTorqueEach()` and `DisableTorqueAt()`, and
  `Reboot()`, are never
  refused for the driver's state: they start the port themselves when needed (and set the bus baud rate when the
  driver was not initialised), need no consent flag, and write even when the input flush fails (the fault is
  reported as `Bus` after the write). A torque-off is sent up to three times per servo: once when a trusted answer
  confirms it, three times when no answer can confirm it (status return level below 2, or the broadcast) or when the
  flush failed. They are the stop an emergency handler can call from any state. The only refusals are an argument
  the bus protocol does not have (`InvalidArgument`: an ID out of range, a family that does not speak the bus
  protocol, an address above 255 in protocol 1.0, or a family with no torque register such as `Generic`: use
  `DisableTorqueAt()` with the servo's own Torque Enable address) and a port that cannot start. `DisableTorqueEach()`
  returns only the first error. On an invalid configuration they still run, with the timeouts held inside 100
  microseconds to 1 second and the reboot wait inside 10 seconds.
- A typed call (`ReadRegister()`, `WriteRegister()`, `SetTorque()`, `ReadFeedback()`, the stop) for a family that does
  not speak the bus protocol (for example an AX/RX/DX servo on a protocol 2.0 bus) is refused before anything is sent,
  because the same register name sits at another address in the other table.
- Every port failure propagates as `Error::Bus`; no call returns a stale or zero value after a failed read, and a
  failed `Result<T>` always carries `T{}`.
- Every wait is bounded: a reply by `Config::reply_timeout_us` for the first byte and `Config::gap_timeout_us` per
  further read, a broadcast ping by `Config::broadcast_timeout_us` per reply, a reboot by `Config::reboot_wait_us`.
  Each receive is also bounded by a number of reads (`4 * FrameBytes + 16`), and a broadcast ping by one reply per
  ID, so no call can block forever, whatever the line carries (the fuzz targets check this with endless noise). The
  time bound is therefore steps times timeout, not one timeout: a line that gives one byte per read can hold a
  call for up to `Config::reply_timeout_us + (4 * FrameBytes + 15) * Config::gap_timeout_us` (about 2 000 s at the
  1 s limit and the default 512-byte buffer), a stop for three times that, and a broadcast ping for that many times
  the reply count. Use short timeouts on a bus whose line you do
  not trust.
- A write is sent once. A ping or read is repeated up to `Config::retries` times after a missing, corrupted or
  invalid reply; nothing else is repeated, except the torque-off calls above.
- `Config{}` is a valid configuration for a protocol 2.0 bus at 57 600 bit/s.
- A missing, misdeclared or `noexcept`-less required hook is a compile error; a missing `SetBaudRate()` gives
  `Unsupported` from `ChangeBusBaud()`; a missing `Log()` logs nothing.
- A multi-servo read (`SyncRead()`, `FastSyncRead()`, `BulkRead()`, `FastBulkRead()`) fills one `DeviceReply` per
  requested ID. When the call returns `Ok`, each device that is `Ok` has valid data in its slice of the output;
  the slices of the others are not meaningful.
- The driver never sends a destructive instruction (Factory Reset, Control Table Restore, Clear, an ID write to
  every servo) unless `Config::allow_destructive` is set.
- The driver sends a packet of at most `FrameBytes` bytes; `GetLastRequest()` returns the bytes as put on the line.

## Depends on

| Dependency | Version | Why |
| --- | --- | --- |
| A C++20 compiler | GCC 10, Clang 13 or later | Concepts constrain the port |
| CMake | 3.16 or later | Generates `dynamixel_version.h` |
| Your port | | A UART at 8N1 with the half-duplex direction handled ([Porting](porting.md)), a delay, optionally the baud rate and logging |

Nothing else: no RTOS, no SDK, no other repository of the org.

## Threading

The driver is not thread-safe and takes no lock. Use each driver object from one task at a time, or guard every
call on it with one mutex; the layer that owns the driver owns that lock, and the bus lock when several driver
objects share a UART. Nothing may run in an ISR: every call can block for a bus transfer or a delay. Separate
objects share no state.

## Errors

`dynamixel::Error` inside `dynamixel::Result<T>`. Values 0 to 9 are the shared base set; values from 16 are specific
to the bus.

| `Error` | Value | Returned when |
| --- | --- | --- |
| `Ok` | 0 | The call succeeded |
| `NotInitialized` | 1 | `EnsureInitialized()` has not succeeded, or `Deinitialize()` ran since |
| `InvalidArgument` | 2 | A `Config` field or argument is out of range, or the packet does not fit `FrameBytes`; nothing was sent |
| `Bus` | 3 | A port hook returned `false`, or `Read()` returned more bytes than asked |
| `Timeout` | 4 | No complete reply within the deadline (nothing, or a truncated packet) |
| `DeviceIdMismatch` | 5 | The reply came from another ID than the one addressed, or the model differs |
| `Crc` | 6 | The CRC-16 (protocol 2.0) or checksum (protocol 1.0) of the reply did not match |
| `Busy` | 7 | Reserved by the shared set; this driver has no pending state and never returns it |
| `DeviceFault` | 8 | The servo reported a hardware alert and `AlertPolicy::Fail` is set |
| `Unsupported` | 9 | The protocol, family, register or port hook does not offer this call |
| `DeviceError` | 16 | The servo rejected the instruction; `GetLastReply()` has the error number |
| `InvalidReply` | 17 | A reply the protocol refuses: header, length, instruction or parameter count |
| `EchoMismatch` | 18 | With `EchoMode::Loopback`, the sent bytes were not read back as sent |
| `LineFault` | 19 | The line carried only `00` or only `FF`, or did not go quiet: stuck, floating |
| `NotPermitted` | 22 | A destructive instruction without `Config::allow_destructive` |

The [API guide](api-guide.md#handle-errors) says what to do about each.

## Memory

No heap, no exceptions, no RTTI, no `std::function`, no `std::string`, no `printf` and no static mutable state. A
driver object holds a reference to the port, a `Config`, two buffers of `FrameBytes` bytes (512 by default), a
64-byte scratch buffer, the last reply, the latched alert and the statistics: about 1.2 KB at the default size.
The largest stack buffer of a call is 5 bytes; the multi-servo reads write into the caller's buffers.

## Platform

Platform-free: the same code runs on any target with a C++20 compiler, including the host tests. The ESP-IDF port
in [`examples/esp32`](examples.md) is an example, not part of the driver.

## Open points

The e-Manual does not specify four behaviours that the driver has to decide. They are checked against the bus
model, not against a servo; confirm them on hardware before relying on them:

| Point | What the driver does |
| --- | --- |
| The ID in the header of the first part of a Fast Sync Read or Fast Bulk Read | Accepts any ID in the header and verifies the parts one by one |
| The ID in the reply to a write of the ID register | Accepts the old and the new ID |
| How a silent servo shifts the replies of the servos after it in a Sync Read | Matches every reply to its ID: a reply that belongs to a later servo marks the earlier ones `Timeout`, and the read goes on |
| The reply deadline | The driver's own: `Config::reply_timeout_us`, 20 ms by default |

## Deviations from the standard

None.
