# Porting

Write the platform port: one class that brings the UART up and moves bytes, plus the half-duplex direction. This
page lists the hooks, the rules a port follows and how to check it.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    - Half duplex line and Tx/Rx direction: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 1.4 Half Duplex; 1.5 Tx, Rx Direction, p. 1
    - Byte to byte time and the packet gap: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 1.6 Byte to Byte Time, p. 1
    - Connector and half-duplex circuit, protocol 1.0 servos: ROBOTIS e-Manual, AX-12A web page, no revision mark, 5.2 Connector Information; 5.4 Communication Circuit, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

The driver never calls your SDK. It calls hooks on a class you derive from `dynamixel::UartInterface<Self>` (in
[`inc/dynamixel_uart_interface.hpp`](../inc/dynamixel_uart_interface.hpp)). The base uses CRTP and declares no hook
itself; the driver template is constrained by the `dynamixel::UartPort` concept, so calls are resolved at compile
time (no virtual functions, nothing to allocate) and a missing or misdeclared hook is a compile error at the point
of use.

## What you implement

| Method | Required | Contract |
| --- | --- | --- |
| `bool EnsureInitialized() noexcept` | yes | Bring the UART up at 8 data bits, no parity, 1 stop bit; return `true` at once afterwards. `false` makes the driver's `EnsureInitialized()` return `Bus` |
| `bool Write(const uint8_t* data, std::size_t len) noexcept` | yes | Switch the line to transmit, send all `len` bytes without gaps, wait until the last stop bit has left the shift register, switch back to receive; `false` on an error |
| `std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept` | yes | Return 1 to `max` bytes as soon as one is there, waiting at most `timeout_us` for the first; 0 when none arrived |
| `bool FlushInput() noexcept` | yes | Drop every received byte not returned by `Read()` yet |
| `void DelayUs(uint32_t us) noexcept` | yes | Wait at least `us` microseconds (reboot and baud change settle times) |
| `bool SetBaudRate(uint32_t baud) noexcept` | no | Reconfigure the UART; without it the port keeps its own rate |
| `void Log(LogLevel, const char* message) noexcept` | no | Print one line; `message` is plain text, never a format string |

## The port

```cpp title="my_port.hpp"
--8<-- "tests/docs/my_port.hpp:port"
```

Rules the port must follow:

- **Raw bytes only.** The driver builds the packets, checks their CRC and finds the reply; the port does not parse
  or retry.
- **`Read()` returns what is there.** It does not wait for `max` bytes: one byte is enough to return, and the driver
  calls it again for the rest of the packet. A `Read()` that returns more than `max` is a `Bus` error.
- **`Read()` rounds the timeout up.** On an RTOS the tick may be a millisecond: round `timeout_us` up to a tick, at
  least one, and never wait less than asked.
- **`Write()` sends the packet in one go.** A pause longer than 1.5 ms between two bytes of a protocol 2.0
  instruction (100 ms in protocol 1.0) makes the servo drop the packet. Do not send byte by byte from a slow loop.
- **`Write()` waits for the last stop bit** before it switches the line to receive. Returning when the transmit
  buffer is empty is too early: the last byte is still in the shift register, and cutting it off corrupts the CRC.
- **Do not let an interrupt delay the turnaround.** The servo answers after its Return Delay Time (500
  microseconds at the factory value); a port that switches back to receive late loses the start of the reply.
- **`FlushInput()` really flushes**: the driver calls it before each request so a stale byte cannot shift a reply.
- **Single wire, TX and RX tied:** leave the echo of the sent bytes in the receive FIFO and set
  `Config::echo = EchoMode::Loopback`; the driver reads it back and compares it.
- **8N1**, no flow control, at the rate of `Config::baud`.

## Bus requirements

The driver states what the servos need from the line as `constexpr` values in
[`inc/dynamixel_uart_interface.hpp`](../inc/dynamixel_uart_interface.hpp) (handbook KB-CPP-123). Check your port's
settings against them with `static_assert`, as the ESP32 port does:

| Constant | Value | Meaning (e-Manual, protocols 1.0 and 2.0) |
| --- | --- | --- |
| `kMaxBaudRate` | 4 500 000 | The fastest rate of the catalogued families (the P series goes to 10.5 Mbit/s) |
| `kFactoryBaudRate` | 57 600 | Factory rate of the X series |
| `kTurnaroundUs` | 500 | Return Delay Time at its factory value (250 counts of 2 microseconds) |
| `kMaxTurnaroundUs` | 508 | Largest Return Delay Time (254 counts) |
| `kInstructionByteGapMaxUsV2` | 1 500 | Longest pause inside a protocol 2.0 instruction packet |
| `kInstructionByteGapMaxUsV1` | 100 000 | Longest pause inside a protocol 1.0 instruction packet |

## Direction control

All servos share one data line, so the master must stop driving it while a servo answers. Pick one:

| Hardware | Port |
| --- | --- |
| UART with a hardware half-duplex mode (ESP32, many STM32 parts) | Enable it; `Write()` is the plain send plus the wait for the transmitter |
| A direction GPIO or RTS on an RS-485 transceiver or a tri-state buffer | `Write()` raises it, sends, waits for the shift register and lowers it |
| TX and RX tied to one wire with an open-drain stage | Nothing to switch; the echo comes back, so set `Config::echo = EchoMode::Loopback` |

See [Hardware](hardware.md#the-data-line) for the circuits.

## Optional hooks

The driver finds the optional hooks at compile time with C++20 `requires`, in one place (`dynamixel::detail`), and
checks the exact signature. A missing hook falls back in a documented, safe way; a hook declared with the wrong
type does not compile.

| Hook | Used for | Without it |
| --- | --- | --- |
| `SetBaudRate()` | `EnsureInitialized()`, `ApplyConfig()` and `ChangeBusBaud()` set the UART rate | The port's own rate stays; `ChangeBusBaud()` returns `Unsupported` |
| `Log()` | One line per lifecycle event or failure | Nothing is logged |

A port with both hooks:

```cpp
--8<-- "tests/docs/ports.cpp:full-port"
```

A `SetBaudRate()` that returns `void`, or lacks `noexcept`, is a compile error: the driver must see a failed
reconfiguration. The [compile-fail tests](testing.md) check this.

## Reference implementations

Every port lives with its example, in `examples/<platform>/`; the driver has none. [Platforms](platforms.md) lists
them and how to add one.

- [`examples/host/host_dynamixel_port.hpp`](../examples/host/host_dynamixel_port.hpp): any PC, against a simulated
  bus. Start from it for a host build or a test bench; it is also the smallest complete port.
- [`examples/esp32/main/esp32_dynamixel_port.hpp`](../examples/esp32/main/esp32_dynamixel_port.hpp): ESP-IDF, the UART
  in RS-485 half-duplex mode with both hooks. Start from it for any ESP-IDF target. Not verified on hardware.

## Check your port

1. Build it: the ports on this page are compiled by the host CI from [`tests/docs/`](../tests/docs/my_port.hpp),
   with the driver instantiated over them.
2. Call `EnsureInitialized()`: `Ok` proves the UART came up.
3. Call `Ping()` on a servo whose ID you know: `Ok` proves that writes and reads work and the direction control
   turns the line round. `Timeout` is the servo's ID, rate, protocol or power; `LineFault` or `EchoMismatch` is
   the wiring ([Troubleshooting](troubleshooting.md)).

## Threads

The driver is not thread-safe. Give each driver object one owner task, or guard every call with one mutex. When
several driver objects share a UART, the port must also serialise the bus. See [Contract](contract.md#threading).

## Editor setup

The clang-format and clang-tidy configuration is in `_config/`, and the repository root has no copy of it. Tools
that take a path use it directly; CI does the same:

| Tool | Setting |
| --- | --- |
| Command line | `clang-format --style=file:_config/.clang-format`, `clang-tidy --config-file=_config/.clang-tidy -p build` |
| VS Code (C/C++ extension) | `"C_Cpp.clang_format_style": "file:${workspaceFolder}/_config/.clang-format"` and `"C_Cpp.codeAnalysis.clangTidy.args": ["--config-file=${workspaceFolder}/_config/.clang-tidy"]` |
| clangd (VS Code clangd, Neovim, Emacs), CLion | These find `.clang-format` and `.clang-tidy` only by name, from the file upwards: copy both to the root once (`cp _config/.clang-format _config/.clang-tidy .`). `.gitignore` lists the copies, so they never reach a commit; copy again after `_config/` changes |

Configure with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` (clangd: `--compile-commands-dir=build`) so the editor sees the
same flags as the build.
