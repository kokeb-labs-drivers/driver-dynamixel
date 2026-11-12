# DYNAMIXEL host example

The driver on your PC, with no hardware: the host port in
[`host_dynamixel_port.hpp`](host_dynamixel_port.hpp) connects the real driver to a simulated bus of two XM430
servos (the bus model the host tests use), and [`main.cpp`](main.cpp) finds the servos, moves one and reads both
with one Sync Read.

## Build and run

From the repository root, with CMake 3.16+ and a C++20 compiler:

```bash
cmake -S examples/host -B build-example
cmake --build build-example
./build-example/dynamixel_host_example
```

## Expected output

```text
DYNAMIXEL driver 0.1.0 on the host
dynamixel: initialised
servo 1: model 1020, firmware 46
servo 2: model 1020, firmware 46
servo 1: 31 C
servo 2: 25 C
torque off
```

The host tests build this folder too and run the example (CTest `dynamixel_host_example`), so CI fails if it stops
compiling or a call fails.

## Files

| File | What it is |
| --- | --- |
| [`main.cpp`](main.cpp) | The example: find the servos, move one, Sync Read, stop |
| [`host_dynamixel_port.hpp`](host_dynamixel_port.hpp) | The host port: UART hooks over the simulated bus, `DelayUs()` and `Log()` |
| [`CMakeLists.txt`](CMakeLists.txt) | Standalone project; adds the driver with `add_subdirectory(../..)` |
