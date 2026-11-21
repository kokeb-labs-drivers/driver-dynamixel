# Figures

<!-- Written by kokeb-docs figures from docs/diagrams/figures/*.py. Do not edit by hand. -->

Every figure on this site is drawn by a Python module from facts extracted from the code, so it is rebuilt,
not redrawn, when the code changes. CI runs `kokeb-docs figures --check` and fails when a
committed figure no longer matches the code. Each figure below lists the files it is drawn from.

## Inside the DYNAMIXEL

<!-- figure: chip-blocks -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-blocks.dark.svg">
  <img src="chip-blocks.svg"
    alt="Block diagram of the DYNAMIXEL: its function blocks, how they connect and which package pins reach
    each block.">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## What the DYNAMIXEL does

<!-- figure: chip-capabilities -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-capabilities.dark.svg">
  <img src="chip-capabilities.svg"
    alt="Capability map of the DYNAMIXEL: Position control (supported): Moves to a goal position in counts,
    within the position limits and the profile velocity and acceleration; Velocity, current and PWM
    modes (supported): Selects the operating mode and sets a velocity, current or PWM goal, by register
    name with units; Feedback (supported): Reads position, velocity, load or current, voltage and
    temperature in one call; Torque enable (supported): Switches the motor on or off; the torque-off
    stop is always available, even with guards on; Find servos (supported): Pings one ID or the whole
    bus, and reads each servo's model number and firmware; Many servos at once (supported): Reads or
    writes the same registers of many servos in one transaction, with an outcome for each servo; ID,
    baud rate and status level (supported): Gives a servo its ID and baud rate, moves the bus to match,
    and sets which replies a servo returns; Reboot, reset and backup (guarded): Reboots a servo, resets
    its control table, clears the multi-turn count and stores or restores a backup; Hardware errors
    (supported): Decodes the error byte and the hardware error status, and latches an alert until it is
    cleared.">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## DYNAMIXEL: servo bus on an MCU's UART through a half-duplex stage

<!-- figure: chip-circuit-typical -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-circuit-typical.dark.svg">
  <img src="chip-circuit-typical.svg"
    alt="Schematic of the DYNAMIXEL servo bus on an mcu's uart through a half-duplex stage: the chip with U2
    MCU (5 V), U3 74LVC2G241 buffers, R1 resistor 10 kΩ, R2 resistor 10 kΩ, R3 resistor 10 kΩ, C1
    capacitor 100 µF; the MCU wires carry the port hooks Write().">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## DYNAMIXEL

<!-- figure: chip-hero -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-hero.dark.svg">
  <img src="chip-hero.svg"
    alt="Overview of the DYNAMIXEL, a family of smart servo actuators: each one has a motor, a gearbox, an
    absolute position sensor and a controller that runs its position, velocity or current loop, and all
    of them share one half-duplex data line, each with its own ID. A MCU talks to it over TTL or RS-485;
    it drives Motor and gear; it reads back Absolute encoder, Temperature, voltage, current. Headline
    figures: 10 to 14.8 V XM430-W350 supply, 12 V recommended; 4096 / rev position resolution, 12 bit;
    4.5 Mbit/s fastest baud rate, from 9600; 253 IDs on one bus, 0 to 252; 4.1 N.m stall torque at 12 V.">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## DYNAMIXEL package, TTL servo

<!-- figure: chip-outline -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-outline.dark.svg">
  <img src="chip-outline.svg"
    alt="Outline of the DYNAMIXEL in TTL servo drawn to scale from above: body 28.5 x 46.5 x 34.0 mm, pitch
    2.5 mm, 3 pins.">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## DYNAMIXEL pinout, TTL servo

<!-- figure: chip-pinout -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-pinout.dark.svg">
  <img src="chip-pinout.svg"
    alt="Pinout of the DYNAMIXEL in TTL servo, top view: 3 pins, each with its name, its electrical type as a
    colour and the driver hook that uses it.">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## Wiring: servo bus on an MCU's UART through a half-duplex stage

<!-- figure: chip-wiring-typical -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-wiring-typical.dark.svg">
  <img src="chip-wiring-typical.svg"
    alt="Wiring of the DYNAMIXEL for the servo bus on an MCU's UART through a half-duplex stage: MCU (5 V)
    UART TX to 74LVC2G241 buffers 2A (Write()); MCU (5 V) UART RX to 74LVC2G241 buffers 1Y (Read()); MCU
    (5 V) GPIO (direction) to 74LVC2G241 buffers 1OE, 2OE (Write()); 74LVC2G241 buffers 2Y, 1A to DATA
    (Write() and Read()). Supplies, grounds and the support parts are listed under the drawing.">
</picture>
<!-- /figure -->

Hardware description: [`docs/hardware.yml`](../hardware.yml)

Drawn from:

- [`docs/hardware.yml`](../hardware.yml)
- [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## One read on the half-duplex data line

<!-- figure: data-line-read -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="data-line-read.dark.svg">
  <img src="data-line-read.svg"
    alt="A read on the DYNAMIXEL data line: the controller sends an instruction packet for four bytes, the
    servo waits its return delay time and answers with a status packet carrying the bytes, an error byte
    and a CRC-16.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/data-line-read.seq.yml`](../model/data-line-read.seq.yml)

Drawn from:

- [`docs/model/data-line-read.seq.yml`](../model/data-line-read.seq.yml)

## The layers, one of them yours

<!-- figure: driver-layers -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="driver-layers.dark.svg">
  <img src="driver-layers.svg"
    alt="Layer diagram: your application calls the DYNAMIXEL core, which sends register reads and writes
    through the UART port contract to the port class you write, which drives the chip; the core and the
    contract are the same on every MCU.">
</picture>
<!-- /figure -->

Figure module: [`docs/diagrams/figures/driver-layers.py`](../../docs/diagrams/figures/driver-layers.py)

Drawn from:

- `public calls` in [`inc/dynamixel.hpp`](../../inc/dynamixel.hpp)
- `UartInterface` in [`inc/dynamixel_uart_interface.hpp`](../../inc/dynamixel_uart_interface.hpp)

## One call through the layers

<!-- figure: one-call-through-the-layers -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="one-call-through-the-layers.dark.svg">
  <img src="one-call-through-the-layers.svg"
    alt="One call through the layers: your code calls ReadRegister({1, X430}, PresentPosition), the core
    exchanges frames with the chip through your port and returns the result to your code.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/one-call-through-the-layers.seq.yml`](../model/one-call-through-the-layers.seq.yml)

Drawn from:

- [`docs/model/one-call-through-the-layers.seq.yml`](../model/one-call-through-the-layers.seq.yml)

## Instruction and status packets

<!-- figure: servo-packet -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="servo-packet.dark.svg">
  <img src="servo-packet.svg"
    alt="Field layout of a DYNAMIXEL protocol 2.0 instruction packet and the status packet that answers it:
    header and reserved byte, packet ID, length, instruction, the status packet's error byte, parameters
    and CRC.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/servo-packet.timing.yml`](../model/servo-packet.timing.yml)

Drawn from:

- [`docs/model/servo-packet.timing.yml`](../model/servo-packet.timing.yml)

## Bytes on the DATA line

<!-- figure: uart-timing -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="uart-timing.dark.svg">
  <img src="uart-timing.svg"
    alt="Two bytes on the half-duplex DATA line of a DYNAMIXEL bus: a start bit, eight data bits and one stop
    bit with no parity, and the longest gap allowed between the bytes of one instruction packet.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/uart-timing.timing.yml`](../model/uart-timing.timing.yml)

Drawn from:

- [`docs/model/uart-timing.timing.yml`](../model/uart-timing.timing.yml)

## Servos on one DATA line

<!-- figure: uart-topology -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="uart-topology.dark.svg">
  <img src="uart-topology.svg"
    alt="A DYNAMIXEL multidrop bus on which the MCU and several servos share one half-duplex DATA line and
    ground, each servo answering to its own ID, and the broadcast ID reaching all of them.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/uart-topology.topology.yml`](../model/uart-topology.topology.yml)

Drawn from:

- [`docs/model/uart-topology.topology.yml`](../model/uart-topology.topology.yml)
