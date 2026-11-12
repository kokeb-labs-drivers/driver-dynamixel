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

## DYNAMIXEL: servo bus on an MCU's UART through a half-duplex stage

<!-- figure: chip-circuit-typical -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="chip-circuit-typical.dark.svg">
  <img src="chip-circuit-typical.svg"
    alt="Schematic of the DYNAMIXEL servo bus on an mcu's uart through a half-duplex stage: the chip with U2
    MCU, U3 Half-duplex stage, C1 capacitor 100 µF; the MCU wires carry the port hooks Write().">
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
