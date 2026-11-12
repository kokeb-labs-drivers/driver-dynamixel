# Control tables

Every servo has a control table: a block of registers that is read and written with the Read and Write instructions.
The addresses and units differ by family. The driver keeps one table per family and one family-neutral name per
register (`Reg`), so `ReadRegister({id, family}, Reg::PresentPosition)` works on any family that has one.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    - Control table, X series (addresses, sizes, units): ROBOTIS e-Manual, XM430-W350 web page, no revision mark, 2 Control Table; 2.1 Control Table of EEPROM Area; 2.2 Control Table of RAM Area, p. 1
    - Specifications, baud rates and ID range: ROBOTIS e-Manual, XM430-W350 web page, no revision mark, 1 Specifications, p. 1
    - Control table, AX / RX / DX series (protocol 1.0): ROBOTIS e-Manual, AX-12A web page, no revision mark, 2 Control Table; 2.2 EEPROM Area; 2.3 RAM Area, p. 1
    - Control table, XL-320: ROBOTIS e-Manual, XL-320 web page, no revision mark, 2 Control Table; 2.2 EEPROM Area; 2.3 RAM Area, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

## Families

`Info(family)` gives the facts below, `FindModel()` the model number to family mapping (47 model numbers), and
`FamilyOf(model)` the family of a ping reply. A model number the catalogue does not list is `Family::Generic`: raw
addresses only.

| `Family` | Models | Protocol | Baud register | Position per turn | Table page of the e-Manual |
| --- | --- | --- | --- | --- | --- |
| `AxRxDx` | AX-12A, AX-12W, RX-10, RX-24F, RX-28, RX-64, DX-113, DX-116, DX-117 | 1.0 | divider | 1024 (300 degrees) | `ax/ax-12a`, `rx/rx-28`, `dx/dx-116` |
| `Mx28`, `Mx64`, `Mx106` | MX-28, MX-12W, MX-64, MX-106 | 1.0 | divider | 4096 | `mx/mx-28`, `mx/mx-64`, `mx/mx-106` |
| `Mx28v2`, `Mx64v2` | MX-28(2.0), MX-64(2.0), MX-106(2.0) | 2.0 | table of 8 | 4096 | `mx/mx-28-2`, `mx/mx-64-2`, `mx/mx-106-2` |
| `Xl320` | XL-320 | 2.0 | table of 4 | 1024 | `x/xl320` |
| `X330` | XL330, XC330, XM335 | 1.0, 2.0 | table of 7 | 4096 | `x/xl330-m288` |
| `X430Basic` | XL430, XC430, 2XL430, 2XC430 | 1.0, 2.0 | table of 8 | 4096 | `x/xl430-w250` |
| `X430` | XM430, XH430, XD430 | 1.0, 2.0 | table of 8 | 4096 | `x/xm430-w350` |
| `X540` | XM540, XH540, XD540 | 1.0, 2.0 | table of 8 | 4096 | `x/xm540-w270` |
| `Xw` | XW430, XW540 | 1.0, 2.0 | table of 8 | 4096 | `x/xw430-t333` |

The pages are under `https://emanual.robotis.com/docs/en/dxl/`. The Pro, P, H, L and Y series are not in the
catalogue: read them with raw addresses and `Family::Generic`.

## Kinds of value

Each register has a `Kind` that says how its bytes decode:

| `Kind` | Registers | Decoding |
| --- | --- | --- |
| `Unsigned` | most of them | little-endian unsigned |
| `Signed` | position, velocity, current and offsets of protocol 2.0 families | two's complement of the register width |
| `SignMag10` | speed, load and torque of protocol 1.0 families | bits 9 to 0 magnitude, bit 10 the direction |
| `Offset2048` | current of the MX-64 and MX-106 (1.0) | value minus 2048 |

`WriteRegister()` checks that the value fits the register and its kind (`InvalidArgument`, nothing sent), refuses a
read-only register, and returns `Unsupported` for a family without the register.

## Units

`Info(family)` and `FindModel(model)` carry the unit of position, velocity and current:

- position: `microdeg_per_count` (for example 88 000 micro-degrees for the 4096-count families);
- velocity: `mrpm_per_count` (milli-rpm per count);
- current: `ModelInfo::current_ua` (micro-amperes per count, 0 when the table has no current register).

`ReadFeedback()` returns position, velocity, effort (load in per mille, or current in counts: `Feedback::EffortKind`),
voltage in 0.1 V and temperature in degrees Celsius, read in one transfer.

## Indirect addresses

The X430, X540, XW and MX(2.0) tables have indirect address and indirect data blocks: map up to 28 (or 56)
scattered registers to one contiguous block, so that a single Read or Sync Read fetches them all.
`Indirect(family, index)` gives the addresses of the pair `index` (1 to 28 in the first block, to 56 in the second
block above address 255); `Info(family).indirect_blocks` says how many blocks a family has.

## Baud rates

`SetServoBaudRate()` writes the servo's Baud Rate register with the value its family's scheme encodes
(`BaudValueFor()`), and `ChangeBusBaud()` then moves the UART. The divider families encode `2 000 000 / (value + 1)`;
the table families list 9 600, 57 600, 115 200, 1 000 000, 2 000 000, 3 000 000, 4 000 000 and 4 500 000 bit/s as the
e-Manual does for their model.
