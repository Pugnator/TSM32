# STM32 TSM for Harleys

A replacement Turn Signal Module (TSM) for Harley-Davidson motorcycles.
Developed against the 2004 Sportster; likely compatible with other Harleys of
the same generation. See [Sportsterpedia](http://sportsterpedia.com/doku.php)
for background.

![PCB](tsm.jpg)

## Contents

- Custom PCB (gerbers and schematic in the corresponding folders)
- Firmware for the STM32F103, built with GCC
- 3D-printable enclosure (STL files; reference print in PETG, 0.3 mm layers)

## When you need it

- You want to preserve the original turn signal switch and instrument cluster, but the OEM TSM is dead or flaky.
- You want to use ballast-free LED turn signals without the "hyperflash" effect.
- You want to develop your own gauge cluster or instrument panel and need a TSM that can talk to the ECM and BCM.
- You need some really working and tested Harley-oriented J1850 code to base your own TSM or gauge cluster project on.

## What the firmware does

- Basic turn-signal functionality: left/right, hazard, and automatic cancel
- Halogen and LED bulb support with per-bulb PWM settings
- TSM/TSSM emulation so the instrument cluster, ECM and BCM see a live module at
  address 0x40: presence broadcasts and the 0x92 security handshake
- J1850-based starter lock (starter disabled while the bike is moving)
- Security PIN immobilizer with a boot settings menu (see below), stored in
  flash-emulated EEPROM

## Security PIN

When a PIN is set, the starter stays disabled after ignition-on until the PIN
is entered on the turn-signal buttons. The J1850 emulation keeps running during
entry, so the cluster lamps stay off.

- Digit: press LEFT the digit's value (1-9, each press blips the left lamp),
  then press RIGHT to commit (right lamp blips)
- Wrong PIN: both lamps flash rapidly six times; after 5 wrong attempts entry
  is ignored for 30 s
- Correct PIN: both lamps give two long flashes and the starter is enabled
- Both buttons held >= 1 s toggles the hazard lights even while locked
- Settings menu: hold both buttons while switching the ignition on (enter the
  PIN first if one is set). Item 1 = set/change PIN; committing an empty first
  digit clears the PIN and disables the lock. The menu times out after 15 s.

## Status

J1850 receive/transmit, TSM emulation and the security PIN are implemented;
J1850 and TSM emulation are tested on a bike.

## Building

Requires `arm-none-eabi` and GNU make. From the project root:

    make

## Configuration

Build- and pin-level options are in [Core/Inc/settings.h](Core/Inc/settings.h).
