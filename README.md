# HV Power Supply Interface

Firmware for the **Adafruit ItsyBitsy M0** (SAMD21G18A) that provides a
serial and I²C control interface for a high-voltage bipolar power supply.

---

## Overview

The board generates a bipolar analog control voltage (±10 V) using two
complementary PWM channels and drives a polarity relay for output polarity
selection.  It accepts ASCII commands over USB-CDC serial and optionally as
an I²C peripheral connected to a MIPS host controller.

Key features:

- **±10 V analog output** via two 12-bit PWM channels (TCC0 and TCC2)
- **Voltage and current setpoint control** with configurable limits
- **Bipolar (reversable) output** with safe relay-switching dwell
- **ADC readbacks** for output voltage and current (10-sample averaged, IIR filtered)
- **USB-CDC ASCII command interface** compatible with the MIPS command protocol
- **I²C peripheral interface** for integration into a MIPS instrument system
- **Non-volatile settings** stored in internal flash (FlashStorage) and optionally
  on an external SPI flash FAT filesystem (survives firmware uploads)
- **DotStar RGB status LED** indicates boot (red), active (green), and idle (off)

---

## Hardware

| Item | Detail |
|------|--------|
| Microcontroller | Adafruit ItsyBitsy M0 (SAMD21G18A, 48 MHz, 256 KB flash, 32 KB RAM) |
| Framework | Arduino (Adafruit SAMD core) |
| Build system | PlatformIO |

### Pin Assignments

| Pin | Direction | Signal | Description |
|-----|-----------|--------|-------------|
| 3   | Output    | POWER  | HIGH = 24 VDC rail enabled |
| 7   | Output    | POL    | Polarity relay — LOW = positive, HIGH = negative |
| 12  | Output    | NEGPWM | PWM negative channel — TCC0/WO[3] |
| 13  | Output    | POSPWM | PWM positive channel — TCC2/WO[1] |
| A0  | Input     | NEG    | Relay feedback — HIGH when negative mode is active |
| A1  | Input     | POS    | Relay feedback — HIGH when positive mode is active |
| A2  | Analog in | VMON   | Output voltage monitor |
| A3  | Analog in | IMON   | Output current monitor |

### Analog Output Architecture

Two PWM channels (12-bit, ~11.7 kHz) produce one bipolar control signal:

```
count > 0:  POSPWM = count,  NEGPWM = 0   → positive output voltage
count < 0:  POSPWM = 0,      NEGPWM = |count| → negative output voltage
count = 0:  both channels = 0 → zero volts
```

The control voltage is scaled 0–10 V for 0–100 % of the configured range.

---

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

Flash to the board:

```bash
pio run --target upload
```

### Dependencies

The following libraries are resolved automatically by PlatformIO:

| Library | Purpose |
|---------|---------|
| Adafruit DotStar | Status LED driver |
| Adafruit SPIFlash | External SPI flash access |
| SdFat – Adafruit Fork | FAT filesystem on SPI flash |
| FlashStorage | Internal flash key-value persistence |
| ArduinoThread | Cooperative thread scheduler |

Additionally, the **GAACE** library collection (located at
`/Users/gordonanderson/Desktop/GAACE/Products/MIPS/Arduino/libraries`)
is required and is referenced via `lib_extra_dirs` in `platformio.ini`.
It provides:

| Module | Purpose |
|--------|---------|
| `commandProcessor` | ASCII serial command dispatcher |
| `WireHelper` | I²C peripheral read/write helpers |
| `Devices` | ADC/DAC calibration (`ADCchan`, `DACchan`, `Value2Counts`, `Counts2Value`, `Filter`) |
| `RingBuffer`, `SerialBuffer` | Buffered I/O |
| `debug` | Debug command subsystem |

---

## Serial Command Interface

Connect via USB at any baud rate (USB-CDC).  Commands follow the MIPS
ASCII protocol — responses begin with ACK (`0x06`) on success or
NAK (`0x15`) on failure.

### System Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `GVER` | — | Return firmware version string |
| `?NAME` / `SNAME,<s>` | string | Get or set device name |
| `SAVE` | — | Save all settings to internal flash |
| `RESTORE` | — | Restore settings from internal flash |
| `BLOAD` | — | Jump to UF2 bootloader (DFU mode) |
| `STWIADD,<hex>` | I²C address | Set I²C peripheral address |
| `GTWIADD` | — | Get I²C peripheral address |

### Output Control

| Command | Arguments | Description |
|---------|-----------|-------------|
| `ON` | — | Enable 24 VDC to power supply |
| `OFF` | — | Disable 24 VDC from power supply |
| `SV,<V>` | 0–20000 V | Set output voltage setpoint |
| `GV` | — | Get output voltage setpoint |
| `SPOL,<POS\|NEG>` | POS or NEG | Set output polarity |
| `GPOL` | — | Get commanded output polarity |
| `SRNG,<V>` | ±20000 V | Set full-scale voltage range |
| `GRNG` | — | Get voltage range |
| `SVMAX,<V>` | 0–20000 V | Set maximum voltage limit |
| `GVMAX` | — | Get maximum voltage limit |
| `SIMAX,<mA>` | 0–100 mA | Set maximum current limit |
| `GIMAX` | — | Get maximum current limit |
| `SRVBLE,<TRUE\|FALSE>` | | Enable/disable bipolar (reversable) output |
| `GRVBLE` | — | Get reversable flag |
| `SCURENA,<TRUE\|FALSE>` | | Enable/disable current-limit protection |
| `GCURENA` | — | Get current monitor enable flag |

### Readbacks

| Command | Returns | Description |
|---------|---------|-------------|
| `VRB` | float (V) | Measured output voltage |
| `IRB` | float (mA) | Measured output current |
| `PRB` | POS or NEG | Measured hardware polarity (relay feedback) |

### SPI Flash Filesystem Commands

| Command | Description |
|---------|-------------|
| `SAVEF` | Save current settings to `default.dat` on SPI flash |
| `LOADF` | Load settings from `default.dat` on SPI flash |
| `SAVECAL` | Save calibration data to `cal.dat` on SPI flash |
| `LOADCAL` | Load calibration data from `cal.dat` on SPI flash |

---

## I²C Interface

The board operates as an I²C peripheral at the address stored in
`data.TWIadd` (default `0x20`).  Command bytes follow the convention:

- `0x00`–`0x7F` — set (write) operations
- `0x80`–`0xFF` — get (read) operations

### I²C Command Bytes

| Byte | Direction | Type | Description |
|------|-----------|------|-------------|
| `0x01` | Set | bool | Power on/off |
| `0x03` | Set | float | Voltage setpoint (V) |
| `0x04` | Set | bool | Polarity (true = positive) |
| `0x05` | Set | float | Voltage range (V) |
| `0x06` | Set | float | Maximum voltage limit (V) |
| `0x07` | Set | float | Maximum current limit (mA) |
| `0x08` | Set | — | Save settings to flash (deferred) |
| `0x09` | Set | — | Restore settings from flash (deferred) |
| `0x0C` | Set | bool | Reversable flag |
| `0x0D` | Set | bool | Current monitor enable |
| `0x27` | Special | — | Enter transparent ASCII serial tunnel |
| `0x7F` | Special | string | Execute one ASCII command; response staged for read |
| `0x81` | Get | bool | Power status |
| `0x82` | Get | uint16 | Available bytes in response buffer |
| `0x83` | Get | float | Voltage setpoint (V) |
| `0x84` | Get | bool | Polarity |
| `0x85` | Get | float | Voltage range (V) |
| `0x86` | Get | float | Maximum voltage limit (V) |
| `0x87` | Get | float | Maximum current limit (mA) |
| `0x88` | Get | float | Voltage readback (V) |
| `0x89` | Get | float | Current readback (mA) |
| `0x8A` | Get | bool | Polarity readback |
| `0x8C` | Get | bool | Reversable flag |
| `0x8D` | Get | bool | Current monitor enable |
| `0xF0` | Get | byte | Device ID (`0x81`) |

All multi-byte values are transferred **little-endian** (LSB first).

---

## Persistent Settings

Settings are stored in two locations:

| Location | Contents | Survives firmware upload? |
|----------|----------|--------------------------|
| Internal flash (FlashStorage) | Full `Data` record | No — erased on upload |
| SPI flash FAT (`default.dat`) | Full `Data` record | Yes |
| SPI flash FAT (`cal.dat`) | Calibration fields only | Yes |

On boot the firmware loads from internal flash. If the signature is invalid
(e.g. after a fresh upload) it falls back to the compiled-in defaults.
Use `LOADF` to restore from the SPI flash after a firmware update.

### Default Values

| Parameter | Default |
|-----------|---------|
| I²C address | `0x20` |
| Power | Off |
| Polarity | Negative |
| Setpoint | 0 V |
| Max voltage | 5000 V |
| Max current | 1.0 mA |
| Range | ±5000 V |
| Reversable | Yes |
| Current monitor | Disabled |

---

## Project Structure

```
src/
  HVpowerSupplyInterface.cpp   Main application — setup, loop, command handlers
  HVpowerSupplyInterface.h     Pin definitions, TWI commands, Data struct
  Timer.cpp                    SAMD21 TCC PWM configuration and output helpers
  Timer.h                      PWM API and PWM_PERIOD constant
platformio.ini                 PlatformIO build configuration
```

---

## License

This project is part of the GAACE instrument control ecosystem.
