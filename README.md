# STM32G070RB Driver Library

This repository contains low-level peripheral drivers for the STM32G070RB microcontroller, developed as a register-level driver library and tested using PlatformIO.

## ✅ Currently Supported Peripherals

- **GPIO Driver**: Full support for pin configuration, mode selection (Input/Output/AltFn/Analog), and speed/pull-up control.
- **SPI Driver**: Verified Master mode transmission with 8-bit/16-bit support and software slave management.

## 🚧 Planned Support
- USART
- I2C
- Timers
- ADC

## 📸 SPI Driver Verification
The SPI driver has been physically verified using a Logic Analyzer.
- **Target**: Nucleo-G070RB (Master)
- **Message**: "Hello World"
- **Status**: Verified clean 8-bit ASCII transmission.

## Project Structure

| Path               | Description                             |
|--------------------|-----------------------------------------|
| `lib/G070/`        | Device-specific header (Register Defs)  |
| `lib/GPIO/`        | Custom GPIO driver implementation       |
| `lib/SPI/`         | Custom SPI driver implementation        |
| `lib/Notes/`       | Hardware pinouts and reference docs     |
| `src/spi.c`        | SPI verification application            |
| `platformio.ini`   | PlatformIO project configuration        |

## Hardware Pinout
Refer to [CN7_CN10_Pinout.md](lib/Notes/CN7_CN10_Pinout.md) for corrected Morpho connector mappings.

