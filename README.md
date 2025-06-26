# STM32G070RB Driver Library

This repository contains low-level peripheral drivers for the STM32G070RB microcontroller, developed using STM32 HAL and tested using PlatformIO.

## Features

- ✅ GPIO driver (basic functionality and testing)
- 🚧 Planned support for additional peripherals:
  - USART
  - I2C
  - SPI
  - Timers
  - ADC

## Project Structure

| Path               | Description                             |
|--------------------|-----------------------------------------|
| `include/`         | Shared header files                     |
| `lib/GPIO/`        | Custom GPIO driver implementation       |
| `src/main.c`       | Main application file (driver testing)  |
| `test/`            | Optional test files                     |
| `platformio.ini`   | PlatformIO project configuration        |
| `README.md`        | Project documentation                   |
