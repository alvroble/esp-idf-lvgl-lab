# ESP-IDF LVGL Lab

An experimental project exploring ESP-IDF, LVGL, and camera integration on the **Waveshare ESP32-S3-Touch-LCD-3.5** board.

The goal is to serve as a lab environment for porting [SeedSigner](https://github.com/SeedSigner/seedsigner) to ESP32.

> **Warning:** This project is being developed with the assistance of AI. Code may contain errors or suboptimal implementations. Review carefully before using in any serious application.

## Current Status

Simple camera stream demo displaying live feed on the LCD.

## Target Hardware

- **Board:** [Waveshare ESP32-S3-Touch-LCD-3.5](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.5)
   - **Display:** 3.5" ST7796 LCD (480x320) via SPI
   - **Touch:** FT6336 capacitive touch via I2C
   - **Camera:** OV5640 module

## Requirements

- **ESP-IDF v5.2+** ([Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/))
- USB-C cable

## Build & Flash

```bash
# Set up ESP-IDF environment (run in each terminal session)
source ~/esp/esp-idf/export.sh

# Set target (first time only)
idf.py set-target esp32s3

# Build
idf.py build

# Flash
idf.py flash

# Monitor serial output (Ctrl+] to exit)
idf.py monitor

# Or all in one
idf.py build flash monitor
```

## License

MIT
