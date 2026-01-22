/**
 * @file board_config.h
 * @brief Pin configuration for Waveshare ESP32-S3-Touch-LCD-3.5
 *
 * This file contains all the GPIO pin definitions for:
 * - LCD display (ST7796 via SPI)
 * - Touch panel (FT6336 via I2C)
 * - Camera module (OV5640)
 *
 * Pin assignments from official Waveshare demo code.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ============================================================================
 * LCD DISPLAY CONFIGURATION (ST7796 - SPI Interface)
 * ============================================================================
 * The display uses SPI for communication. The ST7796 is a 480x320 LCD.
 */

// Display resolution
#define LCD_H_RES               320
#define LCD_V_RES               480

// SPI pins for the LCD
#define LCD_PIN_MOSI            GPIO_NUM_1    // SPI Data Out (MOSI)
#define LCD_PIN_MISO            GPIO_NUM_NC   // SPI Data In (not used)
#define LCD_PIN_CLK             GPIO_NUM_5    // SPI Clock
#define LCD_PIN_CS              GPIO_NUM_NC   // Chip Select (directly connected)
#define LCD_PIN_DC              GPIO_NUM_3    // Data/Command selection
#define LCD_PIN_RST             GPIO_NUM_NC   // Hardware Reset (not connected)
#define LCD_PIN_BL              GPIO_NUM_6    // Backlight control

// SPI configuration
#define LCD_SPI_HOST            SPI2_HOST
#define LCD_SPI_FREQ_HZ         (80 * 1000 * 1000)  // 80 MHz SPI clock

/* ============================================================================
 * I2C BUS CONFIGURATION (Shared between Touch and Camera)
 * ============================================================================
 */

#define I2C_PIN_SDA             GPIO_NUM_8    // I2C Data
#define I2C_PIN_SCL             GPIO_NUM_7    // I2C Clock
#define I2C_PORT_NUM            I2C_NUM_0
#define I2C_FREQ_HZ             (400 * 1000)  // 400 kHz I2C clock

/* ============================================================================
 * TOUCH PANEL CONFIGURATION (FT6336 - I2C Interface)
 * ============================================================================
 * The capacitive touch panel uses I2C for communication.
 */

#define TOUCH_PIN_INT           GPIO_NUM_NC   // Interrupt (not connected)
#define TOUCH_PIN_RST           GPIO_NUM_NC   // Reset (not connected)

/* ============================================================================
 * CAMERA CONFIGURATION (OV5640)
 * ============================================================================
 * Camera interface pins for the OV5640 module.
 * Camera shares the I2C bus with touch panel.
 */

// Camera control pins
#define CAM_PIN_PWDN            -1            // Power down (not used)
#define CAM_PIN_RESET           -1            // Reset (not used)
#define CAM_PIN_XCLK            GPIO_NUM_38   // External clock

// Camera uses shared I2C bus (sccb_i2c_port = 0)
#define CAM_SCCB_I2C_PORT       0

// Camera data pins (directly from ESP32-S3 to OV5640)
#define CAM_PIN_D0              GPIO_NUM_45   // Y2
#define CAM_PIN_D1              GPIO_NUM_47   // Y3
#define CAM_PIN_D2              GPIO_NUM_48   // Y4
#define CAM_PIN_D3              GPIO_NUM_46   // Y5
#define CAM_PIN_D4              GPIO_NUM_42   // Y6
#define CAM_PIN_D5              GPIO_NUM_40   // Y7
#define CAM_PIN_D6              GPIO_NUM_39   // Y8
#define CAM_PIN_D7              GPIO_NUM_21   // Y9

// Camera sync pins
#define CAM_PIN_VSYNC           GPIO_NUM_17   // Vertical sync
#define CAM_PIN_HREF            GPIO_NUM_18   // Horizontal reference
#define CAM_PIN_PCLK            GPIO_NUM_41   // Pixel clock

// Camera settings
#define CAM_XCLK_FREQ_HZ        (20 * 1000 * 1000)  // 20 MHz XCLK

#endif // BOARD_CONFIG_H
