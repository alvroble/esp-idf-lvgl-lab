/**
 * @file main.c
 * @brief Camera Stream Demo for Waveshare ESP32-S3-Touch-LCD-3.5
 *
 * This is a beginner-friendly example that demonstrates:
 * 1. How to initialize an LCD display using ESP-IDF's esp_lcd component
 * 2. How to set up LVGL (Light and Versatile Graphics Library)
 * 3. How to capture frames from a camera using esp32-camera
 * 4. How to display the camera feed on the LCD using LVGL
 *
 * The code is heavily commented to serve as a learning tutorial.
 */

#include <stdio.h>
#include <string.h>

// FreeRTOS includes (ESP-IDF uses FreeRTOS for multitasking)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP-IDF system includes
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

// LCD and display driver includes
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7796.h"

// Touch driver
#include "esp_lcd_touch_ft5x06.h"

// I/O Expander (controls LCD reset on this board)
#include "esp_io_expander_tca9554.h"

// LVGL includes
#include "lvgl.h"
#include "esp_lvgl_port.h"

// Camera driver
#include "esp_camera.h"

// Our board-specific pin configuration
#include "board_config.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Tag for ESP-IDF logging (helps identify where log messages come from)
static const char *TAG = "CameraLCD";

// I2C bus handle (shared between touch and camera)
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

// I/O Expander handle (controls LCD reset)
static esp_io_expander_handle_t io_expander_handle = NULL;

// LVGL display handle
static lv_display_t *lvgl_disp = NULL;

// LVGL image widget that will show the camera feed
static lv_obj_t *camera_canvas = NULL;

// Image descriptor for LVGL (describes the camera frame format)
static lv_image_dsc_t camera_img_dsc;

// Buffer to hold the converted RGB565 image for display
static uint16_t *rgb565_buffer = NULL;

// ============================================================================
// STEP 1: I2C BUS INITIALIZATION (Shared)
// ============================================================================
/**
 * @brief Initialize the shared I2C bus
 *
 * The I2C bus is shared between touch panel and camera.
 */
static void init_i2c_bus(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus...");

    i2c_master_bus_config_t i2c_bus_conf = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_PIN_SDA,
        .scl_io_num = I2C_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle));

    ESP_LOGI(TAG, "I2C bus initialized successfully");
}

// ============================================================================
// STEP 1B: I/O EXPANDER INITIALIZATION (Controls LCD Reset)
// ============================================================================
/**
 * @brief Initialize the TCA9554 I/O Expander
 *
 * The Waveshare board uses a TCA9554 I/O expander to control the LCD reset.
 * PIN 1 of the expander is connected to the LCD reset line.
 */
static void init_io_expander(void)
{
    ESP_LOGI(TAG, "Initializing I/O Expander (TCA9554)...");

    // Create the TCA9554 I/O expander on the I2C bus
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(
        i2c_bus_handle,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
        &io_expander_handle
    ));

    // Configure PIN 1 as output (LCD reset control)
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_handle, IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT));

    // Reset the LCD: pull low, wait, then pull high
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_handle, IO_EXPANDER_PIN_NUM_1, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_handle, IO_EXPANDER_PIN_NUM_1, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "I/O Expander initialized, LCD reset complete");
}

// ============================================================================
// STEP 2: SPI BUS INITIALIZATION
// ============================================================================
/**
 * @brief Initialize the SPI bus for the LCD
 *
 * The LCD uses SPI (Serial Peripheral Interface) for communication.
 * SPI is fast and uses fewer pins than parallel interfaces.
 */
static void init_spi_bus(void)
{
    ESP_LOGI(TAG, "Initializing SPI bus for LCD...");

    // SPI bus configuration structure
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,      // Master Out, Slave In (data to LCD)
        .miso_io_num = LCD_PIN_MISO,      // Master In, Slave Out (data from LCD)
        .sclk_io_num = LCD_PIN_CLK,       // Serial Clock
        .quadwp_io_num = GPIO_NUM_NC,     // Not used (for quad SPI)
        .quadhd_io_num = GPIO_NUM_NC,     // Not used (for quad SPI)
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "SPI bus initialized successfully");
}

// ============================================================================
// STEP 3: LCD PANEL INITIALIZATION
// ============================================================================
/**
 * @brief Initialize the LCD panel (ST7796)
 *
 * @param[out] panel_handle Pointer to store the panel handle
 * @param[out] io_handle Pointer to store the IO handle
 */
static void init_lcd_panel(esp_lcd_panel_handle_t *panel_handle,
                           esp_lcd_panel_io_handle_t *io_handle)
{
    ESP_LOGI(TAG, "Initializing LCD panel (ST7796)...");

    // Configure the LCD backlight pin as output and turn it on
    gpio_config_t bl_gpio_cfg = {
        .pin_bit_mask = (1ULL << LCD_PIN_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bl_gpio_cfg);
    gpio_set_level(LCD_PIN_BL, 1);  // Turn on backlight

    // SPI device configuration for the LCD
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_PIN_CS,
        .dc_gpio_num = LCD_PIN_DC,
        .spi_mode = 0,                    // SPI mode 0 (CPOL=0, CPHA=0)
        .pclk_hz = LCD_SPI_FREQ_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,                // Commands are 8 bits
        .lcd_param_bits = 8,              // Parameters are 8 bits
    };

    // Create the SPI device for the LCD
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
        &io_config,
        io_handle
    ));

    // ST7796 panel configuration
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,             // RGB565
    };

    // Create the ST7796 panel driver
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(*io_handle, &panel_config, panel_handle));

    // Reset and initialize
    ESP_ERROR_CHECK(esp_lcd_panel_reset(*panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(*panel_handle));

    // Invert colors (required for this display)
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(*panel_handle, true));

    // Mirror the display horizontally to correct orientation
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(*panel_handle, true, false));

    // Turn on the display
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*panel_handle, true));

    ESP_LOGI(TAG, "LCD panel initialized successfully");
}

// ============================================================================
// STEP 4: TOUCH PANEL INITIALIZATION
// ============================================================================
/**
 * @brief Initialize the touch panel (FT6336/FT5x06)
 *
 * The FT6336 is a capacitive touch controller that communicates via I2C.
 *
 * @param[out] touch_handle Pointer to store the touch handle
 */
static void init_touch_panel(esp_lcd_touch_handle_t *touch_handle)
{
    ESP_LOGI(TAG, "Initializing touch panel (FT6336)...");

    // Touch panel IO configuration - manually configure for proper I2C speed
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x38,                 // FT6336 I2C address
        .scl_speed_hz = I2C_FREQ_HZ,      // Use our defined I2C frequency
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .dc_bit_offset = 0,
        .flags = {
            .disable_control_phase = 1,
        },
    };

    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &io_config, &io_handle));

    // FT6336 touch controller configuration
    esp_lcd_touch_config_t touch_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_PIN_RST,
        .int_gpio_num = TOUCH_PIN_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(io_handle, &touch_cfg, touch_handle));

    ESP_LOGI(TAG, "Touch panel initialized successfully");
}

// ============================================================================
// STEP 5: LVGL INITIALIZATION
// ============================================================================
/**
 * @brief Initialize LVGL graphics library
 */
static void init_lvgl(esp_lcd_panel_handle_t panel_handle,
                      esp_lcd_panel_io_handle_t io_handle,
                      esp_lcd_touch_handle_t touch_handle)
{
    ESP_LOGI(TAG, "Initializing LVGL...");

    // Initialize the LVGL port (handles LVGL tick and task)
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // Configure LVGL display
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES / 8,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = true,
        },
    };

    // Add the display to LVGL
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    // Configure LVGL touch input
    if (touch_handle != NULL) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lvgl_disp,
            .handle = touch_handle,
        };
        lvgl_port_add_touch(&touch_cfg);
    }

    ESP_LOGI(TAG, "LVGL initialized successfully");
}

// ============================================================================
// STEP 6: CAMERA INITIALIZATION
// ============================================================================
/**
 * @brief Initialize the camera (OV5640)
 *
 * The esp32-camera library handles all the low-level camera communication.
 * Camera shares the I2C bus with touch panel.
 */
static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing camera (OV5640)...");

    // Camera configuration
    camera_config_t camera_config = {
        // Pin configuration (from board_config.h)
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = -1,           // Use existing I2C bus
        .pin_sccb_scl = -1,           // Use existing I2C bus

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        // Clock frequency
        .xclk_freq_hz = CAM_XCLK_FREQ_HZ,

        // Use existing I2C bus (shared with touch)
        .sccb_i2c_port = CAM_SCCB_I2C_PORT,

        // Frame format - RGB565 for direct display
        .pixel_format = PIXFORMAT_RGB565,

        // Frame size - QVGA (320x240) fits nicely on the display
        .frame_size = FRAMESIZE_QVGA,

        // JPEG quality (only used if pixel_format is JPEG)
        .jpeg_quality = 12,

        // Number of frame buffers
        .fb_count = 2,

        // Location of frame buffers - PSRAM for larger buffers
        .fb_location = CAMERA_FB_IN_PSRAM,

        // Grab mode - get the most recent frame
        .grab_mode = CAMERA_GRAB_LATEST,

        // LEDC timer/channel for XCLK generation
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
    };

    // Initialize the camera
    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed with error 0x%x", ret);
        return ret;
    }

    // Get the camera sensor object to adjust settings
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != NULL) {
        // Flip the image vertically (required for this board)
        sensor->set_vflip(sensor, 1);
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

// ============================================================================
// STEP 7: CREATE THE UI
// ============================================================================
/**
 * @brief Create the LVGL user interface
 */
static void create_ui(void)
{
    ESP_LOGI(TAG, "Creating user interface...");

    // Lock LVGL mutex before making any LVGL calls
    lvgl_port_lock(0);

    // Get the active screen
    lv_obj_t *screen = lv_screen_active();

    // Set screen background color to black
    lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);

    // Create a title label
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "Camera Stream Demo");
    lv_obj_set_style_text_color(title, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Allocate buffer for camera image (QVGA = 320x240, RGB565 = 2 bytes per pixel)
    const size_t img_width = 320;
    const size_t img_height = 240;
    const size_t buffer_size = img_width * img_height * sizeof(uint16_t);

    rgb565_buffer = (uint16_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (rgb565_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate image buffer");
        lvgl_port_unlock();
        return;
    }

    // Initialize image descriptor for LVGL
    camera_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    camera_img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    camera_img_dsc.header.w = img_width;
    camera_img_dsc.header.h = img_height;
    camera_img_dsc.data_size = buffer_size;
    camera_img_dsc.data = (const uint8_t *)rgb565_buffer;

    // Create an image widget for the camera feed
    camera_canvas = lv_image_create(screen);
    lv_image_set_src(camera_canvas, &camera_img_dsc);
    lv_obj_align(camera_canvas, LV_ALIGN_CENTER, 0, 15);

    // Create FPS label at the bottom
    lv_obj_t *fps_label = lv_label_create(screen);
    lv_label_set_text(fps_label, "FPS: --");
    lv_obj_set_style_text_color(fps_label, lv_color_make(0, 255, 0), LV_PART_MAIN);
    lv_obj_align(fps_label, LV_ALIGN_BOTTOM_MID, 20, -10);
    lv_obj_set_user_data(screen, fps_label);  // Store reference for later updates

    // Unlock LVGL mutex
    lvgl_port_unlock();

    ESP_LOGI(TAG, "User interface created successfully");
}

// ============================================================================
// STEP 8: CAMERA STREAMING TASK
// ============================================================================
/**
 * @brief Task that captures camera frames and updates the display
 */
static void camera_stream_task(void *param)
{
    ESP_LOGI(TAG, "Starting camera stream task...");

    camera_fb_t *fb = NULL;
    uint32_t frame_count = 0;
    int64_t last_fps_time = esp_timer_get_time();

    while (1) {
        // Capture a frame from the camera
        fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Make sure we got RGB565 format
        if (fb->format == PIXFORMAT_RGB565) {
            // Lock LVGL mutex
            if (lvgl_port_lock(10)) {
                // Copy frame data directly to our buffer
                memcpy(rgb565_buffer, fb->buf, fb->len);

                // Tell LVGL the image has changed
                lv_image_set_src(camera_canvas, &camera_img_dsc);
                lv_obj_invalidate(camera_canvas);

                // Update FPS counter every second
                frame_count++;
                int64_t now = esp_timer_get_time();
                if (now - last_fps_time >= 1000000) {  // 1 second in microseconds
                    float fps = (float)frame_count * 1000000.0f / (float)(now - last_fps_time);
                    lv_obj_t *screen = lv_screen_active();
                    lv_obj_t *fps_label = (lv_obj_t *)lv_obj_get_user_data(screen);
                    if (fps_label != NULL) {
                        lv_label_set_text_fmt(fps_label, "FPS: %.1f", fps);
                    }
                    frame_count = 0;
                    last_fps_time = now;
                }

                lvgl_port_unlock();
            }
        }

        // Return the frame buffer to the camera driver
        esp_camera_fb_return(fb);

        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================
/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Camera Stream Demo Starting...");
    ESP_LOGI(TAG, "  Board: Waveshare ESP32-S3-Touch-LCD-3.5");
    ESP_LOGI(TAG, "========================================");

    // Handles for LCD and touch
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_touch_handle_t touch_handle = NULL;

    // Step 1: Initialize shared I2C bus
    init_i2c_bus();

    // Step 1B: Initialize I/O Expander (resets LCD via TCA9554)
    init_io_expander();

    // Step 2: Initialize SPI bus
    init_spi_bus();

    // Step 3: Initialize LCD panel
    init_lcd_panel(&panel_handle, &io_handle);

    // Step 4: Initialize touch panel
    init_touch_panel(&touch_handle);

    // Step 5: Initialize LVGL
    init_lvgl(panel_handle, io_handle, touch_handle);

    // Step 6: Initialize camera
    esp_err_t ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed! Check connections.");
    }

    // Step 7: Create the UI
    create_ui();

    // Step 8: Start camera streaming task (if camera initialized)
    if (ret == ESP_OK) {
        xTaskCreatePinnedToCore(
            camera_stream_task,     // Task function
            "camera_stream",        // Task name
            4096,                   // Stack size (bytes)
            NULL,                   // Task parameter
            5,                      // Task priority
            NULL,                   // Task handle (not needed)
            1                       // CPU core (1 = APP core)
        );
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Initialization complete!");
    ESP_LOGI(TAG, "  Camera stream should appear on screen.");
    ESP_LOGI(TAG, "========================================");
}
