/**
 * @file main.c
 * @brief Camera Stream Demo for Waveshare ESP32-P4-WiFi6-Touch-LCD-4B
 *
 * This application demonstrates:
 * - MIPI DSI display initialization via BSP
 * - GT911 touch controller via BSP
 * - OV5647 camera streaming via V4L2/MIPI CSI
 * - LVGL 9 UI with camera preview
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// BSP includes (handles display, touch, I2C)
#include "bsp/esp-bsp.h"

// Video component (V4L2 camera interface)
#include "video.h"

// UI component
#include "ui.h"

static const char *TAG = "main";

// ============================================================================
// Configuration
// ============================================================================

// FPS calculation
static int64_t last_fps_time = 0;
static int frame_count = 0;
static int current_fps = 0;

// Video file descriptor
static int video_fd = -1;

// ============================================================================
// Camera Frame Callback
// ============================================================================

/**
 * @brief Callback function called for each camera frame
 *
 * This function is called by the video component for each captured frame.
 * It updates the UI with the new frame data.
 */
static void camera_frame_callback(uint8_t *camera_buf,
                                   uint8_t camera_buf_index,
                                   uint32_t camera_buf_hes,
                                   uint32_t camera_buf_ves,
                                   size_t camera_buf_len)
{
    // Calculate FPS
    int64_t current_time = esp_timer_get_time();
    frame_count++;

    if (current_time - last_fps_time >= 1000000) { // 1 second
        current_fps = frame_count;
        frame_count = 0;
        last_fps_time = current_time;
    }

    // Lock LVGL for thread-safe UI updates
    if (lvgl_port_lock(10)) {
        // Update camera frame in UI
        bool grayscale = ui_camera_is_grayscale();
        ui_camera_set_frame_format(camera_buf_hes, camera_buf_ves, camera_buf_len);
        ui_camera_update_frame(camera_buf, camera_buf_len, grayscale);

        // Update FPS display
        ui_camera_update_fps(current_fps);

        lvgl_port_unlock();
    }
}

// ============================================================================
// Initialization Functions
// ============================================================================

/**
 * @brief Initialize the display and LVGL via BSP
 */
static esp_err_t init_display(void)
{
    ESP_LOGI(TAG, "Initializing display via BSP...");

    // Configure display with LVGL
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = false,
        }
    };

    lv_display_t *display = bsp_display_start_with_config(&cfg);
    if (display == NULL) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Display initialized: %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);
    return ESP_OK;
}

/**
 * @brief Initialize the camera via video component
 */
static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing camera...");

    // Get I2C bus handle from BSP (used for camera sensor communication)
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();

    // Initialize video subsystem
    esp_err_t ret = app_video_main(i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize video system");
        return ret;
    }

    // Open the camera device with RGB565 format
    video_fd = app_video_open(CAM_DEV_PATH, APP_VIDEO_FMT);
    if (video_fd < 0) {
        ESP_LOGE(TAG, "Failed to open camera device");
        return ESP_FAIL;
    }

    // Get actual resolution
    uint32_t width, height;
    ret = app_video_get_resolution(&width, &height);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Camera resolution: %lux%lu", width, height);
    }

    // Set up camera buffers
    ret = app_video_set_bufs(video_fd, CAM_BUF_NUM, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set up camera buffers");
        app_video_close(video_fd);
        return ret;
    }

    // Register frame callback
    ret = app_video_register_frame_operation_cb(camera_frame_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame callback");
        app_video_close(video_fd);
        return ret;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start the camera streaming task
 */
static esp_err_t start_camera_stream(void)
{
    ESP_LOGI(TAG, "Starting camera stream...");

    // Initialize FPS counter
    last_fps_time = esp_timer_get_time();
    frame_count = 0;

    // Start video streaming task on core 1
    esp_err_t ret = app_video_stream_task_start(video_fd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera stream task");
        return ret;
    }

    ESP_LOGI(TAG, "Camera stream started");
    return ESP_OK;
}

// ============================================================================
// Main Application
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "ESP32-P4 Camera Stream Demo");
    ESP_LOGI(TAG, "Board: Waveshare ESP32-P4-WiFi6-Touch-LCD-4B");
    ESP_LOGI(TAG, "===========================================");

    // Log memory info
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM: %lu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // Initialize BSP I2C (shared by touch and camera)
    ESP_ERROR_CHECK(bsp_i2c_init());

    // Initialize display and LVGL
    ESP_ERROR_CHECK(init_display());

    // Lock LVGL for UI initialization
    lvgl_port_lock(0);

    // Set up screen background
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1a1a1a), LV_PART_MAIN);

    // Force initial render
    lv_refr_now(NULL);

    lvgl_port_unlock();

    // Small delay to ensure display is ready
    vTaskDelay(pdMS_TO_TICKS(50));

    // Turn on backlight
    bsp_display_brightness_set(80);

    // Initialize UI component
    lvgl_port_lock(0);
    ESP_ERROR_CHECK(ui_init());
    lvgl_port_unlock();

    ESP_LOGI(TAG, "UI initialized");

    // Initialize camera
    esp_err_t ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed, continuing without camera");
    } else {
        // Start camera streaming
        ret = start_camera_stream();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start camera stream");
        }
    }

    ESP_LOGI(TAG, "Application started successfully");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());

    // Main loop - LVGL is handled by esp_lvgl_port
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
