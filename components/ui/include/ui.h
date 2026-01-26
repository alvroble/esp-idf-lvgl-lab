/**
 * @file ui.h
 * @brief UI Component Public API
 *
 * This header provides the public interface for the UI component.
 * It manages screens and navigation between views.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Screen identifiers
 */
typedef enum {
    UI_SCREEN_CAMERA = 0,   // Camera stream screen
    UI_SCREEN_MAX
} ui_screen_t;

/**
 * @brief Initialize the UI component
 *
 * Must be called after LVGL is initialized.
 *
 * @return ESP_OK on success
 */
esp_err_t ui_init(void);

/**
 * @brief Navigate to a specific screen
 *
 * @param screen The screen to navigate to
 * @return ESP_OK on success
 */
esp_err_t ui_navigate_to(ui_screen_t screen);

/**
 * @brief Get the currently active screen
 *
 * @return Current screen identifier
 */
ui_screen_t ui_get_current_screen(void);

// ============================================================================
// Camera Screen API
// ============================================================================

/**
 * @brief Update the camera frame buffer
 *
 * Call this from the camera task to update the displayed frame.
 *
 * @param frame_data Pointer to RGB565 frame data
 * @param len Length of frame data in bytes
 * @param grayscale If true, convert to grayscale
 */
void ui_camera_update_frame(const uint8_t *frame_data, size_t len, bool grayscale);

/**
 * @brief Update the FPS display
 *
 * @param fps Current frames per second
 */
void ui_camera_update_fps(int fps);

/**
 * @brief Check if grayscale mode is enabled
 *
 * @return true if grayscale mode is active
 */
bool ui_camera_is_grayscale(void);

#ifdef __cplusplus
}
#endif
