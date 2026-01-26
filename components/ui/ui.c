/**
 * @file ui.c
 * @brief UI Component Main Implementation
 *
 * This file manages screen navigation and initialization.
 * Uses LVGL 9 API
 */

#include "ui.h"
#include "ui_common.h"

static const char *TAG = "UI";

// ============================================================================
// External Screen Creation Functions
// ============================================================================

// Declared in ui_screen_camera.c
extern lv_obj_t *ui_screen_camera_create(void);
extern lv_obj_t *ui_screen_camera_get(void);

// ============================================================================
// Private Variables
// ============================================================================

static ui_screen_t current_screen = UI_SCREEN_CAMERA;
static bool ui_initialized = false;

// ============================================================================
// Public Functions
// ============================================================================

esp_err_t ui_init(void)
{
    if (ui_initialized) {
        ESP_LOGW(TAG, "UI already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing UI component...");

    // Lock LVGL
    lvgl_port_lock(0);

    // Create all screens
    ui_screen_camera_create();

    // Load the default screen (camera) - LVGL 9 uses lv_screen_load
    lv_obj_t *camera_scr = ui_screen_camera_get();
    if (camera_scr != NULL) {
        lv_screen_load(camera_scr);
    }

    // Unlock LVGL
    lvgl_port_unlock();

    ui_initialized = true;
    current_screen = UI_SCREEN_CAMERA;

    ESP_LOGI(TAG, "UI component initialized successfully");
    return ESP_OK;
}

esp_err_t ui_navigate_to(ui_screen_t screen)
{
    if (!ui_initialized) {
        ESP_LOGE(TAG, "UI not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (screen >= UI_SCREEN_MAX) {
        ESP_LOGE(TAG, "Invalid screen: %d", screen);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Navigating to screen: %d", screen);

    lvgl_port_lock(0);

    lv_obj_t *target_screen = NULL;

    switch (screen) {
        case UI_SCREEN_CAMERA:
            target_screen = ui_screen_camera_get();
            break;
        default:
            break;
    }

    if (target_screen != NULL) {
        lv_screen_load_anim(target_screen, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
        current_screen = screen;
    }

    lvgl_port_unlock();

    return ESP_OK;
}

ui_screen_t ui_get_current_screen(void)
{
    return current_screen;
}
