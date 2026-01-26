/**
 * @file ui_screen_camera.c
 * @brief Camera Screen Implementation
 *
 * This file implements the camera stream screen with:
 * - Live camera feed display
 * - B/W toggle switch
 * - FPS counter
 * - LIVE indicator
 */

#include <string.h>
#include "ui.h"
#include "ui_common.h"
#include "esp_heap_caps.h"

static const char *TAG = "UI_Camera";

// ============================================================================
// Private Variables
// ============================================================================

// Screen object
static lv_obj_t *camera_screen = NULL;

// Camera image widget
static lv_obj_t *camera_img = NULL;

// Image descriptor for LVGL
static lv_img_dsc_t camera_img_dsc;

// Buffer for camera frame (RGB565)
static uint16_t *camera_buffer = NULL;

// FPS label reference
static lv_obj_t *fps_label = NULL;

// Grayscale mode flag
static bool grayscale_enabled = false;

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Callback for B/W toggle switch
 */
static void bw_switch_event_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    grayscale_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "Grayscale mode: %s", grayscale_enabled ? "ON" : "OFF");
}

/**
 * @brief Create the header bar
 */
static lv_obj_t *create_header(lv_obj_t *parent)
{
    lv_obj_t *header = lv_obj_create(parent);
    lv_obj_set_size(header, lv_pct(100), UI_HEADER_HEIGHT);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(UI_COLOR_HEADER), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(header, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(header, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(header, 0, LV_PART_MAIN);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    // Title label
    lv_obj_t *title = lv_label_create(header);
    lv_label_set_text(title, "Camera Stream");
    lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);

    // B/W toggle switch
    lv_obj_t *bw_switch = lv_switch_create(header);
    lv_obj_set_size(bw_switch, 40, 20);
    lv_obj_align(bw_switch, LV_ALIGN_RIGHT_MID, -50, 0);
    lv_obj_set_style_bg_color(bw_switch, lv_color_hex(UI_COLOR_GRAY), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bw_switch, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(bw_switch, bw_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // B/W label
    lv_obj_t *bw_label = lv_label_create(header);
    lv_label_set_text(bw_label, "B/W");
    lv_obj_set_style_text_color(bw_label, lv_color_hex(UI_COLOR_LIGHT_GRAY), LV_PART_MAIN);
    lv_obj_set_style_text_font(bw_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(bw_label, LV_ALIGN_RIGHT_MID, -10, 0);

    return header;
}

/**
 * @brief Create the footer bar
 */
static lv_obj_t *create_footer(lv_obj_t *parent)
{
    lv_obj_t *footer = lv_obj_create(parent);
    lv_obj_set_size(footer, lv_pct(100), UI_FOOTER_HEIGHT);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(footer, lv_color_hex(UI_COLOR_HEADER), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(footer, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(footer, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(footer, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(footer, 0, LV_PART_MAIN);
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

    // FPS label
    fps_label = lv_label_create(footer);
    lv_label_set_text(fps_label, "FPS: --");
    lv_obj_set_style_text_color(fps_label, lv_color_hex(UI_COLOR_GREEN), LV_PART_MAIN);
    lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(fps_label, LV_ALIGN_RIGHT_MID, -15, 0);

    // LIVE indicator
    lv_obj_t *live_label = lv_label_create(footer);
    lv_label_set_text(live_label, "LIVE");
    lv_obj_set_style_text_color(live_label, lv_color_hex(UI_COLOR_RED), LV_PART_MAIN);
    lv_obj_set_style_text_font(live_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(live_label, LV_ALIGN_LEFT_MID, 15, 0);

    return footer;
}

/**
 * @brief Create the camera view container
 */
static lv_obj_t *create_camera_view(lv_obj_t *parent)
{
    // Allocate buffer for camera image
    const size_t buffer_size = UI_CAMERA_WIDTH * UI_CAMERA_HEIGHT * sizeof(uint16_t);
    camera_buffer = (uint16_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (camera_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate camera buffer");
        return NULL;
    }

    // Initialize image descriptor (LVGL 8 format)
    camera_img_dsc.header.always_zero = 0;
    camera_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    camera_img_dsc.header.w = UI_CAMERA_WIDTH;
    camera_img_dsc.header.h = UI_CAMERA_HEIGHT;
    camera_img_dsc.data_size = buffer_size;
    camera_img_dsc.data = (const uint8_t *)camera_buffer;

    // Container with border
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, UI_CAMERA_WIDTH + 8, UI_CAMERA_HEIGHT + 8);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 5);
    lv_obj_set_style_bg_color(container, lv_color_hex(UI_COLOR_BLACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(container, lv_color_hex(UI_COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(container, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 2, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    // Image widget
    camera_img = lv_img_create(container);
    lv_img_set_src(camera_img, &camera_img_dsc);
    lv_obj_center(camera_img);

    return container;
}

// ============================================================================
// Public Functions
// ============================================================================

/**
 * @brief Create the camera screen
 */
lv_obj_t *ui_screen_camera_create(void)
{
    ESP_LOGI(TAG, "Creating camera screen...");

    // Create a new screen
    camera_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(camera_screen, lv_color_hex(UI_COLOR_BG_DARK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(camera_screen, LV_OPA_COVER, LV_PART_MAIN);

    // Create UI elements
    create_header(camera_screen);
    create_footer(camera_screen);
    create_camera_view(camera_screen);

    ESP_LOGI(TAG, "Camera screen created successfully");
    return camera_screen;
}

/**
 * @brief Get the camera screen object
 */
lv_obj_t *ui_screen_camera_get(void)
{
    return camera_screen;
}

/**
 * @brief Update camera frame
 */
void ui_camera_update_frame(const uint8_t *frame_data, size_t len, bool grayscale)
{
    if (camera_buffer == NULL || camera_img == NULL) {
        return;
    }

    if (grayscale) {
        // Convert RGB565 to grayscale
        uint16_t *src = (uint16_t *)frame_data;
        size_t pixel_count = len / 2;
        for (size_t i = 0; i < pixel_count; i++) {
            uint16_t pixel = src[i];
            // Extract RGB components (byte-swapped RGB565)
            uint8_t r = (pixel >> 3) & 0x1F;
            uint8_t g = ((pixel & 0x07) << 3) | ((pixel >> 13) & 0x07);
            uint8_t b = (pixel >> 8) & 0x1F;
            // Convert to grayscale
            uint8_t gray = (uint8_t)(((r * 8) * 77 + (g * 4) * 150 + (b * 8) * 29) >> 8);
            // Convert back to RGB565 grayscale
            uint8_t gray5 = gray >> 3;
            uint8_t gray6 = gray >> 2;
            camera_buffer[i] = ((gray5 & 0x1F) << 3) | ((gray6 >> 3) & 0x07) |
                              ((gray5 & 0x1F) << 8) | ((gray6 & 0x07) << 13);
        }
    } else {
        // Direct copy
        memcpy(camera_buffer, frame_data, len);
    }

    // Update LVGL image
    lv_img_set_src(camera_img, &camera_img_dsc);
    lv_obj_invalidate(camera_img);
}

/**
 * @brief Update FPS display
 */
void ui_camera_update_fps(int fps)
{
    if (fps_label != NULL) {
        lv_label_set_text_fmt(fps_label, "FPS: %d", fps);
    }
}

/**
 * @brief Check if grayscale mode is enabled
 */
bool ui_camera_is_grayscale(void)
{
    return grayscale_enabled;
}
