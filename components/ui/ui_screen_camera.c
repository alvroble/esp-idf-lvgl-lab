/**
 * @file ui_screen_camera.c
 * @brief Camera Screen Implementation for ESP32-P4
 *
 * This file implements the camera stream screen with:
 * - Live camera feed display
 * - B/W toggle switch
 * - FPS counter
 * - LIVE indicator
 *
 * Uses LVGL 9 API
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

// Camera image container
static lv_obj_t *camera_container = NULL;

// Image descriptor for LVGL 9
static lv_image_dsc_t camera_img_dsc;

// Buffer for camera frame (RGB565)
static uint8_t *camera_buffer = NULL;

// Camera frame format tracking
static uint32_t camera_width = UI_CAMERA_WIDTH;
static uint32_t camera_height = UI_CAMERA_HEIGHT;
static size_t camera_stride = UI_CAMERA_WIDTH * 2;
static size_t camera_buf_size = UI_CAMERA_WIDTH * UI_CAMERA_HEIGHT * 2;
static lv_color_format_t camera_color_format = LV_COLOR_FORMAT_RGB565;
static uint32_t camera_scale = 256; // LVGL scale: 256 = 1.0

// FPS label reference
static lv_obj_t *fps_label = NULL;

// Grayscale mode flag
static bool grayscale_enabled = false;

// ============================================================================
// Private Functions
// ============================================================================

static esp_err_t camera_reconfigure(uint32_t width, uint32_t height, size_t stride,
                                    lv_color_format_t color_format)
{
    const size_t buffer_size = stride * height;
    uint8_t *new_buffer = (uint8_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (new_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate camera buffer (%lu bytes)", (unsigned long)buffer_size);
        return ESP_FAIL;
    }

    if (camera_buffer != NULL) {
        heap_caps_free(camera_buffer);
    }

    camera_buffer = new_buffer;
    camera_width = width;
    camera_height = height;
    camera_stride = stride;
    camera_buf_size = buffer_size;
    camera_color_format = color_format;
    memset(camera_buffer, 0, buffer_size);

    camera_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    camera_img_dsc.header.cf = color_format;
    camera_img_dsc.header.w = width;
    camera_img_dsc.header.h = height;
    camera_img_dsc.header.stride = stride;
    camera_img_dsc.data_size = buffer_size;
    camera_img_dsc.data = camera_buffer;

    if (camera_img != NULL) {
        lv_image_set_src(camera_img, &camera_img_dsc);
    }

    // Resize container + apply scale after buffer/descriptor update
    if (camera_container != NULL && camera_img != NULL) {
        lv_obj_t *parent = lv_obj_get_parent(camera_container);
        int32_t parent_w = parent ? lv_obj_get_width(parent) : 0;
        int32_t parent_h = parent ? lv_obj_get_height(parent) : 0;

        if (parent_w > 0 && parent_h > 0) {
            int32_t available_h = parent_h - UI_HEADER_HEIGHT - UI_FOOTER_HEIGHT - 20;
            int32_t target_w = (parent_w * 80) / 100;
            if (target_w <= 0) {
                target_w = parent_w;
            }

            uint32_t scale_w = (uint32_t)((target_w * 256) / (int32_t)width);
            uint32_t scale_h = (available_h > 0)
                                   ? (uint32_t)((available_h * 256) / (int32_t)height)
                                   : scale_w;
            camera_scale = scale_w < scale_h ? scale_w : scale_h;
            if (camera_scale == 0) {
                camera_scale = 256;
            }

            int32_t target_h = (int32_t)((height * camera_scale) / 256);
            target_w = (int32_t)((width * camera_scale) / 256);

            lv_obj_set_size(camera_container, target_w + 8, target_h + 8);
            lv_image_set_scale(camera_img, camera_scale);
            lv_obj_center(camera_img);
        }
    }

    ESP_LOGI(TAG, "Camera UI format set: %lux%lu stride=%lu cf=%d",
             (unsigned long)width, (unsigned long)height,
             (unsigned long)stride, (int)color_format);
    return ESP_OK;
}

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
    lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE);

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
    lv_obj_remove_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

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
    // Allocate buffer for camera image (RGB565 = 2 bytes per pixel)
    if (camera_reconfigure(camera_width, camera_height, camera_stride, camera_color_format) != ESP_OK) {
        return NULL;
    }

    // Container with border
    camera_container = lv_obj_create(parent);
    lv_obj_set_size(camera_container, camera_width + 8, camera_height + 8);
    lv_obj_align(camera_container, LV_ALIGN_CENTER, 0, 5);
    lv_obj_set_style_bg_color(camera_container, lv_color_hex(UI_COLOR_BLACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(camera_container, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(camera_container, lv_color_hex(UI_COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_border_width(camera_container, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(camera_container, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(camera_container, 2, LV_PART_MAIN);
    lv_obj_remove_flag(camera_container, LV_OBJ_FLAG_SCROLLABLE);

    // Image widget (LVGL 9 uses lv_image_create)
    camera_img = lv_image_create(camera_container);
    lv_image_set_src(camera_img, &camera_img_dsc);
    lv_obj_center(camera_img);

    return camera_container;
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
        if (camera_color_format == LV_COLOR_FORMAT_RGB565) {
            const size_t bpp = 2;
            for (uint32_t y = 0; y < camera_height; y++) {
                const uint16_t *src_row = (const uint16_t *)(frame_data + (y * camera_stride));
                uint16_t *dst_row = (uint16_t *)(camera_buffer + (y * camera_stride));
                for (uint32_t x = 0; x < camera_width; x++) {
                    uint16_t pixel = src_row[x];
                    uint8_t r = (pixel >> 11) & 0x1F;
                    uint8_t g = (pixel >> 5) & 0x3F;
                    uint8_t b = pixel & 0x1F;
                    uint8_t gray = (uint8_t)(((r * 8) * 77 + (g * 4) * 150 + (b * 8) * 29) >> 8);
                    uint8_t gray5 = gray >> 3;
                    uint8_t gray6 = gray >> 2;
                    dst_row[x] = (gray5 << 11) | (gray6 << 5) | gray5;
                }
                if (camera_stride > camera_width * bpp) {
                    memset((uint8_t *)dst_row + (camera_width * bpp), 0, camera_stride - (camera_width * bpp));
                }
            }
        } else if (camera_color_format == LV_COLOR_FORMAT_RGB888) {
            const size_t bpp = 3;
            for (uint32_t y = 0; y < camera_height; y++) {
                const uint8_t *src_row = frame_data + (y * camera_stride);
                uint8_t *dst_row = camera_buffer + (y * camera_stride);
                for (uint32_t x = 0; x < camera_width; x++) {
                    const uint8_t *px = &src_row[x * bpp];
                    uint8_t gray = (uint8_t)((px[0] * 77 + px[1] * 150 + px[2] * 29) >> 8);
                    dst_row[x * bpp + 0] = gray;
                    dst_row[x * bpp + 1] = gray;
                    dst_row[x * bpp + 2] = gray;
                }
                if (camera_stride > camera_width * bpp) {
                    memset(dst_row + (camera_width * bpp), 0, camera_stride - (camera_width * bpp));
                }
            }
        } else {
            size_t copy_len = len < camera_buf_size ? len : camera_buf_size;
            memcpy(camera_buffer, frame_data, copy_len);
            if (copy_len < camera_buf_size) {
                memset(camera_buffer + copy_len, 0, camera_buf_size - copy_len);
            }
        }
    } else {
        size_t copy_len = len < camera_buf_size ? len : camera_buf_size;
        memcpy(camera_buffer, frame_data, copy_len);
        if (copy_len < camera_buf_size) {
            memset(camera_buffer + copy_len, 0, camera_buf_size - copy_len);
        }
    }

    // Update LVGL image
    lv_image_set_src(camera_img, &camera_img_dsc);
    lv_obj_invalidate(camera_img);
}

esp_err_t ui_camera_set_frame_format(uint32_t width, uint32_t height, size_t len)
{
    if (width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t bpp = 2;
    size_t pixel_count = (size_t)width * (size_t)height;
    if (pixel_count > 0) {
        size_t approx_bpp = len / pixel_count;
        if (approx_bpp == 3) {
            bpp = 3;
        }
    }

    size_t stride = width * bpp;
    if (height > 0 && len >= stride * height && (len % height) == 0) {
        stride = len / height;
    }

    lv_color_format_t format = (bpp == 3) ? LV_COLOR_FORMAT_RGB888 : LV_COLOR_FORMAT_RGB565;

    if (camera_buffer != NULL &&
        camera_width == width &&
        camera_height == height &&
        camera_stride == stride &&
        camera_color_format == format) {
        return ESP_OK;
    }

    return camera_reconfigure(width, height, stride, format);
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
