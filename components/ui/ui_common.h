/**
 * @file ui_common.h
 * @brief UI Component Internal Common Definitions
 *
 * Internal header with shared definitions for UI screens.
 */

#pragma once

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// UI Color Palette
// ============================================================================
#define UI_COLOR_BG_DARK       0x1a1a1a    // Dark background
#define UI_COLOR_HEADER        0x16213e    // Dark navy (header/footer)
#define UI_COLOR_ACCENT        0xe94560    // Pink/red accent
#define UI_COLOR_BLACK         0x0f0f0f    // Near black
#define UI_COLOR_GREEN         0x00ff00    // Green (for FPS)
#define UI_COLOR_RED           0xff0000    // Red (for LIVE indicator)
#define UI_COLOR_GRAY          0x555555    // Gray (inactive elements)
#define UI_COLOR_LIGHT_GRAY    0xaaaaaa    // Light gray (labels)

// ============================================================================
// UI Dimensions
// ============================================================================
#define UI_HEADER_HEIGHT       40
#define UI_FOOTER_HEIGHT       30

// ============================================================================
// Camera Screen Dimensions
// ============================================================================
#define UI_CAMERA_WIDTH        320
#define UI_CAMERA_HEIGHT       240

#ifdef __cplusplus
}
#endif
