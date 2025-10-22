// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file lvgl_demo_ui.c
// @brief Simple LVGL demo UI
//
// Based on ESP-IDF PARLIO RGB LED matrix LVGL example

#include "lvgl.h"
#include <esp_log.h>
#include <stdio.h>

static const char *TAG = "demo_ui";

// Animation callback for scrolling text
static void scroll_anim_cb(void *var, int32_t val) {
    lv_obj_t *label = (lv_obj_t *)var;
    lv_obj_set_x(label, val);
}

// Create scrolling text label
static void create_scrolling_label(lv_obj_t *parent, const char *text, int16_t y_pos, uint32_t duration) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);

    // Set text color (white)
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    // Make sure label doesn't cause scrolling
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

    // Get dimensions
    int16_t parent_width = lv_obj_get_width(parent);
    lv_obj_update_layout(label);  // Force layout update to get accurate width
    int16_t label_width = lv_obj_get_width(label);

    // Position at right edge (just outside visible area)
    lv_obj_set_pos(label, parent_width, y_pos);

    // Create animation to scroll from right to left
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, label);
    lv_anim_set_exec_cb(&anim, scroll_anim_cb);
    lv_anim_set_values(&anim, parent_width, -label_width);
    lv_anim_set_duration(&anim, duration);
    lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&anim);
}

// Main UI creation function
void example_lvgl_demo_ui(lv_display_t *disp) {
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    // Set background color to black
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Disable scrollbars on screen
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Get display size
    int16_t scr_width = lv_obj_get_width(scr);
    int16_t scr_height = lv_obj_get_height(scr);

    ESP_LOGI(TAG, "Creating UI for %dx%d display", scr_width, scr_height);

    // Title: Scrolling text
    create_scrolling_label(scr, "HUB75 + LVGL Demo", 10, 5000);

    // Static info label (center)
    lv_obj_t *info_label = lv_label_create(scr);
    lv_label_set_long_mode(info_label, LV_LABEL_LONG_CLIP);
    lv_obj_clear_flag(info_label, LV_OBJ_FLAG_SCROLLABLE);

    // Build info string
    char info_str[64];
    snprintf(info_str, sizeof(info_str), "%dx%d", scr_width, scr_height);
    lv_label_set_text(info_label, info_str);

    // Center the label
    lv_obj_set_style_text_color(info_label, lv_color_make(0, 255, 255), 0);  // Cyan
    lv_obj_align(info_label, LV_ALIGN_CENTER, 0, 0);

    // Scrolling subtitle
    create_scrolling_label(scr, "esp-idf + lvgl", scr_height - 20, 6000);

    ESP_LOGI(TAG, "Demo UI created");
}
