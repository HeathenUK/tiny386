/*
 * Keyboard-driven mouse emulation for Tanmatsu
 *
 * Controls:
 *   Arrow keys: Move cursor (with acceleration)
 *   Space Left: Left click
 *   Space Middle: Middle click
 *   Space Right: Right click
 *   META+Ctrl: Toggle mouse mode on/off (handled by input_bsp.c)
 */

#ifdef USE_BADGE_BSP

#include "mouse_emu.h"
#include "common.h"
#include "toast.h"
#include "../../i8042.h"
#include "bsp/input.h"
#include "esp_log.h"

static const char *TAG = "mouse_emu";

// Mouse emulation state
static bool mouse_active = false;
static int mouse_speed = 5;  // 1-10, default 5

// Key held state
static bool arrow_up_held = false;
static bool arrow_down_held = false;
static bool arrow_left_held = false;
static bool arrow_right_held = false;

// Button state
static bool btn_left_held = false;
static bool btn_middle_held = false;
static bool btn_right_held = false;

// Acceleration state
static uint32_t arrow_hold_ticks = 0;

// Internal cursor position tracking (for edge clamping)
static int16_t cursor_x = 320;
static int16_t cursor_y = 240;

void mouse_emu_init(void)
{
    mouse_active = false;
    mouse_speed = 5;
    arrow_hold_ticks = 0;
    cursor_x = 320;
    cursor_y = 240;
    ESP_LOGI(TAG, "Mouse emulation initialized");
}

void mouse_emu_toggle(void)
{
    mouse_active = !mouse_active;

    // Reset state on toggle
    arrow_up_held = false;
    arrow_down_held = false;
    arrow_left_held = false;
    arrow_right_held = false;
    btn_left_held = false;
    btn_middle_held = false;
    btn_right_held = false;
    arrow_hold_ticks = 0;

    if (mouse_active) {
        // Center cursor on activation
        cursor_x = globals.vga_width / 2;
        cursor_y = globals.vga_height / 2;
        toast_show("Mouse Mode ON");
    } else {
        toast_show("Mouse Mode OFF");
    }

    ESP_LOGI(TAG, "Mouse mode %s", mouse_active ? "enabled" : "disabled");
}

bool mouse_emu_is_active(void)
{
    return mouse_active;
}

bool mouse_emu_handle_nav_key(uint8_t nav_key, bool pressed)
{
    if (!mouse_active) {
        // Toggle is now handled by META+Ctrl in input_bsp.c
        return false;
    }

    // Mouse mode is active - handle keys
    switch (nav_key) {
    case BSP_INPUT_NAVIGATION_KEY_UP:
        arrow_up_held = pressed;
        if (pressed) arrow_hold_ticks = 0;
        return true;

    case BSP_INPUT_NAVIGATION_KEY_DOWN:
        arrow_down_held = pressed;
        if (pressed) arrow_hold_ticks = 0;
        return true;

    case BSP_INPUT_NAVIGATION_KEY_LEFT:
        arrow_left_held = pressed;
        if (pressed) arrow_hold_ticks = 0;
        return true;

    case BSP_INPUT_NAVIGATION_KEY_RIGHT:
        arrow_right_held = pressed;
        if (pressed) arrow_hold_ticks = 0;
        return true;

    case BSP_INPUT_NAVIGATION_KEY_SPACE_L:
        btn_left_held = pressed;
        return true;

    case BSP_INPUT_NAVIGATION_KEY_SPACE_M:
        btn_middle_held = pressed;
        return true;

    case BSP_INPUT_NAVIGATION_KEY_SPACE_R:
        btn_right_held = pressed;
        return true;

    default:
        // Other keys pass through to emulator
        return false;
    }
}

// Calculate movement speed based on hold duration and speed setting
// Returns pixels per tick
static int calculate_speed(uint32_t ticks)
{
    // Speed setting scales the curve (1-10)
    // speed=1: base=1, max=5
    // speed=5: base=2, max=18
    // speed=10: base=4, max=36
    int base_speed = 1 + (mouse_speed / 3);
    int max_speed = 5 + (mouse_speed * 31 / 10);

    // Ramp timing: reach max speed after ~25 ticks (~400ms at 60Hz)
    const int ramp_ticks = 25;

    if (ticks < 5) {
        // Initial slow period for precision (first ~80ms)
        return base_speed;
    } else if (ticks < ramp_ticks) {
        // Linear ramp from base to max
        int progress = ticks - 5;
        int range = max_speed - base_speed;
        return base_speed + (progress * range / (ramp_ticks - 5));
    } else {
        // Full speed
        return max_speed;
    }
}

void mouse_emu_tick(void)
{
    if (!mouse_active) return;

    // Check if any arrow is held
    bool any_arrow = arrow_up_held || arrow_down_held ||
                     arrow_left_held || arrow_right_held;

    if (!any_arrow) {
        arrow_hold_ticks = 0;
        // Still need to send button state changes
    }

    // Calculate movement
    int dx = 0, dy = 0;

    if (any_arrow) {
        int speed = calculate_speed(arrow_hold_ticks);
        arrow_hold_ticks++;

        if (arrow_left_held)  dx -= speed;
        if (arrow_right_held) dx += speed;
        if (arrow_up_held)    dy -= speed;
        if (arrow_down_held)  dy += speed;
    }

    // Update internal cursor position (for edge clamping)
    int max_x = globals.vga_width > 0 ? globals.vga_width - 1 : 639;
    int max_y = globals.vga_height > 0 ? globals.vga_height - 1 : 479;

    int new_x = cursor_x + dx;
    int new_y = cursor_y + dy;

    // Clamp to screen bounds
    if (new_x < 0) new_x = 0;
    if (new_x > max_x) new_x = max_x;
    if (new_y < 0) new_y = 0;
    if (new_y > max_y) new_y = max_y;

    // Calculate actual delta after clamping
    dx = new_x - cursor_x;
    dy = new_y - cursor_y;

    cursor_x = new_x;
    cursor_y = new_y;

    // Build button state
    // PS/2 mouse buttons: bit 0 = left, bit 1 = right, bit 2 = middle
    uint8_t buttons = 0;
    if (btn_left_held)   buttons |= 0x01;
    if (btn_right_held)  buttons |= 0x02;
    if (btn_middle_held) buttons |= 0x04;

    // Send to PS/2 mouse if there's any movement or button state
    // ps2_mouse_event handles deduplication internally
    if (globals.mouse) {
        ps2_mouse_event(globals.mouse, dx, dy, 0, buttons);
    }
}

void mouse_emu_set_speed(int speed)
{
    if (speed < 1) speed = 1;
    if (speed > 10) speed = 10;
    mouse_speed = speed;
}

int mouse_emu_get_speed(void)
{
    return mouse_speed;
}

#endif /* USE_BADGE_BSP */
