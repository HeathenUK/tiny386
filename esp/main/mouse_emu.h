/*
 * Keyboard-driven mouse emulation for Tanmatsu
 * Provides PS/2 mouse input using arrow keys and space bar sections
 */

#ifndef MOUSE_EMU_H
#define MOUSE_EMU_H

#include <stdbool.h>
#include <stdint.h>

// Initialize mouse emulation (call once at startup)
void mouse_emu_init(void);

// Toggle mouse mode on/off (called by input_bsp.c on META+Ctrl)
void mouse_emu_toggle(void);

// Check if mouse mode is active
bool mouse_emu_is_active(void);

// Handle navigation key events (arrows, space sections)
// Returns true if the event was consumed by mouse emulation
bool mouse_emu_handle_nav_key(uint8_t nav_key, bool pressed);

// Tick function - call at ~60Hz to process held keys and send mouse movements
void mouse_emu_tick(void);

// Configuration
void mouse_emu_set_speed(int speed);  // 1-10, default 5
int mouse_emu_get_speed(void);

#endif /* MOUSE_EMU_H */
