/*
 * INI File Selector Screen
 * Shows available .ini files at boot or when switching configs at runtime.
 * Displays a file list sorted by modification time with a settings preview panel.
 */
#ifndef INI_SELECTOR_H
#define INI_SELECTOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Initialize the INI selector, scan for .ini files, and activate it.
 * auto_boot: if true, starts a 5-second countdown to auto-boot the
 *            most recently modified file.  Any keypress cancels the countdown.
 */
void ini_selector_start(bool auto_boot);

/*
 * Render the INI selector to the portrait framebuffer (480x800 RGB565).
 * Called from vga_task when globals.ini_selector_active is true.
 */
void ini_selector_render(uint16_t *fb, int phys_w, int phys_h, int pitch);

/*
 * Feed a key-down scancode to the INI selector.
 * Called from input_task.  Returns 1 if the selector is done (user made
 * a selection or cancelled).
 */
int ini_selector_handle_key(int scancode);

#endif /* INI_SELECTOR_H */
