#ifndef TOAST_H
#define TOAST_H

#include <stdbool.h>

// Toast notification duration in milliseconds
#define TOAST_DURATION_MS 2000

// Show a toast notification message
// Message will be displayed for TOAST_DURATION_MS then auto-hide
void toast_show(const char *message);

// Clear current toast immediately
void toast_clear(void);

// Check if toast is currently visible (for render decisions)
bool toast_is_visible(void);

// Render toast to framebuffer (called from lcd_bsp.c)
// fb: portrait framebuffer (480x800)
void toast_render(uint16_t *fb);

#endif /* TOAST_H */
