#ifndef LED_ACTIVITY_H
#define LED_ACTIVITY_H

#include <stdbool.h>

// Initialize LED activity subsystem
// Must be called after BSP LED init
void led_activity_init(void);

// Signal drive activity (call from IDE/floppy code)
void led_activity_hdd(void);    // Flash LED 4 (red)
void led_activity_floppy(void); // Flash LED 5 (green)

// Must be called periodically to manage LED timeouts
// Call from display loop or main task
void led_activity_tick(void);

#endif /* LED_ACTIVITY_H */
