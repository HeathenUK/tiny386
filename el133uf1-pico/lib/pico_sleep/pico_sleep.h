/*
 * Deep Sleep functionality for RP2350
 * Based on pico-extras pico_sleep component
 * 
 * This provides DORMANT mode sleep with timer wakeup using the
 * RP2350's powman (Power Manager) and LPOSC (Low Power Oscillator).
 */

#ifndef _PICO_SLEEP_H_
#define _PICO_SLEEP_H_

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Dormant clock sources
 */
typedef enum {
    DORMANT_SOURCE_XOSC,   // Crystal oscillator (~12MHz)
    DORMANT_SOURCE_ROSC,   // Ring oscillator (~6.5MHz)
    DORMANT_SOURCE_LPOSC,  // Low power oscillator (~32KHz) - RP2350 only, lowest power
} dormant_source_t;

/**
 * @brief Prepare system for dormant mode by switching to a low power clock source
 * 
 * This reconfigures all clocks to run from the specified dormant source,
 * disables PLLs and unnecessary peripherals to minimize power consumption.
 * 
 * @param dormant_source The clock source to use during dormancy
 */
void sleep_run_from_dormant_source(dormant_source_t dormant_source);

/**
 * @brief Convenience function to prepare for dormant mode using LPOSC
 * 
 * LPOSC provides the lowest power consumption during dormancy.
 */
static inline void sleep_run_from_lposc(void) {
    sleep_run_from_dormant_source(DORMANT_SOURCE_LPOSC);
}

/**
 * @brief Send system to dormant mode for a specified duration
 * 
 * The system will enter dormant mode (all clocks stopped except LPOSC)
 * and wake up after the specified number of milliseconds.
 * 
 * One of the sleep_run_from_* functions MUST be called before this.
 * After waking up, call sleep_power_up() to restore normal clock operation.
 * 
 * @param delay_ms Duration to sleep in milliseconds
 */
void sleep_goto_dormant_for_ms(uint32_t delay_ms);

/**
 * @brief Restore clocks and peripherals after waking from dormant mode
 * 
 * This must be called after waking from dormant mode to restore
 * normal system clock operation. It re-enables PLLs, restores
 * clock sources, and reinitializes UART.
 */
void sleep_power_up(void);

#ifdef __cplusplus
}
#endif

#endif // _PICO_SLEEP_H_
