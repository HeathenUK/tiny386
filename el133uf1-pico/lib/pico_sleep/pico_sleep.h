/*
 * Deep Sleep functionality for RP2350
 * Based on pico-extras pico_sleep component
 * 
 * This provides DORMANT mode sleep with timer wakeup using either:
 * - RP2350's powman timer with LPOSC (Low Power Oscillator)
 * - External DS3231 RTC with alarm-based GPIO wake (more accurate)
 */

#ifndef _PICO_SLEEP_H_
#define _PICO_SLEEP_H_

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========================================================================
// DS3231 External RTC Support
// ========================================================================

/**
 * @brief Initialize DS3231 RTC if present
 * 
 * Scans for DS3231 on I2C bus and configures it if found.
 * If DS3231 is present, it will be used for timekeeping instead of LPOSC.
 * 
 * @param sda_pin SDA GPIO pin number
 * @param scl_pin SCL GPIO pin number  
 * @param int_pin INT/SQW GPIO pin for alarm wake (use -1 to disable wake)
 * @return true if DS3231 found and initialized
 */
bool sleep_init_rtc(int sda_pin, int scl_pin, int int_pin);

/**
 * @brief Check if external RTC (DS3231) is available
 * @return true if DS3231 is present and initialized
 */
bool sleep_has_rtc(void);

/**
 * @brief Get the GPIO pin used for RTC interrupt/wake
 * @return GPIO pin number, or -1 if not configured
 */
int sleep_get_rtc_int_pin(void);

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
 * The system will enter dormant mode and wake up after the specified
 * number of milliseconds.
 * 
 * If DS3231 RTC is available (sleep_has_rtc() == true):
 * - Uses DS3231 alarm for accurate wake timing
 * - Wakes via GPIO interrupt from DS3231 INT pin
 * 
 * Otherwise:
 * - Uses RP2350's powman timer with LPOSC
 * - One of the sleep_run_from_* functions MUST be called before this
 * 
 * @param delay_ms Duration to sleep in milliseconds
 */
void sleep_goto_dormant_for_ms(uint32_t delay_ms);

/**
 * @brief Restore clocks and peripherals after waking from dormant mode
 * 
 * Note: For true deep sleep, this is handled automatically by the
 * wake handler. This function is kept for compatibility.
 */
void sleep_power_up(void);

/**
 * @brief Check if we just woke from deep sleep
 * 
 * @return true if we woke from deep sleep, false otherwise
 */
bool sleep_woke_from_deep_sleep(void);

/**
 * @brief Clear the wake-from-deep-sleep flag
 */
void sleep_clear_wake_flag(void);

/**
 * @brief Get the current RTC time in milliseconds
 * 
 * This timer runs continuously, even during deep sleep (using LPOSC).
 * It starts at 0 on first boot but persists across sleep cycles.
 * 
 * @return Current time in milliseconds since timer was started
 */
uint64_t sleep_get_time_ms(void);

/**
 * @brief Set the RTC time in milliseconds
 * 
 * @param time_ms Time to set in milliseconds
 */
void sleep_set_time_ms(uint64_t time_ms);

/**
 * @brief Get uptime in seconds (convenience function)
 * 
 * @return Seconds since timer started
 */
uint32_t sleep_get_uptime_seconds(void);

/**
 * @brief Get drift-compensated time in milliseconds
 * 
 * Applies the stored drift correction factor to improve accuracy.
 * Use this instead of sleep_get_time_ms() for best accuracy.
 * 
 * @return Corrected time in milliseconds
 */
uint64_t sleep_get_corrected_time_ms(void);

/**
 * @brief Update drift calibration based on NTP sync
 * 
 * Call this when you have accurate NTP time to improve future accuracy.
 * Compares elapsed LPOSC time vs actual elapsed time to calculate drift.
 * 
 * @param accurate_time_ms The accurate (NTP) time in milliseconds
 */
void sleep_calibrate_drift(uint64_t accurate_time_ms);

/**
 * @brief Get the current drift correction in parts-per-million
 * 
 * Positive means LPOSC runs slow, negative means LPOSC runs fast.
 * 
 * @return Drift in PPM
 */
int32_t sleep_get_drift_ppm(void);

/**
 * @brief Set a known drift correction in parts-per-million
 * 
 * @param drift_ppm Drift correction value
 */
void sleep_set_drift_ppm(int32_t drift_ppm);

/**
 * @brief Get the calibrated LPOSC frequency in Hz
 * 
 * Returns the measured LPOSC frequency stored during calibration.
 * Nominal is 32768 Hz, but actual can vary significantly.
 * 
 * @return Calibrated frequency in Hz, or 0 if not calibrated
 */
uint32_t sleep_get_lposc_freq_hz(void);

/**
 * @brief Get LPOSC deviation from nominal in percent * 100
 * 
 * @return Deviation (e.g., 500 = +5.00%, -300 = -3.00%)
 */
int32_t sleep_get_lposc_deviation_centipercent(void);


#ifdef __cplusplus
}
#endif

#endif // _PICO_SLEEP_H_
