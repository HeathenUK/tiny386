/*
 * Deep Sleep functionality for RP2350
 * Uses the official Pico SDK powman functions
 * 
 * This implements TRUE deep sleep where the switched core is powered down.
 * When waking up, execution restarts from a boot vector (like a warm reset)
 * but RAM is preserved.
 * 
 * Supports optional DS3231 external RTC for more accurate timekeeping.
 */

#include "pico_sleep.h"
#include "DS3231.h"

#include <Arduino.h>

// Pico SDK includes
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/powman.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "pico/runtime_init.h"

// For ARM deep sleep
#ifndef __riscv
#include "hardware/structs/scb.h"
#endif

// Magic number for powman boot vector
#define POWMAN_BOOT_MAGIC_NUM 0x706d6e61  // "pmna" in ASCII

// Systick register
#define SYSTICK_CSR (*(volatile uint32_t*)0xE000E010)

// ========================================================================
// DS3231 RTC state
// ========================================================================

static bool _rtc_present = false;
static int _rtc_int_pin = -1;

// ========================================================================
// Sleep state - use powman scratch registers (preserved across reset!)
// ========================================================================

#define SLEEP_SCRATCH_MAGIC    0xDEE95EE7  // Magic value to detect wake
#define SLEEP_SCRATCH_REG      0           // Which scratch register to use

// Drift calibration scratch registers
#define DRIFT_PPM_REG          2           // Drift in parts-per-million (int32_t)
#define LAST_SYNC_LPOSC_HI_REG 3           // LPOSC time at last sync (high 32 bits)
#define LAST_SYNC_LPOSC_LO_REG 4           // LPOSC time at last sync (low 32 bits)
#define LAST_SYNC_NTP_HI_REG   5           // NTP time at last sync (high 32 bits)
#define LAST_SYNC_NTP_LO_REG   6           // NTP time at last sync (low 32 bits)
#define DRIFT_VALID_MAGIC      0xCA1B0000  // Magic + flags in high bits of DRIFT_PPM_REG

static dormant_source_t _dormant_source = DORMANT_SOURCE_LPOSC;

// ========================================================================
// DS3231 RTC functions
// ========================================================================

bool sleep_init_rtc(int sda_pin, int scl_pin, int int_pin) {
    Serial.printf("sleep_init_rtc: SDA=%d, SCL=%d, INT=%d\n", sda_pin, scl_pin, int_pin);
    Serial.flush();
    
    // Determine which I2C bus to use based on pins
    // GPIO 0,1 = I2C0, GPIO 2,3 = I2C1, GPIO 4,5 = I2C0, GPIO 6,7 = I2C1, etc.
    // General rule: (pin / 2) % 2 == 0 -> I2C0, else I2C1
    TwoWire* wire = &Wire;  // Default I2C0
    if (sda_pin == 2 || sda_pin == 6 || sda_pin == 10 || sda_pin == 14 || 
        sda_pin == 18 || sda_pin == 22 || sda_pin == 26) {
        wire = &Wire1;  // I2C1
        Serial.println("  Using Wire1 (I2C1)");
    } else {
        Serial.println("  Using Wire (I2C0)");
    }
    Serial.flush();
    
    // Initialize DS3231
    _rtc_present = rtc.begin(wire, sda_pin, scl_pin);
    _rtc_int_pin = int_pin;
    
    if (_rtc_present) {
        Serial.println("DS3231 RTC detected - will use for timekeeping");
        rtc.printStatus();
        
        // Configure INT pin if specified
        if (int_pin >= 0) {
            pinMode(int_pin, INPUT_PULLUP);
            Serial.printf("RTC INT pin %d configured as input with pullup\n", int_pin);
            
            // Clear any pending alarm
            rtc.clearAlarm1();
        }
        
        // If RTC time is invalid (before 2020), it needs to be set
        time_t rtcTime = rtc.getTime();
        if (rtcTime < 1577836800) {  // Before 2020-01-01
            Serial.println("WARNING: RTC time is invalid (before 2020), needs NTP sync");
        }
    } else {
        Serial.println("DS3231 RTC not found - falling back to LPOSC");
    }
    
    return _rtc_present;
}

bool sleep_has_rtc(void) {
    return _rtc_present;
}

int sleep_get_rtc_int_pin(void) {
    return _rtc_int_pin;
}

// Check if we woke from deep sleep by looking at scratch register
bool sleep_woke_from_deep_sleep(void) {
    return (powman_hw->scratch[SLEEP_SCRATCH_REG] == SLEEP_SCRATCH_MAGIC);
}

void sleep_clear_wake_flag(void) {
    powman_hw->scratch[SLEEP_SCRATCH_REG] = 0;
}

static void sleep_set_wake_flag(void) {
    powman_hw->scratch[SLEEP_SCRATCH_REG] = SLEEP_SCRATCH_MAGIC;
}

// ========================================================================
// RTC / Timer functions
// ========================================================================

uint64_t sleep_get_time_ms(void) {
    // Use DS3231 if available (more accurate)
    if (_rtc_present) {
        return rtc.getTimeMs();
    }
    // Fall back to powman timer
    return powman_timer_get_ms();
}

// Check if the powman timer has a valid time set (not just 0)
bool sleep_has_valid_time(void) {
    return powman_timer_is_running() && powman_timer_get_ms() > 1700000000000ULL;
}

void sleep_set_time_ms(uint64_t time_ms) {
    // Set time on DS3231 if available
    if (_rtc_present) {
        rtc.setTimeMs(time_ms);
        Serial.printf("DS3231: Time set to %llu ms\n", time_ms);
    }
    
    // Also set powman timer (for backup/comparison)
    if (!powman_timer_is_running()) {
        powman_timer_start();
    }
    powman_timer_set_ms(time_ms);
}

uint32_t sleep_get_uptime_seconds(void) {
    return (uint32_t)(powman_timer_get_ms() / 1000);
}

// Initialize the powman timer if not already running
static void ensure_timer_running(void) {
    if (!powman_timer_is_running()) {
        powman_timer_set_ms(0);
        powman_timer_start();
    }
}

// ========================================================================
// Public API
// ========================================================================

// Scratch register for storing calibrated LPOSC frequency
#define LPOSC_FREQ_REG  7

// Get stored LPOSC frequency, or 0 if not calibrated
static uint32_t get_calibrated_lposc_hz(void) {
    uint32_t val = powman_hw->scratch[LPOSC_FREQ_REG];
    // Check for reasonable range (25kHz - 45kHz)
    if (val >= 25000 && val <= 45000) {
        return val;
    }
    return 0;  // Not calibrated or invalid
}

// Public API to get calibrated frequency
uint32_t sleep_get_lposc_freq_hz(void) {
    return get_calibrated_lposc_hz();
}

// Get deviation from nominal 32768 Hz in centipercent (percent * 100)
int32_t sleep_get_lposc_deviation_centipercent(void) {
    uint32_t freq = get_calibrated_lposc_hz();
    if (freq == 0) return 0;
    // deviation = (freq - 32768) / 32768 * 10000
    return (int32_t)(((int64_t)freq - 32768) * 10000 / 32768);
}

// Store calibrated LPOSC frequency
static void set_calibrated_lposc_hz(uint32_t freq_hz) {
    powman_hw->scratch[LPOSC_FREQ_REG] = freq_hz;
}

// Measure actual LPOSC frequency by comparing to XOSC
// This should only be called when XOSC is running (before deep sleep)
// NOTE: Does NOT touch the main timer - uses a separate measurement approach
static uint32_t measure_lposc_frequency(void) {
    Serial.println("  [LPOSC] Calibrating frequency...");
    Serial.flush();
    
    // We'll measure by temporarily switching the timer to LPOSC,
    // counting ticks over 500ms, then switching back.
    // We need to save and restore the timer value.
    
    uint64_t saved_time = powman_timer_get_ms();
    Serial.printf("  [LPOSC] Current timer: %llu ms\n", saved_time);
    Serial.flush();
    
    // Switch to LPOSC with assumed 32768 Hz
    // The SDK function will stop/restart the timer
    powman_timer_set_1khz_tick_source_lposc_with_hz(32768);
    delay(20);  // Let it stabilize
    
    // Ensure running
    if (!powman_timer_is_running()) {
        powman_timer_start();
        delay(5);
    }
    
    Serial.println("  [LPOSC] Measuring for 500ms...");
    Serial.flush();
    
    // Measure LPOSC ticks over 500ms (using XOSC-based delay/micros)
    uint64_t t1 = powman_timer_get_ms();
    uint32_t start_us = micros();
    
    delay(500);
    
    uint64_t t2 = powman_timer_get_ms();
    uint32_t elapsed_us = micros() - start_us;
    
    int64_t lposc_ticks = (int64_t)(t2 - t1);
    float elapsed_sec = elapsed_us / 1000000.0f;
    float expected_ticks = elapsed_sec * 1000.0f;
    
    // Calculate result
    uint32_t actual_freq = 32768;
    if (lposc_ticks > 0 && expected_ticks > 0) {
        float ratio = lposc_ticks / expected_ticks;
        actual_freq = (uint32_t)(32768.0f * ratio);
        
        if (actual_freq < 27000 || actual_freq > 38000) {
            Serial.printf("  [LPOSC] WARNING: Unusual freq %lu Hz, using nominal\n", actual_freq);
            actual_freq = 32768;
        } else {
            Serial.printf("  [LPOSC] Result: %lu Hz (%.2f%% from nominal)\n", 
                          actual_freq, (ratio - 1.0f) * 100.0f);
        }
    } else {
        Serial.println("  [LPOSC] ERROR: Invalid measurement, using nominal");
    }
    Serial.flush();
    
    // Return the frequency - caller will reconfigure with this value and restore time
    return actual_freq;
}

void sleep_run_from_dormant_source(dormant_source_t dormant_source) {
    _dormant_source = dormant_source;

    Serial.println("  [1] Checking powman timer...");
    Serial.flush();

    if (dormant_source == DORMANT_SOURCE_LPOSC) {
        // Check current timer state
        bool timer_running = powman_timer_is_running();
        bool using_lposc = (powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) != 0;
        bool using_xosc = (powman_hw->timer & POWMAN_TIMER_USING_XOSC_BITS) != 0;
        uint64_t time_before = powman_timer_get_ms();
        
        Serial.printf("  [2] Timer state: running=%d, LPOSC=%d, XOSC=%d, time=%llu ms\n", 
                      timer_running, using_lposc, using_xosc, time_before);
        Serial.flush();
        
        if (timer_running && using_lposc) {
            // Already set up from previous cycle - don't touch it!
            Serial.println("  [3] Timer already on LPOSC, preserving");
            uint32_t cal_freq = get_calibrated_lposc_hz();
            if (cal_freq > 0) {
                Serial.printf("  [4] Using calibrated freq: %lu Hz\n", cal_freq);
            }
            Serial.flush();
            return;
        }
        
        // SAVE the current timer value - we need to preserve it through LPOSC setup
        uint64_t saved_time = time_before;
        Serial.printf("  [3] Saving timer value: %llu ms\n", saved_time);
        Serial.flush();
        
        // Track if we did calibration measurement (takes ~600ms)
        bool did_measurement = false;
        
        // Check if we have a stored LPOSC calibration
        uint32_t lposc_freq = get_calibrated_lposc_hz();
        
        if (lposc_freq == 0) {
            // No calibration - measure LPOSC frequency now (while XOSC is running)
            lposc_freq = measure_lposc_frequency();
            set_calibrated_lposc_hz(lposc_freq);
            Serial.printf("  [4] Stored new calibration: %lu Hz\n", lposc_freq);
            did_measurement = true;
        } else {
            Serial.printf("  [4] Using stored calibration: %lu Hz\n", lposc_freq);
        }
        Serial.flush();
        
        // Switch to LPOSC with CALIBRATED frequency
        Serial.println("  [5] Configuring LPOSC with calibrated frequency...");
        Serial.flush();
        powman_timer_set_1khz_tick_source_lposc_with_hz(lposc_freq);
        delay(10);
        
        // Ensure timer is running
        if (!powman_timer_is_running()) {
            Serial.println("  [5b] Timer stopped, starting...");
            powman_timer_start();
            delay(5);
        }
        
        // RESTORE the saved timer value, accounting for elapsed overhead
        // Measurement takes ~600ms, just switching takes ~50ms
        uint64_t elapsed_overhead = did_measurement ? 650 : 50;
        uint64_t restored_time = saved_time + elapsed_overhead;
        
        Serial.printf("  [6] Restoring timer: %llu + %llu = %llu ms\n", 
                      saved_time, elapsed_overhead, restored_time);
        Serial.flush();
        
        powman_timer_stop();
        delay(1);
        powman_timer_set_ms(restored_time);
        powman_timer_start();
        delay(1);
        
        uint64_t verify_time = powman_timer_get_ms();
        Serial.printf("  [7] Timer now: %llu ms (running=%d, LPOSC=%d)\n",
                      verify_time,
                      powman_timer_is_running(),
                      (powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) ? 1 : 0);
        
        // Verify timer is counting correctly
        uint64_t t1 = powman_timer_get_ms();
        delay(100);
        uint64_t t2 = powman_timer_get_ms();
        Serial.printf("  [8] 100ms test: %llu -> %llu (delta=%llu, expected ~100)\n", t1, t2, t2-t1);
        Serial.flush();
    }
}

void sleep_goto_dormant_for_ms(uint32_t delay_ms) {
    // Set flag so we know we're waking from sleep (survives reset!)
    sleep_set_wake_flag();
    
    // Configure power states:
    // Sleep state: SW core OFF, keep XIP cache + SRAM (0x07)
    // Wake state: Everything ON (0x0f)
    powman_power_state sleep_state = 0x07;  // XIP, SRAM0, SRAM1 (no SW core)
    powman_power_state wake_state = 0x0f;   // All on
    
    // ========================================================================
    // DS3231 RTC Wake (preferred - more accurate)
    // ========================================================================
    if (_rtc_present && _rtc_int_pin >= 0) {
        Serial.println("  [sleep] Using DS3231 RTC for wake");
        Serial.flush();
        
        // Set alarm on DS3231
        rtc.setAlarm1(delay_ms);
        
        // Configure GPIO wake on the INT pin (active low when alarm triggers)
        // The DS3231 INT pin goes LOW when alarm fires and stays low until cleared
        Serial.printf("  [sleep] Configuring GPIO%d for wake (low level)\n", _rtc_int_pin);
        Serial.flush();
        
        // Enable GPIO wake in powman
        // powman_enable_gpio_wakeup(slot, gpio, edge, high)
        // slot: 0-3 (which GPIO wake source to use)
        // gpio: the GPIO pin number
        // edge: true = edge triggered, false = level triggered
        // high: true = high/rising, false = low/falling
        // DS3231 INT is active-low and stays low, so use level-triggered low
        powman_enable_gpio_wakeup(0, _rtc_int_pin, false, false);  // Slot 0, level-triggered, low
        
        bool valid = powman_configure_wakeup_state(sleep_state, wake_state);
        if (!valid) {
            Serial.println("  ERROR: Invalid wakeup state configuration!");
            sleep_clear_wake_flag();
            return;
        }
        
        Serial.printf("  [sleep] Entering DEEP SLEEP for %lu ms (DS3231 alarm)...\n", delay_ms);
        Serial.println("  [sleep] Will reboot on wake - check sleep_woke_from_deep_sleep()");
        Serial.flush();
        delay(100);
        
        // Power down the switched core
        powman_set_power_state(sleep_state);
        __wfi();
        
        // We should never reach here
        Serial.println("  ERROR: Should not reach here!");
        return;
    }
    
    // ========================================================================
    // LPOSC Timer Wake (fallback)
    // ========================================================================
    if (!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS)) {
        Serial.println("  ERROR: Timer not using LPOSC and no RTC available!");
        sleep_clear_wake_flag();
        return;
    }
    
    uint64_t alarm_time = powman_timer_get_ms() + delay_ms;
    Serial.printf("  [sleep] Using LPOSC timer, alarm at: %llu ms (in %lu ms)\n", alarm_time, delay_ms);
    Serial.flush();
    
    // Set up alarm wakeup
    powman_enable_alarm_wakeup_at_ms(alarm_time);
    
    bool valid = powman_configure_wakeup_state(sleep_state, wake_state);
    if (!valid) {
        Serial.println("  ERROR: Invalid wakeup state configuration!");
        sleep_clear_wake_flag();
        return;
    }
    
    Serial.println("  [sleep] Entering DEEP SLEEP (core powered down)...");
    Serial.println("  [sleep] Will reboot on wake - check sleep_woke_from_deep_sleep()");
    Serial.flush();
    delay(100);
    
    // Power down the switched core - THIS IS THE DEEP SLEEP
    powman_set_power_state(sleep_state);
    __wfi();
    
    // We should never reach here
    Serial.println("  ERROR: Should not reach here!");
}

void sleep_power_up(void) {
    // Not needed for deep sleep - system reboots on wake
    // Use sleep_woke_from_deep_sleep() to detect wake state
}

// ========================================================================
// Drift compensation functions
// ========================================================================

int32_t sleep_get_drift_ppm(void) {
    uint32_t val = powman_hw->scratch[DRIFT_PPM_REG];
    // Check if drift has been calibrated (magic in high bits)
    if ((val & 0xFFFF0000) != DRIFT_VALID_MAGIC) {
        return 0;  // Not calibrated yet
    }
    // Drift is stored in low 16 bits as signed value
    int16_t drift = (int16_t)(val & 0xFFFF);
    return (int32_t)drift * 100;  // Stored as drift/100 to fit in 16 bits
}

void sleep_set_drift_ppm(int32_t drift_ppm) {
    // Clamp to ±3,276,700 ppm (±327%) - should be way more than needed
    if (drift_ppm > 3276700) drift_ppm = 3276700;
    if (drift_ppm < -3276700) drift_ppm = -3276700;
    // Store as drift/100 in low 16 bits, magic in high bits
    int16_t stored = (int16_t)(drift_ppm / 100);
    powman_hw->scratch[DRIFT_PPM_REG] = DRIFT_VALID_MAGIC | (uint16_t)stored;
}

static void store_sync_point(uint64_t lposc_ms, uint64_t ntp_ms) {
    powman_hw->scratch[LAST_SYNC_LPOSC_HI_REG] = (uint32_t)(lposc_ms >> 32);
    powman_hw->scratch[LAST_SYNC_LPOSC_LO_REG] = (uint32_t)(lposc_ms & 0xFFFFFFFF);
    powman_hw->scratch[LAST_SYNC_NTP_HI_REG] = (uint32_t)(ntp_ms >> 32);
    powman_hw->scratch[LAST_SYNC_NTP_LO_REG] = (uint32_t)(ntp_ms & 0xFFFFFFFF);
}

static bool get_sync_point(uint64_t* lposc_ms, uint64_t* ntp_ms) {
    // Check if we have valid calibration data (magic number in drift register)
    uint32_t drift_reg = powman_hw->scratch[DRIFT_PPM_REG];
    if ((drift_reg & 0xFFFF0000) != DRIFT_VALID_MAGIC) {
        return false;
    }
    
    *lposc_ms = ((uint64_t)powman_hw->scratch[LAST_SYNC_LPOSC_HI_REG] << 32) |
                 powman_hw->scratch[LAST_SYNC_LPOSC_LO_REG];
    *ntp_ms = ((uint64_t)powman_hw->scratch[LAST_SYNC_NTP_HI_REG] << 32) |
               powman_hw->scratch[LAST_SYNC_NTP_LO_REG];
    
    // Sanity checks:
    // - NTP time should be after 2023 (1700000000000 ms)
    // - NTP time should be before 2100 (4102444800000 ms)
    // - LPOSC and NTP should be reasonably close (within 24 hours = 86400000 ms)
    if (*ntp_ms < 1700000000000ULL || *ntp_ms > 4102444800000ULL) {
        return false;  // Invalid NTP time
    }
    if (*lposc_ms < 1700000000000ULL || *lposc_ms > 4102444800000ULL) {
        return false;  // Invalid LPOSC time  
    }
    
    return true;
}

void sleep_calibrate_drift(uint64_t accurate_time_ms) {
    Serial.println("  [calibrate] Starting...");
    Serial.flush();
    
    // ========================================================================
    // DS3231 RTC - just set time, no drift calibration needed
    // ========================================================================
    if (_rtc_present) {
        Serial.println("  [calibrate] Using DS3231 RTC (crystal accurate, no drift cal needed)");
        rtc.setTimeMs(accurate_time_ms);
        
        // Also keep powman timer in sync for backup
        if (!powman_timer_is_running()) {
            powman_timer_start();
            delay(1);
        }
        powman_timer_set_ms(accurate_time_ms);
        
        // Store sync point for reference
        store_sync_point(accurate_time_ms, accurate_time_ms);
        
        Serial.printf("  [calibrate] DS3231 and powman set to: %llu ms\n", accurate_time_ms);
        Serial.flush();
        return;
    }
    
    // ========================================================================
    // LPOSC fallback - needs drift calibration
    // ========================================================================
    
    // Ensure timer is running before we try to read/write it
    bool was_running = powman_timer_is_running();
    if (!was_running) {
        Serial.println("  [calibrate] Timer not running, starting...");
        Serial.flush();
        powman_timer_start();
        delay(10);
    }
    
    // Read the CURRENT timer value (before we overwrite it with NTP time)
    uint64_t current_timer = powman_timer_get_ms();
    Serial.printf("  [calibrate] Current timer: %llu ms (was_running=%d)\n", 
                  current_timer, was_running);
    Serial.flush();
    
    uint64_t last_sync_timer, last_sync_ntp;
    if (get_sync_point(&last_sync_timer, &last_sync_ntp)) {
        int64_t lposc_elapsed = (int64_t)(current_timer - last_sync_timer);
        int64_t ntp_elapsed = (int64_t)(accurate_time_ms - last_sync_ntp);
        
        Serial.printf("  [calibrate] Last sync: timer=%llu, ntp=%llu\n", 
                      last_sync_timer, last_sync_ntp);
        Serial.printf("  [calibrate] Elapsed: LPOSC=%lld ms, NTP=%lld ms\n", 
                      lposc_elapsed, ntp_elapsed);
        Serial.flush();
        
        // Only calibrate if values make sense
        if (lposc_elapsed > 10000 && ntp_elapsed > 10000 && 
            lposc_elapsed < 86400000LL && ntp_elapsed < 86400000LL) {
            
            int64_t drift_ppm = ((lposc_elapsed - ntp_elapsed) * 1000000LL) / ntp_elapsed;
            int32_t old_drift = sleep_get_drift_ppm();
            int32_t new_drift = (old_drift == 0) ? (int32_t)drift_ppm 
                                                  : (old_drift * 3 + (int32_t)drift_ppm) / 4;
            
            Serial.printf("  [calibrate] Drift: %lld ppm (smoothed: %d)\n", drift_ppm, new_drift);
            sleep_set_drift_ppm(new_drift);
        } else {
            Serial.println("  [calibrate] Skipping - values out of range");
        }
    } else {
        Serial.println("  [calibrate] First sync - no previous data");
    }
    Serial.flush();
    
    // Set the timer - use explicit stop/set/start sequence for reliability
    Serial.println("  [calibrate] Setting timer...");
    Serial.flush();
    
    if (powman_timer_is_running()) {
        powman_timer_stop();
        delay(1);
    }
    powman_timer_set_ms(accurate_time_ms);
    powman_timer_start();
    delay(1);
    
    // Store sync point AFTER setting timer
    store_sync_point(accurate_time_ms, accurate_time_ms);
    
    // Verify
    uint64_t verify = powman_timer_get_ms();
    Serial.printf("  [calibrate] Timer set to: %llu ms (verify: %llu)\n", 
                  accurate_time_ms, verify);
    Serial.flush();
}

uint64_t sleep_get_corrected_time_ms(void) {
    // DS3231 is crystal-accurate, no drift correction needed
    if (_rtc_present) {
        return rtc.getTimeMs();
    }
    
    // LPOSC fallback - apply drift correction
    uint64_t current_timer = powman_timer_get_ms();
    
    int32_t drift_ppm = sleep_get_drift_ppm();
    if (drift_ppm == 0) {
        return current_timer;
    }
    
    uint64_t last_sync_timer, last_sync_ntp;
    if (!get_sync_point(&last_sync_timer, &last_sync_ntp)) {
        // No sync point, return raw time
        return current_timer;
    }
    
    // Calculate elapsed time since last sync (as measured by LPOSC)
    int64_t lposc_elapsed = (int64_t)(current_timer - last_sync_timer);
    
    // Apply drift correction to the elapsed time
    // If drift_ppm > 0, LPOSC runs fast, so we need to SUBTRACT time
    // If drift_ppm < 0, LPOSC runs slow, so we need to ADD time
    int64_t correction = (lposc_elapsed * drift_ppm) / 1000000LL;
    
    // Corrected time = sync NTP time + elapsed - drift correction
    // (subtract because drift_ppm represents how much EXTRA time LPOSC counted)
    uint64_t corrected_time = last_sync_ntp + lposc_elapsed - correction;
    
    return corrected_time;
}
