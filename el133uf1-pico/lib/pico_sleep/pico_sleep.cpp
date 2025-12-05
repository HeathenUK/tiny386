/*
 * Deep Sleep functionality for RP2350
 * Uses the official Pico SDK powman functions
 * 
 * This implements TRUE deep sleep where the switched core is powered down.
 * When waking up, execution restarts from a boot vector (like a warm reset)
 * but RAM is preserved.
 */

#include "pico_sleep.h"

#include <Arduino.h>

// Pico SDK includes
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/powman.h"
#include "hardware/sync.h"
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
    return powman_timer_get_ms();
}

void sleep_set_time_ms(uint64_t time_ms) {
    // Ensure timer is running before setting time
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
static uint32_t measure_lposc_frequency(void) {
    // Use rosc_hw->count to count LPOSC cycles
    // Actually, we'll use a simpler approach: 
    // Set timer to LPOSC, wait 100ms via delay() (XOSC-based), measure timer ticks
    
    Serial.println("  Calibrating LPOSC frequency...");
    
    // Make sure timer is running on LPOSC
    bool was_running = powman_timer_is_running();
    if (!was_running) {
        powman_timer_set_ms(0);
        powman_timer_start();
    }
    
    // Switch to LPOSC with assumed 32768 Hz initially
    powman_timer_set_1khz_tick_source_lposc_with_hz(32768);
    
    // Wait for timer to stabilize
    delay(10);
    
    // Measure: use XOSC-based delay() as reference, count LPOSC timer ticks
    uint64_t t1 = powman_timer_get_ms();
    uint32_t start_us = micros();  // micros() uses XOSC
    
    delay(500);  // Wait 500ms (XOSC-accurate)
    
    uint64_t t2 = powman_timer_get_ms();
    uint32_t elapsed_us = micros() - start_us;
    
    // Calculate actual LPOSC frequency
    // If LPOSC were exactly 32768 Hz, we'd see 500 timer ticks
    // actual_freq = assumed_freq * (actual_ticks / expected_ticks)
    // But we need to account for the assumed frequency
    
    int64_t lposc_ticks = (int64_t)(t2 - t1);
    float elapsed_sec = elapsed_us / 1000000.0f;
    
    // The timer gives 1 tick per ms when calibrated correctly
    // So lposc_ticks should equal elapsed_ms if calibrated
    // actual_lposc_hz = 32768 * (lposc_ticks / elapsed_ms) 
    
    float expected_ticks = elapsed_sec * 1000.0f;  // Expected ms
    float ratio = lposc_ticks / expected_ticks;
    uint32_t actual_freq = (uint32_t)(32768.0f * ratio);
    
    Serial.printf("  Measured: %lld ticks in %.1f ms (ratio=%.4f)\n", 
                  lposc_ticks, elapsed_sec * 1000.0f, ratio);
    Serial.printf("  LPOSC actual frequency: %lu Hz (nominal 32768 Hz)\n", actual_freq);
    Serial.printf("  Deviation: %+.2f%%\n", (ratio - 1.0f) * 100.0f);
    
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
        
        // First time setup - need to configure timer
        if (!timer_running) {
            Serial.println("  [3] Timer not running, starting...");
            Serial.flush();
            powman_timer_start();
        }
        
        // Check if we have a stored LPOSC calibration
        uint32_t lposc_freq = get_calibrated_lposc_hz();
        
        if (lposc_freq == 0) {
            // No calibration - measure LPOSC frequency now (while XOSC is running)
            lposc_freq = measure_lposc_frequency();
            set_calibrated_lposc_hz(lposc_freq);
            Serial.printf("  [4] Stored calibration: %lu Hz\n", lposc_freq);
        } else {
            Serial.printf("  [4] Using stored calibration: %lu Hz\n", lposc_freq);
        }
        
        // Check time before switching source
        uint64_t time_pre_switch = powman_timer_get_ms();
        Serial.printf("  [5] Time before LPOSC switch: %llu ms\n", time_pre_switch);
        Serial.flush();
        
        // Switch to LPOSC with CALIBRATED frequency
        powman_timer_set_1khz_tick_source_lposc_with_hz(lposc_freq);
        
        // Check time immediately after switch
        uint64_t time_post_switch = powman_timer_get_ms();
        int64_t switch_delta = (int64_t)time_post_switch - (int64_t)time_pre_switch;
        
        Serial.printf("  [6] Time after LPOSC switch: %llu ms (delta: %+lld ms)\n", 
                      time_post_switch, switch_delta);
        Serial.printf("  [7] Timer now: running=%d, LPOSC=%d\n",
                      powman_timer_is_running(),
                      (powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) ? 1 : 0);
        
        // Verify timer is counting correctly
        uint64_t t1 = powman_timer_get_ms();
        delay(100);
        uint64_t t2 = powman_timer_get_ms();
        Serial.printf("  [8] 100ms test: %llu -> %llu (delta=%llu, expected ~100)\n", t1, t2, t2-t1);
        Serial.flush();
        
        // Warn if significant time was lost during switch
        if (switch_delta < -100 || switch_delta > 100) {
            Serial.printf("  WARNING: %+lld ms jump during clock switch!\n", switch_delta);
        }
    }
}

void sleep_goto_dormant_for_ms(uint32_t delay_ms) {
    if (!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS)) {
        Serial.println("  ERROR: Timer not using LPOSC!");
        return;
    }
    
    uint64_t alarm_time = powman_timer_get_ms() + delay_ms;
    Serial.printf("  [sleep] Alarm at: %llu ms (in %lu ms)\n", alarm_time, delay_ms);
    
    // Set flag so we know we're waking from sleep (survives reset!)
    sleep_set_wake_flag();
    
    // Set up alarm wakeup
    powman_enable_alarm_wakeup_at_ms(alarm_time);
    
    // Configure power states:
    // Sleep state: SW core OFF, keep XIP cache + SRAM (0x07)
    // Wake state: Everything ON (0x0f)
    powman_power_state sleep_state = 0x07;  // XIP, SRAM0, SRAM1 (no SW core)
    powman_power_state wake_state = 0x0f;   // All on
    
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
    // On wake, the chip will REBOOT but the scratch register preserves our flag
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
    // Check if we have valid calibration data
    if ((powman_hw->scratch[DRIFT_PPM_REG] & 0xFFFF0000) != DRIFT_VALID_MAGIC) {
        return false;
    }
    *lposc_ms = ((uint64_t)powman_hw->scratch[LAST_SYNC_LPOSC_HI_REG] << 32) |
                 powman_hw->scratch[LAST_SYNC_LPOSC_LO_REG];
    *ntp_ms = ((uint64_t)powman_hw->scratch[LAST_SYNC_NTP_HI_REG] << 32) |
               powman_hw->scratch[LAST_SYNC_NTP_LO_REG];
    return (*lposc_ms > 0 && *ntp_ms > 1700000000000ULL);  // Sanity check
}

void sleep_calibrate_drift(uint64_t accurate_time_ms) {
    uint64_t current_lposc = powman_timer_get_ms();
    
    uint64_t last_lposc, last_ntp;
    if (get_sync_point(&last_lposc, &last_ntp)) {
        // Calculate elapsed time on both clocks
        int64_t lposc_elapsed = (int64_t)(current_lposc - last_lposc);
        int64_t ntp_elapsed = (int64_t)(accurate_time_ms - last_ntp);
        
        // Only calibrate if we have meaningful elapsed time (>10 seconds)
        if (lposc_elapsed > 10000 && ntp_elapsed > 10000) {
            // drift_ppm = (ntp_elapsed - lposc_elapsed) / lposc_elapsed * 1000000
            // Positive = LPOSC runs slow, negative = LPOSC runs fast
            int64_t drift_ppm = ((ntp_elapsed - lposc_elapsed) * 1000000LL) / lposc_elapsed;
            
            // Get existing drift and apply exponential smoothing (75% old, 25% new)
            int32_t old_drift = sleep_get_drift_ppm();
            int32_t new_drift;
            if (old_drift == 0) {
                new_drift = (int32_t)drift_ppm;  // First calibration
            } else {
                new_drift = (old_drift * 3 + (int32_t)drift_ppm) / 4;  // Smoothed
            }
            
            Serial.printf("  Drift calibration: LPOSC=%lldms, NTP=%lldms, measured=%lldppm, smoothed=%dppm\n",
                          lposc_elapsed, ntp_elapsed, drift_ppm, new_drift);
            
            sleep_set_drift_ppm(new_drift);
        }
    }
    
    // Store this sync point for next calibration
    store_sync_point(current_lposc, accurate_time_ms);
    
    // Also update the raw timer to match NTP (for sleep_get_time_ms)
    powman_timer_set_ms(accurate_time_ms);
}

uint64_t sleep_get_corrected_time_ms(void) {
    uint64_t current_lposc = powman_timer_get_ms();
    
    uint64_t last_lposc, last_ntp;
    if (!get_sync_point(&last_lposc, &last_ntp)) {
        // No calibration data, return raw time
        return current_lposc;
    }
    
    int32_t drift_ppm = sleep_get_drift_ppm();
    if (drift_ppm == 0) {
        return current_lposc;
    }
    
    // Calculate corrected time from last sync point
    int64_t lposc_elapsed = (int64_t)(current_lposc - last_lposc);
    
    // Apply drift correction: corrected = elapsed * (1 + drift_ppm/1000000)
    int64_t correction = (lposc_elapsed * drift_ppm) / 1000000LL;
    uint64_t corrected_time = last_ntp + lposc_elapsed + correction;
    
    return corrected_time;
}
