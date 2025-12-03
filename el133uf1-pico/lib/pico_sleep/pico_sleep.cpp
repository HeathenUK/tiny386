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
// Public API
// ========================================================================

void sleep_run_from_dormant_source(dormant_source_t dormant_source) {
    _dormant_source = dormant_source;

    Serial.println("  [1] Setting up powman timer...");
    Serial.flush();

    if (dormant_source == DORMANT_SOURCE_LPOSC) {
        // Start timer if not running
        if (!powman_timer_is_running()) {
            Serial.println("  [2] Starting timer...");
            Serial.flush();
            powman_timer_set_ms(0);
            powman_timer_start();
        }
        
        Serial.println("  [3] Switching to LPOSC...");
        Serial.flush();
        
        // Switch to LPOSC (keeps running during deep sleep)
        powman_timer_set_1khz_tick_source_lposc();
        
        Serial.printf("  [4] Timer using LPOSC: %d\n", 
                      (powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) ? 1 : 0);
        
        // Verify timer is counting
        uint64_t t1 = powman_timer_get_ms();
        delay(100);
        uint64_t t2 = powman_timer_get_ms();
        Serial.printf("  [5] Timer test: %llu -> %llu (delta=%llu)\n", t1, t2, t2-t1);
        Serial.flush();
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
