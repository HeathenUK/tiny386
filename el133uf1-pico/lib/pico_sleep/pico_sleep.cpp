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
// Sleep state - preserved across deep sleep (in RAM)
// ========================================================================

static volatile bool _woke_from_deep_sleep = false;
static dormant_source_t _dormant_source = DORMANT_SOURCE_LPOSC;

// ========================================================================
// Wake-up callback - user provides this
// ========================================================================
static void (*_wake_callback)(void) = nullptr;

void sleep_set_wake_callback(void (*callback)(void)) {
    _wake_callback = callback;
}

// ========================================================================
// Wake-up handler - called when we wake from deep sleep
// This runs instead of main() after deep sleep
// ========================================================================

void __not_in_flash_func(deep_sleep_wake_handler)() {
    // Reinitialize the C runtime (but preserve RAM)
    runtime_init();
    
    // Mark that we woke from deep sleep
    _woke_from_deep_sleep = true;
    
    // Re-enable clocks
    clocks_hw->sleep_en0 = ~0u;
    clocks_hw->sleep_en1 = ~0u;
    clocks_init();
    
    // Re-enable systick
    SYSTICK_CSR |= 1;
    
    // Switch powman timer back to XOSC
    powman_timer_set_1khz_tick_source_xosc();
    
    // Reinitialize serial
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n\n========================================");
    Serial.println("*** WOKE FROM DEEP SLEEP! ***");
    Serial.println("========================================\n");
    
    // Call user's wake callback if set
    if (_wake_callback) {
        _wake_callback();
    }
    
    // Enter infinite loop (user callback should handle everything)
    while(1) {
        delay(1000);
    }
}

// ========================================================================
// Public API
// ========================================================================

bool sleep_woke_from_deep_sleep(void) {
    return _woke_from_deep_sleep;
}

void sleep_clear_wake_flag(void) {
    _woke_from_deep_sleep = false;
}

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
    
    // Set up alarm wakeup
    powman_enable_alarm_wakeup_at_ms(alarm_time);
    
    // Set up boot vector for wake-up
    // When we wake, execution will jump to deep_sleep_wake_handler
    uintptr_t boot_addr = (uintptr_t)deep_sleep_wake_handler;
#ifndef __riscv
    boot_addr |= 1;  // OR with 1 for ARM thumb mode
#endif
    
    uintptr_t stack_pointer;
    asm volatile("mov %0, sp" : "=r" (stack_pointer));
    
    // Write boot vector to powman registers
    powman_hw->boot[0] = POWMAN_BOOT_MAGIC_NUM;
    powman_hw->boot[1] = (~POWMAN_BOOT_MAGIC_NUM + 1) ^ boot_addr;  // -magic ^ addr
    powman_hw->boot[2] = stack_pointer;
    powman_hw->boot[3] = boot_addr;
    
    Serial.printf("  [sleep] Boot vector: 0x%08lx, SP: 0x%08lx\n", boot_addr, stack_pointer);
    
    // Configure power states:
    // Sleep state: SW core OFF, keep XIP cache + SRAM (0x07)
    // Wake state: Everything ON (0x0f)
    powman_power_state sleep_state = 0x07;  // XIP, SRAM0, SRAM1 (no SW core)
    powman_power_state wake_state = 0x0f;   // All on
    
    bool valid = powman_configure_wakeup_state(sleep_state, wake_state);
    if (!valid) {
        Serial.println("  ERROR: Invalid wakeup state configuration!");
        return;
    }
    
    Serial.println("  [sleep] Entering DEEP SLEEP (core powered down)...");
    Serial.flush();
    
    // Wait for serial to complete
    delay(100);
    
    // Power down the switched core - THIS IS THE DEEP SLEEP
    // Execution will NOT continue past this point
    // When alarm fires, we'll wake at deep_sleep_wake_handler()
    powman_set_power_state(sleep_state);
    __wfi();
    
    // We should never reach here - wake goes to boot vector
    Serial.println("  ERROR: Should not reach here!");
}

void sleep_power_up(void) {
    // This is now handled by deep_sleep_wake_handler
    // Called automatically on wake
}
