/*
 * Deep Sleep functionality for RP2350
 * Uses the official Pico SDK powman functions
 */

#include "pico_sleep.h"

#include <Arduino.h>

// Pico SDK includes
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/powman.h"  // Official SDK powman header
#include "hardware/sync.h"
#include "pico/runtime_init.h"

// For ARM deep sleep
#ifndef __riscv
#include "hardware/structs/scb.h"
#endif

// Systick register
#define SYSTICK_CSR (*(volatile uint32_t*)0xE000E010)

// ========================================================================
// ROSC functions (from pico-extras, not in main SDK)
// ========================================================================

static inline void rosc_write(io_rw_32 *addr, uint32_t value) {
    hw_clear_bits(&rosc_hw->status, ROSC_STATUS_BADWRITE_BITS);
    *addr = value;
}

static void rosc_disable(void) {
    uint32_t tmp = rosc_hw->ctrl;
    tmp &= (~ROSC_CTRL_ENABLE_BITS);
    tmp |= (ROSC_CTRL_ENABLE_VALUE_DISABLE << ROSC_CTRL_ENABLE_LSB);
    rosc_write(&rosc_hw->ctrl, tmp);
    while(rosc_hw->status & ROSC_STATUS_STABLE_BITS);
}

static void rosc_set_dormant(void) {
    rosc_write(&rosc_hw->dormant, ROSC_DORMANT_VALUE_DORMANT);
    while(!(rosc_hw->status & ROSC_STATUS_STABLE_BITS));
}

static void rosc_enable(void) {
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);
    while (!(rosc_hw->status & ROSC_STATUS_STABLE_BITS));
}

// ========================================================================
// Sleep implementation
// ========================================================================

static dormant_source_t _dormant_source = DORMANT_SOURCE_LPOSC;

void sleep_run_from_dormant_source(dormant_source_t dormant_source) {
    _dormant_source = dormant_source;

    Serial.println("  [1] Setting up powman timer for LPOSC...");
    Serial.flush();

    if (dormant_source == DORMANT_SOURCE_LPOSC) {
        Serial.printf("  [2] Timer running: %d\n", powman_timer_is_running() ? 1 : 0);
        Serial.flush();
        
        // Start timer if not running
        if (!powman_timer_is_running()) {
            Serial.println("  [3] Starting and configuring timer...");
            Serial.flush();
            powman_timer_set_ms(0);
            powman_timer_start();
        }
        
        Serial.println("  [4] Switching to LPOSC tick source...");
        Serial.flush();
        
        // Use official SDK function to switch to LPOSC
        powman_timer_set_1khz_tick_source_lposc();
        
        Serial.printf("  [5] Timer now using LPOSC: %d\n", 
                      (powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) ? 1 : 0);
        Serial.flush();
        
        // Verify timer is counting
        uint64_t t1 = powman_timer_get_ms();
        delay(100);
        uint64_t t2 = powman_timer_get_ms();
        Serial.printf("  [6] Timer test: %llu -> %llu (delta=%llu)\n", t1, t2, t2-t1);
        Serial.flush();
    }
}

static void go_dormant(void) {
#ifndef __riscv
    // ARM Cortex-M33: Set SLEEPDEEP bit for deep sleep
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
#endif

    // Put the oscillator into dormant mode
    // This blocks until woken by an interrupt (like the powman alarm)
    if (_dormant_source == DORMANT_SOURCE_XOSC) {
        xosc_dormant();
    } else {
        // For LPOSC, we put ROSC dormant but LPOSC keeps running for the timer
        rosc_set_dormant();
    }
    
    // If we get here, we've been woken up
}

void sleep_goto_dormant_for_ms(uint32_t delay_ms) {
    if (_dormant_source != DORMANT_SOURCE_LPOSC) {
        Serial.println("Warning: Timed dormant requires LPOSC source!");
        return;
    }
    
    if (!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS)) {
        Serial.println("  [sleep] ERROR: Timer not using LPOSC!");
        return;
    }
    
    uint64_t current_ms = powman_timer_get_ms();
    uint64_t alarm_time = current_ms + delay_ms;
    
    Serial.printf("  [sleep] Current: %llu ms, Alarm: %llu ms\n", current_ms, alarm_time);
    Serial.flush();
    
    // Use official SDK function to set up alarm wakeup
    Serial.println("  [sleep] Setting alarm wakeup...");
    Serial.flush();
    powman_enable_alarm_wakeup_at_ms(alarm_time);
    
    Serial.printf("  [sleep] Timer reg: 0x%08lx\n", powman_hw->timer);
    Serial.printf("  [sleep] INTE reg: 0x%08lx\n", powman_hw->inte);
    Serial.println("  [sleep] Entering low power sleep...");
    Serial.flush();
    
    // Wait for serial
    for (volatile int i = 0; i < 500000; i++);
    
    // Disable systick to prevent tick interrupts
    SYSTICK_CSR &= ~1;
    
    // Only enable powman clock during sleep
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS;
    clocks_hw->sleep_en1 = 0x0;
    
    // Stop unused clocks
    clock_stop(clk_adc);
    clock_stop(clk_usb);
#if PICO_RP2350
    clock_stop(clk_hstx);
#endif
    
    // Disable PLLs
    pll_deinit(pll_sys);
    pll_deinit(pll_usb);
    
    // Switch to LPOSC (32kHz) - system runs very slow but uses less power
    clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_LPOSC_CLKSRC, 0, 32*KHZ, 32*KHZ);
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0, 32*KHZ, 32*KHZ);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, 32*KHZ, 32*KHZ);
    
    // Disable XOSC to save power
    xosc_disable();
    
    // Set SLEEPDEEP for ARM
#ifndef __riscv
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
#endif
    
    // Wait for interrupt (powman alarm) - this is where the CPU sleeps
    // The CPU will halt here until the powman alarm fires
    __wfi();
    
    // --- We wake up here when the alarm fires ---
    
    powman_disable_alarm_wakeup();
}

void sleep_power_up(void) {
    rosc_enable();
    xosc_init();
    
    clocks_hw->sleep_en0 = ~0u;
    clocks_hw->sleep_en1 = ~0u;
    
    clocks_init();
    
    SYSTICK_CSR |= 1;
    
    // Switch powman back to XOSC
    uint64_t restore_ms = powman_timer_get_ms();
    powman_timer_set_1khz_tick_source_xosc();
    powman_timer_set_ms(restore_ms);
    
    Serial.end();
    Serial.begin(115200);
    for (volatile int i = 0; i < 1000000; i++);
}
