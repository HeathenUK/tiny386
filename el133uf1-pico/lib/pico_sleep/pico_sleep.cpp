/*
 * Deep Sleep functionality for RP2350
 * Based on pico-extras pico_sleep component
 * 
 * Uses direct register access to powman since the higher-level
 * powman API may not be linked in arduino-pico builds.
 */

#include "pico_sleep.h"

#include <Arduino.h>

// Pico SDK includes
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/powman.h"
#include "hardware/regs/powman.h"
#include "hardware/sync.h"
#include "pico/runtime_init.h"

// Systick register (avoid including full header that might conflict)
#define SYSTICK_CSR (*(volatile uint32_t*)0xE000E010)

// For ARM deep sleep
#ifndef __riscv
#include "hardware/structs/scb.h"
#endif

// Password for powman writes
#define POWMAN_PASSWORD (0x5AFE << 16)

// ========================================================================
// ROSC (Ring Oscillator) functions from pico-extras
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
// POWMAN helper functions (direct register access)
// ========================================================================

static inline void powman_write(io_rw_32 *reg, uint32_t value) {
    *reg = POWMAN_PASSWORD | (value & 0xFFFF);
}

static inline void powman_set_bits(io_rw_32 *reg, uint32_t bits) {
    hw_set_bits(reg, POWMAN_PASSWORD | bits);
}

static inline void powman_clear_bits(io_rw_32 *reg, uint32_t bits) {
    hw_clear_bits(reg, POWMAN_PASSWORD | bits);
}

static bool powman_timer_is_running(void) {
    return powman_hw->timer & POWMAN_TIMER_RUN_BITS;
}

static void powman_timer_stop(void) {
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_RUN_BITS);
}

static void powman_timer_start(void) {
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_RUN_BITS);
}

static uint64_t powman_timer_get_ms(void) {
    uint32_t hi = powman_hw->read_time_upper;
    uint32_t lo;
    do {
        lo = powman_hw->read_time_lower;
        uint32_t next_hi = powman_hw->read_time_upper;
        if (hi == next_hi) break;
        hi = next_hi;
    } while (true);
    return ((uint64_t)hi << 32u) | lo;
}

static void powman_timer_set_ms(uint64_t time_ms) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    powman_write(&powman_hw->set_time_15to0, time_ms & 0xffff);
    powman_write(&powman_hw->set_time_31to16, (time_ms >> 16) & 0xffff);
    powman_write(&powman_hw->set_time_47to32, (time_ms >> 32) & 0xffff);
    powman_write(&powman_hw->set_time_63to48, (time_ms >> 48) & 0xffff);
    if (was_running) powman_timer_start();
}

static void powman_timer_set_1khz_tick_source_lposc(void) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    
    // LPOSC frequency is ~32kHz
    uint32_t lposc_freq_hz = 32768;
    uint32_t lposc_freq_khz = lposc_freq_hz / 1000;  // 32
    uint32_t lposc_freq_khz_frac16 = (lposc_freq_hz % 1000) * 65536 / 1000;  // 768 * 65536 / 1000
    
    powman_write(&powman_hw->lposc_freq_khz_int, lposc_freq_khz);
    powman_write(&powman_hw->lposc_freq_khz_frac, lposc_freq_khz_frac16);
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_USE_LPOSC_BITS);
    
    if (was_running) {
        powman_timer_start();
        while(!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS));
    }
}

static void powman_timer_set_1khz_tick_source_xosc(void) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    
    // XOSC frequency is typically 12MHz
    uint32_t xosc_freq_hz = XOSC_HZ;
    uint32_t xosc_freq_khz = xosc_freq_hz / 1000;
    uint32_t xosc_freq_khz_frac16 = (xosc_freq_hz % 1000) * 65536 / 1000;
    
    powman_write(&powman_hw->xosc_freq_khz_int, xosc_freq_khz);
    powman_write(&powman_hw->xosc_freq_khz_frac, xosc_freq_khz_frac16);
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_USE_XOSC_BITS);
    
    if (was_running) {
        powman_timer_start();
        while(!(powman_hw->timer & POWMAN_TIMER_USING_XOSC_BITS));
    }
}

static void powman_timer_disable_alarm(void) {
    powman_clear_bits(&powman_hw->inte, POWMAN_INTE_TIMER_BITS);
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
}

static void powman_timer_clear_alarm(void) {
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_BITS);
}

static void powman_enable_alarm_wakeup_at_ms(uint64_t alarm_time_ms) {
    // Enable timer interrupt
    powman_set_bits(&powman_hw->inte, POWMAN_INTE_TIMER_BITS);
    
    // Disable alarm first to set time
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
    
    // Set alarm time
    powman_write(&powman_hw->alarm_time_15to0, alarm_time_ms & 0xffff);
    powman_write(&powman_hw->alarm_time_31to16, (alarm_time_ms >> 16) & 0xffff);
    powman_write(&powman_hw->alarm_time_47to32, (alarm_time_ms >> 32) & 0xffff);
    powman_write(&powman_hw->alarm_time_63to48, (alarm_time_ms >> 48) & 0xffff);
    
    // Clear any pending alarm
    powman_timer_clear_alarm();
    
    // Enable alarm and powerup on alarm
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS | POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
}

static void powman_disable_alarm_wakeup(void) {
    powman_timer_disable_alarm();
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
}

// ========================================================================
// Sleep implementation
// ========================================================================

static dormant_source_t _dormant_source = DORMANT_SOURCE_LPOSC;

void sleep_run_from_dormant_source(dormant_source_t dormant_source) {
    _dormant_source = dormant_source;

    Serial.println("  [1] Setting up dormant source...");
    Serial.flush();

    uint32_t src_hz;
    uint32_t clk_ref_src;
    
    switch (dormant_source) {
        case DORMANT_SOURCE_XOSC:
            src_hz = XOSC_HZ;
            clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC;
            break;
        case DORMANT_SOURCE_ROSC:
            src_hz = 6500 * KHZ;
            clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH;
            break;
        case DORMANT_SOURCE_LPOSC:
        default:
            src_hz = 32 * KHZ;
            clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_LPOSC_CLKSRC;
            break;
    }

    // IMPORTANT: Switch powman timer to LPOSC BEFORE disabling XOSC
    if (dormant_source == DORMANT_SOURCE_LPOSC) {
        Serial.printf("  [2] Timer running: %d, reg: 0x%08lx\n", 
                      powman_timer_is_running(), powman_hw->timer);
        Serial.flush();
        
        // Initialize timer if not running
        if (!powman_timer_is_running()) {
            Serial.println("  [3] Starting timer...");
            Serial.flush();
            powman_timer_set_ms(0);
            powman_timer_start();
        }
        
        Serial.println("  [4] Getting current time...");
        Serial.flush();
        uint64_t current_ms = powman_timer_get_ms();
        Serial.printf("  [5] Current time: %llu ms\n", current_ms);
        Serial.flush();
        
        Serial.println("  [6] Switching to LPOSC tick source...");
        Serial.flush();
        powman_timer_set_1khz_tick_source_lposc();
        
        Serial.println("  [7] Restoring time...");
        Serial.flush();
        powman_timer_set_ms(current_ms);
        
        Serial.println("  [8] Waiting for LPOSC switch...");
        Serial.flush();
        
        // Wait for switch to complete with timeout
        uint32_t timeout = 100000;
        while (!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) && timeout--) {
            // busy wait
        }
        
        if (timeout == 0) {
            Serial.printf("  [!] LPOSC switch timeout! Timer reg: 0x%08lx\n", powman_hw->timer);
            Serial.flush();
            return;
        }
        Serial.println("  [9] LPOSC switch complete");
        Serial.flush();
    }

    Serial.println("  [10] Disabling systick...");
    Serial.flush();
    SYSTICK_CSR &= ~1;

    Serial.println("  [11] Configuring clk_ref...");
    Serial.flush();
    clock_configure(clk_ref,
                    clk_ref_src,
                    0,
                    src_hz,
                    src_hz);

    Serial.println("  [12] Configuring clk_sys...");
    Serial.flush();
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0,
                    src_hz,
                    src_hz);

    // After this point, Serial won't work reliably (clock too slow)
    // So we stop debug output here

    // Stop unused clocks
    clock_stop(clk_adc);
    clock_stop(clk_usb);
#if PICO_RP2350
    clock_stop(clk_hstx);
#endif

    // CLK_PERI = clk_sys
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    src_hz,
                    src_hz);

    // Disable PLLs
    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    // Now safe to disable the oscillator we're not using
    if (dormant_source == DORMANT_SOURCE_XOSC) {
        rosc_disable();
    } else if (dormant_source == DORMANT_SOURCE_ROSC || dormant_source == DORMANT_SOURCE_LPOSC) {
        xosc_disable();
    }
}

static void processor_deep_sleep(void) {
#ifdef __riscv
    // RISC-V deep sleep - would need riscv.h includes
    // For now, just use inline assembly
    asm volatile("wfi");
#else
    // ARM Cortex-M33 deep sleep
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
#endif
}

static void go_dormant(void) {
    if (_dormant_source == DORMANT_SOURCE_XOSC) {
        xosc_dormant();
    } else {
        rosc_set_dormant();
    }
}

void sleep_goto_dormant_for_ms(uint32_t delay_ms) {
    if (_dormant_source != DORMANT_SOURCE_LPOSC) {
        Serial.println("Warning: Timed dormant requires LPOSC source!");
        return;
    }
    
    // Timer should already be set up by sleep_run_from_dormant_source
    if (!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS)) {
        Serial.println("  [sleep] ERROR: Timer not using LPOSC!");
        return;
    }
    
    // Ensure timer is running
    if (!powman_timer_is_running()) {
        Serial.println("  [sleep] Starting timer...");
        powman_timer_start();
    }
    
    uint64_t current_ms = powman_timer_get_ms();
    Serial.printf("  [sleep] Current time: %llu ms\n", current_ms);
    
    // Set alarm time
    uint64_t alarm_time = current_ms + delay_ms;
    Serial.printf("  [sleep] Alarm set for: %llu ms (in %lu ms)\n", alarm_time, delay_ms);
    
    // Only enable powman clock during sleep
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS;
    clocks_hw->sleep_en1 = 0x0;
    
    // Configure alarm for wakeup
    powman_enable_alarm_wakeup_at_ms(alarm_time);
    
    Serial.printf("  [sleep] Timer reg: 0x%08lx (ALARM_ENAB=%d, PWRUP_ON_ALARM=%d)\n", 
                  powman_hw->timer,
                  (powman_hw->timer & POWMAN_TIMER_ALARM_ENAB_BITS) ? 1 : 0,
                  (powman_hw->timer & POWMAN_TIMER_PWRUP_ON_ALARM_BITS) ? 1 : 0);
    
    Serial.println("  [sleep] Entering dormant mode NOW...");
    Serial.flush();
    
    // Small busy wait since delay() might not work with clocks configured for dormant
    for (volatile int i = 0; i < 100000; i++);
    
    // Enable deep sleep at processor level
    processor_deep_sleep();
    
    // Go dormant - execution stops here until wakeup
    go_dormant();
    
    // --- We wake up here after the alarm ---
    
    powman_disable_alarm_wakeup();
}

void sleep_power_up(void) {
    // Re-enable ring oscillator first
    rosc_enable();
    
    // Re-enable crystal oscillator
    xosc_init();
    
    // Reset sleep enable registers to allow all clocks
    clocks_hw->sleep_en0 = ~0u;
    clocks_hw->sleep_en1 = ~0u;
    
    // Restore all clocks (PLLs, etc.)
    clocks_init();
    
    // Re-enable systick
    SYSTICK_CSR |= 1;
    
    // Switch powman timer back to XOSC for accuracy
    uint64_t restore_ms = powman_timer_get_ms();
    powman_timer_set_1khz_tick_source_xosc();
    powman_timer_set_ms(restore_ms);
    
    // Reinitialize serial
    Serial.end();
    Serial.begin(115200);
    
    // Give serial time to initialize
    for (volatile int i = 0; i < 1000000; i++);
}
