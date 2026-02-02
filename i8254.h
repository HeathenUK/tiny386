#ifndef I8254_H
#define I8254_H
#include <stdint.h>

#define PIT_FREQ 1193182

typedef struct PITState PITState;
PITState *i8254_init(int irq, void *pic, void (*set_irq)(void *pic, int irq, int level));
void i8254_update_irq(PITState *pit);
uint32_t i8254_ioport_read(PITState *pit, uint32_t addr1);
void i8254_ioport_write(PITState *pit, uint32_t addr, uint32_t val);

int pit_get_out(PITState *pit, int channel);
int pit_get_gate(PITState *pit, int channel);
int pit_get_initial_count(PITState *pit, int channel);
int pit_get_mode(PITState *pit, int channel);
void pit_set_gate(PITState *pit, int channel, int val);

/* Get current virtual tick count for audio timing synchronization.
 * Returns tick count based on wall-clock time and PIT programming,
 * representing how many ticks WOULD have elapsed on real hardware. */
uint64_t pit_get_virtual_ticks(PITState *pit, int channel);

/* Get the number of pending channel 0 interrupts we're behind.
 * Used for burst catch-up when CPU can't keep up with high-frequency timers. */
int pit_get_pending_irqs(PITState *pit);

/* Fire a single PIT IRQ and advance counter. For burst catch-up processing.
 * Returns true if an IRQ was fired, false if nothing pending. */
#include <stdbool.h>
bool pit_fire_single_irq(PITState *pit);

#ifdef BUILD_ESP32
/* Optimized burst functions that avoid repeated get_uticks() calls.
 * pit_burst_start: validates mode, returns pending count and cached timing info.
 * pit_burst_fire: fires one IRQ without re-checking, returns true if fired. */
int pit_burst_start(PITState *pit, uint32_t *out_d, int *out_irq);
bool pit_burst_fire(PITState *pit, uint32_t d, int irq);
#endif

#endif /* I8254_H */
