#ifndef GUS_H
#define GUS_H

#include <stdint.h>

typedef struct GUSState GUSState;

/* I/O port handlers - base range (0x2X0) */
uint32_t gus_read(void *opaque, uint32_t port);
void gus_write(void *opaque, uint32_t port, uint32_t val);

/* I/O port handlers - base+0x100 range (0x3X0) */
uint32_t gus_read_gf1(void *opaque, uint32_t port);
void gus_write_gf1(void *opaque, uint32_t port, uint32_t val);

/* 16-bit data port access (0x3X4) */
uint32_t gus_read_gf1_16(void *opaque, uint32_t port);
void gus_write_gf1_16(void *opaque, uint32_t port, uint32_t val);

/* Audio callback - produces stereo int16 output at 44100 Hz */
void gus_audio_callback(void *opaque, uint8_t *stream, int free);

GUSState *gus_new(
    int base_port,    /* 0x240 */
    int gus_irq,      /* e.g. 7 */
    int dma_channel,  /* e.g. 1 */
    void *isa_dma,
    void *pic,
    void (*set_irq)(void *pic, int irq, int level));

#endif /* GUS_H */
