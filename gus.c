/*
 * Gravis Ultrasound (GUS) emulation - clean room implementation
 *
 * Implements the GF1 synthesizer chip register model, 32-voice wavetable
 * synthesis, DMA upload, volume ramping, and timer functionality.
 *
 * Written from publicly available GUS SDK documentation, Linux kernel
 * headers (include/sound/gus.h), and hardware register specifications.
 */

#include "gus.h"
#include "i8257.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void *pcmalloc(long size);
void *bigmalloc(size_t size);

/* #define DEBUG_GUS */

#ifdef DEBUG_GUS
#define gus_log(...) fprintf(stderr, "gus: " __VA_ARGS__)
#else
#define gus_log(...)
#endif

/* GF1 clock: 9,878,400 Hz (half of 19.7568 MHz crystal) */
#define GF1_CLOCK 9878400

/* Output sample rate with N active voices: GF1_CLOCK / (N * 16) */
/* With 14 voices: 44100 Hz exactly */
#define OUTPUT_RATE 44100

/* GUS onboard DRAM size (1MB max) */
#define GUS_RAM_SIZE (1024 * 1024)
#define GUS_RAM_MASK (GUS_RAM_SIZE - 1)

/* Volume table size: 4096 logarithmic steps */
#define VOL_TABLE_SIZE 4096

/* IRQ status register bits (port base+0x6, active high) */
#define IRQ_STAT_MIDI_TX   0x01
#define IRQ_STAT_MIDI_RX   0x02
#define IRQ_STAT_TIMER1    0x04
#define IRQ_STAT_TIMER2    0x08
#define IRQ_STAT_WAVE      0x20
#define IRQ_STAT_RAMP      0x40
#define IRQ_STAT_DMA_TC    0x80

/* Voice control register bits */
#define VC_STOPPED      0x01
#define VC_STOP         0x02
#define VC_16BIT        0x04
#define VC_LOOP         0x08
#define VC_BIDIR        0x10
#define VC_IRQ_ENABLE   0x20
#define VC_BACKWARD     0x40
#define VC_IRQ_PENDING  0x80

/* Volume ramp control register bits */
#define RC_STOPPED      0x01
#define RC_STOP         0x02
#define RC_ROLLOVER     0x04
#define RC_LOOP         0x08
#define RC_BIDIR        0x10
#define RC_IRQ_ENABLE   0x20
#define RC_BACKWARD     0x40
#define RC_IRQ_PENDING  0x80

/* Reset register bits */
#define RESET_RUN       0x01
#define RESET_DAC       0x02
#define RESET_MASTER_IRQ 0x04

/* DMA control register bits */
#define DMA_ENABLE      0x01
#define DMA_READ        0x02  /* 0=read from GUS, 1=write to GUS (confusing naming) */
#define DMA_WIDTH_16    0x04
#define DMA_RATE0       0x08
#define DMA_RATE1       0x10
#define DMA_INVERT_MSB  0x20
#define DMA_IRQ_ENABLE  0x40
#define DMA_TC_IRQ      0x40  /* same bit, read-back means TC pending */

typedef struct GUSVoice {
    /* Control */
    uint8_t  voice_ctrl;    /* Reg 0x00: playback control */
    uint8_t  ramp_ctrl;     /* Reg 0x0D: volume ramp control */

    /* Wave addresses (fixed-point 20.9 format) */
    uint32_t start_addr;    /* Loop start */
    uint32_t end_addr;      /* Loop end */
    uint32_t current_addr;  /* Current playback position */

    /* Frequency */
    uint16_t freq_ctrl;     /* Reg 0x01: step increment */

    /* Volume (16-bit logarithmic) */
    uint16_t current_vol;   /* Reg 0x09 */
    uint8_t  ramp_start;    /* Reg 0x07: upper 8 bits of ramp start volume */
    uint8_t  ramp_end;      /* Reg 0x08: upper 8 bits of ramp end volume */
    uint8_t  ramp_rate;     /* Reg 0x06: ramp speed */

    /* Panning */
    uint8_t  pan;           /* Reg 0x0C: 0=left, 7=center, 15=right */
} GUSVoice;

struct GUSState {
    /* Hardware configuration */
    uint16_t base_port;
    uint8_t  gus_irq;
    uint8_t  dma_channel;

    /* PIC/DMA references */
    void *pic;
    void (*set_irq)(void *pic, int irq, int level);
    IsaDma *isa_dma;

    /* GF1 onboard DRAM */
    uint8_t *gus_ram;

    /* Voice state */
    GUSVoice voices[32];
    uint8_t  active_voices;     /* Number of active voices (14-32) */
    uint8_t  current_voice;     /* Selected via voice select register */

    /* Register access */
    uint8_t  reg_select;        /* Global register select */
    uint8_t  mix_control;       /* Mix control register */

    /* IRQ state */
    uint8_t  irq_status;        /* IRQ status register */

    /* Voice IRQ queue */
    uint8_t  irq_queue[32];     /* Pending voice IRQ values */
    int      irq_queue_head;
    int      irq_queue_tail;
    int      irq_queue_count;

    /* DMA state */
    uint8_t  dma_control;       /* GF1 DMA control register */
    uint16_t dma_addr;          /* DMA target address in GUS RAM */
    int      dma_tc_pending;    /* DMA terminal count IRQ pending */

    /* Timers */
    uint8_t  timer_control;     /* GF1 register 0x45 */
    uint8_t  timer1_count;      /* GF1 register 0x46 */
    uint8_t  timer2_count;      /* GF1 register 0x47 */
    uint8_t  adlib_timer_ctrl;  /* AdLib-compat timer control byte */
    int      timer1_running;
    int      timer2_running;
    uint32_t timer1_accum;      /* Accumulator (in output sample ticks) */
    uint32_t timer2_accum;

    /* DRAM I/O address (for direct peek/poke via port 0x347) */
    uint32_t dram_addr;         /* 20-bit, from regs 0x43/0x44 */

    /* Reset state */
    uint8_t  reset_reg;         /* Register 0x4C */

    /* IRQ/DMA latch mechanism */
    int      irq_latch_select;  /* bit 6 of mix_control: 1=IRQ, 0=DMA */

    /* Volume lookup table */
    int32_t  vol_table[VOL_TABLE_SIZE];

    /* Resampling state for >14 active voices */
    uint32_t resample_accum;
};

/* ----------------------------------------------------------------
 * Volume table initialization
 * 4096 logarithmic steps, ~0.0235 dB each, ~96 dB dynamic range
 * ---------------------------------------------------------------- */
static void gus_init_vol_table(GUSState *s)
{
    /* Build table: index 0 = silence, index 4095 = max amplitude.
     * Each step multiplies by ~1.002709201 (10^(0.0235/20)).
     * We store as 16-bit linear amplitude values. */
    s->vol_table[0] = 0;
    double v = 1.0;
    for (int i = VOL_TABLE_SIZE - 1; i >= 1; i--) {
        s->vol_table[i] = (int32_t)(v * 65535.0 + 0.5);
        if (s->vol_table[i] > 65535)
            s->vol_table[i] = 65535;
        v /= 1.002709201;
    }
}

/* ----------------------------------------------------------------
 * 16-bit sample address conversion
 * Upper 2 bits select 256KB bank, remaining 17 bits are word-addressed
 * ---------------------------------------------------------------- */
static inline uint32_t gus_addr_16bit(uint32_t addr)
{
    return ((addr & 0xC0000) | ((addr & 0x1FFFF) << 1)) & GUS_RAM_MASK;
}

/* ----------------------------------------------------------------
 * IRQ management
 * ---------------------------------------------------------------- */
static void gus_update_irq(GUSState *s)
{
    if (s->irq_status)
        s->set_irq(s->pic, s->gus_irq, 1);
    else
        s->set_irq(s->pic, s->gus_irq, 0);
}

static void gus_irq_queue_push(GUSState *s, uint8_t val)
{
    if (s->irq_queue_count >= 32)
        return;
    s->irq_queue[s->irq_queue_tail] = val;
    s->irq_queue_tail = (s->irq_queue_tail + 1) & 31;
    s->irq_queue_count++;
}

static uint8_t gus_irq_queue_pop(GUSState *s)
{
    if (s->irq_queue_count == 0)
        return 0xE0; /* No IRQ pending: bits 5+6 set */
    uint8_t val = s->irq_queue[s->irq_queue_head];
    s->irq_queue_head = (s->irq_queue_head + 1) & 31;
    s->irq_queue_count--;

    /* If queue is now empty, clear wave/ramp IRQ status bits */
    if (s->irq_queue_count == 0) {
        s->irq_status &= ~(IRQ_STAT_WAVE | IRQ_STAT_RAMP);
        gus_update_irq(s);
    }
    return val;
}

static void gus_raise_wave_irq(GUSState *s, int voice)
{
    if (!(s->reset_reg & RESET_MASTER_IRQ))
        return;
    s->irq_status |= IRQ_STAT_WAVE;
    /* Queue entry: voice number in bits 0-4, bit 6 set = not ramp, bit 5 clear = wave */
    gus_irq_queue_push(s, (voice & 0x1F) | 0x40);
    gus_update_irq(s);
}

static void gus_raise_ramp_irq(GUSState *s, int voice)
{
    if (!(s->reset_reg & RESET_MASTER_IRQ))
        return;
    s->irq_status |= IRQ_STAT_RAMP;
    /* Queue entry: voice number in bits 0-4, bit 6 clear = ramp IRQ */
    gus_irq_queue_push(s, (voice & 0x1F) | 0x20);
    gus_update_irq(s);
}

/* ----------------------------------------------------------------
 * DMA transfer handler (called by i8257 DMA controller)
 * Transfers data from system RAM into GUS onboard DRAM
 * ---------------------------------------------------------------- */
static int gus_dma_handler(void *opaque, int nchan, int dma_pos, int dma_len)
{
    GUSState *s = opaque;

    if (!(s->dma_control & DMA_ENABLE))
        return dma_pos;

    /* Target address in GUS DRAM: dma_addr << 4 */
    uint32_t gus_addr = (uint32_t)s->dma_addr << 4;
    int invert_msb = (s->dma_control & DMA_INVERT_MSB) ? 1 : 0;

    /* Transfer in chunks */
    uint8_t buf[256];
    int remaining = dma_len - dma_pos;
    if (remaining <= 0)
        remaining = dma_len;

    while (remaining > 0) {
        int chunk = remaining;
        if (chunk > (int)sizeof(buf))
            chunk = (int)sizeof(buf);

        int copied = i8257_dma_read_memory(s->isa_dma, nchan, buf, dma_pos, chunk);
        if (copied <= 0)
            break;

        /* Copy into GUS RAM */
        for (int i = 0; i < copied; i++) {
            uint8_t byte = buf[i];
            if (invert_msb)
                byte ^= 0x80;
            s->gus_ram[gus_addr & GUS_RAM_MASK] = byte;
            gus_addr++;
        }

        dma_pos = (dma_pos + copied) % dma_len;
        remaining -= copied;
    }

    /* Transfer complete */
    s->dma_control &= ~DMA_ENABLE;
    s->dma_tc_pending = 1;

    if (s->dma_control & DMA_IRQ_ENABLE) {
        s->irq_status |= IRQ_STAT_DMA_TC;
        gus_update_irq(s);
    }

    /* Release DMA request */
    i8257_dma_release_DREQ(s->isa_dma, s->dma_channel);

    return dma_pos;
}

/* ----------------------------------------------------------------
 * Voice synthesis
 * ---------------------------------------------------------------- */

/* Fetch a sample from GUS RAM with linear interpolation */
static inline int32_t gus_fetch_sample(GUSState *s, GUSVoice *v)
{
    uint32_t addr_int = v->current_addr >> 9;
    uint32_t frac = v->current_addr & 0x1FF;
    int32_t s0, s1;

    if (v->voice_ctrl & VC_16BIT) {
        uint32_t a0 = gus_addr_16bit(addr_int);
        uint32_t a1 = gus_addr_16bit(addr_int + 1);
        s0 = (int16_t)(s->gus_ram[a0] | (s->gus_ram[(a0 + 1) & GUS_RAM_MASK] << 8));
        s1 = (int16_t)(s->gus_ram[a1] | (s->gus_ram[(a1 + 1) & GUS_RAM_MASK] << 8));
    } else {
        s0 = ((int32_t)s->gus_ram[addr_int & GUS_RAM_MASK] - 128) << 8;
        s1 = ((int32_t)s->gus_ram[(addr_int + 1) & GUS_RAM_MASK] - 128) << 8;
    }

    /* Linear interpolation */
    return (s0 * (int32_t)(511 - frac) + s1 * (int32_t)frac) >> 9;
}

/* Apply logarithmic volume */
static inline int32_t gus_apply_volume(GUSState *s, GUSVoice *v, int32_t sample)
{
    int vol_idx = (v->current_vol >> 4) & 0xFFF;
    int32_t vol = s->vol_table[vol_idx];
    return (sample * vol) >> 16;
}

/* Handle voice reaching a boundary (start or end address) */
static void gus_voice_boundary(GUSState *s, GUSVoice *v, int voice_num)
{
    if (v->voice_ctrl & VC_BIDIR) {
        /* Bidirectional: reverse direction */
        v->voice_ctrl ^= VC_BACKWARD;
        if (v->voice_ctrl & VC_BACKWARD)
            v->current_addr = v->end_addr;
        else
            v->current_addr = v->start_addr;
    } else if (v->voice_ctrl & VC_LOOP) {
        /* Forward loop: snap back to start */
        if (v->voice_ctrl & VC_BACKWARD)
            v->current_addr = v->end_addr;
        else
            v->current_addr = v->start_addr;
    } else {
        /* No loop: stop voice */
        v->voice_ctrl |= VC_STOPPED | VC_STOP;
    }

    /* Fire wave IRQ if enabled */
    if (v->voice_ctrl & VC_IRQ_ENABLE) {
        v->voice_ctrl |= VC_IRQ_PENDING;
        gus_raise_wave_irq(s, voice_num);
    }
}

/* Advance voice playback position */
static inline void gus_advance_voice(GUSState *s, GUSVoice *v, int voice_num)
{
    uint32_t step = v->freq_ctrl >> 1;

    if (v->voice_ctrl & VC_BACKWARD) {
        if (v->current_addr < v->start_addr + step)
            gus_voice_boundary(s, v, voice_num);
        else
            v->current_addr -= step;
    } else {
        v->current_addr += step;
        if (v->current_addr >= v->end_addr)
            gus_voice_boundary(s, v, voice_num);
    }
}

/* Handle volume ramp reaching a boundary */
static void gus_ramp_boundary(GUSState *s, GUSVoice *v, int voice_num)
{
    if (v->ramp_ctrl & RC_BIDIR) {
        /* Bidirectional: reverse ramp direction */
        v->ramp_ctrl ^= RC_BACKWARD;
        if (v->ramp_ctrl & RC_BACKWARD)
            v->current_vol = (uint16_t)v->ramp_end << 8;
        else
            v->current_vol = (uint16_t)v->ramp_start << 8;
    } else if (v->ramp_ctrl & RC_LOOP) {
        /* Looping ramp: snap back */
        if (v->ramp_ctrl & RC_BACKWARD)
            v->current_vol = (uint16_t)v->ramp_end << 8;
        else
            v->current_vol = (uint16_t)v->ramp_start << 8;
    } else {
        /* Stop ramp */
        v->ramp_ctrl |= RC_STOPPED | RC_STOP;
    }

    /* Fire ramp IRQ if enabled */
    if (v->ramp_ctrl & RC_IRQ_ENABLE) {
        v->ramp_ctrl |= RC_IRQ_PENDING;
        gus_raise_ramp_irq(s, voice_num);
    }
}

/* Advance volume ramp */
static inline void gus_advance_ramp(GUSState *s, GUSVoice *v, int voice_num)
{
    if (v->ramp_ctrl & (RC_STOPPED | RC_STOP))
        return;

    /* Rate register: bits 0-5 = increment, bits 6-7 = scale (multiply by 8^scale) */
    int increment = v->ramp_rate & 0x3F;
    int scale = (v->ramp_rate >> 6) & 0x03;
    /* Scale: 0=1x, 1=8x, 2=64x, 3=512x */
    static const int scale_mult[] = {1, 8, 64, 512};
    int step = increment * scale_mult[scale];

    if (v->ramp_ctrl & RC_BACKWARD) {
        /* Descending ramp */
        int target = (uint16_t)v->ramp_start << 8;
        if ((int)v->current_vol - step <= target) {
            v->current_vol = target;
            gus_ramp_boundary(s, v, voice_num);
        } else {
            v->current_vol -= step;
        }
    } else {
        /* Ascending ramp */
        int target = (uint16_t)v->ramp_end << 8;
        if ((int)v->current_vol + step >= target) {
            v->current_vol = target;
            gus_ramp_boundary(s, v, voice_num);
        } else {
            v->current_vol += step;
        }
    }
}

/* Timer tick - called once per output sample */
static inline void gus_tick_timers(GUSState *s)
{
    /* Timer 1: 80us base period
     * At 44100 Hz output, each sample = ~22.676 us
     * Timer fires every (256 - count) * 80 us
     * We accumulate fractional ticks using a fixed-point counter.
     * Ticks per sample (scaled by 1024): 80 * 1024 / 22.676 ~= 3612 */
    if (s->timer1_running && (s->timer_control & 0x04)) {
        s->timer1_accum += 3612;
        uint32_t period = ((uint32_t)(256 - s->timer1_count)) * 1024;
        if (period == 0) period = 1;
        if (s->timer1_accum >= period) {
            s->timer1_accum -= period;
            if (!(s->adlib_timer_ctrl & 0x40)) {
                /* Timer 1 not masked */
                s->irq_status |= IRQ_STAT_TIMER1;
                gus_update_irq(s);
            }
        }
    }

    /* Timer 2: 320us base period
     * Ticks per sample (scaled by 1024): 320 * 1024 / 22.676 ~= 14449 */
    if (s->timer2_running && (s->timer_control & 0x08)) {
        s->timer2_accum += 14449;
        uint32_t period = ((uint32_t)(256 - s->timer2_count)) * 1024;
        if (period == 0) period = 1;
        if (s->timer2_accum >= period) {
            s->timer2_accum -= period;
            if (!(s->adlib_timer_ctrl & 0x20)) {
                /* Timer 2 not masked */
                s->irq_status |= IRQ_STAT_TIMER2;
                gus_update_irq(s);
            }
        }
    }
}

/* Main audio callback - produces stereo int16 output at 44100 Hz */
void gus_audio_callback(void *opaque, uint8_t *stream, int free)
{
    GUSState *s = opaque;
    int16_t *out = (int16_t *)stream;
    int num_samples = free / 4; /* stereo 16-bit = 4 bytes per frame */

    if (!(s->reset_reg & RESET_DAC)) {
        memset(stream, 0, free);
        return;
    }

    /*
     * Output rate scaling: the real GF1 runs at 9878400/(active_voices*16).
     * With 14 voices that's 44100 Hz (our output rate). With more voices the
     * internal rate drops. We handle this by stepping the voice synthesis at
     * the internal rate and producing output samples at 44100 Hz via simple
     * sample-and-hold when needed.
     */
    int nv = s->active_voices;
    if (nv < 14) nv = 14;
    if (nv > 32) nv = 32;
    uint32_t internal_rate = GF1_CLOCK / (nv * 16);

    /* Fixed-point step: how many internal ticks per output sample (16.16) */
    uint32_t step = ((uint64_t)internal_rate << 16) / OUTPUT_RATE;

    for (int i = 0; i < num_samples; i++) {
        int32_t mix_left = 0, mix_right = 0;

        /* Advance the internal synthesis by the right amount */
        s->resample_accum += step;
        int synth_ticks = s->resample_accum >> 16;
        s->resample_accum &= 0xFFFF;

        for (int tick = 0; tick < synth_ticks; tick++) {
            mix_left = 0;
            mix_right = 0;

            for (int v = 0; v < nv; v++) {
                GUSVoice *voice = &s->voices[v];

                /* Skip stopped voices */
                if (voice->voice_ctrl & (VC_STOPPED | VC_STOP))
                    continue;

                /* Fetch and process sample */
                int32_t sample = gus_fetch_sample(s, voice);
                sample = gus_apply_volume(s, voice, sample);

                /* Apply panning: 0=full left, 7-8=center, 15=full right.
                 * Split at midpoint so center gives equal amplitude. */
                int pan = voice->pan & 0x0F;
                int ls, rs;
                if (pan <= 7) { ls = 7; rs = pan; }
                else { ls = 15 - pan; rs = 7; }
                mix_left  += (sample * ls) / 7;
                mix_right += (sample * rs) / 7;

                /* Advance voice position */
                gus_advance_voice(s, voice, v);

                /* Advance volume ramp */
                gus_advance_ramp(s, voice, v);
            }

            gus_tick_timers(s);
        }

        /* If no synth ticks this output sample, hold previous value (already in mix) */
        if (synth_ticks == 0) {
            /* Produce the current state without advancing */
            for (int v = 0; v < nv; v++) {
                GUSVoice *voice = &s->voices[v];
                if (voice->voice_ctrl & (VC_STOPPED | VC_STOP))
                    continue;
                int32_t sample = gus_fetch_sample(s, voice);
                sample = gus_apply_volume(s, voice, sample);
                int pan = voice->pan & 0x0F;
                mix_left  += (sample * (15 - pan)) / 15;
                mix_right += (sample * pan) / 15;
            }
        }

        /* Clamp to 16-bit range */
        if (mix_left > 32767) mix_left = 32767;
        if (mix_left < -32768) mix_left = -32768;
        if (mix_right > 32767) mix_right = 32767;
        if (mix_right < -32768) mix_right = -32768;

        out[i * 2]     = (int16_t)mix_left;
        out[i * 2 + 1] = (int16_t)mix_right;
    }
}

/* ----------------------------------------------------------------
 * GF1 register write (via reg_select + data ports)
 * ---------------------------------------------------------------- */

/* Combine two 16-bit register values into a 32-bit fixed-point address */
static inline uint32_t gus_make_addr(uint16_t high, uint16_t low)
{
    return ((uint32_t)high << 7) | ((uint32_t)(low >> 9) & 0x7F);
}

/* Split a 32-bit fixed-point address into high 16-bit register value */
static inline uint16_t gus_addr_high(uint32_t addr)
{
    return (uint16_t)(addr >> 7);
}

/* Split a 32-bit fixed-point address into low 16-bit register value */
static inline uint16_t gus_addr_low(uint32_t addr)
{
    return (uint16_t)((addr & 0x7F) << 9);
}

static void gus_reg_write16(GUSState *s, uint8_t reg, uint16_t val)
{
    GUSVoice *v = &s->voices[s->current_voice & 0x1F];

    switch (reg) {
    case 0x01: /* Frequency control */
        v->freq_ctrl = val;
        break;
    case 0x02: /* Start address high */
        v->start_addr = (v->start_addr & 0x7F) | ((uint32_t)val << 7);
        break;
    case 0x03: /* Start address low */
        v->start_addr = (v->start_addr & ~0x7F) | ((val >> 9) & 0x7F);
        break;
    case 0x04: /* End address high */
        v->end_addr = (v->end_addr & 0x7F) | ((uint32_t)val << 7);
        break;
    case 0x05: /* End address low */
        v->end_addr = (v->end_addr & ~0x7F) | ((val >> 9) & 0x7F);
        break;
    case 0x09: /* Current volume */
        v->current_vol = val;
        break;
    case 0x0A: /* Current address high */
        v->current_addr = (v->current_addr & 0x7F) | ((uint32_t)val << 7);
        break;
    case 0x0B: /* Current address low */
        v->current_addr = (v->current_addr & ~0x7F) | ((val >> 9) & 0x7F);
        break;
    case 0x42: /* DMA start address */
        s->dma_addr = val;
        break;
    case 0x43: /* DRAM address low */
        s->dram_addr = (s->dram_addr & 0xF0000) | val;
        break;
    case 0x44: /* DRAM address high (only 4 bits used) */
        /* This is actually only 8-bit via data_high port, but handle 16-bit too */
        s->dram_addr = (s->dram_addr & 0x0FFFF) | (((uint32_t)val & 0x0F) << 16);
        break;
    default:
        gus_log("reg_write16 unhandled reg %02x = %04x\n", reg, val);
        break;
    }
}

static void gus_reg_write8(GUSState *s, uint8_t reg, uint8_t val)
{
    GUSVoice *v = &s->voices[s->current_voice & 0x1F];

    switch (reg) {
    case 0x00: /* Voice control */
        v->voice_ctrl = val & 0x7F; /* Bit 7 (IRQ pending) is read-only */
        break;
    case 0x06: /* Volume ramp rate */
        v->ramp_rate = val;
        break;
    case 0x07: /* Volume ramp start */
        v->ramp_start = val;
        break;
    case 0x08: /* Volume ramp end */
        v->ramp_end = val;
        break;
    case 0x0C: /* Pan position */
        v->pan = val & 0x0F;
        break;
    case 0x0D: /* Volume ramp control */
        v->ramp_ctrl = val & 0x7F; /* Bit 7 (IRQ pending) is read-only */
        break;
    case 0x0E: /* Active voice count (write: set to N-1, range 13..31) */
        s->active_voices = (val & 0x3F) + 1;
        if (s->active_voices < 14) s->active_voices = 14;
        if (s->active_voices > 32) s->active_voices = 32;
        break;
    case 0x41: /* DMA control */
        s->dma_control = val;
        if (val & DMA_ENABLE) {
            /* Start DMA transfer */
            s->dma_tc_pending = 0;
            i8257_dma_hold_DREQ(s->isa_dma, s->dma_channel);
        }
        break;
    case 0x44: /* DRAM address high (8-bit access) */
        s->dram_addr = (s->dram_addr & 0x0FFFF) | (((uint32_t)val & 0x0F) << 16);
        break;
    case 0x45: /* Timer control */
        s->timer_control = val;
        break;
    case 0x46: /* Timer 1 count */
        s->timer1_count = val;
        break;
    case 0x47: /* Timer 2 count */
        s->timer2_count = val;
        break;
    case 0x4C: /* Reset register */
        {
            uint8_t old = s->reset_reg;
            /* When bit 0 transitions, bits 1-2 are auto-cleared */
            if ((old & RESET_RUN) != (val & RESET_RUN)) {
                s->reset_reg = val & RESET_RUN;
            } else {
                s->reset_reg = val & 0x07;
            }

            /* If entering reset (bit 0 = 0), stop everything */
            if (!(s->reset_reg & RESET_RUN)) {
                for (int i = 0; i < 32; i++) {
                    s->voices[i].voice_ctrl |= VC_STOPPED | VC_STOP;
                    s->voices[i].ramp_ctrl |= RC_STOPPED | RC_STOP;
                }
                s->irq_status = 0;
                s->irq_queue_head = 0;
                s->irq_queue_tail = 0;
                s->irq_queue_count = 0;
                gus_update_irq(s);
            }
        }
        break;
    default:
        gus_log("reg_write8 unhandled reg %02x = %02x\n", reg, val);
        break;
    }
}

/* ----------------------------------------------------------------
 * GF1 register read
 * ---------------------------------------------------------------- */
static uint16_t gus_reg_read16(GUSState *s, uint8_t reg)
{
    GUSVoice *v = &s->voices[s->current_voice & 0x1F];

    switch (reg) {
    case 0x80: return v->voice_ctrl | (v->voice_ctrl & VC_IRQ_PENDING ? 0x80 : 0);
    case 0x81: return v->freq_ctrl;
    case 0x82: return gus_addr_high(v->start_addr);
    case 0x83: return gus_addr_low(v->start_addr);
    case 0x84: return gus_addr_high(v->end_addr);
    case 0x85: return gus_addr_low(v->end_addr);
    case 0x89: return v->current_vol;
    case 0x8A: return gus_addr_high(v->current_addr);
    case 0x8B: return gus_addr_low(v->current_addr);
    default:
        gus_log("reg_read16 unhandled reg %02x\n", reg);
        return 0;
    }
}

static uint8_t gus_reg_read8(GUSState *s, uint8_t reg)
{
    GUSVoice *v = &s->voices[s->current_voice & 0x1F];

    switch (reg) {
    case 0x80: /* Voice control (read-back) */
        return v->voice_ctrl;
    case 0x82: /* Start address high (upper byte) */
        return (uint8_t)(gus_addr_high(v->start_addr) >> 8);
    case 0x83: /* Start address low (upper byte) */
        return (uint8_t)(gus_addr_low(v->start_addr) >> 8);
    case 0x84: /* End address high (upper byte) */
        return (uint8_t)(gus_addr_high(v->end_addr) >> 8);
    case 0x85: /* End address low (upper byte) */
        return (uint8_t)(gus_addr_low(v->end_addr) >> 8);
    case 0x86: /* Volume ramp rate */
        return v->ramp_rate;
    case 0x87: /* Volume ramp start */
        return v->ramp_start;
    case 0x88: /* Volume ramp end */
        return v->ramp_end;
    case 0x89: /* Current volume (upper byte) */
        return (uint8_t)(v->current_vol >> 8);
    case 0x8A: /* Current address high (upper byte) */
        return (uint8_t)(gus_addr_high(v->current_addr) >> 8);
    case 0x8B: /* Current address low (upper byte) */
        return (uint8_t)(gus_addr_low(v->current_addr) >> 8);
    case 0x8C: /* Pan position */
        return v->pan;
    case 0x8D: /* Volume ramp control (read-back) */
        return v->ramp_ctrl;
    case 0x8E: /* Active voice count (read-back) */
        return (s->active_voices - 1) & 0x3F;
    case 0x8F: /* Voice IRQ read - pops from IRQ queue */
        return gus_irq_queue_pop(s);
    case 0x41: /* DMA control read-back */
        {
            uint8_t ret = s->dma_control;
            if (s->dma_tc_pending) {
                ret |= DMA_TC_IRQ;
                /* Reading clears the DMA TC IRQ */
                s->dma_tc_pending = 0;
                s->irq_status &= ~IRQ_STAT_DMA_TC;
                gus_update_irq(s);
            }
            return ret;
        }
    case 0x45: return s->timer_control;
    case 0x49: /* Record DMA control - stub */
        return 0;
    case 0x4C: return s->reset_reg;
    default:
        gus_log("reg_read8 unhandled reg %02x\n", reg);
        return 0;
    }
}

/* ----------------------------------------------------------------
 * I/O port handlers - base range (0x2X0)
 * ---------------------------------------------------------------- */
uint32_t gus_read(void *opaque, uint32_t port)
{
    GUSState *s = opaque;
    uint32_t offset = port - s->base_port;

    switch (offset) {
    case 0x00: /* Mix control register (write-only, reads return 0xFF) */
        return 0xFF;
    case 0x06: /* IRQ status register */
        return s->irq_status;
    case 0x08: /* Timer control (AdLib-compatible status) */
        {
            /* AdLib-compatible timer status:
             * Bit 7: IRQ flag (any timer)
             * Bit 6: Timer 1 overflow
             * Bit 5: Timer 2 overflow */
            uint8_t ret = 0;
            if (s->irq_status & IRQ_STAT_TIMER1)
                ret |= 0xC0; /* bit 7 + bit 6 */
            if (s->irq_status & IRQ_STAT_TIMER2)
                ret |= 0xA0; /* bit 7 + bit 5 */
            return ret;
        }
    case 0x0F: /* Board revision / IRQDMA control read */
        return 0x05; /* GUS Classic rev 3.7+ identifier */
    default:
        return 0xFF;
    }
}

void gus_write(void *opaque, uint32_t port, uint32_t val)
{
    GUSState *s = opaque;
    uint32_t offset = port - s->base_port;

    switch (offset) {
    case 0x00: /* Mix control register */
        s->mix_control = val;
        s->irq_latch_select = (val >> 6) & 1;
        break;
    case 0x08: /* Timer control (AdLib-compatible) - register select */
        /* Games write 0x04 here to select timer control mode */
        break;
    case 0x09: /* Timer data (AdLib-compatible) */
        s->adlib_timer_ctrl = val;
        if (val & 0x80) {
            /* Reset timer IRQ flags */
            s->irq_status &= ~(IRQ_STAT_TIMER1 | IRQ_STAT_TIMER2);
            gus_update_irq(s);
        }
        s->timer1_running = (val & 0x01) ? 1 : 0;
        s->timer2_running = (val & 0x02) ? 1 : 0;
        break;
    case 0x0B: /* IRQ/DMA control register (latched by mix control bit 6) */
        if (s->irq_latch_select) {
            /* IRQ channel select - we ignore this since IRQ is fixed in config */
            gus_log("IRQ latch write: %02x\n", val);
        } else {
            /* DMA channel select - we ignore this since DMA is fixed in config */
            gus_log("DMA latch write: %02x\n", val);
        }
        break;
    case 0x0F: /* Board register control */
        break;
    default:
        gus_log("base write unhandled offset %02x = %02x\n", offset, val);
        break;
    }
}

/* ----------------------------------------------------------------
 * I/O port handlers - GF1 range (base + 0x100)
 * ---------------------------------------------------------------- */
uint32_t gus_read_gf1(void *opaque, uint32_t port)
{
    GUSState *s = opaque;
    uint32_t offset = port - (s->base_port + 0x100);

    switch (offset) {
    case 0x00: /* MIDI control/status (stub) */
        return 0;
    case 0x01: /* MIDI data (stub) */
        return 0xFF;
    case 0x02: /* Voice select (write-only) */
        return s->current_voice;
    case 0x03: /* Register select (write-only typically, but readable) */
        return s->reg_select;
    case 0x04: /* Data low - 16-bit read (low byte only for 8-bit access) */
        return gus_reg_read16(s, s->reg_select) & 0xFF;
    case 0x05: /* Data high - 8-bit read */
        return gus_reg_read8(s, s->reg_select);
    case 0x07: /* DRAM I/O - read byte from GUS RAM */
        return s->gus_ram[s->dram_addr & GUS_RAM_MASK];
    default:
        return 0xFF;
    }
}

void gus_write_gf1(void *opaque, uint32_t port, uint32_t val)
{
    GUSState *s = opaque;
    uint32_t offset = port - (s->base_port + 0x100);

    switch (offset) {
    case 0x00: /* MIDI control (stub) */
        break;
    case 0x01: /* MIDI data (stub) */
        break;
    case 0x02: /* Voice select */
        s->current_voice = val & 0x1F;
        break;
    case 0x03: /* Register select */
        s->reg_select = val;
        break;
    case 0x04: /* Data low - 8-bit write (low byte of 16-bit register) */
        /* 8-bit writes to 0x3X4 - rarely used alone, usually 16-bit */
        gus_reg_write8(s, s->reg_select, val);
        break;
    case 0x05: /* Data high - 8-bit write */
        gus_reg_write8(s, s->reg_select, val);
        break;
    case 0x07: /* DRAM I/O - write byte to GUS RAM */
        s->gus_ram[s->dram_addr & GUS_RAM_MASK] = val;
        break;
    default:
        gus_log("gf1 write unhandled offset %02x = %02x\n", offset, val);
        break;
    }
}

/* 16-bit data port access */
uint32_t gus_read_gf1_16(void *opaque, uint32_t port)
{
    GUSState *s = opaque;
    return gus_reg_read16(s, s->reg_select);
}

void gus_write_gf1_16(void *opaque, uint32_t port, uint32_t val)
{
    GUSState *s = opaque;
    gus_reg_write16(s, s->reg_select, val & 0xFFFF);
}

/* ----------------------------------------------------------------
 * AdLib-compatible timer ports (0x388/0x389)
 * When GUS is active, these ports are handled by the GUS instead of
 * the OPL chip, providing timer functionality for GUS drivers.
 * ---------------------------------------------------------------- */
static uint8_t adlib_reg_select; /* Latched AdLib register address */

uint32_t gus_adlib_read(void *opaque, uint32_t port)
{
    GUSState *s = opaque;
    /* Port 0x388: AdLib status register
     * Bit 7: IRQ flag (any timer)
     * Bit 6: Timer 1 overflow
     * Bit 5: Timer 2 overflow */
    uint8_t ret = 0;
    if (s->irq_status & IRQ_STAT_TIMER1)
        ret |= 0xC0;
    if (s->irq_status & IRQ_STAT_TIMER2)
        ret |= 0xA0;
    return ret;
}

void gus_adlib_write(void *opaque, uint32_t port, uint32_t val)
{
    GUSState *s = opaque;
    if ((port & 0xF) == 0x8) {
        /* Port 0x388: register address select */
        adlib_reg_select = val;
    } else {
        /* Port 0x389: register data write */
        if (adlib_reg_select == 0x04) {
            /* AdLib register 0x04 = timer control */
            s->adlib_timer_ctrl = val;
            if (val & 0x80) {
                /* Reset timer IRQ flags */
                s->irq_status &= ~(IRQ_STAT_TIMER1 | IRQ_STAT_TIMER2);
                gus_update_irq(s);
            }
            s->timer1_running = (val & 0x01) ? 1 : 0;
            s->timer2_running = (val & 0x02) ? 1 : 0;
        }
        /* Other AdLib registers (0x01, 0x02, 0x03) are timer count values
         * on the real OPL chip, but GUS uses its own GF1 registers (0x46/0x47)
         * for timer counts, so we ignore non-0x04 writes here. */
    }
}

/* ----------------------------------------------------------------
 * Initialization
 * ---------------------------------------------------------------- */
GUSState *gus_new(
    int base_port,
    int gus_irq,
    int dma_channel,
    void *isa_dma,
    void *pic,
    void (*set_irq)(void *pic, int irq, int level))
{
    GUSState *s = pcmalloc(sizeof(GUSState));
    memset(s, 0, sizeof(GUSState));

    s->base_port = base_port;
    s->gus_irq = gus_irq;
    s->dma_channel = dma_channel;
    s->isa_dma = isa_dma;
    s->pic = pic;
    s->set_irq = set_irq;

    /* Allocate GUS onboard DRAM (1MB from PSRAM on ESP32) */
    s->gus_ram = bigmalloc(GUS_RAM_SIZE);
    memset(s->gus_ram, 0, GUS_RAM_SIZE);

    /* Default: 14 active voices (44100 Hz output) */
    s->active_voices = 14;

    /* Start in reset state */
    s->reset_reg = 0;
    for (int i = 0; i < 32; i++) {
        s->voices[i].voice_ctrl = VC_STOPPED | VC_STOP;
        s->voices[i].ramp_ctrl = RC_STOPPED | RC_STOP;
    }

    /* Build volume lookup table */
    gus_init_vol_table(s);

    /* Register DMA handler */
    i8257_dma_register_channel(s->isa_dma, s->dma_channel, gus_dma_handler, s);

    gus_log("initialized at port %03x, IRQ %d, DMA %d\n",
            base_port, gus_irq, dma_channel);

    return s;
}
