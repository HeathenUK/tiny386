/*
 * Dummy VGA device
 * 
 * Copyright (c) 2003-2017 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include "vga.h"
#include "pci.h"

#include "esp_attr.h"
#include "esp_timer.h"
#include "common.h"

#include "driver/bitscrambler_loopback.h"

void *pcmalloc(long size);
/* Allocate from PSRAM outside emulator pool (for caches, etc.) */
#include "esp_heap_caps.h"
static inline void *psram_malloc(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
}

/* PIE (SIMD) functions for ESP32-P4 - defined in vga_pie.S */
extern void pie_write_8pixels(uint16_t *dst, uint16_t *colors);
extern void pie_write_8pixels_doubled(uint16_t *dst, uint16_t *colors);
extern void pie_memcpy_128(void *dst, const void *src, size_t n);
extern void pie_render_8pixels_mode13(uint16_t *dst, const uint8_t *indices, const uint16_t *palette);

//#define DEBUG_VBE
//#define DEBUG_VGA_REG

/*
 * Planar expansion lookup table for EGA/VGA 16-color modes.
 * For each byte value (0-255), provides the 8 individual bits (MSB first).
 * planar_expand[byte][bit_idx] = (byte >> (7 - bit_idx)) & 1
 *
 * This replaces expensive per-pixel bit shifting with a simple table lookup,
 * dramatically improving EGA mode rendering performance.
 */
/* No DRAM_ATTR: with BitScrambler active this table is only used on the
 * software fallback path, so PSRAM placement is acceptable.  Frees 2KB
 * internal SRAM for BitScrambler DMA buffers. */
static const uint8_t planar_expand[256][8] = {
#define PE(n) {((n)>>7)&1,((n)>>6)&1,((n)>>5)&1,((n)>>4)&1,((n)>>3)&1,((n)>>2)&1,((n)>>1)&1,(n)&1}
#define PE4(n) PE(n),PE(n+1),PE(n+2),PE(n+3)
#define PE16(n) PE4(n),PE4(n+4),PE4(n+8),PE4(n+12)
#define PE64(n) PE16(n),PE16(n+16),PE16(n+32),PE16(n+48)
    PE64(0), PE64(64), PE64(128), PE64(192)
#undef PE
#undef PE4
#undef PE16
#undef PE64
};

/*
 * BitScrambler-accelerated planar VGA rendering.
 * Converts 4 interleaved plane bytes to 8 packed 4-bit pixel indices in HW,
 * ~40x faster than the software planar_expand table approach.
 * Falls back to software path if BitScrambler init fails.
 */
BITSCRAMBLER_PROGRAM(bs_planar_prog, "planar_bs");

static bitscrambler_handle_t bs_planar_handle = NULL;

/* Batch up to 32 dirty lines per BitScrambler DMA call (amortizes overhead) */
#define BS_BATCH_LINES 32
#define BS_MAX_LINE_BYTES 324  /* 80 bytes/plane * 4 + 4 for pixel panning */
#define BS_DMA_PAD 64          /* Extra bytes to absorb FIFO drain (must be cache-line aligned) */
static uint8_t *bs_dma_in = NULL;   /* Gathered planar VRAM data + padding */
static uint8_t *bs_dma_out = NULL;  /* Packed I4 output from BitScrambler + padding */

/* Palette pair LUT: maps byte (2 packed nibbles) -> 2 RGB565 pixels as uint32_t */
static uint32_t bs_pal_pair[256];
/* Palette double LUT: maps nibble (1 pixel index) -> doubled RGB565 pixel */
static uint32_t bs_pal_double[16];
static uint16_t bs_pal_single[16];  /* Single nibble → RGB565 for odd panning */
/* bs_pal_last_hash: hash of palette used to build LUTs — 0 means "never built" */
static uint32_t bs_pal_last_hash = 0;

/* Debug flags for tracing fast path usage (reset on mode change) */
static int ega_fast_logged = 0;
static int vga256_fast_logged = 0;

/* VGA I/O trace ring buffer for crash diagnostics.
 * Records last N VGA port I/O and memory accesses.
 * Dumped on VMM crash to diagnose VGA driver init failures. */
#define VGA_TRACE_SIZE 2048
typedef struct {
	uint32_t addr;     /* port address or VGA memory offset */
	uint32_t val;      /* value read or written */
	uint8_t type;      /* 0=port_rd, 1=port_wr, 2=mem_rd, 3=mem_wr */
	uint8_t sr2;       /* plane write mask at time of access */
	uint8_t gr4;       /* read map select at time of access */
	uint8_t gr5;       /* mode register (write mode + read mode) */
} VGATraceEntry;
static VGATraceEntry vga_trace_ring[VGA_TRACE_SIZE];
static int vga_trace_idx = 0;
static uint32_t vga_trace_total = 0;
/* vga_trace_record and vga_dump_io_trace defined after VGAState struct */

/*
 * Mode 13h Performance Optimizations for ESP32-P4
 *
 * 1. DRAM Palette Cache: 256-entry RGB565 palette in internal SRAM (not PSRAM)
 *    Note: IRAM_ATTR crashes on ESP32-P4 RISC-V because IRAM requires 32-bit
 *    aligned access, but uint16_t allows 16-bit access. DRAM_ATTR places data
 *    in internal SRAM without this restriction.
 *    Palette lookups: ~3-5 cycles (DRAM) vs ~20+ cycles (PSRAM)
 *
 * 2. Dirty Line Tracking: Bitmap of which scanlines changed since last frame
 *    Skip rendering unchanged lines (helps scrollers, partial updates)
 *
 * 3. VRAM Line Prefetch: Copy scanline to internal SRAM for faster access
 *
 * References:
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-guides/memory-types.html
 */

/* Palette cache in internal SRAM - 256 entries * 2 bytes = 512 bytes
 * DRAM_ATTR places in internal SRAM (faster than PSRAM) */
static DRAM_ATTR uint16_t vga256_palette_cache[256] __attribute__((aligned(4)));
static int vga256_palette_valid = 0;

/* Adaptive frame skipping - skip up to N frames when rendering is slow
 * When enabled, skips frames if previous render exceeded threshold */
int vga_frame_skip_max = 0;  /* 0 = disabled, 1-10 = max frames to skip */

static int vga_frame_skip_count = 0;  /* Consecutive frames skipped */
static uint64_t vga_last_render_time = 0;  /* Time of last render (microseconds) */
static uint64_t vga_last_render_duration = 0;  /* Duration of last render */
#define VGA_FRAME_SKIP_THRESHOLD_US 40000  /* Skip if render takes >40ms */

/* Dirty line tracking in internal SRAM.
 * 512 covers Mode 12h (480 lines) and all smaller modes. */
#define VGA_MAX_LINES 512
static DRAM_ATTR uint32_t vga_dirty_lines[VGA_MAX_LINES / 32];
static DRAM_ATTR uint32_t vga_prev_line_hash[VGA_MAX_LINES];

/* Fast hash function for change detection (FNV-1a variant) */
static inline uint32_t vga_hash_line(const uint8_t *data, int len)
{
    uint32_t hash = 2166136261u;
    /* Process 16 bytes at a time for speed */
    const uint32_t *data32 = (const uint32_t *)data;
    while (len >= 16) {
        hash ^= data32[0];
        hash *= 16777619u;
        hash ^= data32[1];
        hash *= 16777619u;
        hash ^= data32[2];
        hash *= 16777619u;
        hash ^= data32[3];
        hash *= 16777619u;
        data32 += 4;
        len -= 16;
    }
    data = (const uint8_t *)data32;
    while (len >= 4) {
        hash ^= *(const uint32_t *)data;
        hash *= 16777619u;
        data += 4;
        len -= 4;
    }
    return hash;
}

/* Mark a scanline as dirty */
static inline void vga_mark_line_dirty(int line)
{
    if (line < VGA_MAX_LINES)
        vga_dirty_lines[line >> 5] |= (1u << (line & 31));
}

/* Check if a scanline is dirty */
static inline int vga_is_line_dirty(int line)
{
    if (line >= VGA_MAX_LINES) return 1;
    return (vga_dirty_lines[line >> 5] >> (line & 31)) & 1;
}

/* Clear all dirty flags (call after frame rendered) */
static inline void vga_clear_dirty_lines(void)
{
    for (int i = 0; i < VGA_MAX_LINES / 32; i++)
        vga_dirty_lines[i] = 0;
}

/* Mark all lines dirty (call on mode change or palette change) */
static inline void vga_mark_all_dirty(void)
{
    for (int i = 0; i < VGA_MAX_LINES / 32; i++)
        vga_dirty_lines[i] = 0xFFFFFFFF;
    bs_pal_last_hash = 0;  /* Force palette LUT rebuild on next frame */
}

/* Invalidate palette cache (call when DAC registers change) */
static inline void vga256_invalidate_palette(void)
{
    vga256_palette_valid = 0;
    vga_mark_all_dirty();
}

/*
 * Initialize BitScrambler for hardware-accelerated planar VGA rendering.
 * Allocates DMA buffers in internal SRAM (preferred) or PSRAM (fallback).
 * Falls back to software rendering if BitScrambler setup fails entirely.
 */
void vga_bitscrambler_init(void)
{
    if (bs_planar_handle)
        return;  /* Already initialized */

    size_t buf_size = BS_BATCH_LINES * BS_MAX_LINE_BYTES + BS_DMA_PAD;

    esp_err_t err = bitscrambler_loopback_create(
        &bs_planar_handle,
        SOC_BITSCRAMBLER_ATTACH_PARL_IO,  /* Unused peripheral, just need DMA channel */
        buf_size);
    if (err != ESP_OK) {
        fprintf(stderr, "VGA: WARNING: BitScrambler init failed (err=%d), using software planar path\n", err);
        bs_planar_handle = NULL;
        return;
    }

    err = bitscrambler_load_program(bs_planar_handle, bs_planar_prog);
    if (err != ESP_OK) {
        fprintf(stderr, "VGA: WARNING: BitScrambler program load failed (err=%d), using software planar path\n", err);
        bitscrambler_free(bs_planar_handle);
        bs_planar_handle = NULL;
        return;
    }

    /* Try internal SRAM first for DMA buffers (lowest latency, cache-coherent) */
    bs_dma_in = heap_caps_aligned_calloc(64, 1, buf_size,
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    bs_dma_out = heap_caps_aligned_calloc(64, 1, buf_size,
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (bs_dma_in && bs_dma_out) {
        fprintf(stderr, "VGA: BitScrambler planar accel ready (2x%uB DMA in SRAM)\n",
                (unsigned)buf_size);
        return;
    }

    /* SRAM allocation failed — free partial and try PSRAM */
    free(bs_dma_in);
    free(bs_dma_out);
    bs_dma_in = NULL;
    bs_dma_out = NULL;

    fprintf(stderr, "VGA: WARNING: SRAM too tight for %uB DMA buffers, trying PSRAM\n",
            (unsigned)(buf_size * 2));

    bs_dma_in = heap_caps_aligned_calloc(64, 1, buf_size,
        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    bs_dma_out = heap_caps_aligned_calloc(64, 1, buf_size,
        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);

    if (bs_dma_in && bs_dma_out) {
        fprintf(stderr, "VGA: BitScrambler planar accel ready (2x%uB DMA in PSRAM — slower)\n",
                (unsigned)buf_size);
        return;
    }

    /* Even PSRAM failed — fall back to software */
    free(bs_dma_in);
    free(bs_dma_out);
    bs_dma_in = NULL;
    bs_dma_out = NULL;
    bitscrambler_free(bs_planar_handle);
    bs_planar_handle = NULL;
    fprintf(stderr, "VGA: WARNING: DMA buffer alloc failed, using software planar path\n");
}

#define MSR_COLOR_EMULATION 0x01
#define MSR_PAGE_SELECT     0x20

#define ST01_V_RETRACE      0x08
#define ST01_DISP_ENABLE    0x01

#define VBE_DISPI_INDEX_ID              0x0
#define VBE_DISPI_INDEX_XRES            0x1
#define VBE_DISPI_INDEX_YRES            0x2
#define VBE_DISPI_INDEX_BPP             0x3
#define VBE_DISPI_INDEX_ENABLE          0x4
#define VBE_DISPI_INDEX_BANK            0x5
#define VBE_DISPI_INDEX_VIRT_WIDTH      0x6
#define VBE_DISPI_INDEX_VIRT_HEIGHT     0x7
#define VBE_DISPI_INDEX_X_OFFSET        0x8
#define VBE_DISPI_INDEX_Y_OFFSET        0x9
#define VBE_DISPI_INDEX_VIDEO_MEMORY_64K 0xa
#define VBE_DISPI_INDEX_NB              0xb

#define VBE_DISPI_ID0                   0xB0C0
#define VBE_DISPI_ID1                   0xB0C1
#define VBE_DISPI_ID2                   0xB0C2
#define VBE_DISPI_ID3                   0xB0C3
#define VBE_DISPI_ID4                   0xB0C4
#define VBE_DISPI_ID5                   0xB0C5

#define VBE_DISPI_DISABLED              0x00
#define VBE_DISPI_ENABLED               0x01
#define VBE_DISPI_GETCAPS               0x02
#define VBE_DISPI_8BIT_DAC              0x20
#define VBE_DISPI_LFB_ENABLED           0x40
#define VBE_DISPI_NOCLEARMEM            0x80

#define FB_ALLOC_ALIGN (1 << 20)

#define MAX_TEXT_WIDTH 132
#define MAX_TEXT_HEIGHT 60

struct FBDevice {
    /* the following is set by the device */
    int width;
    int height;
    int stride; /* current stride in bytes */
    uint8_t *fb_data; /* current pointer to the pixel data */
};

struct VGAState {
    FBDevice *fb_dev;
    int graphic_mode;
    uint32_t cursor_blink_time;
    int cursor_visible_phase;
    uint32_t retrace_time;
    int retrace_phase;
    int render_pending;  /* Set by 3DA polling optimization when V_RETRACE entered */
    int force_8dm;
    int force_graphic_clear;  /* Force clear when switching to graphics mode */

    uint8_t *vga_ram;
    uint64_t dirty_pages;  /* Written by VRAM stores but unused (TE sync replaces double buffering) */
    int vga_ram_size;

    uint8_t sr_index;
    uint8_t sr[8];
    uint8_t gr_index;
    uint8_t gr[16];
    uint8_t ar_index;
    uint8_t ar[21];
    int ar_flip_flop;
    uint8_t cr_index;
    uint8_t cr[256]; /* CRT registers */
    uint8_t msr; /* Misc Output Register */
    uint8_t fcr; /* Feature Control Register */
    uint8_t st00; /* status 0 */
    uint8_t st01; /* status 1 */
    uint8_t dac_state;
    uint8_t dac_sub_index;
    uint8_t dac_read_index;
    uint8_t dac_write_index;
    uint8_t dac_8bit;
    uint8_t dac_cache[3]; /* used when writing */
    uint8_t palette[768];
    uint8_t palette_before_switch[768]; /* For mid-frame palette switching (top) */
    uint8_t palette_after_switch[768];  /* For mid-frame palette switching (bottom) */
    int palette_switch_scanline;        /* -1 if no switch, else scanline number */
    uint32_t palette_generation;        /* Incremented on DAC writes for dirty tracking */
    uint32_t palette_before_gen;        /* Generation of palette_before_switch */
    uint32_t palette_after_gen;         /* Generation of palette_after_switch */
    int hblank_poll_count;              /* Count of 0x3DA reads (hblank counting) */
    /* Render snapshots - copied at start of render to avoid race conditions */
    uint8_t palette_snapshot[768];
    uint8_t palette_before_snapshot[768];
    uint8_t palette_after_snapshot[768];
    int palette_switch_snapshot;        /* Snapshot of palette_switch_scanline */
    int32_t bank_offset;

    uint32_t latch;

    /* Mode X optimization: cached state for fast-path writes */
    uint8_t cached_plane_mask;    /* sr[VGA_SEQ_PLANE_WRITE] - which planes to write */
    uint8_t cached_write_mode;    /* gr[VGA_GFX_MODE] & 3 - VGA write mode 0-3 */
    uint8_t cached_chain4;        /* sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M */
    uint8_t cached_memmap_mode;   /* (gr[VGA_GFX_MISC] >> 2) & 3 */
    uint8_t cached_simple_write;  /* 0=slow, 1=wm1 fast, 2=wm0 fast */

    /* text mode state */
    uint32_t last_palette[16];
#ifndef FULL_UPDATE
    uint16_t last_ch_attr[MAX_TEXT_WIDTH * MAX_TEXT_HEIGHT];
#endif
    uint32_t last_width;
    uint32_t last_height;
    uint16_t last_line_offset;
    uint16_t last_start_addr;
    uint16_t last_cursor_offset;
    uint8_t last_cursor_start;
    uint8_t last_cursor_end;

#ifdef TEXT_RENDER_OPT
    /* Text rendering optimizations */
    uint32_t text_hash;           /* CRC32 of text buffer for frame skip */
    uint32_t last_text_start;     /* Last text buffer start address */
    uint32_t last_text_size;      /* Last text buffer size */
    int scroll_lines;             /* Detected scroll amount (positive=up, negative=down) */

    /* Glyph tile cache: LRU cache of pre-rendered glyph+fg+bg combinations */
    #define GLYPH_CACHE_SIZE 512  /* 512 entries * 256 bytes = 128KB (allocated from free PSRAM) */
    #define GLYPH_TILE_SIZE (8 * 16 * 2)  /* 8x16 pixels * 2 bytes RGB565 */
    uint8_t *glyph_cache_tiles;   /* GLYPH_CACHE_SIZE * GLYPH_TILE_SIZE bytes */
    uint32_t glyph_cache_keys[GLYPH_CACHE_SIZE];  /* char | (fg << 8) | (bg << 12) | (cheight << 16) */
    uint8_t glyph_cache_lru[GLYPH_CACHE_SIZE];    /* LRU counter */
    uint8_t glyph_cache_counter;  /* Global LRU counter */

    /* Font IRAM cache */
    uint8_t *font_cache;          /* 256 * 16 * 4 = 16KB in IRAM */
    uint32_t font_cache_base;     /* VGA RAM offset of cached font */
#endif

    /* VBE extension */
    uint16_t vbe_index;
    uint16_t vbe_regs[VBE_DISPI_INDEX_NB];
    uint32_t vbe_start_addr;
    uint32_t vbe_line_offset;

#if defined(SCALE_3_2) || defined(SWAPXY)
#ifndef LCD_WIDTH
#define LCD_WIDTH 2048
#endif
    uint8_t tmpbuf[(LCD_WIDTH > 720 ? LCD_WIDTH : 720) * 3 * 2];
#endif
};

static VGAState *vga_trace_state = NULL;

static inline void vga_trace_record(uint32_t addr, uint32_t val, uint8_t type, VGAState *s) {
	VGATraceEntry *e = &vga_trace_ring[vga_trace_idx];
	e->addr = addr;
	e->val = val;
	e->type = type;
	e->sr2 = s->sr[0x02];  /* plane write mask */
	e->gr4 = s->gr[0x04];  /* read map select */
	e->gr5 = s->gr[0x05];  /* mode register */
	vga_trace_idx = (vga_trace_idx + 1) & (VGA_TRACE_SIZE - 1);
	vga_trace_total++;
}

void vga_dump_io_trace(void) {
	if (!vga_trace_state) return;
	VGAState *s = vga_trace_state;
	static const char *type_names[] = {"PR", "PW", "MR", "MW"};

	/* Dump current VGA register state */
	fprintf(stderr, "=== VGA state: msr=%02x sr1=%02x sr2=%02x sr4=%02x "
		"gr0=%02x gr1=%02x gr3=%02x gr4=%02x gr5=%02x gr6=%02x gr8=%02x "
		"ar10=%02x cr17=%02x ===\n",
		s->msr, s->sr[1], s->sr[2], s->sr[4],
		s->gr[0], s->gr[1], s->gr[3], s->gr[4], s->gr[5], s->gr[6], s->gr[8],
		s->ar[0x10], s->cr[0x17]);
	fprintf(stderr, "  CRTC: cr01=%02x cr06=%02x cr07=%02x cr12=%02x\n",
		s->cr[0x01], s->cr[0x06], s->cr[0x07], s->cr[0x12]);

	/* Dump ring buffer (oldest first) */
	int count = (vga_trace_total < VGA_TRACE_SIZE) ? (int)vga_trace_total : VGA_TRACE_SIZE;
	int start = (vga_trace_total < VGA_TRACE_SIZE) ? 0 :
		(vga_trace_idx & (VGA_TRACE_SIZE - 1));
	fprintf(stderr, "=== VGA I/O trace (last %d of %u) ===\n", count, vga_trace_total);
	for (int i = 0; i < count; i++) {
		VGATraceEntry *e = &vga_trace_ring[(start + i) & (VGA_TRACE_SIZE - 1)];
		if (e->type <= 1) {
			/* Port I/O */
			fprintf(stderr, "  %s %03x=%02x\n", type_names[e->type], e->addr, e->val);
		} else {
			/* Memory I/O - show plane context */
			fprintf(stderr, "  %s [%05x]=%02x sr2=%x gr4=%x gr5=%02x\n",
				type_names[e->type], e->addr, e->val,
				e->sr2, e->gr4, e->gr5);
		}
	}
}

uint32_t get_uticks();
static int after_eq(uint32_t a, uint32_t b)
{
    return (a - b) < (1u << 31);
}

#if BPP == 32
static void vga_draw_glyph8(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol)
{
    uint32_t font_data, xorcol;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint32_t *)d)[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        ((uint32_t *)d)[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[7] = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}

static void vga_draw_glyph9(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol,
                            int dup9)
{
    uint32_t font_data, xorcol, v;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint32_t *)d)[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        ((uint32_t *)d)[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        v = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[7] = v;
        if (dup9)
            ((uint32_t *)d)[8] = v;
        else
            ((uint32_t *)d)[8] = bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}
#elif BPP == 16
static void vga_draw_glyph8(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol)
{
    uint32_t font_data, xorcol;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint16_t *)d)[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        ((uint16_t *)d)[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[7] = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}

static void vga_draw_glyph9(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol,
                            int dup9)
{
    uint32_t font_data, xorcol, v;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint16_t *)d)[0] = ((-((font_data >> 7)) & xorcol) ^ bgcol);
        ((uint16_t *)d)[1] = ((-((font_data >> 6) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[2] = ((-((font_data >> 5) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[3] = ((-((font_data >> 4) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[4] = ((-((font_data >> 3) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[5] = ((-((font_data >> 2) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[6] = ((-((font_data >> 1) & 1) & xorcol) ^ bgcol);
        v = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[7] = v;
        if (dup9)
            ((uint16_t *)d)[8] = v;
        else
            ((uint16_t *)d)[8] = bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}

static inline uint32_t c69(uint16_t c)
{
    // 0000 0000 0000 rrrr rggg gggb bbbb
    // 0000 0rrr rr00 0ggg ggg0 000b bbbb
    return (c & 0x1f) | ((c & 0x7e0) << 4) | ((c & 0xf800) << 7);
}

static inline uint16_t c96(uint32_t c)
{
    // 0000 0rrr rr00 0ggg ggg0 000b bbbb
    // 0000 0000 0000 rrrr rggg gggb bbbb
    uint16_t t = (c & 0x1f) | ((c & 0x7e00) >> 4) | ((c & 0x7c0000) >> 7);
#ifdef SWAP_BYTEORDER_BPP16
    return (t << 8) | (t >> 8);
#else
    return t;
#endif
}

static void scale_3_2(uint8_t *dst, int dst_stride, uint8_t *src, int w)
{
   const static int shift[4][4] = {
       { 2, 0, 1, 0 },
       { 1, 2, 0, 0 },
       { 0, 0, 2, 1 },
       { 0, 1, 0, 2 }
   };
   int ww = w / 3 * 2;
   int idx = 0;
   for (int j = 0; j < 2; j++, idx ^= 2,
#ifdef SWAPXY
                dst += BPP / 8
#else
                dst += dst_stride
#endif
           ) {
       uint8_t *dst1 = dst;
       uint8_t *src1 = src + (j * w) * (BPP / 8);
       for (int k = 0; k < ww; k++, idx ^= 1) {
           int kk = k / 2 * 3 + (k & 1);
           uint16_t *p0 = (uint16_t *) (src1 + kk * (BPP / 8));
           uint16_t *p1 = p0 + 1;
           uint16_t *p2 = p0 + w;
           uint16_t *p3 = p1 + w;
           int sh0 = shift[idx][0];
           int sh1 = shift[idx][1];
           int sh2 = shift[idx][2];
           int sh3 = shift[idx][3];
           *(uint16_t *)dst1 = c96(((c69(*p0) << sh0) + (c69(*p1) << sh1) +
                                    (c69(*p2) << sh2) + (c69(*p3) << sh3)) >> 3);
#ifdef SWAPXY
           dst1 += dst_stride;
#else
           dst1 += BPP / 8;
#endif
       }
   }
}

static void scale_3_3(uint8_t *dst, int dst_stride, uint8_t *src, int w)
{
   for (int j = 0; j < 3; j++,
#ifdef SWAPXY
                dst += BPP / 8
#else
                dst += dst_stride
#endif
           ) {
       uint8_t *dst1 = dst;
       uint8_t *src1 = src + (j * w) * (BPP / 8);
       for (int k = 0; k < w; k++) {
           *(uint16_t *)dst1 = *(uint16_t *) (src1 + k * (BPP / 8));
#ifdef SWAPXY
           dst1 += dst_stride;
#else
           dst1 += BPP / 8;
#endif
       }
   }
}

#else
#error "bad bpp"
#endif

static const uint8_t cursor_glyph[32] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

#if BPP == 32
static inline int c6_to_8(int v)
{
    int b;
    v &= 0x3f;
    b = v & 1;
    return (v << 2) | (b << 1) | b;
}

static inline unsigned int rgb_to_pixel(unsigned int r, unsigned int g,
                                        unsigned int b)
{
    return (r << 16) | (g << 8) | b;
}

static int update_palette256(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    v = 0;
    for(i = 0; i < 256; i++) {
        if (s->dac_8bit) {
          col = rgb_to_pixel(s->palette_snapshot[v],
                             s->palette_snapshot[v + 1],
                             s->palette_snapshot[v + 2]);
        } else {
          col = rgb_to_pixel(c6_to_8(s->palette_snapshot[v]),
                             c6_to_8(s->palette_snapshot[v + 1]),
                             c6_to_8(s->palette_snapshot[v + 2]));
        }
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
        v += 3;
    }
    return full_update;
}

static int update_palette16(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    for(i = 0; i < 16; i++) {
        v = s->ar[i];
        if (s->ar[0x10] & 0x80)
            v = ((s->ar[0x14] & 0xf) << 4) | (v & 0xf);
        else
            v = ((s->ar[0x14] & 0xc) << 4) | (v & 0x3f);
        v = v * 3;
        col = (c6_to_8(s->palette_snapshot[v]) << 16) |
            (c6_to_8(s->palette_snapshot[v + 1]) << 8) |
            c6_to_8(s->palette_snapshot[v + 2]);
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
    }
    return full_update;
}
#elif BPP == 16
static int update_palette256(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    v = 0;
    for(i = 0; i < 256; i++) {
        if (s->dac_8bit) {
            col = ((s->palette_snapshot[v + 2] >> 3)) |
                ((s->palette_snapshot[v + 1] >> 2) << 5) |
                ((s->palette_snapshot[v] >> 3) << 11);
        } else {
            col = (s->palette_snapshot[v + 2] >> 1) |
                ((s->palette_snapshot[v + 1]) << 5) |
                ((s->palette_snapshot[v] >> 1) << 11);
        }
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
        v += 3;
    }
    /* Update IRAM palette cache for fast Mode 13h rendering */
    if (full_update || !vga256_palette_valid) {
        for (i = 0; i < 256; i++) {
            vga256_palette_cache[i] = (uint16_t)palette[i];
        }
        vga256_palette_valid = 1;
        vga_mark_all_dirty();  /* Palette changed, redraw everything */
    }
    return full_update;
}

static int update_palette16(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    for(i = 0; i < 16; i++) {
        v = s->ar[i];
        if (s->ar[0x10] & 0x80)
            v = ((s->ar[0x14] & 0xf) << 4) | (v & 0xf);
        else
            v = ((s->ar[0x14] & 0xc) << 4) | (v & 0x3f);
        v = v * 3;
        col = (s->palette_snapshot[v + 2] >> 1) |
              ((s->palette_snapshot[v + 1]) << 5) |
              ((s->palette_snapshot[v] >> 1) << 11);
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
    }
    return full_update;
}
#else
#error "bad bpp"
#endif

/* Quick hash of 16-color palette for change detection */
static uint32_t hash_palette16(const uint32_t *palette16, int cpe_mask)
{
    uint32_t h = 0x811c9dc5u;  /* FNV-1a */
    for (int i = 0; i < 16; i++) {
        h ^= palette16[i & cpe_mask];
        h *= 0x01000193u;
    }
    h ^= (uint32_t)cpe_mask;
    h *= 0x01000193u;
    return h ? h : 1;  /* Never return 0 (0 = "never built") */
}

/* Build BitScrambler palette pair LUTs from a 16-color palette.
 * bs_pal_pair[byte] = (rgb565[lo_nibble]) | (rgb565[hi_nibble] << 16)
 * bs_pal_double[idx] = rgb565 | (rgb565 << 16)  (pixel doubled for xdiv==2)
 * Returns 1 if LUTs were rebuilt, 0 if unchanged. */
static int build_bs_palette_luts(const uint32_t *palette16, int cpe_mask)
{
    uint32_t h = hash_palette16(palette16, cpe_mask);
    if (h == bs_pal_last_hash)
        return 0;  /* Palette unchanged since last build */

    for (int b = 0; b < 256; b++) {
        uint16_t lo = palette16[(b & 0xf) & cpe_mask];
        uint16_t hi = palette16[((b >> 4) & 0xf) & cpe_mask];
        bs_pal_pair[b] = (uint32_t)lo | ((uint32_t)hi << 16);
    }
    for (int i = 0; i < 16; i++) {
        uint16_t c = palette16[i & cpe_mask];
        bs_pal_double[i] = (uint32_t)c | ((uint32_t)c << 16);
        bs_pal_single[i] = c;
    }
    bs_pal_last_hash = h;
    return 1;
}

/* Expand BitScrambler packed nibbles to RGB565 framebuffer with pixel panning.
 * packed: BS output, vram_line_bytes: display bytes/line, pan: 0-7 pixel offset */
static inline void bs_expand_line(uint32_t *fb32, const uint8_t *packed,
                                  int vram_line_bytes, int xdiv, int pan) {
    if (xdiv == 2) {
        if (pan == 0) {
            const uint32_t *p32 = (const uint32_t *)packed;
            for (int i = 0, nw = vram_line_bytes / 4; i < nw; i++) {
                uint32_t p = p32[i];
                fb32[0] = bs_pal_double[p & 0xf];
                fb32[1] = bs_pal_double[(p >> 4) & 0xf];
                fb32[2] = bs_pal_double[(p >> 8) & 0xf];
                fb32[3] = bs_pal_double[(p >> 12) & 0xf];
                fb32[4] = bs_pal_double[(p >> 16) & 0xf];
                fb32[5] = bs_pal_double[(p >> 20) & 0xf];
                fb32[6] = bs_pal_double[(p >> 24) & 0xf];
                fb32[7] = bs_pal_double[p >> 28];
                fb32 += 8;
            }
        } else {
            int nib = pan, dp = vram_line_bytes * 2;
            for (int px = 0; px < dp; px++, nib++)
                fb32[px] = bs_pal_double[(nib & 1) ?
                    (packed[nib >> 1] >> 4) & 0xf : packed[nib >> 1] & 0xf];
        }
    } else {
        if (pan == 0) {
            for (int i = 0; i < vram_line_bytes; i++)
                fb32[i] = bs_pal_pair[packed[i]];
        } else if (!(pan & 1)) {
            packed += pan >> 1;
            for (int i = 0; i < vram_line_bytes; i++)
                fb32[i] = bs_pal_pair[packed[i]];
        } else {
            uint16_t *fb16 = (uint16_t *)fb32;
            int nib = pan, dp = vram_line_bytes * 2;
            for (int px = 0; px < dp; px++, nib++)
                fb16[px] = bs_pal_single[(nib & 1) ?
                    (packed[nib >> 1] >> 4) & 0xf : packed[nib >> 1] & 0xf];
        }
    }
}

/* VGA CRT controller register indices */
#define VGA_CRTC_H_TOTAL        0
#define VGA_CRTC_H_DISP         1
#define VGA_CRTC_H_BLANK_START  2
#define VGA_CRTC_H_BLANK_END    3
#define VGA_CRTC_H_SYNC_START   4
#define VGA_CRTC_H_SYNC_END     5
#define VGA_CRTC_V_TOTAL        6
#define VGA_CRTC_OVERFLOW       7
#define VGA_CRTC_PRESET_ROW     8
#define VGA_CRTC_MAX_SCAN       9
#define VGA_CRTC_CURSOR_START   0x0A
#define VGA_CRTC_CURSOR_END     0x0B
#define VGA_CRTC_START_HI       0x0C
#define VGA_CRTC_START_LO       0x0D
#define VGA_CRTC_CURSOR_HI      0x0E
#define VGA_CRTC_CURSOR_LO      0x0F
#define VGA_CRTC_V_SYNC_START   0x10
#define VGA_CRTC_V_SYNC_END     0x11
#define VGA_CRTC_V_DISP_END     0x12
#define VGA_CRTC_OFFSET         0x13
#define VGA_CRTC_UNDERLINE      0x14
#define VGA_CRTC_V_BLANK_START  0x15
#define VGA_CRTC_V_BLANK_END    0x16
#define VGA_CRTC_MODE           0x17
#define VGA_CRTC_LINE_COMPARE   0x18
#define VGA_CRTC_REGS           VGA_CRT_C

/* VGA sequencer register indices */
#define VGA_SEQ_RESET           0x00
#define VGA_SEQ_CLOCK_MODE      0x01
#define VGA_SEQ_PLANE_WRITE     0x02
#define VGA_SEQ_CHARACTER_MAP   0x03
#define VGA_SEQ_MEMORY_MODE     0x04

/* VGA sequencer register bit masks */
#define VGA_SR01_CHAR_CLK_8DOTS 0x01 /* bit 0: character clocks 8 dots wide are generated */
#define VGA_SR01_SCREEN_OFF     0x20 /* bit 5: Screen is off */
#define VGA_SR02_ALL_PLANES     0x0F /* bits 3-0: enable access to all planes */
#define VGA_SR04_EXT_MEM        0x02 /* bit 1: allows complete mem access to 256K */
#define VGA_SR04_SEQ_MODE       0x04 /* bit 2: directs system to use a sequential addressing mode */
#define VGA_SR04_CHN_4M         0x08 /* bit 3: selects modulo 4 addressing for CPU access to display memory */

/* VGA graphics controller register indices */
#define VGA_GFX_SR_VALUE        0x00
#define VGA_GFX_SR_ENABLE       0x01
#define VGA_GFX_COMPARE_VALUE   0x02
#define VGA_GFX_DATA_ROTATE     0x03
#define VGA_GFX_PLANE_READ      0x04
#define VGA_GFX_MODE            0x05
#define VGA_GFX_MISC            0x06
#define VGA_GFX_COMPARE_MASK    0x07
#define VGA_GFX_BIT_MASK        0x08

/* VGA graphics controller bit masks */
#define VGA_GR06_GRAPHICS_MODE  0x01

static bool vbe_enabled(VGAState *s)
{
    return s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED;
}

/*
 * Sanity check vbe register writes.
 *
 * As we don't have a way to signal errors to the guest in the bochs
 * dispi interface we'll go adjust the registers to the closest valid
 * value.
 */
static void vbe_fixup_regs(VGAState *s)
{
    uint16_t *r = s->vbe_regs;
    uint32_t bits, linelength, /*maxy,*/ offset;

    if (!vbe_enabled(s)) {
        /* vbe is turned off -- nothing to do */
        return;
    }

    /* check depth */
    switch (r[VBE_DISPI_INDEX_BPP]) {
    case 4:
    case 8:
    case 16:
    case 24:
    case 32:
        bits = r[VBE_DISPI_INDEX_BPP];
        break;
    case 15:
        bits = 16;
        break;
    default:
        bits = r[VBE_DISPI_INDEX_BPP] = 8;
        break;
    }

    /* check width */
    r[VBE_DISPI_INDEX_XRES] &= ~7u;
    if (r[VBE_DISPI_INDEX_XRES] == 0) {
        r[VBE_DISPI_INDEX_XRES] = 8;
    }
//    if (r[VBE_DISPI_INDEX_XRES] > VBE_DISPI_MAX_XRES) {
//        r[VBE_DISPI_INDEX_XRES] = VBE_DISPI_MAX_XRES;
//    }
    r[VBE_DISPI_INDEX_VIRT_WIDTH] &= ~7u;
//    if (r[VBE_DISPI_INDEX_VIRT_WIDTH] > VBE_DISPI_MAX_XRES) {
//        r[VBE_DISPI_INDEX_VIRT_WIDTH] = VBE_DISPI_MAX_XRES;
//    }
    if (r[VBE_DISPI_INDEX_VIRT_WIDTH] < r[VBE_DISPI_INDEX_XRES]) {
        r[VBE_DISPI_INDEX_VIRT_WIDTH] = r[VBE_DISPI_INDEX_XRES];
    }

    /* check height */
    linelength = r[VBE_DISPI_INDEX_VIRT_WIDTH] * bits / 8;
//    maxy = s->vbe_size / linelength;
    if (r[VBE_DISPI_INDEX_YRES] == 0) {
        r[VBE_DISPI_INDEX_YRES] = 1;
    }
//    if (r[VBE_DISPI_INDEX_YRES] > VBE_DISPI_MAX_YRES) {
//        r[VBE_DISPI_INDEX_YRES] = VBE_DISPI_MAX_YRES;
//    }
//    if (r[VBE_DISPI_INDEX_YRES] > maxy) {
//        r[VBE_DISPI_INDEX_YRES] = maxy;
//    }

    /* check offset */
//    if (r[VBE_DISPI_INDEX_X_OFFSET] > VBE_DISPI_MAX_XRES) {
//        r[VBE_DISPI_INDEX_X_OFFSET] = VBE_DISPI_MAX_XRES;
//    }
//    if (r[VBE_DISPI_INDEX_Y_OFFSET] > VBE_DISPI_MAX_YRES) {
//        r[VBE_DISPI_INDEX_Y_OFFSET] = VBE_DISPI_MAX_YRES;
//    }
    offset = r[VBE_DISPI_INDEX_X_OFFSET] * bits / 8;
    offset += r[VBE_DISPI_INDEX_Y_OFFSET] * linelength;
//    if (offset + r[VBE_DISPI_INDEX_YRES] * linelength > s->vbe_size) {
//        r[VBE_DISPI_INDEX_Y_OFFSET] = 0;
//        offset = r[VBE_DISPI_INDEX_X_OFFSET] * bits / 8;
//        if (offset + r[VBE_DISPI_INDEX_YRES] * linelength > s->vbe_size) {
//            r[VBE_DISPI_INDEX_X_OFFSET] = 0;
//            offset = 0;
//        }
//    }

    /* update vga state */
//    r[VBE_DISPI_INDEX_VIRT_HEIGHT] = maxy;
    s->vbe_line_offset = linelength;
    s->vbe_start_addr  = offset / 4;
}

static void vbe_update_vgaregs(VGAState *s)
{
    int h, shift_control;

    if (!vbe_enabled(s)) {
        /* vbe is turned off -- nothing to do */
        return;
    }

    /* graphic mode + memory map 1 */
    s->gr[VGA_GFX_MISC] = (s->gr[VGA_GFX_MISC] & ~0x0c) | 0x04 |
        VGA_GR06_GRAPHICS_MODE;
    s->cr[VGA_CRTC_MODE] |= 3; /* no CGA modes */
    s->cr[VGA_CRTC_OFFSET] = s->vbe_line_offset >> 3;
    /* width */
    s->cr[VGA_CRTC_H_DISP] =
        (s->vbe_regs[VBE_DISPI_INDEX_XRES] >> 3) - 1;
    /* height (only meaningful if < 1024) */
    h = s->vbe_regs[VBE_DISPI_INDEX_YRES] - 1;
    s->cr[VGA_CRTC_V_DISP_END] = h;
    s->cr[VGA_CRTC_OVERFLOW] = (s->cr[VGA_CRTC_OVERFLOW] & ~0x42) |
        ((h >> 7) & 0x02) | ((h >> 3) & 0x40);
    /* line compare to 1023 */
    s->cr[VGA_CRTC_LINE_COMPARE] = 0xff;
    s->cr[VGA_CRTC_OVERFLOW] |= 0x10;
    s->cr[VGA_CRTC_MAX_SCAN] |= 0x40;

    if (s->vbe_regs[VBE_DISPI_INDEX_BPP] == 4) {
        shift_control = 0;
        s->sr/*_vbe*/[VGA_SEQ_CLOCK_MODE] &= ~8; /* no double line */
    } else {
        shift_control = 2;
        /* set chain 4 mode */
        s->sr/*_vbe*/[VGA_SEQ_MEMORY_MODE] |= VGA_SR04_CHN_4M;
        /* activate all planes */
        s->sr/*_vbe*/[VGA_SEQ_PLANE_WRITE] |= VGA_SR02_ALL_PLANES;
    }
    s->gr[VGA_GFX_MODE] = (s->gr[VGA_GFX_MODE] & ~0x60) |
        (shift_control << 5);
    s->cr[VGA_CRTC_MAX_SCAN] &= ~0x9f; /* no double scan */
}

#ifdef TEXT_RENDER_OPT
/* Fast CRC32 for text buffer hash - uses table lookup.
 * No DRAM_ATTR: text mode is not performance-critical enough to justify
 * 1KB of internal SRAM; freed for BitScrambler DMA buffers. */
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d09, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cd9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd706b3,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static uint32_t IRAM_ATTR text_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xffffffff;
    while (len >= 4) {
        crc = crc32_table[(crc ^ data[0]) & 0xff] ^ (crc >> 8);
        crc = crc32_table[(crc ^ data[1]) & 0xff] ^ (crc >> 8);
        crc = crc32_table[(crc ^ data[2]) & 0xff] ^ (crc >> 8);
        crc = crc32_table[(crc ^ data[3]) & 0xff] ^ (crc >> 8);
        data += 4;
        len -= 4;
    }
    while (len--) {
        crc = crc32_table[(crc ^ *data++) & 0xff] ^ (crc >> 8);
    }
    return crc ^ 0xffffffff;
}

/* Detect scroll by comparing old and new text buffers.
 * Returns number of lines scrolled (positive = up, negative = down, 0 = no scroll) */
static int detect_text_scroll(const uint16_t *old_attrs, const uint16_t *new_vram,
                               int width, int height, uint32_t line_offset)
{
    /* Check for 1-line scroll up (most common case) */
    int line_words = width;
    int total_words = width * height;

    /* Compare line 0 of old with line 1 of new */
    const uint16_t *old_line0 = old_attrs;
    const uint16_t *new_line1 = new_vram + line_offset / 2;

    int match_count = 0;
    for (int i = 0; i < line_words && i < (height - 1) * width; i++) {
        if (old_line0[i] == new_line1[i]) match_count++;
    }

    /* If >80% match, likely a scroll up */
    if (match_count > line_words * 4 / 5) {
        /* Verify more lines match the scroll pattern */
        int lines_matched = 0;
        for (int y = 0; y < height - 1; y++) {
            const uint16_t *old_line = old_attrs + y * width;
            const uint16_t *new_line = new_vram + (y + 1) * line_offset / 2;
            int line_match = 0;
            for (int x = 0; x < width; x++) {
                if (old_line[x] == *(uint16_t *)((uint8_t *)new_vram + (y + 1) * line_offset + x * 2))
                    line_match++;
            }
            if (line_match > width * 4 / 5) lines_matched++;
        }
        if (lines_matched > (height - 1) * 3 / 4) return 1;  /* Scroll up by 1 */
    }

    return 0;  /* No scroll detected */
}

/* Look up glyph in cache, returns pointer to tile or NULL if not cached */
static uint8_t *glyph_cache_lookup(VGAState *s, uint32_t key)
{
    if (!s->glyph_cache_tiles) return NULL;

    for (int i = 0; i < GLYPH_CACHE_SIZE; i++) {
        if (s->glyph_cache_keys[i] == key) {
            s->glyph_cache_lru[i] = s->glyph_cache_counter;
            return s->glyph_cache_tiles + i * GLYPH_TILE_SIZE;
        }
    }
    return NULL;
}

/* Store glyph in cache, evicting LRU entry if needed */
static uint8_t *glyph_cache_store(VGAState *s, uint32_t key)
{
    if (!s->glyph_cache_tiles) return NULL;

    /* Find LRU entry */
    int lru_idx = 0;
    uint8_t oldest = 255;
    for (int i = 0; i < GLYPH_CACHE_SIZE; i++) {
        if (s->glyph_cache_keys[i] == 0xffffffff) {
            lru_idx = i;
            break;
        }
        uint8_t age = s->glyph_cache_counter - s->glyph_cache_lru[i];
        if (age > oldest || oldest == 255) {
            oldest = age;
            lru_idx = i;
        }
    }

    s->glyph_cache_keys[lru_idx] = key;
    s->glyph_cache_lru[lru_idx] = ++s->glyph_cache_counter;
    return s->glyph_cache_tiles + lru_idx * GLYPH_TILE_SIZE;
}

/* Render a glyph to a tile buffer (8x16 RGB565) */
static void render_glyph_tile(uint8_t *tile, const uint8_t *font_ptr,
                               int cheight, uint32_t fgcol, uint32_t bgcol)
{
    uint16_t *dst = (uint16_t *)tile;
    uint32_t xorcol = bgcol ^ fgcol;

    for (int y = 0; y < cheight; y++) {
        uint8_t font_data = font_ptr[0];
        dst[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        dst[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        dst[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        dst[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        dst[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        dst[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        dst[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        dst[7] = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        font_ptr += 4;
        dst += 8;
    }
}

/* Update font cache from VGA RAM - called when font pointer changes */
static void update_font_cache(VGAState *s, const uint8_t *font_base)
{
    if (!s->font_cache) return;

    uint32_t font_offset = font_base - s->vga_ram;
    if (font_offset == s->font_cache_base) return;  /* Font unchanged */

    /* Copy font data to cache (256 chars * 16 rows * 4 bytes stride) */
    for (int ch = 0; ch < 256; ch++) {
        const uint8_t *src = font_base + ch * 32 * 4;  /* 32 rows * 4 bytes per char in VGA RAM */
        uint8_t *dst = s->font_cache + ch * 16 * 4;    /* 16 rows * 4 bytes in cache */
        for (int row = 0; row < 16; row++) {
            dst[row * 4] = src[row * 4];
        }
    }
    s->font_cache_base = font_offset;

    /* Invalidate glyph cache since font changed */
    memset(s->glyph_cache_keys, 0xff, sizeof(s->glyph_cache_keys));
}
#endif /* TEXT_RENDER_OPT */

/* the text refresh is just for debugging and initial boot message, so
   it is very incomplete */
static void vga_text_refresh(VGAState *s,
                             SimpleFBDrawFunc *redraw_func, void *opaque,
                             int full_update)
{
    FBDevice *fb_dev = s->fb_dev;
    int width, height, cwidth, cheight, cy, cx, x1, y1, width1, height1;
    int cx_min, cx_max, dup9;
    uint32_t ch_attr, line_offset, start_addr, ch_addr, ch_addr1, ch, cattr;
    uint8_t *vga_ram, *dst;
    const uint8_t *font_ptr;
    uint32_t fgcol, bgcol, cursor_offset, cursor_start, cursor_end;
    uint32_t now = get_uticks();
    if (after_eq(now, s->cursor_blink_time)) {
        s->cursor_blink_time = now + 266666;
        s->cursor_visible_phase = !s->cursor_visible_phase;
    }

    /* Snapshot palette at render start to avoid race conditions */
    memcpy(s->palette_snapshot, s->palette, 768);

    full_update = full_update || update_palette16(s, s->last_palette);

    vga_ram = s->vga_ram;

    const uint8_t *font_base[2];
    uint32_t v = s->sr[0x3];
    font_base[0] = vga_ram + (((v >> 4) & 1) | ((v << 1) & 6)) * 8192 * 4 + 2;
    font_base[1] = vga_ram + (((v >> 5) & 1) | ((v >> 1) & 6)) * 8192 * 4 + 2;

#ifdef TEXT_RENDER_OPT
    /* Update font cache if needed (copy font to faster memory) */
    update_font_cache(s, font_base[0]);
#endif

    line_offset = s->cr[0x13];
    line_offset <<= 3;

    start_addr = s->cr[0x0d] | (s->cr[0x0c] << 8);
    
    cheight = (s->cr[9] & 0x1f) + 1;
    cwidth = 8;
    if (!s->force_8dm && !(s->sr[1] & 0x01))
        cwidth++;

    width = (s->cr[0x01] + 1);
    height = s->cr[0x12] |
        ((s->cr[0x07] & 0x02) << 7) |
        ((s->cr[0x07] & 0x40) << 3);
    height = (height + 1) / cheight;

#ifdef TEXT_RENDER_OPT
    /* Text buffer hash check - skip render if unchanged */
    {
        uint32_t text_size = width * height * 2;  /* 2 bytes per char (char + attr) */
        uint32_t text_start = start_addr * 4;
        if (text_start + text_size <= (uint32_t)s->vga_ram_size) {
            uint32_t hash = text_crc32(vga_ram + text_start, text_size);
            /* Also include cursor position in hash to detect cursor movement */
            hash ^= (s->cr[0x0e] << 24) | (s->cr[0x0f] << 16) | (s->cursor_visible_phase << 8);

            if (hash == s->text_hash && !full_update &&
                s->last_text_start == text_start && s->last_text_size == text_size) {
                /* Text buffer unchanged - skip rendering entirely */
                return;
            }

            /* Check for scroll and optimize if detected */
            if (!full_update && s->last_text_start == text_start &&
                s->last_text_size == text_size && s->last_ch_attr[0] != 0xffff) {
                int scroll = detect_text_scroll(s->last_ch_attr,
                    (uint16_t *)(vga_ram + text_start), width, height, line_offset);
                if (scroll == 1) {
                    /* Scroll up by 1 line - memmove framebuffer */
                    int w1 = width * cwidth;
                    int line_bytes = w1 * (BPP / 8);
                    int total_lines = height * cheight;
                    uint8_t *fb = fb_dev->fb_data;
                    memmove(fb, fb + line_bytes * cheight,
                            line_bytes * (total_lines - cheight));
                    /* Only need to render the last line */
                    /* Mark all lines except last as unchanged */
                    for (int y = 0; y < height - 1; y++) {
                        for (int x = 0; x < width; x++) {
                            s->last_ch_attr[y * width + x] =
                                *(uint16_t *)(vga_ram + text_start + (y + 1) * line_offset + x * 2);
                        }
                    }
                    s->scroll_lines = 1;
                }
            }

            s->text_hash = hash;
            s->last_text_start = text_start;
            s->last_text_size = text_size;
        }
    }
#endif

    width1 = width * cwidth;
    height1 = height * cheight;
#if defined(SCALE_3_2) || defined(SWAPXY)
#ifdef SCALE_3_2
    if (fb_dev->width * 3 / 2 < width1 || fb_dev->height * 3 / 2 < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT || cheight > 16)
        return; /* not enough space */
    x1 = (fb_dev->width * 3 / 2 - width1) / 3;
    y1 = (fb_dev->height * 3 / 2 - height1) / 3;
    full_update = 1;
#else
    if (fb_dev->width < width1 || fb_dev->height < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT || cheight > 16)
        return; /* not enough space */
    x1 = (fb_dev->width - width1) / 2;
    y1 = (fb_dev->height - height1) / 2;
    full_update = 1;
#endif
#else
    if (fb_dev->width < width1 || fb_dev->height < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT)
        return; /* not enough space */
    /* For Tanmatsu, render at 0,0 and let PPA handle scaling/centering.
     * Text mode outputs at native resolution (no pixel doubling). */
    x1 = 0;
    y1 = 0;
    globals.vga_mode_width = width1;
    globals.vga_mode_height = height1;
    globals.vga_pixel_double = 1;
    int stride = fb_dev->stride;
#endif
    if (s->last_line_offset != line_offset ||
        s->last_start_addr != start_addr ||
        s->last_width != width ||
        s->last_height != height) {
        s->last_line_offset = line_offset;
        s->last_start_addr = start_addr;
        s->last_width = width;
        s->last_height = height;
        full_update = 1;
    }
       
    /* update cursor position */
    cursor_offset = ((s->cr[0x0e] << 8) | s->cr[0x0f]) - start_addr;
    cursor_start = s->cr[0xa];
    cursor_end = s->cr[0xb];
    if (cursor_offset != s->last_cursor_offset ||
        cursor_start != s->last_cursor_start ||
        cursor_end != s->last_cursor_end) {
#ifndef FULL_UPDATE
        /* force refresh of characters with the cursor */
        if (s->last_cursor_offset < MAX_TEXT_WIDTH * MAX_TEXT_HEIGHT)
            s->last_ch_attr[s->last_cursor_offset] = -1;
        if (cursor_offset < MAX_TEXT_WIDTH * MAX_TEXT_HEIGHT)
            s->last_ch_attr[cursor_offset] = -1;
#endif
        s->last_cursor_offset = cursor_offset;
        s->last_cursor_start = cursor_start;
        s->last_cursor_end = cursor_end;
    }

    ch_addr1 = (start_addr * 4);
    cursor_offset = (start_addr + cursor_offset) * 4;
    
#if 0
    printf("text refresh %dx%d font=%dx%d start_addr=0x%x line_offset=0x%x\n",
           width, height, cwidth, cheight, start_addr, line_offset);
#endif
#if defined(SCALE_3_2) || defined(SWAPXY)
    int cb = 6;
    int nb = (width + cb - 1) / cb;
    int cxbegin = 0;
    int cxend = cb > width ? width : cb;
    int stride = (cxend - cxbegin) * cwidth * (BPP / 8);
    for (int b = 0; b < nb; b++)
    {
    int yt = 0;
    int yy = 0;
    ch_addr1 = (start_addr * 4) + cxbegin * 4;
#endif
    for(cy = 0; cy < height; cy++) {
        ch_addr = ch_addr1;
#if defined(SCALE_3_2) || defined(SWAPXY)
        dst = s->tmpbuf + yt * stride;
#else
        dst = fb_dev->fb_data + (y1 + cy * cheight) * stride + x1 * (BPP / 8);
#endif
        cx_min = width;
        cx_max = -1;
#if defined(SCALE_3_2) || defined(SWAPXY)
        for(cx = 0; cx < cxend - cxbegin; cx++) {
#else
        for(cx = 0; cx < width; cx++) {
#endif
            ch_attr = *(uint16_t *)(vga_ram + (ch_addr & 0x1fffe));
#ifdef FULL_UPDATE
            if (1) {
#else
            if (full_update || ch_attr != s->last_ch_attr[cy * width + cx] || cursor_offset == ch_addr) {
                s->last_ch_attr[cy * width + cx] = ch_attr;
#endif
                cx_min = cx_min > cx ? cx : cx_min;
                cx_max = cx_max < cx ? cx : cx_max;
                ch = ch_attr & 0xff;
                cattr = ch_attr >> 8;

                bgcol = s->last_palette[cattr >> 4];
                fgcol = s->last_palette[cattr & 0x0f];
#ifdef TEXT_RENDER_OPT
                /* Use font cache if available and using primary font */
                int use_alt_font = (cattr >> 3) & 1;
                if (s->font_cache && !use_alt_font) {
                    font_ptr = s->font_cache + ch * 16 * 4;
                } else {
                    font_ptr = font_base[use_alt_font] + 32 * 4 * ch;
                }
#else
                font_ptr = font_base[(cattr >> 3) & 1] + 32 * 4 * ch;
#endif
                if (cwidth == 8) {
#ifdef TEXT_RENDER_OPT
                    /* Try glyph cache first */
                    uint32_t cache_key = ch | ((cattr & 0x0f) << 8) | ((cattr >> 4) << 12) | (cheight << 16);
                    uint8_t *tile = glyph_cache_lookup(s, cache_key);
                    if (tile) {
                        /* Cache hit - copy pre-rendered tile */
                        uint8_t *d = dst;
                        uint8_t *t = tile;
                        for (int row = 0; row < cheight; row++) {
                            memcpy(d, t, 16);  /* 8 pixels * 2 bytes */
                            d += stride;
                            t += 16;
                        }
                    } else {
                        /* Cache miss - render and store */
                        tile = glyph_cache_store(s, cache_key);
                        if (tile) {
                            render_glyph_tile(tile, font_ptr, cheight, fgcol, bgcol);
                            /* Copy to framebuffer */
                            uint8_t *d = dst;
                            uint8_t *t = tile;
                            for (int row = 0; row < cheight; row++) {
                                memcpy(d, t, 16);
                                d += stride;
                                t += 16;
                            }
                        } else {
                            /* No cache available - direct render */
                            vga_draw_glyph8(dst, stride, font_ptr, cheight, fgcol, bgcol);
                        }
                    }
#else
                    vga_draw_glyph8(dst, stride, font_ptr, cheight,
                                    fgcol, bgcol);
#endif
                } else {
                    dup9 = 0;
                    if (ch >= 0xb0 && ch <= 0xdf && (s->ar[0x10] & 0x04))
                        dup9 = 1;
                    vga_draw_glyph9(dst, stride, font_ptr, cheight,
                                    fgcol, bgcol, dup9);
                }
                /* cursor display */
                if (cursor_offset == ch_addr && !(cursor_start & 0x20) && s->cursor_visible_phase) {
                    int line_start, line_last, h;
                    uint8_t *dst1;
                    line_start = cursor_start & 0x1f;
                    line_last = cursor_end & 0x1f;
                    if (line_last > cheight - 1)
                        line_last = cheight - 1;

                    if (line_last >= line_start && line_start < cheight) {
                        h = line_last - line_start + 1;
                        dst1 = dst + stride * line_start;
                        if (cwidth == 8) {
                            vga_draw_glyph8(dst1, stride,
                                            cursor_glyph,
                                            h, fgcol, bgcol);
                        } else {
                            vga_draw_glyph9(dst1, stride,
                                            cursor_glyph,
                                            h, fgcol, bgcol, 1);
                        }
                    }
                }
            }
            ch_addr += 4;
            dst += (BPP / 8) * cwidth;
        }
#if defined(SCALE_3_2) || defined(SWAPXY)
        int k;
        for (k = 0; k < yt + cheight - 1; k += 3) {
#ifdef SCALE_3_2
#ifdef SWAPXY
                int ii0 = (BPP / 8) * ((y1 + yy) + (x1 + cxbegin * cwidth * 2 / 3) * fb_dev->height);
#else
                int ii0 = (BPP / 8) * ((y1 + yy) * fb_dev->width + x1 + cxbegin * cwidth * 2 / 3);
#endif
                scale_3_2(fb_dev->fb_data + ii0, fb_dev->stride,
                          s->tmpbuf + k * stride, stride / (BPP / 8));
                yy += 2;
#else
#ifdef SWAPXY
                int ii0 = (BPP / 8) * ((y1 + yy) + (x1 + cxbegin * cwidth) * fb_dev->height);
#else
                int ii0 = (BPP / 8) * ((y1 + yy) * fb_dev->width + x1 + cxbegin * cwidth);
#endif
                scale_3_3(fb_dev->fb_data + ii0, fb_dev->stride,
                          s->tmpbuf + k * stride, stride / (BPP / 8));
                yy += 3;
#endif
        }
        yt = k - (yt + cheight - 1);
        if (yt != 0) {
                yt = 3 - yt;
                memcpy(s->tmpbuf, s->tmpbuf + (k - 3) * stride, yt * stride);
        }
#endif
//        if (cx_max >= cx_min) {
//            redraw_func(opaque,
//                        x1 + cx_min * cwidth, y1 + cy * cheight,
//                        (cx_max - cx_min + 1) * cwidth, cheight);
//        }
        ch_addr1 += line_offset;
    }
#if defined(SCALE_3_2) || defined(SWAPXY)
    cxbegin += cb;
    cxend += cb;
    if (cxend > width) cxend = width;
    stride = (cxend - cxbegin) * cwidth * (BPP / 8);
    }
#endif
    redraw_func(opaque, 0, 0, fb_dev->width, fb_dev->height);
}

static void vga_graphic_refresh(VGAState *s,
                                SimpleFBDrawFunc *redraw_func, void *opaque,
                                int full_update)
{
    FBDevice *fb_dev = s->fb_dev;

    /* Snapshot all palette data at render start to avoid race conditions
     * with CPU core modifying palettes during render (palette animation).
     * Use generation counters to skip copy when palettes haven't changed. */
    static uint32_t last_palette_gen = 0;
    static uint32_t last_before_gen = 0;
    static uint32_t last_after_gen = 0;
    if (s->palette_generation != last_palette_gen) {
        memcpy(s->palette_snapshot, s->palette, 768);
        last_palette_gen = s->palette_generation;
    }
    if (s->palette_before_gen != last_before_gen) {
        memcpy(s->palette_before_snapshot, s->palette_before_switch, 768);
        last_before_gen = s->palette_before_gen;
    }
    if (s->palette_after_gen != last_after_gen) {
        memcpy(s->palette_after_snapshot, s->palette_after_switch, 768);
        last_after_gen = s->palette_after_gen;
    }
    s->palette_switch_snapshot = s->palette_switch_scanline;

    int w = (s->cr[0x01] + 1) * 8;
    int h = s->cr[0x12] |
        ((s->cr[0x07] & 0x02) << 7) |
        ((s->cr[0x07] & 0x40) << 3);
    h++;

    int shift_control = (s->gr[0x05] >> 5) & 3;

    /* Detect mode changes and clear framebuffer to avoid leftover pixels */
    static int last_w = 0, last_h = 0, last_sc = -1;
    /* Force clear when transitioning from text/blank to graphics mode */
    if (s->force_graphic_clear) {
        last_w = last_h = 0;
        last_sc = -1;
        s->force_graphic_clear = 0;
    }
    if (w != last_w || h != last_h || shift_control != last_sc) {
#ifdef DEBUG_VGA
        fprintf(stderr, "VGA: w=%d h=%d shift_ctrl=%d cr01=%02x cr13=%02x sr01=%02x gr05=%02x ar10=%02x ar12=%02x\n",
                w, h, shift_control, s->cr[0x01], s->cr[0x13], s->sr[0x01], s->gr[0x05],
                s->ar[0x10], s->ar[0x12]);
        /* Print AR palette (attribute controller palette mapping) */
        fprintf(stderr, "VGA AR palette: ");
        for (int i = 0; i < 16; i++)
            fprintf(stderr, "%02x ", s->ar[i]);
        fprintf(stderr, "\n");
#endif
        /* Clear entire framebuffer on mode change to remove old frame remnants */
        memset(fb_dev->fb_data, 0, fb_dev->stride * fb_dev->height);
        /* Reset mid-frame palette switching - new mode may not use it */
        s->palette_switch_scanline = -1;
        last_w = w; last_h = h; last_sc = shift_control;
        /* Reset fast path debug logging on mode change */
        ega_fast_logged = 0;
        vga256_fast_logged = 0;
        /* Invalidate dirty tracking on mode change */
        vga_mark_all_dirty();
        vga256_invalidate_palette();
        memset(vga_prev_line_hash, 0, sizeof(vga_prev_line_hash));
        fprintf(stderr, "VGA mode: %dx%d sc=%d xdiv=%d bpp=%d cr17=0x%02x\n",
                w, h, shift_control, (s->sr[0x01] & 8) ? 2 : 1,
                (shift_control >= 2) ? 8 : 4, s->cr[0x17]);
    }

#ifdef DEBUG_VGA
    /* Debug timing for performance analysis */
    static uint32_t frame_count = 0;
    static uint32_t slow_frame_count = 0;
    static uint32_t last_report = 0;
    static uint32_t last_palette_dump = 0;
    uint32_t t_start = get_uticks();

    /* Periodic AR palette dump (every 5 seconds) to catch mid-session changes */
    if (t_start - last_palette_dump > 5000000) {
        fprintf(stderr, "VGA AR[0-15]: ");
        for (int i = 0; i < 16; i++)
            fprintf(stderr, "%02x ", s->ar[i]);
        fprintf(stderr, "AR14=%02x\n", s->ar[0x14]);
        last_palette_dump = t_start;
    }
#endif
    int double_scan = (s->cr[0x09] >> 7);
    int multi_scan, multi_run;
    if (shift_control != 1) {
        multi_scan = (((s->cr[0x09] & 0x1f) + 1) << double_scan) - 1;
    } else {
        /* in CGA modes, multi_scan is ignored */
        /* XXX: is it correct ? */
        multi_scan = double_scan;
    }
    /* For Tanmatsu 256-color modes, skip CPU vertical doubling - PPA will scale.
     * This outputs native 200 lines instead of doubled 400 for mode 13h. */
    int tanmatsu_vdouble = 0;
    if (shift_control >= 2 && !vbe_enabled(s) && multi_scan > 0) {
        tanmatsu_vdouble = multi_scan + 1;  /* Remember original for height calc */
        multi_scan = 0;
    }
    multi_run = multi_scan;

    uint32_t start_addr = s->cr[0x0d] | (s->cr[0x0c] << 8);
    uint32_t line_offset = s->cr[0x13];
    line_offset <<= 3;
    uint32_t line_compare = s->cr[0x18] |
        ((s->cr[0x07] & 0x10) << 4) |
        ((s->cr[0x09] & 0x40) << 3);
    if (vbe_enabled(s)) {
        line_offset = s->vbe_line_offset;
        start_addr = s->vbe_start_addr;
        line_compare = 0x3ff;
    }
    uint32_t addr1 = 4 * start_addr;
    /* VGA address counter wrap: 16-bit counter wraps at 64KB (byte mode),
     * 128KB (word mode), or 256KB (dword mode).  In our interleaved format
     * (4 bytes per VGA byte), multiply by 4.  VBE modes don't wrap. */
    uint32_t vram_addr_mask;
    if (vbe_enabled(s))
        vram_addr_mask = s->vga_ram_size - 1;
    else if (s->cr[0x14] & 0x40)
        vram_addr_mask = 0xFFFFF;    /* DW mode: 256K×4 = 1MB */
    else if (s->cr[0x17] & 0x40)
        vram_addr_mask = 0x3FFFF;    /* Byte mode: 64K×4 = 256KB */
    else
        vram_addr_mask = 0x7FFFF;    /* Word mode: 128K×4 = 512KB */
    if (vram_addr_mask >= (uint32_t)s->vga_ram_size)
        vram_addr_mask = s->vga_ram_size - 1;
    uint32_t vram_wrap_size = vram_addr_mask + 1; /* wrap boundary */
    addr1 &= vram_addr_mask;
    uint8_t *vram = s->vga_ram;
    uint32_t palette[256];
    uint32_t palette_before[16];  /* For mid-frame palette switching (top) */
    uint32_t palette_after[16];   /* For mid-frame palette switching (bottom) */
    int palette_switch_line = -1; /* Only set if we build alternate palettes */
    int xdiv = 1;
    int bpp = 4;
    if (shift_control == 0 || shift_control == 1) {
        update_palette16(s, palette);
        /* Build alternate palettes if mid-frame switch detected (using snapshots) */
        if (s->palette_switch_snapshot >= 0) {
            palette_switch_line = s->palette_switch_snapshot;
            /* Build "before" palette from pre-switch DAC state (for top of screen) */
            for (int i = 0; i < 16; i++) {
                int v = s->ar[i];
                if (s->ar[0x10] & 0x80)
                    v = ((s->ar[0x14] & 0xf) << 4) | (v & 0xf);
                else
                    v = ((s->ar[0x14] & 0xc) << 4) | (v & 0x3f);
                v = v * 3;
#if BPP == 32
                palette_before[i] = (c6_to_8(s->palette_before_snapshot[v]) << 16) |
                    (c6_to_8(s->palette_before_snapshot[v + 1]) << 8) |
                    c6_to_8(s->palette_before_snapshot[v + 2]);
                palette_after[i] = (c6_to_8(s->palette_after_snapshot[v]) << 16) |
                    (c6_to_8(s->palette_after_snapshot[v + 1]) << 8) |
                    c6_to_8(s->palette_after_snapshot[v + 2]);
#elif BPP == 16
                palette_before[i] = (s->palette_before_snapshot[v + 2] >> 1) |
                    ((s->palette_before_snapshot[v + 1]) << 5) |
                    ((s->palette_before_snapshot[v] >> 1) << 11);
                palette_after[i] = (s->palette_after_snapshot[v + 2] >> 1) |
                    ((s->palette_after_snapshot[v + 1]) << 5) |
                    ((s->palette_after_snapshot[v] >> 1) << 11);
#endif
            }
        }
        if (s->sr[0x01] & 8) {
            xdiv = 2;
            w *= 2;
        }
    } else {
        if (!vbe_enabled(s)) {
            update_palette256(s, palette);
            /* For Tanmatsu, output native resolution and let PPA handle scaling.
             * CRTC gives output resolution (640x400), native is (320x200).
             * xdiv=1 means no software pixel doubling - PPA will scale 2x.
             * Also halve height since we skipped multi_scan above. */
            xdiv = 1;
            w = w / 2;  /* Native width: 640->320, 800->400, etc. */
            if (tanmatsu_vdouble > 0)
                h = h / tanmatsu_vdouble;  /* Native height: 400->200 */
            bpp = 8;
        } else {
            bpp = s->vbe_regs[VBE_DISPI_INDEX_BPP];
            if (bpp == 8)
                update_palette256(s, palette);
        }
    }

    int y1 = 0;
    int i0 = 0;
#if defined(SCALE_3_2) || defined(SWAPXY)
#ifdef SCALE_3_2
    int hx = fb_dev->height * 3 / 2;
    int wx = fb_dev->width * 3 / 2;
    if (h < hx)
#ifdef SWAPXY
        i0 += (hx - h) / 3 * (BPP / 8);
#else
        i0 += (hx - h) / 3 * fb_dev->stride;
#endif
    else
        h = hx;
    if (w < wx)
#ifdef SWAPXY
        i0 += (wx - w) / 3 * fb_dev->stride;
#else
        i0 += (wx - w) / 3 * (BPP / 8);
#endif
    else
        w = wx;
#else
    int hx = fb_dev->height;
    int wx = fb_dev->width;
    if (h < hx)
        i0 += (hx - h) / 2 * (BPP / 8);
    else
        h = hx;
    if (w < wx)
        i0 += (wx - w) / 2 * fb_dev->stride;
    else
        w = wx;
#endif
    int yyt = 0;
    int yy = 0;
#else
    int hx = fb_dev->height;
    int wx = fb_dev->width;
    /* For Tanmatsu, don't center - render at 0,0 and let PPA handle scaling.
     * Report native VGA dimensions and pixel doubling factor for PPA. */
    globals.vga_mode_width = (w < wx) ? w : wx;
    globals.vga_mode_height = (h < hx) ? h : hx;
    /* For 256-color modes, we output native (half width) and PPA doubles.
     * For text/EGA modes, output is already at intended resolution. */
    globals.vga_pixel_double = (shift_control >= 2 && !vbe_enabled(s)) ? 2 : 1;
    if (h > hx) h = hx;
    if (w > wx) w = wx;
    /* i0 stays 0 - no centering offset */
#endif

    /* VGA split screen: line_compare is the first line that resets display
     * address to 0. When split_y >= h, no split — the comparison never matches. */
    int split_y;
    if (tanmatsu_vdouble > 0 && line_compare < (uint32_t)(h * tanmatsu_vdouble))
        split_y = line_compare / tanmatsu_vdouble;
    else
        split_y = (line_compare < (uint32_t)h) ? (int)line_compare : h;

    /* Pixel panning: sub-character-clock horizontal shift (AR[0x13]) */
    int panning;
    if (shift_control <= 1)
        panning = s->ar[0x13] & 7;      /* Planar/EGA: 0-7 pixel shift */
    else
        panning = (s->ar[0x13] >> 1) & 3; /* 256-color: 0-3 pixel shift */

    /* BitScrambler batch state — accumulate dirty planar lines, DMA in one shot */
    int bs_batch_count = 0;
    int bs_batch_y[BS_BATCH_LINES];
    int bs_vram_line_bytes = 0;
    int bs_gather_stride = 0;
    int bs_xdiv = 0;
    int bs_panning = 0;
    uint32_t *bs_batch_palette = NULL;  /* palette active for current batch */

    for (int y = 0; y < h; y++) {
        /* VGA split screen: reset display address at line_compare */
        if (y == split_y) {
            addr1 = 0;
            y1 = 0;
            multi_run = multi_scan;
            if (!(s->ar[0x10] & 0x20))
                panning = 0;  /* Clear pixel panning in split screen area */
        }

        /* Select palette based on scanline for mid-frame palette switching */
        uint32_t *cur_palette;
        if (palette_switch_line >= 0) {
            cur_palette = (y < palette_switch_line) ? palette_before : palette_after;
        } else {
            cur_palette = palette;
        }

        uint32_t addr = addr1;
        if (!(s->cr[0x17] & 1)) {
            int shift;
            /* CGA compatibility handling */
            shift = 14 + ((s->cr[0x17] >> 6) & 1);
            addr = (addr & ~(1 << shift)) | ((y1 & 1) << shift);
        }
        if (!(s->cr[0x17] & 2)) {
            addr = (addr & ~0x8000) | ((y1 & 2) << 14);
        }
        /* 256-color byte-level pixel panning (mode 13h) */
        if (shift_control >= 2)
            addr += panning;
        addr &= vram_addr_mask;

        /*
         * Fast path for EGA planar modes (16-color) with BitScrambler + dirty tracking.
         * Conditions: BPP==16, shift_control==0, no scaling, standard CRT mode
         *
         * BitScrambler path: gather dirty lines into DMA batch, HW bit permutation,
         * then palette pair LUT expansion. ~40x faster than software for the permutation
         * step, plus dirty tracking skips unchanged lines entirely.
         *
         * Falls back to software planar_expand table if BitScrambler unavailable.
         */
#if BPP == 16 && !defined(SCALE_3_2) && !defined(SWAPXY)
        if (shift_control == 0 && (s->cr[0x17] & 3) == 3) {
            int cpe_mask = s->ar[0x12] & 0x0f;
            int src_bytes = (w / xdiv) >> 3;
            int vram_line_bytes = src_bytes * 4;

            if (y == 0 && !ega_fast_logged) {
                fprintf(stderr, "VGA: planar fast path active (w=%d xdiv=%d cpe=0x%x bs=%s)\n",
                        w, xdiv, cpe_mask, bs_planar_handle ? "HW" : "SW");
                ega_fast_logged = 1;
            }

            /* --- BitScrambler hardware path --- */
            {
            int gather_bytes = vram_line_bytes + (panning > 0 ? 4 : 0);
            if (bs_planar_handle && gather_bytes <= BS_MAX_LINE_BYTES) {
                /* Rebuild palette LUTs if palette changed (hash-based detection) */
                build_bs_palette_luts(cur_palette, cpe_mask);

                /* Dirty line check via hash comparison */
#if 0 /* DIAG: disable to test framebuffer corruption theory */
                if (y < VGA_MAX_LINES) {
                    uint32_t line_hash = vga_hash_line(vram + addr, vram_line_bytes);
                    if (line_hash == vga_prev_line_hash[y] && !vga_is_line_dirty(y))
                        goto next_line;  /* Unchanged — skip */
                    vga_prev_line_hash[y] = line_hash;
                }
#endif

                /* Flush batch if palette or panning changed */
                if (bs_batch_count > 0 &&
                    (cur_palette != bs_batch_palette || panning != bs_panning)) {
                    size_t total = bs_batch_count * bs_gather_stride;
                    size_t padded = total + BS_DMA_PAD;
                    memset(bs_dma_in + total, 0, BS_DMA_PAD);
                    bitscrambler_loopback_run(bs_planar_handle,
                        bs_dma_in, padded, bs_dma_out, padded, NULL);
                    for (int b = 0; b < bs_batch_count; b++) {
                        uint32_t *fb32 = (uint32_t *)(fb_dev->fb_data + i0 +
                            bs_batch_y[b] * fb_dev->stride);
                        bs_expand_line(fb32, bs_dma_out + b * bs_gather_stride,
                                       bs_vram_line_bytes, bs_xdiv, bs_panning);
                    }
                    bs_batch_count = 0;
                    bs_pal_last_hash = 0;
                    build_bs_palette_luts(cur_palette, cpe_mask);
                }

                /* Gather this dirty line into DMA input buffer (wrap-aware) */
                if (addr + gather_bytes <= vram_wrap_size) {
                    memcpy(bs_dma_in + bs_batch_count * gather_bytes,
                           vram + addr, gather_bytes);
                } else {
                    uint32_t first = vram_wrap_size - addr;
                    uint8_t *dst = bs_dma_in + bs_batch_count * gather_bytes;
                    memcpy(dst, vram + addr, first);
                    memcpy(dst + first, vram, gather_bytes - first);
                }
                bs_batch_y[bs_batch_count] = y;
                bs_batch_count++;
                bs_vram_line_bytes = vram_line_bytes;
                bs_gather_stride = gather_bytes;
                bs_xdiv = xdiv;
                bs_panning = panning;
                bs_batch_palette = cur_palette;

                /* Flush when batch full */
                if (bs_batch_count >= BS_BATCH_LINES) {
                    size_t total = bs_batch_count * gather_bytes;
                    size_t padded = total + BS_DMA_PAD;
                    memset(bs_dma_in + total, 0, BS_DMA_PAD);
                    bitscrambler_loopback_run(bs_planar_handle,
                        bs_dma_in, padded, bs_dma_out, padded, NULL);
                    for (int b = 0; b < bs_batch_count; b++) {
                        uint32_t *fb32 = (uint32_t *)(fb_dev->fb_data + i0 +
                            bs_batch_y[b] * fb_dev->stride);
                        bs_expand_line(fb32, bs_dma_out + b * gather_bytes,
                                       vram_line_bytes, xdiv, panning);
                    }
                    bs_batch_count = 0;
                }
                goto next_line;
            }
            }

            /* --- Software fallback (no BitScrambler) --- */
            {
                uint16_t *fb_row = (uint16_t *)(fb_dev->fb_data + i0 + y * fb_dev->stride);

                /* Copy VRAM line into stack buffer for sequential PSRAM read.
                 * Read extra 4 bytes (one character clock) when pixel panning. */
                int read_bytes = vram_line_bytes + (panning > 0 ? 4 : 0);
                uint8_t linebuf[408];
                const uint8_t *src_line;
                if (read_bytes <= (int)sizeof(linebuf)) {
                    if (addr + read_bytes <= vram_wrap_size) {
                        memcpy(linebuf, vram + addr, read_bytes);
                    } else {
                        uint32_t first = vram_wrap_size - addr;
                        memcpy(linebuf, vram + addr, first);
                        memcpy(linebuf + first, vram, read_bytes - first);
                    }
                    src_line = linebuf;
                } else {
                    src_line = vram + addr;
                }

                if (panning == 0) {
                /* No panning: fast 2-byte unrolled loop */
                int bx = 0;
                for (; bx + 1 < src_bytes; bx += 2) {
                    int off = 4 * bx;
                    uint32_t planes0 = *(const uint32_t *)(src_line + off);
                    uint32_t planes1 = *(const uint32_t *)(src_line + off + 4);
                    const uint8_t *p0 = planar_expand[planes0 & 0xFF];
                    const uint8_t *p1 = planar_expand[(planes0 >> 8) & 0xFF];
                    const uint8_t *p2 = planar_expand[(planes0 >> 16) & 0xFF];
                    const uint8_t *p3 = planar_expand[planes0 >> 24];
                    const uint8_t *q0 = planar_expand[planes1 & 0xFF];
                    const uint8_t *q1 = planar_expand[(planes1 >> 8) & 0xFF];
                    const uint8_t *q2 = planar_expand[(planes1 >> 16) & 0xFF];
                    const uint8_t *q3 = planar_expand[planes1 >> 24];
                    if (xdiv == 2) {
                        uint32_t *fb32 = (uint32_t *)fb_row;
                        for (int i = 0; i < 8; i++) {
                            uint32_t c = cur_palette[(p0[i] | (p1[i]<<1) | (p2[i]<<2) | (p3[i]<<3)) & cpe_mask];
                            fb32[i] = c | (c << 16);
                        }
                        for (int i = 0; i < 8; i++) {
                            uint32_t c = cur_palette[(q0[i] | (q1[i]<<1) | (q2[i]<<2) | (q3[i]<<3)) & cpe_mask];
                            fb32[8+i] = c | (c << 16);
                        }
                        fb_row += 32;
                    } else {
                        for (int i = 0; i < 8; i++)
                            fb_row[i] = cur_palette[(p0[i] | (p1[i]<<1) | (p2[i]<<2) | (p3[i]<<3)) & cpe_mask];
                        for (int i = 0; i < 8; i++)
                            fb_row[8+i] = cur_palette[(q0[i] | (q1[i]<<1) | (q2[i]<<2) | (q3[i]<<3)) & cpe_mask];
                        fb_row += 16;
                    }
                }
                for (; bx < src_bytes; bx++) {
                    int off = 4 * bx;
                    uint32_t planes0 = *(const uint32_t *)(src_line + off);
                    const uint8_t *p0 = planar_expand[planes0 & 0xFF];
                    const uint8_t *p1 = planar_expand[(planes0 >> 8) & 0xFF];
                    const uint8_t *p2 = planar_expand[(planes0 >> 16) & 0xFF];
                    const uint8_t *p3 = planar_expand[planes0 >> 24];
                    if (xdiv == 2) {
                        uint32_t *fb32 = (uint32_t *)fb_row;
                        for (int i = 0; i < 8; i++) {
                            uint32_t c = cur_palette[(p0[i] | (p1[i]<<1) | (p2[i]<<2) | (p3[i]<<3)) & cpe_mask];
                            fb32[i] = c | (c << 16);
                        }
                        fb_row += 16;
                    } else {
                        for (int i = 0; i < 8; i++)
                            fb_row[i] = cur_palette[(p0[i] | (p1[i]<<1) | (p2[i]<<2) | (p3[i]<<3)) & cpe_mask];
                        fb_row += 8;
                    }
                }
                } else {
                /* Pixel panning: three-part render (partial first byte, middle, partial last) */
                int total_bytes = src_bytes + 1;
                int total_pixels = src_bytes * 8;
                int px_out = 0;
                for (int bx = 0; bx < total_bytes && px_out < total_pixels; bx++) {
                    uint32_t planes0 = *(const uint32_t *)(src_line + 4 * bx);
                    const uint8_t *p0 = planar_expand[planes0 & 0xFF];
                    const uint8_t *p1 = planar_expand[(planes0 >> 8) & 0xFF];
                    const uint8_t *p2 = planar_expand[(planes0 >> 16) & 0xFF];
                    const uint8_t *p3 = planar_expand[planes0 >> 24];
                    int istart = (bx == 0) ? panning : 0;
                    int iend = 8;
                    if (px_out + (iend - istart) > total_pixels)
                        iend = istart + (total_pixels - px_out);
                    if (xdiv == 2) {
                        uint32_t *fb32 = (uint32_t *)(fb_row + px_out * 2);
                        for (int i = istart; i < iend; i++) {
                            uint32_t c = cur_palette[(p0[i] | (p1[i]<<1) | (p2[i]<<2) | (p3[i]<<3)) & cpe_mask];
                            fb32[i - istart] = c | (c << 16);
                        }
                    } else {
                        for (int i = istart; i < iend; i++)
                            fb_row[px_out + (i - istart)] = cur_palette[(p0[i] | (p1[i]<<1) | (p2[i]<<2) | (p3[i]<<3)) & cpe_mask];
                    }
                    px_out += iend - istart;
                }
                }
            }
            goto next_line;
        }

        /*
         * Fast path for VGA 256-color mode (mode 13h and similar)
         * Conditions: BPP==16, shift_control==2, bpp==8, xdiv==2
         */
        if (shift_control == 2 && bpp == 8 && xdiv == 2) {
            /* Debug: trace fast path usage (only on first scanline of first frame) */
            if (y == 0 && !vga256_fast_logged) {
                fprintf(stderr, "VGA: VGA256 (w=%d h=%d i0=%d stride=%d fbw=%d)\n",
                        w, h, i0, fb_dev->stride, fb_dev->width);
                vga256_fast_logged = 1;
            }
            uint16_t *fb_row = (uint16_t *)(fb_dev->fb_data + i0 + y * fb_dev->stride);
            /* Calculate source bytes: use min of display width and VRAM line width */
            /* line_offset is the VRAM bytes per line (CR[0x13] << 3) */
            int src_from_display = w / 2;  /* Source pixels based on display width */
            int src_from_vram = (line_offset > 0) ? line_offset : src_from_display;
            int src_bytes = (src_from_display < src_from_vram) ? src_from_display : src_from_vram;
            int num_chunks = src_bytes >> 3;  /* Process 8 source pixels at a time */

            for (int chunk = 0; chunk < num_chunks; chunk++) {
                uint8_t *vram_ptr = vram + addr + chunk * 8;
                /* Read 8 palette indices and look up colors */
                uint16_t colors[8] __attribute__((aligned(16)));
                colors[0] = palette[vram_ptr[0]];
                colors[1] = palette[vram_ptr[1]];
                colors[2] = palette[vram_ptr[2]];
                colors[3] = palette[vram_ptr[3]];
                colors[4] = palette[vram_ptr[4]];
                colors[5] = palette[vram_ptr[5]];
                colors[6] = palette[vram_ptr[6]];
                colors[7] = palette[vram_ptr[7]];

                /* Write 16 pixels (8 source pixels doubled) using 32-bit stores */
                uint32_t *fb32 = (uint32_t *)fb_row;
                fb32[0] = colors[0] | (colors[0] << 16);
                fb32[1] = colors[1] | (colors[1] << 16);
                fb32[2] = colors[2] | (colors[2] << 16);
                fb32[3] = colors[3] | (colors[3] << 16);
                fb32[4] = colors[4] | (colors[4] << 16);
                fb32[5] = colors[5] | (colors[5] << 16);
                fb32[6] = colors[6] | (colors[6] << 16);
                fb32[7] = colors[7] | (colors[7] << 16);
                fb_row += 16;
            }
            goto next_line;
        }

        /*
         * Optimized fast path for VGA 256-color native output (Tanmatsu with PPA scaling)
         * Conditions: shift_control==2, bpp==8, xdiv==1 (no CPU pixel doubling)
         *
         * Optimizations:
         * 1. DRAM palette cache - lookups ~3-5 cycles vs ~20+ from PSRAM
         * 2. Dirty line tracking - skip unchanged scanlines (hash-based)
         * 3. VRAM line prefetch - copy to internal SRAM for sequential access
         * 4. PIE SIMD stores - 8 pixels per 128-bit write
         */
        if (shift_control == 2 && bpp == 8 && xdiv == 1) {
            if (y == 0 && !vga256_fast_logged) {
                fprintf(stderr, "VGA: VGA256 native optimized (w=%d h=%d) DRAM palette + dirty tracking\n",
                        w, h);
                vga256_fast_logged = 1;
            }

            int src_bytes = w;  /* Native: 1 byte = 1 pixel */
            uint8_t *vram_line = vram + addr;

            /* Check if this line changed using fast hash comparison */
            if (y < VGA_MAX_LINES) {
                uint32_t line_hash = vga_hash_line(vram_line, src_bytes);
                if (line_hash == vga_prev_line_hash[y] && !vga_is_line_dirty(y)) {
                    /* Line unchanged - skip rendering */
                    goto next_line;
                }
                vga_prev_line_hash[y] = line_hash;
            }

            /* Process 16 pixels per iteration with aggressive unrolling */
            const uint16_t *pal = vga256_palette_cache;
            uint32_t *fb32 = (uint32_t *)(fb_dev->fb_data + i0 + y * fb_dev->stride);
            const uint8_t *src = vram_line;
            int pixels_left = src_bytes;

            /* Main loop: 16 pixels at a time */
            while (pixels_left >= 16) {
                /* Pack pairs of colors into 32-bit words for efficient stores */
                uint32_t c0 = pal[src[0]] | (pal[src[1]] << 16);
                uint32_t c1 = pal[src[2]] | (pal[src[3]] << 16);
                uint32_t c2 = pal[src[4]] | (pal[src[5]] << 16);
                uint32_t c3 = pal[src[6]] | (pal[src[7]] << 16);
                uint32_t c4 = pal[src[8]] | (pal[src[9]] << 16);
                uint32_t c5 = pal[src[10]] | (pal[src[11]] << 16);
                uint32_t c6 = pal[src[12]] | (pal[src[13]] << 16);
                uint32_t c7 = pal[src[14]] | (pal[src[15]] << 16);
                fb32[0] = c0; fb32[1] = c1; fb32[2] = c2; fb32[3] = c3;
                fb32[4] = c4; fb32[5] = c5; fb32[6] = c6; fb32[7] = c7;
                fb32 += 8;
                src += 16;
                pixels_left -= 16;
            }
            /* Handle remaining 8 pixels if any */
            if (pixels_left >= 8) {
                uint32_t c0 = pal[src[0]] | (pal[src[1]] << 16);
                uint32_t c1 = pal[src[2]] | (pal[src[3]] << 16);
                uint32_t c2 = pal[src[4]] | (pal[src[5]] << 16);
                uint32_t c3 = pal[src[6]] | (pal[src[7]] << 16);
                fb32[0] = c0; fb32[1] = c1; fb32[2] = c2; fb32[3] = c3;
            }
            goto next_line;
        }
#endif

        /* Cache for EGA planar mode - pre-compute all 8 colors when VRAM changes */
        uint32_t cached_byte_addr = ~0u;
        uint32_t cached_colors[8];  /* Pre-computed RGB colors for 8 source pixels */
        for (int x = 0; x < w; x++) {
            int x1 = x / xdiv;
            uint32_t color;
            if (shift_control == 0) {
                /* EGA 16-color planar mode - use lookup tables for fast bit extraction */
                uint32_t byte_addr = addr + 4 * (x1 >> 3);
                int bit = x1 & 7;
                /* Pre-compute all 8 colors when byte address changes */
                if (byte_addr != cached_byte_addr) {
                    cached_byte_addr = byte_addr;
                    uint8_t v0 = vram[byte_addr];
                    uint8_t v1 = vram[byte_addr + 1];
                    uint8_t v2 = vram[byte_addr + 2];
                    uint8_t v3 = vram[byte_addr + 3];
                    /* AR[0x12] Color Plane Enable - mask which planes contribute */
                    int cpe_mask = s->ar[0x12] & 0x0f;
                    {
                        const uint8_t *p0 = planar_expand[v0];
                        const uint8_t *p1 = planar_expand[v1];
                        const uint8_t *p2 = planar_expand[v2];
                        const uint8_t *p3 = planar_expand[v3];
                        for (int i = 0; i < 8; i++) {
                            int k = (p0[i] | (p1[i] << 1) | (p2[i] << 2) | (p3[i] << 3)) & cpe_mask;
                            cached_colors[i] = cur_palette[k];
                        }
                    }
                }
                /* Hot path: just index into pre-computed colors - no table lookups! */
                color = cached_colors[bit];
            } else if (shift_control == 1) {
                int k = ((vram[addr + 4 * (x1 >> 3) + ((x1 & 4) >> 2)] >>
                          (6 - 2 * (x1 & 3))) & 3);
                color = cur_palette[k];
            } else
#if BPP == 32
            {
                switch (bpp) {
                case 8: {
                    int k = vram[addr + x1];
                    color = cur_palette[k];
                    break;
                }
                case 15: {
                    int k = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    int b = (k & ((1 << 5) - 1)) << 3;
                    int g = ((k >> 5) & ((1 << 5) - 1)) << 3;
                    int r = ((k >> 10) & ((1 << 5) - 1)) << 3;
                    color = b | (g << 8) | (r << 16);
                    break;
                }
                case 16: {
                    int k = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    int b = (k & ((1 << 5) - 1)) << 3;
                    int g = ((k >> 5) & ((1 << 6) - 1)) << 2;
                    int r = ((k >> 11) & ((1 << 5) - 1)) << 3;
                    color = b | (g << 8) | (r << 16);
                    break;
                }
                case 24: {
                    color = vram[addr + 3 * x1] |
                        (vram[addr + 3 * x1 + 1] << 8) |
                        (vram[addr + 3 * x1 + 2] << 16);
                    break;
                }
                case 32: {
                    color = vram[addr + 4 * x1] |
                        (vram[addr + 4 * x1 + 1] << 8) |
                        (vram[addr + 4 * x1 + 2] << 16) |
                        (vram[addr + 4 * x1 + 3] << 24);
                    break;
                }
                default:
                    fprintf(stderr, "vga bpp is %d\n", bpp);
                    abort();
                }
            }
            int i = (BPP / 8) * (y * fb_dev->width + x) + i0;
            fb_dev->fb_data[i + 0] = color;
            fb_dev->fb_data[i + 1] = color >> 8;
            fb_dev->fb_data[i + 2] = color >> 16;
            fb_dev->fb_data[i + 3] = color >> 24;
#elif BPP == 16
            {
                switch (bpp) {
                case 8: {
                    color = cur_palette[vram[addr + x1]];
                    break;
                }
                case 15: {
                    int k = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    color = (k & 0x1f) | ((k & ~0x1f) << 1);
                    break;
                }
                case 16: {
                    color = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    break;
                }
                case 24: {
                    color = ((vram[addr + 3 * x1] >> 3)) |
                        ((vram[addr + 3 * x1 + 1] >> 2) << 5) |
                        ((vram[addr + 3 * x1 + 2] >> 3) << 11);
                    break;
                }
                case 32: {
                    color = ((vram[addr + 4 * x1] >> 3)) |
                        ((vram[addr + 4 * x1 + 1] >> 2) << 5) |
                        ((vram[addr + 4 * x1 + 2] >> 3) << 11);
                    break;
                }
                default:
                    fprintf(stderr, "vga bpp is %d\n", bpp);
                    abort();
                }
            }
#if defined(SCALE_3_2) || defined(SWAPXY)
            int i = (BPP / 8) * (yyt * w + x);
            s->tmpbuf[i + 0] = color;
            s->tmpbuf[i + 1] = color >> 8;
#else
            int i = (BPP / 8) * (y * fb_dev->width + x) + i0;
            fb_dev->fb_data[i + 0] = color;
            fb_dev->fb_data[i + 1] = color >> 8;
#endif
#else
#error "bad bpp"
#endif
        }
next_line:
        if (!multi_run) {
            int mask = (s->cr[0x17] & 3) ^ 3;
            if ((y1 & mask) == mask) {
                addr1 += line_offset;
                addr1 &= vram_addr_mask;
            }
            y1++;
            multi_run = multi_scan;
        } else {
            multi_run--;
        }
#if defined(SCALE_3_2) || defined(SWAPXY)
        yyt++;
        if (yyt == 3) {
#ifdef SWAPXY
            int ii0 = (BPP / 8) * yy + i0;
#else
            int ii0 = (BPP / 8) * (yy * fb_dev->width) + i0;
#endif
#ifdef SCALE_3_2
            scale_3_2(fb_dev->fb_data + ii0, fb_dev->stride, s->tmpbuf, w);
            yyt = 0;
            yy += 2;
#else
            scale_3_3(fb_dev->fb_data + ii0, fb_dev->stride, s->tmpbuf, w);
            yyt = 0;
            yy += 3;
#endif
        }
#endif
    }

    /* Flush remaining BitScrambler batch (lines accumulated but not yet DMA'd) */
    if (bs_batch_count > 0 && bs_planar_handle) {
        size_t total = bs_batch_count * bs_gather_stride;
        size_t padded = total + BS_DMA_PAD;
        memset(bs_dma_in + total, 0, BS_DMA_PAD);
        bitscrambler_loopback_run(bs_planar_handle,
            bs_dma_in, padded, bs_dma_out, padded, NULL);
        for (int b = 0; b < bs_batch_count; b++) {
            uint32_t *fb32 = (uint32_t *)(fb_dev->fb_data + i0 +
                bs_batch_y[b] * fb_dev->stride);
            bs_expand_line(fb32, bs_dma_out + b * bs_gather_stride,
                           bs_vram_line_bytes, bs_xdiv, bs_panning);
        }
    }

    /* Clear dirty line flags after frame is rendered */
    vga_clear_dirty_lines();

#ifdef DEBUG_VGA
    /* Debug timing report */
    uint32_t t_end = get_uticks();
    uint32_t elapsed = t_end - t_start;
    frame_count++;
    if (elapsed > 50000) {  /* > 50ms is slow */
        slow_frame_count++;
    }
    if (t_end - last_report > 5000000) {  /* Report every 5 seconds */
        if (slow_frame_count > 0) {
            fprintf(stderr, "VGA perf: %u frames, %u slow (>50ms), last=%uus, w=%d h=%d sc=%d\n",
                    frame_count, slow_frame_count, elapsed, w, h, shift_control);
        }
        frame_count = 0;
        slow_frame_count = 0;
        last_report = t_end;
    }
#endif

    redraw_func(opaque, 0, 0, fb_dev->width, fb_dev->height);
}

static void simplefb_clear(FBDevice *fb_dev,
               SimpleFBDrawFunc *redraw_func, void *opaque)
{
    memset(fb_dev->fb_data, 0, fb_dev->width * fb_dev->height * (BPP / 8));
}

int vga_step(VGAState *s)
{
    uint32_t now = get_uticks();
    int ret = 0;

    /* Check if 3DA polling optimization already triggered a render */
    if (s->render_pending) {
        s->render_pending = 0;
        ret = 1;
    }

    if (after_eq(now, s->retrace_time)) {
        if (s->retrace_phase == 0) {
            s->st01 |= ST01_DISP_ENABLE;
            s->retrace_phase = 1;
            s->retrace_time = now + 833;
        } else if (s->retrace_phase == 1) {
            s->st01 |= ST01_V_RETRACE;
            s->retrace_phase = 2;
            s->retrace_time = now + 833;
            ret = 1;
        } else {
            s->st01 &= ~(ST01_V_RETRACE | ST01_DISP_ENABLE);
            s->retrace_phase = 0;
            /* Reset mid-frame palette tracking for new frame */
            s->hblank_poll_count = 0;
            s->retrace_time = now + 15000/3;
        }
    }

    return ret;
}

void vga_refresh(VGAState *s,
                 SimpleFBDrawFunc *redraw_func, void *opaque, int full_update)
{
    FBDevice *fb_dev = s->fb_dev;
    int graphic_mode;

    /* Adaptive frame skipping - skip if previous render was slow */
    if (vga_frame_skip_max > 0 && !full_update) {
        uint64_t now = esp_timer_get_time();
        /* Check if we should skip this frame */
        if (vga_last_render_duration > VGA_FRAME_SKIP_THRESHOLD_US &&
            vga_frame_skip_count < vga_frame_skip_max) {
            /* Skip this frame - previous render was slow */
            vga_frame_skip_count++;
            globals.vga_frame_gen++;  /* Still increment gen for tracking */
            return;
        }
        /* Reset skip counter if we're rendering */
        vga_frame_skip_count = 0;
        vga_last_render_time = now;
    }

    if (!(s->ar_index & 0x20)) {
        /* blank */
        graphic_mode = 0;
    } else if (s->gr[0x06] & 1) {
        /* graphic mode */
        graphic_mode = 2;
    } else {
        /* text mode */
        graphic_mode = 1;
    }

    if (graphic_mode != s->graphic_mode) {
        s->graphic_mode = graphic_mode;
        full_update = 1;
        s->cursor_blink_time = get_uticks();
        simplefb_clear(fb_dev, redraw_func, opaque);
        /* Force vga_graphic_refresh to re-clear on mode change */
        s->force_graphic_clear = 1;
        /* Signal to reset dynamic batch size - new program starting */
        extern struct Globals globals;
        globals.batch_reset_pending = true;
    }

    if (s->graphic_mode == 2) {
        vga_graphic_refresh(s, redraw_func, opaque, full_update);
    } else if (s->graphic_mode == 1) {
        vga_text_refresh(s, redraw_func, opaque, full_update);
    }

    /* Track render duration for adaptive frame skip */
    if (vga_frame_skip_max > 0) {
        vga_last_render_duration = esp_timer_get_time() - vga_last_render_time;
    }
    /* Increment frame generation counter for dirty tracking.
     * lcd_bsp.c can skip PPA rotation if this hasn't changed. */
    globals.vga_frame_gen++;
}

/* force some bits to zero */
static const uint8_t sr_mask[8] = {
    (uint8_t)~0xfc,
    (uint8_t)~0xc2,
    (uint8_t)~0xf0,
    (uint8_t)~0xc0,
    (uint8_t)~0xf1,
    (uint8_t)~0xff,
    (uint8_t)~0xff,
    (uint8_t)~0x00,
};

static const uint8_t gr_mask[16] = {
    (uint8_t)~0xf0, /* 0x00 */
    (uint8_t)~0xf0, /* 0x01 */
    (uint8_t)~0xf0, /* 0x02 */
    (uint8_t)~0xe0, /* 0x03 */
    (uint8_t)~0xfc, /* 0x04 */
    (uint8_t)~0x84, /* 0x05 */
    (uint8_t)~0xf0, /* 0x06 */
    (uint8_t)~0xf0, /* 0x07 */
    (uint8_t)~0x00, /* 0x08 */
    (uint8_t)~0xff, /* 0x09 */
    (uint8_t)~0xff, /* 0x0a */
    (uint8_t)~0xff, /* 0x0b */
    (uint8_t)~0xff, /* 0x0c */
    (uint8_t)~0xff, /* 0x0d */
    (uint8_t)~0xff, /* 0x0e */
    (uint8_t)~0xff, /* 0x0f */
};

uint32_t vga_ioport_read(VGAState *s, uint32_t addr)
{
    int val, index;

    /* check port range access depending on color/monochrome mode */
    if ((addr >= 0x3b0 && addr <= 0x3bf && (s->msr & MSR_COLOR_EMULATION)) ||
        (addr >= 0x3d0 && addr <= 0x3df && !(s->msr & MSR_COLOR_EMULATION))) {
        val = 0xff;
    } else {
        switch(addr) {
        case 0x3c0:
            if (s->ar_flip_flop == 0) {
                val = s->ar_index;
            } else {
                val = 0;
            }
            break;
        case 0x3c1:
            index = s->ar_index & 0x1f;
            if (index < 21)
                val = s->ar[index];
            else
                val = 0;
            break;
        case 0x3c2:
            /* ISR0: toggle switch sense bit on each read (real VGA behavior) */
            s->st00 ^= 0x10;
            val = s->st00;
            break;
        case 0x3c4:
            val = s->sr_index;
            break;
        case 0x3c5:
            val = s->sr[s->sr_index];
#ifdef DEBUG_VGA_REG
            printf("vga: read SR%x = 0x%02x\n", s->sr_index, val);
#endif
            break;
        case 0x3c7:
            val = s->dac_state;
            break;
        case 0x3c8:
            val = s->dac_write_index;
            break;
        case 0x3c9:
            val = s->palette[s->dac_read_index * 3 + s->dac_sub_index];
            if (++s->dac_sub_index == 3) {
                s->dac_sub_index = 0;
                s->dac_read_index++;
            }
            break;
        case 0x3ca:
            val = s->fcr;
            break;
        case 0x3cc:
            val = s->msr;
            break;
        case 0x3ce:
            val = s->gr_index;
            break;
        case 0x3cf:
            val = s->gr[s->gr_index];
#ifdef DEBUG_VGA_REG
            printf("vga: read GR%x = 0x%02x\n", s->gr_index, val);
#endif
            break;
        case 0x3b4:
        case 0x3d4:
            val = s->cr_index;
            break;
        case 0x3b5:
        case 0x3d5:
            val = s->cr[s->cr_index];
#ifdef DEBUG_VGA_REG
            printf("vga: read CR%x = 0x%02x\n", s->cr_index, val);
#endif
            break;
        case 0x3ba:
        case 0x3da:
            /* Retrace status register - with polling optimization */
            val = s->st01;
            s->ar_flip_flop = 0;
            /*
             * Polling optimization: DOS games poll 0x3DA in tight loops.
             * We detect rapid consecutive polls and fast-forward the state
             * machine, but only during actual tight loops (polls <20us apart).
             */
            {
                static uint32_t poll_count = 0;
                static uint32_t last_poll_report = 0;
                static uint32_t last_poll_time = 0;
                static uint32_t rapid_poll_count = 0;
                uint32_t now = get_uticks();
                poll_count++;

                /* Detect tight polling loop: polls within 20us of each other */
                if (now - last_poll_time < 20) {
                    rapid_poll_count++;
                } else {
                    rapid_poll_count = 0;  /* Gap detected, reset */
                }
                last_poll_time = now;

                /*
                 * Fast-forward logic: Only if we're in a tight polling loop
                 * (500+ rapid consecutive polls = ~10ms of tight polling)
                 * AND in display period (phase 0), skip to retrace.
                 */
                const uint32_t rapid_threshold = 500;
                int should_advance = after_eq(now, s->retrace_time);

                /* Fast-forward from display period if polling heavily */
                if (!should_advance && rapid_poll_count > rapid_threshold &&
                    s->retrace_phase == 0) {
                    should_advance = 1;
                }

                if (should_advance) {
                    rapid_poll_count = 0;
                    if (s->retrace_phase == 0) {
                        s->st01 |= ST01_DISP_ENABLE;
                        s->retrace_phase = 1;
                        s->retrace_time = now + 833;
                    } else if (s->retrace_phase == 1) {
                        s->st01 |= ST01_V_RETRACE;
                        s->retrace_phase = 2;
                        s->retrace_time = now + 833;
                        s->render_pending = 1;  /* Signal vga_step to trigger render */
                    } else {
                        s->st01 &= ~(ST01_V_RETRACE | ST01_DISP_ENABLE);
                        s->retrace_phase = 0;
                        /* Reset mid-frame palette tracking for new frame */
                        s->hblank_poll_count = 0;
                        s->retrace_time = now + 15000/3;
                    }
                    val = s->st01;
                }

                /* Track hblank polling for mid-frame palette detection */
                if (s->retrace_phase == 0) {
                    s->hblank_poll_count++;
                }

#ifdef DEBUG_VGA
                /* Debug report - disabled by default to reduce overhead */
                if (now - last_poll_report > 1000000) {
                    if (poll_count > 1000) {
                        fprintf(stderr, "VGA: 3DA polled %u/sec st01=%02x\n", poll_count, val);
                    }
                    poll_count = 0;
                    last_poll_report = now;
                }
#endif
            }
            break;
        default:
            val = 0x00;
            break;
        }
    }
#if defined(DEBUG_VGA)
    printf("VGA: read addr=0x%04x data=0x%02x\n", addr, val);
#endif
    /* Skip retrace polling (3DA/3BA) and DAC (3C8/3C9) from trace - too noisy */
    if (addr != 0x3da && addr != 0x3ba && addr != 0x3c9 && addr != 0x3c8)
        vga_trace_record(addr, val, 0, s);
    return val;
}

/* Update cached VGA state for fast-path optimizations.
 * Called when SR or GR registers change. */
static inline void vga_update_cached_state(VGAState *s)
{
    s->cached_plane_mask = s->sr[VGA_SEQ_PLANE_WRITE];
    s->cached_write_mode = s->gr[VGA_GFX_MODE] & 3;
    s->cached_chain4 = (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) ? 1 : 0;
    s->cached_memmap_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;

    /* Base conditions for any fast path:
     * - Not chain-4 mode (Mode X uses unchained)
     * - Not odd/even mode
     * - Memory map mode 0 or 1 */
    int base_ok = !s->cached_chain4 &&
                  !(s->gr[VGA_GFX_MODE] & 0x10) &&
                  (s->cached_memmap_mode <= 1);

    /* cached_simple_write encodes which fast paths are available:
     * 0 = no fast path
     * 1 = write mode 1 fast path only (latch copy)
     * 2 = write mode 0 fast path (simple byte write)
     * 3 = both fast paths available */
    s->cached_simple_write = 0;

    if (base_ok) {
        /* Write mode 1 (latch copy) ignores bit mask, rotation, set/reset */
        if (s->cached_write_mode == 1) {
            s->cached_simple_write = 1;
        }
        /* Write mode 0: simple if no rotation, no set/reset, no logic ops, full bit mask */
        else if (s->cached_write_mode == 0 &&
                 s->gr[VGA_GFX_DATA_ROTATE] == 0 &&
                 s->gr[VGA_GFX_SR_ENABLE] == 0 &&
                 s->gr[VGA_GFX_BIT_MASK] == 0xff) {
            s->cached_simple_write = 2;
        }
    }
}

void vga_ioport_write(VGAState *s, uint32_t addr, uint32_t val)
{
    int index;
    /* Skip DAC index/data (3C8/3C9) from trace - too noisy during palette loads */
    if (addr != 0x3c9 && addr != 0x3c8)
        vga_trace_record(addr, val, 1, s);

    /* check port range access depending on color/monochrome mode */
    if ((addr >= 0x3b0 && addr <= 0x3bf && (s->msr & MSR_COLOR_EMULATION)) ||
        (addr >= 0x3d0 && addr <= 0x3df && !(s->msr & MSR_COLOR_EMULATION)))
        return;

#ifdef DEBUG_VGA
    printf("VGA: write addr=0x%04x data=0x%02x\n", addr, val);
#endif

    switch(addr) {
    case 0x3c0:
        if (s->ar_flip_flop == 0) {
            val &= 0x3f;
            s->ar_index = val;
        } else {
            index = s->ar_index & 0x1f;
            switch(index) {
            case 0x00 ... 0x0f:
                s->ar[index] = val & 0x3f;
                break;
            case 0x10:
                s->ar[index] = val & ~0x10;
                break;
            case 0x11:
                s->ar[index] = val;
                break;
            case 0x12:
                s->ar[index] = val & ~0xc0;
                break;
            case 0x13:
                s->ar[index] = val & ~0xf0;
                break;
            case 0x14:
                s->ar[index] = val & ~0xf0;
                break;
            default:
                break;
            }
        }
        s->ar_flip_flop ^= 1;
        break;
    case 0x3c2:
        s->msr = val & ~0x10;
        break;
    case 0x3c4:
        s->sr_index = val & 7;
        break;
    case 0x3c5:
#ifdef DEBUG_VGA_REG
        printf("vga: write SR%x = 0x%02x\n", s->sr_index, val);
#endif
        s->sr[s->sr_index] = val & sr_mask[s->sr_index];
        /* Update cached state when plane mask or memory mode changes */
        if (s->sr_index == VGA_SEQ_PLANE_WRITE || s->sr_index == VGA_SEQ_MEMORY_MODE)
            vga_update_cached_state(s);
        break;
    case 0x3c7:
        s->dac_read_index = val;
        s->dac_sub_index = 0;
        s->dac_state = 3;
        break;
    case 0x3c8:
        s->dac_write_index = val;
        s->dac_sub_index = 0;
        s->dac_state = 0;
        break;
    case 0x3c9:
        s->dac_cache[s->dac_sub_index] = val;
        if (++s->dac_sub_index == 3) {
            /* Mid-frame palette switching detection:
             * - LOW poll count (< 50): "before" palette writes (frame start, top of screen)
             * - HIGH poll count (> 400): "after" palette writes (mid-frame, bottom of screen)
             *
             * Key insight: once we detect mid-frame switching, we KEEP palette_switch_scanline
             * set permanently. This ensures render always sees the split, avoiding race conditions
             * where render might catch us during the frame-start write window. */
            int is_mid_frame_write = (s->hblank_poll_count > 400);

            /* Always apply the write to the main palette */
            memcpy(&s->palette[s->dac_write_index * 3], s->dac_cache, 3);
            s->palette_generation++;

            if (is_mid_frame_write) {
                /* Mid-frame write - this is the "after" palette (bottom of screen) */
                if (s->palette_switch_scanline < 0) {
                    /* First time detecting mid-frame switch: set the scanline and
                     * capture current state as "before" (it has the top palette) */
                    s->palette_switch_scanline = s->hblank_poll_count / 8;
                    if (s->palette_switch_scanline > 400)
                        s->palette_switch_scanline = 400;
                    memcpy(s->palette_before_switch, s->palette, 768);
                    s->palette_before_gen = s->palette_generation;
                }
                /* Always update "after" palette with current state */
                memcpy(s->palette_after_switch, s->palette, 768);
                s->palette_after_gen = s->palette_generation;
            } else if (s->palette_switch_scanline >= 0 && s->hblank_poll_count < 50) {
                /* Frame-start write while mid-frame switching is active.
                 * Update the "before" palette to reflect the new top-of-screen colors.
                 * DON'T reset palette_switch_scanline - keep split rendering active. */
                memcpy(s->palette_before_switch, s->palette, 768);
                s->palette_before_gen = s->palette_generation;
            }
            s->dac_sub_index = 0;
            s->dac_write_index++;
        }
        break;
    case 0x3ce:
        s->gr_index = val & 0x0f;
        break;
    case 0x3cf:
#ifdef DEBUG_VGA_REG
        printf("vga: write GR%x = 0x%02x\n", s->gr_index, val);
#endif
        s->gr[s->gr_index] = val & gr_mask[s->gr_index];
        /* Update cached state when write mode, misc, bit mask, rotate, or set/reset changes */
        if (s->gr_index == VGA_GFX_MODE || s->gr_index == VGA_GFX_MISC ||
            s->gr_index == VGA_GFX_BIT_MASK || s->gr_index == VGA_GFX_DATA_ROTATE ||
            s->gr_index == VGA_GFX_SR_ENABLE)
            vga_update_cached_state(s);
        break;
    case 0x3b4:
    case 0x3d4:
        s->cr_index = val;
        break;
    case 0x3b5:
    case 0x3d5:
#ifdef DEBUG_VGA_REG
        printf("vga: write CR%x = 0x%02x\n", s->cr_index, val);
#endif
        /* handle CR0-7 protection */
        if ((s->cr[0x11] & 0x80) && s->cr_index <= 7) {
            printf("VGA: CRTC write CR%02x=%02x BLOCKED by CR11 protection (CR11=%02x)\n",
                   s->cr_index, val, s->cr[0x11]);
            /* can always write bit 4 of CR7 */
            if (s->cr_index == 7)
                s->cr[7] = (s->cr[7] & ~0x10) | (val & 0x10);
            return;
        }
        s->cr[s->cr_index] = val;
        break;
    case 0x3ba:
    case 0x3da:
        s->fcr = val & 0x10;
        break;
    }
}

#define VGA_IO(base) \
static uint32_t vga_read_ ## base(void *opaque, uint32_t addr, int size_log2)\
{\
    return vga_ioport_read(opaque, base + addr);\
}\
static void vga_write_ ## base(void *opaque, uint32_t addr, uint32_t val, int size_log2)\
{\
    return vga_ioport_write(opaque, base + addr, val);\
}

void vbe_write(VGAState *s, uint32_t offset, uint32_t val)
{
    if (offset == 0) {
        s->vbe_index = val;
    } else {
#ifdef DEBUG_VBE
        printf("VBE write: index=0x%04x val=0x%04x\n", s->vbe_index, val);
#endif
        switch(s->vbe_index) {
        case VBE_DISPI_INDEX_ID:
            if (val >= VBE_DISPI_ID0 && val <= VBE_DISPI_ID5)
                s->vbe_regs[s->vbe_index] = val;
            break;
        case VBE_DISPI_INDEX_ENABLE:
            if ((val & VBE_DISPI_ENABLED) &&
                !(s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED)) {
                s->vbe_regs[VBE_DISPI_INDEX_VIRT_WIDTH] =
                    s->vbe_regs[VBE_DISPI_INDEX_XRES];
                s->vbe_regs[VBE_DISPI_INDEX_VIRT_HEIGHT] =
                    s->vbe_regs[VBE_DISPI_INDEX_YRES];
                s->vbe_regs[VBE_DISPI_INDEX_X_OFFSET] = 0;
                s->vbe_regs[VBE_DISPI_INDEX_Y_OFFSET] = 0;
            } else {
                s->bank_offset = 0;
            }
            s->dac_8bit = (val & VBE_DISPI_8BIT_DAC) > 0;
            s->vbe_regs[s->vbe_index] = val;
            vbe_fixup_regs(s);
            vbe_update_vgaregs(s);
            /* clear the screen */
            if (!(val & VBE_DISPI_NOCLEARMEM)) {
                uint32_t clear_size = s->vbe_regs[VBE_DISPI_INDEX_YRES] * s->vbe_line_offset;
                memset(s->vga_ram, 0, clear_size);
                /* Mark all affected pages dirty for double buffering */
                uint32_t end_page = (clear_size - 1) >> 12;
                for (uint32_t p = 0; p <= end_page && p < 64; p++)
                    s->dirty_pages |= (1ULL << p);
            }
            break;
        case VBE_DISPI_INDEX_XRES:
        case VBE_DISPI_INDEX_YRES:
        case VBE_DISPI_INDEX_BPP:
        case VBE_DISPI_INDEX_VIRT_WIDTH:
        case VBE_DISPI_INDEX_VIRT_HEIGHT:
        case VBE_DISPI_INDEX_X_OFFSET:
        case VBE_DISPI_INDEX_Y_OFFSET:
            s->vbe_regs[s->vbe_index] = val;
            vbe_fixup_regs(s);
            vbe_update_vgaregs(s);
            break;
        case VBE_DISPI_INDEX_BANK:
            val &= (s->vga_ram_size >> 16) - 1;
            s->vbe_regs[s->vbe_index] = val;
            s->bank_offset = (val << 16);
            break;
        }
    }
}

uint32_t vbe_read(VGAState *s, uint32_t offset)
{
    uint32_t val;

    if (offset == 0) {
        val = s->vbe_index;
    } else {
        if (s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
            switch(s->vbe_index) {
            case VBE_DISPI_INDEX_XRES:
#ifdef SCALE_3_2
                val = s->fb_dev->width * 3 / 2;
#else
                val = s->fb_dev->width;
#endif
                break;
            case VBE_DISPI_INDEX_YRES:
#ifdef SCALE_3_2
                val = s->fb_dev->height * 3 / 2;
#else
                val = s->fb_dev->height;
#endif
                break;
            case VBE_DISPI_INDEX_BPP:
                val = 32;
                break;
            default:
                goto read_reg;
            }
        } else {
        read_reg:
            if (s->vbe_index < VBE_DISPI_INDEX_NB)
                val = s->vbe_regs[s->vbe_index];
            else
                val = 0;
        }
#ifdef DEBUG_VBE
        printf("VBE read: index=0x%04x val=0x%04x\n", s->vbe_index, val);
#endif
    }
    return val;
}

#define cbswap_32(__x) \
((uint32_t)( \
                (((uint32_t)(__x) & (uint32_t)0x000000ffUL) << 24) | \
                (((uint32_t)(__x) & (uint32_t)0x0000ff00UL) <<  8) | \
                (((uint32_t)(__x) & (uint32_t)0x00ff0000UL) >>  8) | \
                (((uint32_t)(__x) & (uint32_t)0xff000000UL) >> 24) ))

#ifdef HOST_WORDS_BIGENDIAN
#define PAT(x) cbswap_32(x)
#else
#define PAT(x) (x)
#endif

#ifdef HOST_WORDS_BIGENDIAN
#define GET_PLANE(data, p) (((data) >> (24 - (p) * 8)) & 0xff)
#else
#define GET_PLANE(data, p) (((data) >> ((p) * 8)) & 0xff)
#endif

static const uint32_t mask16[16] = {
    PAT(0x00000000),
    PAT(0x000000ff),
    PAT(0x0000ff00),
    PAT(0x0000ffff),
    PAT(0x00ff0000),
    PAT(0x00ff00ff),
    PAT(0x00ffff00),
    PAT(0x00ffffff),
    PAT(0xff000000),
    PAT(0xff0000ff),
    PAT(0xff00ff00),
    PAT(0xff00ffff),
    PAT(0xffff0000),
    PAT(0xffff00ff),
    PAT(0xffffff00),
    PAT(0xffffffff),
};

#define VGA_SEQ_RESET           0x00
#define VGA_SEQ_CLOCK_MODE      0x01
#define VGA_SEQ_PLANE_WRITE     0x02
#define VGA_SEQ_CHARACTER_MAP   0x03
#define VGA_SEQ_MEMORY_MODE     0x04

#define VGA_SR01_CHAR_CLK_8DOTS 0x01 /* bit 0: character clocks 8 dots wide are generated */
#define VGA_SR01_SCREEN_OFF     0x20 /* bit 5: Screen is off */
#define VGA_SR02_ALL_PLANES     0x0F /* bits 3-0: enable access to all planes */
#define VGA_SR04_EXT_MEM        0x02 /* bit 1: allows complete mem access to 256K */
#define VGA_SR04_SEQ_MODE       0x04 /* bit 2: directs system to use a sequential addressing mode */
#define VGA_SR04_CHN_4M         0x08 /* bit 3: selects modulo 4 addressing for CPU access to display memory */

#define VGA_GFX_SR_VALUE        0x00
#define VGA_GFX_SR_ENABLE       0x01
#define VGA_GFX_COMPARE_VALUE   0x02
#define VGA_GFX_DATA_ROTATE     0x03
#define VGA_GFX_PLANE_READ      0x04
#define VGA_GFX_MODE            0x05
#define VGA_GFX_MISC            0x06
#define VGA_GFX_COMPARE_MASK    0x07
#define VGA_GFX_BIT_MASK        0x08

//#define DEBUG_VGA_MEM
//#define TARGET_FMT_plx "%x"
void IRAM_ATTR vga_mem_write16(VGAState *s, uint32_t addr, uint16_t val16)
{
    if (!(s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M)) {
        vga_mem_write(s, addr, val16);
        vga_mem_write(s, addr + 1, val16 >> 8);
        return;
    }
    uint32_t val = val16;

    int memory_map_mode, plane, mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif
    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return;
        addr += s->bank_offset;
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return;
        break;
    }

    /* chain 4 mode : simplest access */
    plane = addr & 3;
    mask = (1 << plane);
    if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
        /* Mark 4KB page dirty for double buffering (addr is byte offset) */
        s->dirty_pages |= (1ULL << (addr >> 12));
        * (uint16_t *) &(s->vga_ram[addr]) = val;
    }
}

void IRAM_ATTR vga_mem_write32(VGAState *s, uint32_t addr, uint32_t val)
{
    if (!(s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M)) {
        vga_mem_write(s, addr, val);
        vga_mem_write(s, addr + 1, val >> 8);
        vga_mem_write(s, addr + 2, val >> 16);
        vga_mem_write(s, addr + 3, val >> 24);
        return;
    }

    int memory_map_mode, plane, mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif
    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return;
        addr += s->bank_offset;
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return;
        break;
    }

    /* chain 4 mode : simplest access */
    plane = addr & 3;
    mask = (1 << plane);
    if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
        /* Mark 4KB page dirty for double buffering (addr is byte offset) */
        s->dirty_pages |= (1ULL << (addr >> 12));
        * (uint32_t *) &(s->vga_ram[addr]) = val;
    }
}

bool IRAM_ATTR vga_mem_write_string(VGAState *s, uint32_t addr, uint8_t *buf, int len)
{
    int memory_map_mode, plane, mask;

    /* Ultra-fast Mode X path - write mode 0 with single plane */
    if (s->cached_simple_write == 2 && s->cached_memmap_mode == 1) {
        uint8_t pmask = s->cached_plane_mask;
        addr &= 0xffff;
        addr += s->bank_offset;

        /* Only handle single-plane writes for string operations */
        if ((pmask & (pmask - 1)) == 0 && pmask != 0) {
            int target_plane = __builtin_ctz(pmask);

            /* Bounds check */
            if ((addr + len) * 4 > s->vga_ram_size)
                return false;

            /* Mark dirty pages for double buffering (addr is dword offset) */
            uint32_t start_page = addr >> 10;
            uint32_t end_page = (addr + len - 1) >> 10;
            for (uint32_t p = start_page; p <= end_page && p < 64; p++)
                s->dirty_pages |= (1ULL << p);

            /* Write each byte to the selected plane */
            uint8_t *dst = s->vga_ram + addr * 4 + target_plane;
            for (int i = 0; i < len; i++) {
                *dst = buf[i];
                dst += 4;
            }
            return true;
        }
    }

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return false;
        addr += s->bank_offset;
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return false;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return false;
        break;
    }

    if (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
        /* chain 4 mode : simplest access */
        plane = addr & 3;
        mask = (1 << plane);
        if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
            /* Mark dirty pages for double buffering (addr is byte offset) */
            uint32_t start_page = addr >> 12;
            uint32_t end_page = (addr + len - 1) >> 12;
            for (uint32_t p = start_page; p <= end_page && p < 64; p++)
                s->dirty_pages |= (1ULL << p);
            memcpy(s->vga_ram + addr, buf, len);
            return true;
        }
        return false;
    }

    /* Mode X fast path: write mode 0 with single plane enabled */
    if (s->cached_simple_write == 2) {
        uint8_t pmask = s->cached_plane_mask;

        /* Only handle single-plane writes for string operations */
        if ((pmask & (pmask - 1)) == 0 && pmask != 0) {
            int target_plane = __builtin_ctz(pmask);

            /* Bounds check */
            if ((addr + len) * 4 > s->vga_ram_size)
                return false;

            /* Mark dirty pages for double buffering (addr is dword offset) */
            uint32_t start_page = addr >> 10;
            uint32_t end_page = (addr + len - 1) >> 10;
            for (uint32_t p = start_page; p <= end_page && p < 64; p++)
                s->dirty_pages |= (1ULL << p);

            /* Write each byte to the selected plane */
            uint8_t *dst = s->vga_ram + addr * 4 + target_plane;
            for (int i = 0; i < len; i++) {
                *dst = buf[i];
                dst += 4;  /* Skip to next dword (same plane, next offset) */
            }
            return true;
        }
    }

    return false;
}

void IRAM_ATTR vga_mem_write(VGAState *s, uint32_t addr, uint8_t val8)
{
    uint32_t val = val8;

    int memory_map_mode, plane, write_mode, b, func_select, mask;
    uint32_t write_mask, bit_mask, set_mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif

    /* Ultra-fast Mode X path - skip all mode checks when cached state is valid */
    uint8_t fast = s->cached_simple_write;
    if (fast && s->cached_memmap_mode == 1) {
        addr &= 0xffff;  /* 64KB window */
        addr += s->bank_offset;
        if (fast == 1) {
            /* Write mode 1: latch copy */
            if (addr * 4 >= s->vga_ram_size)
                return;
            /* Mark 4KB page dirty for double buffering (addr is dword offset) */
            s->dirty_pages |= (1ULL << (addr >> 10));
            uint8_t pmask = s->cached_plane_mask;
            if (pmask == 0x0f) {
                ((uint32_t *)s->vga_ram)[addr] = s->latch;
            } else {
                uint32_t wmask = mask16[pmask];
                uint32_t *p = &((uint32_t *)s->vga_ram)[addr];
                *p = (*p & ~wmask) | (s->latch & wmask);
            }
            return;
        } else {
            /* Write mode 0: direct write all planes (fast == 2) */
            if (addr * 4 + 3 >= s->vga_ram_size)
                return;
            /* Mark 4KB page dirty for double buffering (addr is dword offset) */
            s->dirty_pages |= (1ULL << (addr >> 10));
            uint32_t val32 = val | (val << 8) | (val << 16) | (val << 24);
            uint8_t pmask = s->cached_plane_mask;
            if (pmask == 0x0f) {
                ((uint32_t *)s->vga_ram)[addr] = val32;
            } else {
                uint32_t wmask = mask16[pmask];
                uint32_t *p = &((uint32_t *)s->vga_ram)[addr];
                *p = (*p & ~wmask) | (val32 & wmask);
            }
            return;
        }
    }

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return;
        addr += s->bank_offset;
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return;
        break;
    }

    if (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
        /* chain 4 mode : simplest access */
        plane = addr & 3;
        mask = (1 << plane);
        if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
            /* Mark 4KB page dirty for double buffering */
            s->dirty_pages |= (1ULL << (addr >> 12));
            s->vga_ram[addr] = val;
#ifdef DEBUG_VGA_MEM
            printf("vga: chain4: [0x" TARGET_FMT_plx "]\n", addr);
#endif
//            s->plane_updated |= mask; /* only used to detect font change */
//            memory_region_set_dirty(&s->vram, addr, 1);
        }
    } else if (s->gr[VGA_GFX_MODE] & 0x10) {
        /* odd/even mode (aka text mode mapping) */
        /* Clip to 32KB: text/CGA modes never exceed this. Prevents VRAM
         * corruption when Windows VMM fakes Mode 3 with memory_map_mode=1
         * (64KB window) instead of mode 3 (32KB window). */
        if (addr >= 0x8000)
            return;
        plane = (s->gr[VGA_GFX_PLANE_READ] & 2) | (addr & 1);
        mask = (1 << plane);
        if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
            addr = ((addr & ~1) << 1) | plane;
            if (addr >= s->vga_ram_size) {
                return;
            }
            /* Mark 4KB page dirty for double buffering */
            s->dirty_pages |= (1ULL << (addr >> 12));
            s->vga_ram[addr] = val;
#ifdef DEBUG_VGA_MEM
            printf("vga: odd/even: [0x" TARGET_FMT_plx "]\n", addr);
#endif
//            s->plane_updated |= mask; /* only used to detect font change */
//            memory_region_set_dirty(&s->vram, addr, 1);
        }
    } else {
        /* standard VGA latched access (Mode X / planar) */
        uint8_t fast = s->cached_simple_write;

        /* Fast path for write mode 1 (latch copy) - ignores bit mask */
        if (fast == 1) {
            uint8_t pmask = s->cached_plane_mask;

            /* Bounds check: addr is the dword offset into vga_ram */
            if (addr * 4 >= s->vga_ram_size)
                return;

            /* Mark 4KB page dirty for double buffering (addr is dword offset) */
            s->dirty_pages |= (1ULL << (addr >> 10));

            {
                uint32_t latch_val = s->latch;
                if (pmask == 0x0f) {
                    ((uint32_t *)s->vga_ram)[addr] = latch_val;
                } else {
                    uint32_t wmask = mask16[pmask];
                    ((uint32_t *)s->vga_ram)[addr] =
                        (((uint32_t *)s->vga_ram)[addr] & ~wmask) |
                        (latch_val & wmask);
                }
            }
            return;
        }

        /* Fast path for write mode 0 (simple byte write, no special features) */
        if (fast == 2) {
            uint8_t pmask = s->cached_plane_mask;

            /* Bounds check: addr is the dword offset into vga_ram */
            if (addr * 4 >= s->vga_ram_size)
                return;

            /* Mark 4KB page dirty for double buffering (addr is dword offset) */
            s->dirty_pages |= (1ULL << (addr >> 10));

            /* Replicate byte to 32 bits */
            uint32_t val32 = val8 | (val8 << 8);
            val32 |= val32 << 16;

            if (pmask == 0x0f) {
                /* All 4 planes - direct 32-bit write */
                ((uint32_t *)s->vga_ram)[addr] = val32;
            } else if ((pmask & (pmask - 1)) == 0 && pmask != 0) {
                /* Single plane enabled (power of 2): direct byte write
                 * This is the most common Mode X case for pixel drawing */
                int plane = __builtin_ctz(pmask);  /* Count trailing zeros */
                s->vga_ram[addr * 4 + plane] = val8;
            } else {
                /* Multiple (but not all) planes */
                uint32_t wmask = mask16[pmask];
                ((uint32_t *)s->vga_ram)[addr] =
                    (((uint32_t *)s->vga_ram)[addr] & ~wmask) |
                    (val32 & wmask);
            }
            return;
        }

        /* Slow path counter */

        /* Slow path: full VGA latched write logic */
        write_mode = s->gr[VGA_GFX_MODE] & 3;
        switch(write_mode) {
        default:
        case 0:
            /* rotate */
            b = s->gr[VGA_GFX_DATA_ROTATE] & 7;
            val = ((val >> b) | (val << (8 - b))) & 0xff;
            val |= val << 8;
            val |= val << 16;

            /* apply set/reset mask */
            set_mask = mask16[s->gr[VGA_GFX_SR_ENABLE]];
            val = (val & ~set_mask) |
                (mask16[s->gr[VGA_GFX_SR_VALUE]] & set_mask);
            bit_mask = s->gr[VGA_GFX_BIT_MASK];
            break;
        case 1:
            val = s->latch;
            goto do_write;
        case 2:
            val = mask16[val & 0x0f];
            bit_mask = s->gr[VGA_GFX_BIT_MASK];
            break;
        case 3:
            /* rotate */
            b = s->gr[VGA_GFX_DATA_ROTATE] & 7;
            val = (val >> b) | (val << (8 - b));

            bit_mask = s->gr[VGA_GFX_BIT_MASK] & val;
            val = mask16[s->gr[VGA_GFX_SR_VALUE]];
            break;
        }

        /* apply logical operation */
        func_select = s->gr[VGA_GFX_DATA_ROTATE] >> 3;
        switch(func_select) {
        case 0:
        default:
            /* nothing to do */
            break;
        case 1:
            /* and */
            val &= s->latch;
            break;
        case 2:
            /* or */
            val |= s->latch;
            break;
        case 3:
            /* xor */
            val ^= s->latch;
            break;
        }

        /* apply bit mask */
        bit_mask |= bit_mask << 8;
        bit_mask |= bit_mask << 16;
        val = (val & bit_mask) | (s->latch & ~bit_mask);

    do_write:
        /* mask data according to sr[2] */
        mask = s->sr[VGA_SEQ_PLANE_WRITE];
//        s->plane_updated |= mask; /* only used to detect font change */
        write_mask = mask16[mask];
        if (addr * sizeof(uint32_t) >= s->vga_ram_size) {
            return;
        }
        /* Mark 4KB page dirty for double buffering (addr is dword offset) */
        s->dirty_pages |= (1ULL << (addr >> 10));
        ((uint32_t *)s->vga_ram)[addr] =
            (((uint32_t *)s->vga_ram)[addr] & ~write_mask) |
            (val & write_mask);
#ifdef DEBUG_VGA_MEM
        printf("vga: latch: [0x" TARGET_FMT_plx "] mask=0x%08x val=0x%08x\n",
               addr * 4, write_mask, val);
#endif
//        memory_region_set_dirty(&s->vram, addr << 2, sizeof(uint32_t));
    }
}

uint8_t IRAM_ATTR vga_mem_read(VGAState *s, uint32_t addr)
{
    int memory_map_mode, plane;
    uint32_t ret;

    /* Fast path for Mode X reads (read mode 0, unchained planar)
     * Conditions: not chain4, not odd/even, memory map mode 0 or 1, read mode 0 */
    if (!s->cached_chain4 &&
        !(s->gr[VGA_GFX_MODE] & 0x10) &&  /* not odd/even */
        !(s->gr[VGA_GFX_MODE] & 0x08) &&  /* read mode 0 */
        s->cached_memmap_mode <= 1) {

        addr &= 0x1ffff;
        if (s->cached_memmap_mode == 1) {
            if (addr >= 0x10000)
                return 0xff;
            addr += s->bank_offset;
        }

        /* Bounds check */
        if (addr * 4 >= s->vga_ram_size)
            return 0xff;

        /* Load latch and return selected plane */
        s->latch = ((uint32_t *)s->vga_ram)[addr];
        plane = s->gr[VGA_GFX_PLANE_READ];
        return GET_PLANE(s->latch, plane);
    }

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return 0xff;
        addr += s->bank_offset;
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return 0xff;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return 0xff;
        break;
    }

    if (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
        /* chain 4 mode : simplest access */
//        assert(addr < s->vram_size);
        ret = s->vga_ram[addr];
    } else if (s->gr[VGA_GFX_MODE] & 0x10) {
        /* odd/even mode (aka text mode mapping) */
        if (addr >= 0x8000)
            return 0xff;
        plane = (s->gr[VGA_GFX_PLANE_READ] & 2) | (addr & 1);
        addr = ((addr & ~1) << 1) | plane;
        if (addr >= s->vga_ram_size) {
            return 0xff;
        }
        ret = s->vga_ram[addr];
    } else {
        /* standard VGA latched access */
        if (addr * sizeof(uint32_t) >= s->vga_ram_size) {//s->vram_size) {
            return 0xff;
        }
        s->latch = ((uint32_t *)s->vga_ram)[addr];

        if (!(s->gr[VGA_GFX_MODE] & 0x08)) {
            /* read mode 0 */
            plane = s->gr[VGA_GFX_PLANE_READ];
            ret = GET_PLANE(s->latch, plane);
        } else {
            /* read mode 1 */
            ret = (s->latch ^ mask16[s->gr[VGA_GFX_COMPARE_VALUE]]) &
                mask16[s->gr[VGA_GFX_COMPARE_MASK]];
            ret |= ret >> 16;
            ret |= ret >> 8;
            ret = (~ret) & 0xff;
        }
    }
    return ret;
}

uint16_t IRAM_ATTR vga_mem_read16(VGAState *s, uint32_t addr)
{
    int memory_map_mode;

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;

    /* Fast path for chain-4 mode (mode 13h) with memory map mode 1 */
    if ((s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) && memory_map_mode == 1) {
        if (addr >= 0x10000 - 1)
            return vga_mem_read(s, addr) | ((uint16_t)vga_mem_read(s, addr + 1) << 8);
        addr += s->bank_offset;
        if (addr + 1 < s->vga_ram_size)
            return *(uint16_t *)(s->vga_ram + addr);
    }

    /* Fall back to byte-by-byte for other modes */
    return vga_mem_read(s, addr) | ((uint16_t)vga_mem_read(s, addr + 1) << 8);
}

uint32_t IRAM_ATTR vga_mem_read32(VGAState *s, uint32_t addr)
{
    int memory_map_mode;

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;

    /* Fast path for chain-4 mode (mode 13h) with memory map mode 1 */
    if ((s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) && memory_map_mode == 1) {
        if (addr >= 0x10000 - 3)
            return vga_mem_read16(s, addr) | ((uint32_t)vga_mem_read16(s, addr + 2) << 16);
        addr += s->bank_offset;
        if (addr + 3 < s->vga_ram_size)
            return *(uint32_t *)(s->vga_ram + addr);
    }

    /* Fall back to 16-bit reads for other modes */
    return vga_mem_read16(s, addr) | ((uint32_t)vga_mem_read16(s, addr + 2) << 16);
}

/* Bulk read from VGA memory - returns true if fast path used */
bool IRAM_ATTR vga_mem_read_string(VGAState *s, uint32_t addr, uint8_t *buf, int len)
{
    int memory_map_mode;

    if (len <= 0)
        return true;

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;

    /* Fast path for chain-4 mode (mode 13h) with memory map mode 1 */
    if ((s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) && memory_map_mode == 1) {
        if (addr + len > 0x10000)
            return false;  /* would cross boundary */
        uint32_t phys_addr = addr + s->bank_offset;
        if (phys_addr + len <= s->vga_ram_size) {
            memcpy(buf, s->vga_ram + phys_addr, len);
                return true;
        }
    }

    /* Mode X fast path: read mode 0, unchained planar */
    if (!s->cached_chain4 &&
        !(s->gr[VGA_GFX_MODE] & 0x10) &&  /* not odd/even */
        !(s->gr[VGA_GFX_MODE] & 0x08) &&  /* read mode 0 */
        s->cached_memmap_mode <= 1) {

        if (s->cached_memmap_mode == 1) {
            if (addr + len > 0x10000)
                return false;
            addr += s->bank_offset;
        }

        /* Bounds check */
        if ((addr + len) * 4 > s->vga_ram_size)
            return false;

        int plane = s->gr[VGA_GFX_PLANE_READ];
        uint32_t *vram32 = (uint32_t *)s->vga_ram;

        /* Read each byte from the selected plane, loading latch each time */
        for (int i = 0; i < len; i++) {
            s->latch = vram32[addr + i];
            buf[i] = GET_PLANE(s->latch, plane);
        }
        return true;
    }

    return false;  /* Use slow path */
}

/* Returns direct VGA RAM pointer if chain-4 mode is active, NULL otherwise.
 * This allows bypassing callbacks for mode 13h memory access. */
uint8_t *vga_get_direct_ptr(VGAState *s)
{
    int memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;

    /* Only enable direct access for chain-4 mode with memory map mode 1
     * (standard 64KB at A0000, as used by mode 13h) and bank offset 0 */
    if ((s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) &&
        memory_map_mode == 1 &&
        s->bank_offset == 0) {
        return (uint8_t *)s->vga_ram;
    }
    return NULL;
}

/* Returns text mode state for CPU inline fast path + HLE.
 * Sets *ram to VGA RAM pointer when in standard text mode, NULL otherwise. */
void vga_get_text_state(VGAState *s, uint8_t **ram, uint32_t *base,
                        uint32_t *end, uint64_t **dirty)
{
    int memmap = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    int is_odd_even = (s->gr[VGA_GFX_MODE] & 0x10) != 0;
    int is_chain4 = (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) != 0;
    /* Standard text mode: odd/even mapping, not chain-4,
     * memory map mode 2 (B0000) or 3 (B8000),
     * normal plane selection (planes 0/1, not font planes 2/3) */
    if (is_odd_even && !is_chain4 && (memmap == 2 || memmap == 3) &&
        !(s->gr[VGA_GFX_PLANE_READ] & 2)) {
        *ram = s->vga_ram;
        *base = (memmap == 3) ? 0xB8000 : 0xB0000;
        *end = *base + 0x8000;
        *dirty = &s->dirty_pages;
    } else {
        *ram = NULL;
        *base = *end = 0;
        *dirty = NULL;
    }
}

/* Set CRTC cursor position registers (called from HLE) */
void vga_set_cursor_pos(VGAState *s, uint16_t addr)
{
    s->cr[0x0E] = addr >> 8;
    s->cr[0x0F] = addr & 0xFF;
}

/* Set CRTC cursor shape registers (called from HLE) */
void vga_set_cursor_shape(VGAState *s, uint8_t start, uint8_t end)
{
    s->cr[0x0A] = start & 0x3F;
    s->cr[0x0B] = end & 0x1F;
}

/* Debug: report why direct mode is disabled */
void vga_debug_direct_conditions(VGAState *s)
{
    int memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    int chain4 = (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) ? 1 : 0;
    fprintf(stderr, "VGA direct mode DISABLED: chain4=%d memmap=%d bank=%d\n",
            chain4, memory_map_mode, s->bank_offset);
}

/* Bulk VGA-to-VGA copy for Mode X latch operations (scrolling/blitting)
 * Performs: for each byte: read src (loads latch), write dst (copies latch) */
bool IRAM_ATTR vga_mem_copy_string(VGAState *s, uint32_t dst, uint32_t src, int len)
{
    /* Only handle Mode X with write mode 1 (latch copy) */
    if (s->cached_simple_write != 1)
        return false;

    int memory_map_mode = s->cached_memmap_mode;
    if (memory_map_mode > 1)
        return false;

    /* Adjust addresses */
    dst &= 0x1ffff;
    src &= 0x1ffff;
    if (memory_map_mode == 1) {
        if (dst >= 0x10000 || src >= 0x10000)
            return false;
        dst += s->bank_offset;
        src += s->bank_offset;
    }

    /* Bounds check */
    if ((dst + len) * 4 > s->vga_ram_size || (src + len) * 4 > s->vga_ram_size)
        return false;

    /* Mark dirty pages for double buffering (dst is dword offset) */
    uint32_t start_page = dst >> 10;
    uint32_t end_page = (dst + len - 1) >> 10;
    for (uint32_t p = start_page; p <= end_page && p < 64; p++)
        s->dirty_pages |= (1ULL << p);

    uint8_t pmask = s->cached_plane_mask;
    uint32_t *vram32 = (uint32_t *)s->vga_ram;

    if (pmask == 0x0f) {
        /* All 4 planes - bulk 32-bit copy */
        for (int i = 0; i < len; i++) {
            vram32[dst + i] = vram32[src + i];
        }
    } else {
        /* Partial planes - masked copy */
        uint32_t wmask = ((pmask & 1) ? 0x000000ff : 0) |
                         ((pmask & 2) ? 0x0000ff00 : 0) |
                         ((pmask & 4) ? 0x00ff0000 : 0) |
                         ((pmask & 8) ? 0xff000000 : 0);
        for (int i = 0; i < len; i++) {
            uint32_t latch = vram32[src + i];
            vram32[dst + i] = (vram32[dst + i] & ~wmask) | (latch & wmask);
        }
    }

    /* Update latch to final value (for subsequent operations) */
    s->latch = vram32[src + len - 1];

    return true;
}

/* Get Mode X state for CPU inline fast path
 * Returns NULL for vga_modex_ram if Mode X inline is not possible */
void vga_get_modex_state(VGAState *s, uint8_t **ram, uint32_t *ram_size,
                         uint8_t *write_mode, uint8_t *plane_mask,
                         uint8_t *read_plane, uint32_t **latch)
{
    /* DISABLED - inline Mode X causes graphics corruption.
     * The VGA-to-VGA copy (iomem_copy_string) still works. */
    (void)s;
    *ram = NULL;
    *ram_size = 0;
    *write_mode = 0xFF;
    *plane_mask = 0;
    *read_plane = 0;
    *latch = NULL;
}

/* Mark VGA lines dirty after direct write (addr is VGA-relative, 0-0xFFFF) */
void vga_direct_mark_dirty(VGAState *s, uint32_t addr, int len)
{
    /* Mark 4KB pages dirty for double buffering (addr is byte offset) */
    uint32_t start_page = addr >> 12;
    uint32_t end_page = (addr + len - 1) >> 12;
    for (uint32_t p = start_page; p <= end_page && p < 64; p++)
        s->dirty_pages |= (1ULL << p);

    /* For mode 13h: 320 pixels per line, addr / 320 = line number */
    int line_start = addr / 320;
    int line_end = (addr + len - 1) / 320;

    /* Clamp to valid range */
    if (line_start < 0) line_start = 0;
    if (line_end >= VGA_MAX_LINES) line_end = VGA_MAX_LINES - 1;

    /* Mark lines dirty using existing mechanism */
    for (int y = line_start; y <= line_end; y++) {
        vga_mark_line_dirty(y);
    }
}

static void vga_initmode(VGAState *s);

VGAState *vga_init(char *vga_ram, int vga_ram_size,
                   uint8_t *fb, int width, int height)
{
    VGAState *s;

    /* Initialize BitScrambler for hardware-accelerated planar rendering */
    vga_bitscrambler_init();

    s = pcmalloc(sizeof(*s));
    memset(s, 0, sizeof(*s));
    FBDevice *fb_dev = pcmalloc(sizeof(FBDevice));
    s->fb_dev = fb_dev;
    memset(s->fb_dev, 0, sizeof(FBDevice));
    s->graphic_mode = 0;
    s->cursor_blink_time = get_uticks();
    s->cursor_visible_phase = 1;
    s->retrace_time = get_uticks();
    s->retrace_phase = 0;
    s->palette_switch_scanline = -1;  /* No mid-frame switch initially */
    s->hblank_poll_count = 0;
    fb_dev->width = width;
    fb_dev->height = height;
#ifdef SWAPXY
    fb_dev->stride = height * (BPP / 8);
#else
    fb_dev->stride = width * (BPP / 8);
#endif
    fb_dev->fb_data = fb;

    s->vga_ram = (uint8_t *) vga_ram;
    s->vga_ram_size = vga_ram_size;

    s->vbe_regs[VBE_DISPI_INDEX_ID] = VBE_DISPI_ID5;
    s->vbe_regs[VBE_DISPI_INDEX_VIDEO_MEMORY_64K] = s->vga_ram_size >> 16;

#ifdef TEXT_RENDER_OPT
    /* Try internal RAM first for font cache (16KB) - accessed every char */
    s->font_cache = heap_caps_malloc(256 * 16 * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s->font_cache) s->font_cache = psram_malloc(256 * 16 * 4);

    /* Glyph tile cache (128KB) - try internal RAM, fall back to PSRAM */
    s->glyph_cache_tiles = heap_caps_malloc(GLYPH_CACHE_SIZE * GLYPH_TILE_SIZE,
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s->glyph_cache_tiles)
        s->glyph_cache_tiles = psram_malloc(GLYPH_CACHE_SIZE * GLYPH_TILE_SIZE);
    if (s->glyph_cache_tiles) {
        memset(s->glyph_cache_tiles, 0, GLYPH_CACHE_SIZE * GLYPH_TILE_SIZE);
        memset(s->glyph_cache_keys, 0xff, sizeof(s->glyph_cache_keys));  /* Invalid keys */
    }
    s->glyph_cache_counter = 0;
    s->font_cache_base = 0xffffffff;
#endif

    vga_initmode(s);

    /* Initialize cached state for Mode X fast path */
    vga_update_cached_state(s);
    vga_trace_state = s;

    return s;
}

void vga_set_force_8dm(VGAState *s, int v)
{
    s->force_8dm = v;
}

PCIDevice *vga_pci_init(VGAState *s, PCIBus *bus,
                        void *o, void (*set_bar)(void *, int, uint32_t, bool))
{
    PCIDevice *d;
    d = pci_register_device(bus, "VGA", -1, 0x1234, 0x1111, 0x00, 0x0300);

    uint32_t bar_size;
    bar_size = 1;
    while (bar_size < s->vga_ram_size)
        bar_size <<= 1;
    pci_register_bar(d, 0, bar_size, PCI_ADDRESS_SPACE_MEM, o, set_bar);
    return d;
}

// from vgabios
// stdvga mode 2
const static uint8_t pal_ega[] = {
    0x00,0x00,0x00, 0x00,0x00,0x2a, 0x00,0x2a,0x00, 0x00,0x2a,0x2a,
    0x2a,0x00,0x00, 0x2a,0x00,0x2a, 0x2a,0x2a,0x00, 0x2a,0x2a,0x2a,
    0x00,0x00,0x15, 0x00,0x00,0x3f, 0x00,0x2a,0x15, 0x00,0x2a,0x3f,
    0x2a,0x00,0x15, 0x2a,0x00,0x3f, 0x2a,0x2a,0x15, 0x2a,0x2a,0x3f,
    0x00,0x15,0x00, 0x00,0x15,0x2a, 0x00,0x3f,0x00, 0x00,0x3f,0x2a,
    0x2a,0x15,0x00, 0x2a,0x15,0x2a, 0x2a,0x3f,0x00, 0x2a,0x3f,0x2a,
    0x00,0x15,0x15, 0x00,0x15,0x3f, 0x00,0x3f,0x15, 0x00,0x3f,0x3f,
    0x2a,0x15,0x15, 0x2a,0x15,0x3f, 0x2a,0x3f,0x15, 0x2a,0x3f,0x3f,
    0x15,0x00,0x00, 0x15,0x00,0x2a, 0x15,0x2a,0x00, 0x15,0x2a,0x2a,
    0x3f,0x00,0x00, 0x3f,0x00,0x2a, 0x3f,0x2a,0x00, 0x3f,0x2a,0x2a,
    0x15,0x00,0x15, 0x15,0x00,0x3f, 0x15,0x2a,0x15, 0x15,0x2a,0x3f,
    0x3f,0x00,0x15, 0x3f,0x00,0x3f, 0x3f,0x2a,0x15, 0x3f,0x2a,0x3f,
    0x15,0x15,0x00, 0x15,0x15,0x2a, 0x15,0x3f,0x00, 0x15,0x3f,0x2a,
    0x3f,0x15,0x00, 0x3f,0x15,0x2a, 0x3f,0x3f,0x00, 0x3f,0x3f,0x2a,
    0x15,0x15,0x15, 0x15,0x15,0x3f, 0x15,0x3f,0x15, 0x15,0x3f,0x3f,
    0x3f,0x15,0x15, 0x3f,0x15,0x3f, 0x3f,0x3f,0x15, 0x3f,0x3f,0x3f
};

const static uint8_t actl[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07,
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x0c, 0x00, 0x0f, 0x08 };

const static uint8_t sequ[] = { 0x00, 0x03, 0x00, 0x02 };

const static uint8_t grdc[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x0f, 0xff };

const static uint8_t crtc[] = {
    0x5f, 0x4f, 0x50, 0x82, 0x55, 0x81, 0xbf, 0x1f,
    0x00, 0x4f, 0x0d, 0x0e, 0x00, 0x00, 0x00, 0x00,
    0x9c, 0x8e, 0x8f, 0x28, 0x1f, 0x96, 0xb9, 0xa3,
    0xff };

const static uint8_t vgafont16[256 * 16] = {
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7e, 0x81, 0xa5, 0x81, 0x81, 0xbd, 0x99, 0x81, 0x81, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7e, 0xff, 0xdb, 0xff, 0xff, 0xc3, 0xe7, 0xff, 0xff, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x6c, 0xfe, 0xfe, 0xfe, 0xfe, 0x7c, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x18, 0x3c, 0x3c, 0xe7, 0xe7, 0xe7, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x18, 0x3c, 0x7e, 0xff, 0xff, 0x7e, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3c, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xc3, 0xc3, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x66, 0x42, 0x42, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0x99, 0xbd, 0xbd, 0x99, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x00, 0x00, 0x1e, 0x0e, 0x1a, 0x32, 0x78, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3f, 0x33, 0x3f, 0x30, 0x30, 0x30, 0x30, 0x70, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7f, 0x63, 0x7f, 0x63, 0x63, 0x63, 0x63, 0x67, 0xe7, 0xe6, 0xc0, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x18, 0x18, 0xdb, 0x3c, 0xe7, 0x3c, 0xdb, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfe, 0xf8, 0xf0, 0xe0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x02, 0x06, 0x0e, 0x1e, 0x3e, 0xfe, 0x3e, 0x1e, 0x0e, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7f, 0xdb, 0xdb, 0xdb, 0x7b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x7c, 0xc6, 0x60, 0x38, 0x6c, 0xc6, 0xc6, 0x6c, 0x38, 0x0c, 0xc6, 0x7c, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0c, 0xfe, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0xfe, 0x60, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x66, 0xff, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x7c, 0x7c, 0x38, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x66, 0x66, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x6c, 0x6c, 0xfe, 0x6c, 0x6c, 0x6c, 0xfe, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x7c, 0xc6, 0xc2, 0xc0, 0x7c, 0x06, 0x06, 0x86, 0xc6, 0x7c, 0x18, 0x18, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xc2, 0xc6, 0x0c, 0x18, 0x30, 0x60, 0xc6, 0x86, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x76, 0xdc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x30, 0x30, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x0c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x3c, 0xff, 0x3c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc3, 0xc3, 0xdb, 0xdb, 0xc3, 0xc3, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x38, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x06, 0x3c, 0x06, 0x06, 0x06, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0c, 0x1c, 0x3c, 0x6c, 0xcc, 0xfe, 0x0c, 0x0c, 0x0c, 0x1e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0xc0, 0xc0, 0xc0, 0xfc, 0x06, 0x06, 0x06, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x60, 0xc0, 0xc0, 0xfc, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0xc6, 0x06, 0x06, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0x7e, 0x06, 0x06, 0x06, 0x0c, 0x78, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0x0c, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xde, 0xde, 0xde, 0xdc, 0xc0, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x66, 0x66, 0x66, 0x66, 0xfc, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc2, 0xc0, 0xc0, 0xc0, 0xc0, 0xc2, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xf8, 0x6c, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x6c, 0xf8, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x62, 0x66, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc2, 0xc0, 0xc0, 0xde, 0xc6, 0xc6, 0x66, 0x3a, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1e, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0xcc, 0xcc, 0xcc, 0x78, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xe6, 0x66, 0x66, 0x6c, 0x78, 0x78, 0x6c, 0x66, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xf0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x62, 0x66, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xe7, 0xff, 0xff, 0xdb, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0xe6, 0xf6, 0xfe, 0xde, 0xce, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xd6, 0xde, 0x7c, 0x0c, 0x0e, 0x00, 0x00,
 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x6c, 0x66, 0x66, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0x60, 0x38, 0x0c, 0x06, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xff, 0xdb, 0x99, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xdb, 0xdb, 0xff, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x18, 0x3c, 0x66, 0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xff, 0xc3, 0x86, 0x0c, 0x18, 0x30, 0x60, 0xc1, 0xc3, 0xff, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0x70, 0x38, 0x1c, 0x0e, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x10, 0x38, 0x6c, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00,
 0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xe0, 0x60, 0x60, 0x78, 0x6c, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1c, 0x0c, 0x0c, 0x3c, 0x6c, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x6c, 0x64, 0x60, 0xf0, 0x60, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0xcc, 0x78, 0x00,
 0x00, 0x00, 0xe0, 0x60, 0x60, 0x6c, 0x76, 0x66, 0x66, 0x66, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x06, 0x06, 0x00, 0x0e, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x66, 0x3c, 0x00,
 0x00, 0x00, 0xe0, 0x60, 0x60, 0x66, 0x6c, 0x78, 0x78, 0x6c, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xe6, 0xff, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xf0, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0x0c, 0x1e, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x76, 0x66, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x60, 0x38, 0x0c, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x10, 0x30, 0x30, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x36, 0x1c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xdb, 0xdb, 0xff, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x66, 0x3c, 0x18, 0x3c, 0x66, 0xc3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7e, 0x06, 0x0c, 0xf8, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xcc, 0x18, 0x30, 0x60, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0e, 0x18, 0x18, 0x18, 0x70, 0x18, 0x18, 0x18, 0x18, 0x0e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x70, 0x18, 0x18, 0x18, 0x0e, 0x18, 0x18, 0x18, 0x18, 0x70, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc2, 0xc0, 0xc0, 0xc0, 0xc2, 0x66, 0x3c, 0x0c, 0x06, 0x7c, 0x00, 0x00,
 0x00, 0x00, 0xcc, 0x00, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0c, 0x18, 0x30, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x10, 0x38, 0x6c, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xcc, 0x00, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x38, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x3c, 0x66, 0x60, 0x60, 0x66, 0x3c, 0x0c, 0x06, 0x3c, 0x00, 0x00, 0x00,
 0x00, 0x10, 0x38, 0x6c, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0x00, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x66, 0x00, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x3c, 0x66, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xc6, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x38, 0x6c, 0x38, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x30, 0x60, 0x00, 0xfe, 0x66, 0x60, 0x7c, 0x60, 0x60, 0x66, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x3b, 0x1b, 0x7e, 0xd8, 0xdc, 0x77, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3e, 0x6c, 0xcc, 0xcc, 0xfe, 0xcc, 0xcc, 0xcc, 0xcc, 0xce, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x10, 0x38, 0x6c, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x30, 0x78, 0xcc, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7e, 0x06, 0x0c, 0x78, 0x00,
 0x00, 0xc6, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xc6, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x18, 0x7e, 0xc3, 0xc0, 0xc0, 0xc0, 0xc3, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x64, 0x60, 0xf0, 0x60, 0x60, 0x60, 0x60, 0xe6, 0xfc, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0x66, 0x3c, 0x18, 0xff, 0x18, 0xff, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xfc, 0x66, 0x66, 0x7c, 0x62, 0x66, 0x6f, 0x66, 0x66, 0x66, 0xf3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0e, 0x1b, 0x18, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0xd8, 0x70, 0x00, 0x00,
 0x00, 0x18, 0x30, 0x60, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0c, 0x18, 0x30, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x30, 0x60, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x30, 0x60, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x76, 0xdc, 0x00, 0xdc, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x76, 0xdc, 0x00, 0xc6, 0xe6, 0xf6, 0xfe, 0xde, 0xce, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x3c, 0x6c, 0x6c, 0x3e, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x60, 0xc0, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xc0, 0xc0, 0xc2, 0xc6, 0xcc, 0x18, 0x30, 0x60, 0xce, 0x9b, 0x06, 0x0c, 0x1f, 0x00, 0x00,
 0x00, 0xc0, 0xc0, 0xc2, 0xc6, 0xcc, 0x18, 0x30, 0x66, 0xce, 0x96, 0x3e, 0x06, 0x06, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6c, 0xd8, 0x6c, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x6c, 0x36, 0x6c, 0xd8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44,
 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa,
 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xf6, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf6, 0x06, 0xf6, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x06, 0xf6, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf6, 0x06, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x30, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x30, 0x37, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf7, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xf7, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x30, 0x37, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf7, 0x00, 0xf7, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xff, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x18, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0,
 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xdc, 0xd8, 0xd8, 0xd8, 0xdc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0xd8, 0xcc, 0xc6, 0xc6, 0xc6, 0xcc, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0xc6, 0xc6, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xfe, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0xfe, 0xc6, 0x60, 0x30, 0x18, 0x30, 0x60, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xd8, 0xd8, 0xd8, 0xd8, 0xd8, 0x70, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xc0, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x76, 0xdc, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x7e, 0x18, 0x3c, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0x6c, 0x6c, 0x6c, 0x6c, 0xee, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1e, 0x30, 0x18, 0x0c, 0x3e, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xdb, 0xdb, 0xdb, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x03, 0x06, 0x7e, 0xdb, 0xdb, 0xf3, 0x7e, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1c, 0x30, 0x60, 0x60, 0x7c, 0x60, 0x60, 0x60, 0x30, 0x1c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x30, 0x18, 0x0c, 0x06, 0x0c, 0x18, 0x30, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x0c, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0c, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0e, 0x1b, 0x1b, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xd8, 0xd8, 0xd8, 0x70, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x7e, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xdc, 0x00, 0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0f, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0xec, 0x6c, 0x6c, 0x3c, 0x1c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xd8, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x70, 0xd8, 0x30, 0x60, 0xc8, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void vga_initmode(VGAState *s)
{
    for (int i = 0; i < 64*3; i++)
        s->palette[i] = pal_ega[i];

    for (int i = 0; i <= 0x13; i++)
        s->ar[i] = actl[i];
    s->ar[0x14] = 0;

    s->sr[0] = 0x3;
    for (int i = 0; i < 4; i++)
        s->sr[i + 1] = sequ[i];

    for (int i = 0; i <= 8; i++)
        s->gr[i] = grdc[i];

    for (int i = 0; i <= 0x18; i++)
        s->cr[i] = crtc[i];

    s->msr = 0x67;

    // clear screen
    for (int i = 0; i < s->vga_ram_size / 4; i++) {
        s->vga_ram[i * 4] = 0x20;
        s->vga_ram[i * 4 + 1] = 0x07;
    }

    // load font
    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < 16; j++) {
            s->vga_ram[i * 32 * 4 + j * 4 + 2] = vgafont16[i * 16 + j];
        }
    }

    s->ar_index = 0x20;
}

/* VGA register snapshot/validation for boot diagnostics */
void vga_snapshot_regs(VGAState *s, uint8_t *sr, uint8_t *gr, uint8_t *cr,
                       uint8_t *ar, uint8_t *msr)
{
    memcpy(sr, s->sr, 5);
    memcpy(gr, s->gr, 9);
    memcpy(cr, s->cr, 25);
    memcpy(ar, s->ar, 21);
    *msr = s->msr;
}

void vga_validate_mode(VGAState *s, int mode)
{
    /* Expected register values for standard VGA modes */
    static const uint8_t mode03_sr[5] = {0x00,0x00,0x03,0x00,0x02};
    static const uint8_t mode03_gr[9] = {0x00,0x00,0x00,0x00,0x00,0x10,0x0E,0x00,0xFF};
    static const uint8_t mode03_cr[25] = {
        0x5F,0x4F,0x50,0x82,0x55,0x81,0xBF,0x1F,0x00,0x4F,0x0D,0x0E,0x00,
        0x00,0x00,0x00,0x9C,0x8E,0x8F,0x28,0x1F,0x96,0xB9,0xA3,0xFF};
    static const uint8_t mode03_ar[21] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x14,0x07,0x38,0x39,0x3A,0x3B,
        0x3C,0x3D,0x3E,0x3F,0x0C,0x00,0x0F,0x08,0x00};
    static const uint8_t mode03_msr = 0x67;

    static const uint8_t mode10_sr[5] = {0x00,0x01,0x0F,0x00,0x06};
    static const uint8_t mode10_gr[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x0F,0xFF};
    static const uint8_t mode10_cr[25] = {
        0x5F,0x4F,0x50,0x82,0x54,0x80,0xBF,0x1F,0x00,0x40,0x00,0x00,0x00,
        0x00,0x00,0x00,0x83,0x85,0x5D,0x28,0x0F,0x63,0xBA,0xE3,0xFF};
    static const uint8_t mode10_ar[21] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x14,0x07,0x38,0x39,0x3A,0x3B,
        0x3C,0x3D,0x3E,0x3F,0x01,0x00,0x0F,0x00,0x00};
    static const uint8_t mode10_msr = 0xA3;

    const uint8_t *exp_sr, *exp_gr, *exp_cr, *exp_ar;
    uint8_t exp_msr;
    const char *mode_name;

    if (mode == 0x03) {
        exp_sr = mode03_sr; exp_gr = mode03_gr; exp_cr = mode03_cr;
        exp_ar = mode03_ar; exp_msr = mode03_msr; mode_name = "03h";
    } else if (mode == 0x10) {
        exp_sr = mode10_sr; exp_gr = mode10_gr; exp_cr = mode10_cr;
        exp_ar = mode10_ar; exp_msr = mode10_msr; mode_name = "10h";
    } else {
        fprintf(stderr, "VGA validate: no reference for mode %02xh\n", mode);
        return;
    }

    bool any_diff = false;
    char buf[256];
    int pos;

    if (s->msr != exp_msr) {
        fprintf(stderr, "VGA mode %s: MSR=%02x (expected %02x)\n",
                mode_name, s->msr, exp_msr);
        any_diff = true;
    }

    /* Check SR */
    pos = 0;
    for (int i = 0; i < 5; i++) {
        if (s->sr[i] != exp_sr[i])
            pos += snprintf(buf + pos, sizeof(buf) - pos, " SR%d=%02x(exp %02x)", i, s->sr[i], exp_sr[i]);
    }
    if (pos > 0) { fprintf(stderr, "VGA mode %s:%s\n", mode_name, buf); any_diff = true; }

    /* Check GR */
    pos = 0;
    for (int i = 0; i < 9; i++) {
        if (s->gr[i] != exp_gr[i])
            pos += snprintf(buf + pos, sizeof(buf) - pos, " GR%d=%02x(exp %02x)", i, s->gr[i], exp_gr[i]);
    }
    if (pos > 0) { fprintf(stderr, "VGA mode %s:%s\n", mode_name, buf); any_diff = true; }

    /* Check CR */
    pos = 0;
    for (int i = 0; i < 25; i++) {
        if (s->cr[i] != exp_cr[i])
            pos += snprintf(buf + pos, sizeof(buf) - pos, " CR%02x=%02x(exp %02x)", i, s->cr[i], exp_cr[i]);
    }
    if (pos > 0) { fprintf(stderr, "VGA mode %s:%s\n", mode_name, buf); any_diff = true; }

    /* Check AR */
    pos = 0;
    for (int i = 0; i < 21; i++) {
        if (s->ar[i] != exp_ar[i])
            pos += snprintf(buf + pos, sizeof(buf) - pos, " AR%02x=%02x(exp %02x)", i, s->ar[i], exp_ar[i]);
    }
    if (pos > 0) { fprintf(stderr, "VGA mode %s:%s\n", mode_name, buf); any_diff = true; }

    if (!any_diff)
        fprintf(stderr, "VGA mode %s: all registers match expected values\n", mode_name);
}

/* Validate current VGA state against a known mode — callable from i386.c */
void vga_validate_current(int mode)
{
    if (vga_trace_state)
        vga_validate_mode(vga_trace_state, mode);
}

/* Dump current VGA register state for boot monitor */
/* Dump last N non-blank lines from text mode screen buffer */
void vga_dump_screen_text(int max_lines)
{
    VGAState *s = vga_trace_state;
    if (!s) return;
    /* Only meaningful in text mode (GR6 bit 0 = 0) */
    if (s->gr[6] & 1) return;  /* graphics mode, skip */
    int cols = s->cr[1] + 1;   /* CRTC end horizontal display + 1 */
    if (cols < 40 || cols > 132) cols = 80;
    int rows = 25;
    /* Text buffer in VGA RAM: start_addr from CRTC, then *4 for VGA RAM offset.
     * Each char is 2 bytes (char + attr) in VGA RAM at stride 2. */
    uint32_t start_addr = s->cr[0x0d] | (s->cr[0x0c] << 8);
    uint32_t text_start = start_addr * 4;
    uint8_t *vram = s->vga_ram + text_start;
    uint32_t text_size = cols * rows * 2;
    if (text_start + text_size > (uint32_t)s->vga_ram_size) return;
    /* Find last non-blank line */
    int last_line = -1;
    for (int r = rows - 1; r >= 0; r--) {
        for (int c = 0; c < cols; c++) {
            uint8_t ch = vram[(r * cols + c) * 2]; /* char byte, stride=2 */
            if (ch != ' ' && ch != 0) { last_line = r; goto found; }
        }
    }
found:
    if (last_line < 0) return;
    int first_line = last_line - max_lines + 1;
    if (first_line < 0) first_line = 0;
    fprintf(stderr, "Screen text (lines %d-%d):\n", first_line, last_line);
    for (int r = first_line; r <= last_line; r++) {
        char line[136];
        int len = 0;
        for (int c = 0; c < cols && len < (int)sizeof(line)-1; c++) {
            uint8_t ch = vram[(r * cols + c) * 2];
            line[len++] = (ch >= 0x20 && ch < 0x7f) ? ch : '.';
        }
        /* trim trailing spaces */
        while (len > 0 && line[len-1] == ' ') len--;
        line[len] = '\0';
        fprintf(stderr, "  |%s|\n", line);
    }
}

void vga_dump_regs_summary(void)
{
    VGAState *s = vga_trace_state;
    if (!s) return;
    fprintf(stderr, "VGA state: mode=%02xh MSR=%02x\n",
        s->gr[6] & 1 ? 0xFF : 0x03, s->msr);  /* rough guess */
    fprintf(stderr, "  SR:");
    for (int i = 0; i < 5; i++) fprintf(stderr, " %02x", s->sr[i]);
    fprintf(stderr, "  GR:");
    for (int i = 0; i < 9; i++) fprintf(stderr, " %02x", s->gr[i]);
    fprintf(stderr, "\n  CR:");
    for (int i = 0; i < 25; i++) fprintf(stderr, " %02x", s->cr[i]);
    fprintf(stderr, "\n  AR:");
    for (int i = 0; i < 21; i++) fprintf(stderr, " %02x", s->ar[i]);
    fprintf(stderr, "\n");
}

/* Dump VRAM plane data and VGA register state to a file.
 * Writes diagnostic info for planar modes to help debug rendering artifacts. */
void vga_dump_vram_to_file(VGAState *s, const char *path, int fb_height)
{
    if (!s || !s->vga_ram) return;
    FILE *f = fopen(path, "w");
    if (!f) return;

    int shift_control = (s->gr[0x05] >> 5) & 3;
    fprintf(f, "VGA sc=%d gr5=%02x sr2=%02x wm=%d\n",
            shift_control, s->gr[5], s->sr[2], s->gr[5] & 3);
    fprintf(f, "SR:");
    for (int i = 0; i < 5; i++) fprintf(f, " %02x", s->sr[i]);
    fprintf(f, "  GR:");
    for (int i = 0; i < 9; i++) fprintf(f, " %02x", s->gr[i]);
    fprintf(f, "\nCR:");
    for (int i = 0; i < 25; i++) fprintf(f, " %02x", s->cr[i]);
    fprintf(f, "\nAR:");
    for (int i = 0; i < 21; i++) fprintf(f, " %02x", s->ar[i]);
    fprintf(f, "\n\n");

    if (shift_control != 0) {
        fprintf(f, "Not planar mode (sc=%d)\n", shift_control);
        fclose(f);
        return;
    }

    uint32_t line_offset = s->cr[0x13];
    line_offset <<= 3;  /* interleaved: CR[0x13] * 8, matches rendering code */
    uint32_t start_addr = (s->cr[0x0c] << 8) | s->cr[0x0d];
    int dots9 = !(s->sr[0x01] & 1);
    int w = (s->cr[0x01] + 1) * (dots9 ? 9 : 8);
    int xdiv = (s->sr[0x01] & 8) ? 2 : 1;
    int src_bytes = (w / xdiv) >> 3;
    char *vram = s->vga_ram;
    uint32_t addr1 = 4 * start_addr;  /* matches rendering: addr1 = 4 * start_addr */

    fprintf(f, "Planar: w=%d xdiv=%d src_bytes=%d line_offset=%lu start_addr=%lu addr1=%lu\n\n",
            w, xdiv, src_bytes, (unsigned long)line_offset,
            (unsigned long)start_addr, (unsigned long)addr1);

    /* Dump lines at key positions — focus on bottom half where bars appear */
    int lines[] = {0, 120, 240, 290, 320, 360, 400, 440, 460, 470};
    int nlines = sizeof(lines) / sizeof(lines[0]);
    for (int li = 0; li < nlines; li++) {
        int y = lines[li];
        if (y < 0 || y >= fb_height) continue;
        uint32_t addr = addr1 + y * line_offset;
        int nbytes = (src_bytes < 20) ? src_bytes : 20;
        fprintf(f, "Y%d (addr=%lu):\n", y, (unsigned long)addr);
        for (int p = 0; p < 4; p++) {
            fprintf(f, "  P%d:", p);
            for (int x = 0; x < nbytes; x++)
                fprintf(f, " %02x", (uint8_t)vram[addr + x * 4 + p]);
            fprintf(f, "\n");
        }
        fprintf(f, "\n");
    }

    fclose(f);
}
