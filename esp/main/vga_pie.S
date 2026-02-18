/*
 * PIE-optimized VGA rendering functions for ESP32-P4
 * Uses 128-bit vector stores for writing 8 pixels at once
 */

#include "sdkconfig.h"

    .text
    .align 4

/*
 * Write 8 RGB565 pixels to framebuffer using PIE 128-bit store
 *
 * void pie_write_8pixels(uint16_t *dst, uint16_t *colors);
 *   a0 = destination pointer (must be 16-byte aligned)
 *   a1 = pointer to 8 uint16_t colors (must be 16-byte aligned)
 */
    .global pie_write_8pixels
    .type   pie_write_8pixels, @function
pie_write_8pixels:
    /* Load 8 colors (128 bits) from colors array */
    esp.vld.128.ip q0, a1, 0
    /* Store 8 pixels (128 bits) to framebuffer */
    esp.vst.128.ip q0, a0, 0
    ret
    .size   pie_write_8pixels, . - pie_write_8pixels

/*
 * Write 8 RGB565 pixels doubled (16 pixels total) for xdiv=2 modes
 * Takes 8 colors and writes each one twice: [A,A,B,B,C,C,D,D,E,E,F,F,G,G,H,H]
 *
 * void pie_write_8pixels_doubled(uint16_t *dst, uint16_t *colors);
 *   a0 = destination pointer (must be 16-byte aligned)
 *   a1 = pointer to 8 uint16_t colors
 *
 * Note: No interleave instruction available, so we manually construct
 * the doubled output using scalar operations and two 128-bit stores
 */
    .global pie_write_8pixels_doubled
    .type   pie_write_8pixels_doubled, @function
pie_write_8pixels_doubled:
    /* Load colors to general registers and construct doubled pattern */
    /* First 4 colors -> first 128-bit store (8 pixels) */
    lhu     t0, 0(a1)       /* color 0 */
    lhu     t1, 2(a1)       /* color 1 */
    lhu     t2, 4(a1)       /* color 2 */
    lhu     t3, 6(a1)       /* color 3 */

    /* Pack pairs: t0|t0, t1|t1, t2|t2, t3|t3 */
    slli    t4, t0, 16
    or      t4, t4, t0      /* t4 = color0 | color0 */
    slli    t5, t1, 16
    or      t5, t5, t1      /* t5 = color1 | color1 */
    slli    t6, t2, 16
    or      t6, t6, t2      /* t6 = color2 | color2 */
    slli    a7, t3, 16
    or      a7, a7, t3      /* a7 = color3 | color3 */

    /* Store first 8 pixels (4 colors doubled) */
    sw      t4, 0(a0)
    sw      t5, 4(a0)
    sw      t6, 8(a0)
    sw      a7, 12(a0)

    /* Second 4 colors -> second 128-bit store (8 pixels) */
    lhu     t0, 8(a1)       /* color 4 */
    lhu     t1, 10(a1)      /* color 5 */
    lhu     t2, 12(a1)      /* color 6 */
    lhu     t3, 14(a1)      /* color 7 */

    slli    t4, t0, 16
    or      t4, t4, t0
    slli    t5, t1, 16
    or      t5, t5, t1
    slli    t6, t2, 16
    or      t6, t6, t2
    slli    a7, t3, 16
    or      a7, a7, t3

    sw      t4, 16(a0)
    sw      t5, 20(a0)
    sw      t6, 24(a0)
    sw      a7, 28(a0)

    ret
    .size   pie_write_8pixels_doubled, . - pie_write_8pixels_doubled

/*
 * Fast memcpy using PIE 128-bit loads/stores
 * Copies n bytes (must be multiple of 16, addresses must be 16-byte aligned)
 *
 * void pie_memcpy_128(void *dst, const void *src, size_t n);
 */
    .global pie_memcpy_128
    .type   pie_memcpy_128, @function
pie_memcpy_128:
    beqz    a2, .Lmemcpy_done
    srli    a2, a2, 4       /* n /= 16 (number of 128-bit chunks) */
.Lmemcpy_loop:
    esp.vld.128.ip q0, a1, 16
    esp.vst.128.ip q0, a0, 16
    addi    a2, a2, -1
    bnez    a2, .Lmemcpy_loop
.Lmemcpy_done:
    ret
    .size   pie_memcpy_128, . - pie_memcpy_128

/*
 * Fast 32-bit value fill using PIE 128-bit stores
 * Fills n bytes with a repeated 32-bit value
 * n must be a multiple of 16, dst must be 16-byte aligned
 *
 * void pie_memset32_128(void *dst, uint32_t val, size_t n);
 */
    .global pie_memset32_128
    .type   pie_memset32_128, @function
pie_memset32_128:
    beqz    a2, .Lms32_done
    srli    a2, a2, 4       /* n /= 16 (number of 128-bit chunks) */
    /* Broadcast 32-bit value to 128-bit q0 via stack */
    addi    sp, sp, -16
    sw      a1, 0(sp)
    sw      a1, 4(sp)
    sw      a1, 8(sp)
    sw      a1, 12(sp)
    mv      a3, sp
    esp.vld.128.ip q0, a3, 0
    addi    sp, sp, 16
.Lms32_loop:
    esp.vst.128.ip q0, a0, 16
    addi    a2, a2, -1
    bnez    a2, .Lms32_loop
.Lms32_done:
    ret
    .size   pie_memset32_128, . - pie_memset32_128

/*
 * Render 8 Mode 13h pixels: load indices, lookup palette, store RGB565
 * Interleaves operations to hide memory latency
 *
 * void pie_render_8pixels_mode13(uint16_t *dst, const uint8_t *indices, const uint16_t *palette);
 *   a0 = destination pointer (should be 16-byte aligned)
 *   a1 = pointer to 8 palette indices
 *   a2 = pointer to 256-entry RGB565 palette
 */
    .global pie_render_8pixels_mode13
    .type   pie_render_8pixels_mode13, @function
pie_render_8pixels_mode13:
    /* Load 8 indices as two 32-bit words */
    lw      t0, 0(a1)           /* indices 0-3 */
    lw      t1, 4(a1)           /* indices 4-7 */

    /* Extract index 0 and start lookup (hide latency with other extracts) */
    andi    t2, t0, 0xFF        /* idx0 */
    slli    t2, t2, 1           /* idx0 * 2 (byte offset) */
    add     t2, a2, t2

    srli    t3, t0, 8
    andi    t3, t3, 0xFF        /* idx1 */
    slli    t3, t3, 1
    add     t3, a2, t3

    srli    t4, t0, 16
    andi    t4, t4, 0xFF        /* idx2 */
    slli    t4, t4, 1
    add     t4, a2, t4

    srli    t5, t0, 24          /* idx3 (no mask needed) */
    slli    t5, t5, 1
    add     t5, a2, t5

    /* Load first 4 colors (interleaved with index 4-7 calculation) */
    lhu     a3, 0(t2)           /* color0 */

    andi    t2, t1, 0xFF        /* idx4 */
    slli    t2, t2, 1
    add     t2, a2, t2

    lhu     a4, 0(t3)           /* color1 */

    srli    t3, t1, 8
    andi    t3, t3, 0xFF        /* idx5 */
    slli    t3, t3, 1
    add     t3, a2, t3

    lhu     a5, 0(t4)           /* color2 */

    srli    t4, t1, 16
    andi    t4, t4, 0xFF        /* idx6 */
    slli    t4, t4, 1
    add     t4, a2, t4

    lhu     a6, 0(t5)           /* color3 */

    srli    t5, t1, 24          /* idx7 */
    slli    t5, t5, 1
    add     t5, a2, t5

    /* Load colors 4-7 */
    lhu     a7, 0(t2)           /* color4 */
    lhu     t0, 0(t3)           /* color5 */
    lhu     t1, 0(t4)           /* color6 */
    lhu     t2, 0(t5)           /* color7 */

    /* Pack into 32-bit words for efficient store */
    slli    t3, a4, 16
    or      t3, t3, a3          /* colors[1:0] */
    slli    t4, a6, 16
    or      t4, t4, a5          /* colors[3:2] */
    slli    t5, t0, 16
    or      t5, t5, a7          /* colors[5:4] */
    slli    t6, t2, 16
    or      t6, t6, t1          /* colors[7:6] */

    /* Store 16 bytes (8 pixels) */
    sw      t3, 0(a0)
    sw      t4, 4(a0)
    sw      t5, 8(a0)
    sw      t6, 12(a0)

    ret
    .size   pie_render_8pixels_mode13, . - pie_render_8pixels_mode13

/*
 * Render a complete glyph using PIE SIMD and bit expansion LUT.
 * For each font row, loads a pre-computed 128-bit mask from LUT,
 * ANDs with broadcast xorcol, XORs with broadcast bgcol, stores 8 RGB565 pixels.
 * ~10 instructions per row vs ~50 for scalar C.
 *
 * void pie_render_glyph(uint8_t *tile, const uint8_t *font_ptr, int cheight,
 *                        uint32_t fgcol, uint32_t bgcol, const uint16_t *mask_table);
 *   a0 = tile output pointer (16-byte aligned, 16 bytes per row)
 *   a1 = font_ptr (font data, 4-byte stride between rows)
 *   a2 = cheight (number of rows, typically 16)
 *   a3 = fgcol (16-bit RGB565 foreground color)
 *   a4 = bgcol (16-bit RGB565 background color)
 *   a5 = mask_table (256 entries x 16 bytes, 16-byte aligned)
 */
    .global pie_render_glyph
    .type   pie_render_glyph, @function
pie_render_glyph:
    beqz    a2, .Lrg_done

    /* Compute xorcol = fgcol ^ bgcol */
    xor     t0, a3, a4

    /* Broadcast xorcol to q1 (8 x 16-bit) via stack.
     * PIE load/store requires a-register as base pointer. */
    addi    sp, sp, -32
    slli    t1, t0, 16
    or      t0, t0, t1          /* t0 = xorcol | xorcol */
    sw      t0, 0(sp)
    sw      t0, 4(sp)
    sw      t0, 8(sp)
    sw      t0, 12(sp)
    mv      a3, sp              /* a3 = stack base (a-reg for PIE) */
    esp.vld.128.ip q1, a3, 0

    /* Broadcast bgcol to q2 (8 x 16-bit) via stack */
    slli    t0, a4, 16
    or      t0, t0, a4          /* t0 = bgcol | bgcol */
    sw      t0, 16(sp)
    sw      t0, 20(sp)
    sw      t0, 24(sp)
    sw      t0, 28(sp)
    addi    a3, sp, 16          /* a3 = stack+16 (a-reg for PIE) */
    esp.vld.128.ip q2, a3, 0

    addi    sp, sp, 32

.Lrg_loop:
    /* Load font byte, compute mask table offset */
    lbu     t0, 0(a1)
    slli    t0, t0, 4           /* font_data * 16 */
    add     a3, a5, t0          /* a3 = &mask_table[font_data] (a-reg) */

    /* Load 128-bit mask: 8 x (0x0000 or 0xFFFF) */
    esp.vld.128.ip q0, a3, 0

    /* result = (mask & xorcol) ^ bgcol */
    esp.andq q0, q0, q1
    esp.xorq q0, q0, q2

    /* Store 8 pixels, advance tile ptr by 16 */
    esp.vst.128.ip q0, a0, 16

    addi    a1, a1, 4           /* font_ptr += 4 */
    addi    a2, a2, -1
    bnez    a2, .Lrg_loop

.Lrg_done:
    ret
    .size   pie_render_glyph, . - pie_render_glyph

/*
 * Copy tile rows to framebuffer using PIE 128-bit load/store.
 * Source rows are contiguous (16 bytes each), dest rows are spaced by stride.
 * ~5 instructions per row vs ~11 for scalar memcpy loop.
 *
 * void pie_copy_tile_to_fb(uint8_t *dst, const uint8_t *tile, int cheight, int stride);
 *   a0 = framebuffer destination (16-byte aligned)
 *   a1 = tile source (16-byte aligned, 16 bytes per row contiguous)
 *   a2 = cheight (number of rows)
 *   a3 = stride (framebuffer row stride in bytes)
 */
    .global pie_copy_tile_to_fb
    .type   pie_copy_tile_to_fb, @function
pie_copy_tile_to_fb:
    beqz    a2, .Lct_done

.Lct_loop:
    esp.vld.128.ip q0, a1, 16  /* load 16 bytes from tile, advance src */
    esp.vst.128.ip q0, a0, 0   /* store 16 bytes to framebuffer */
    add     a0, a0, a3          /* advance dst by stride */
    addi    a2, a2, -1
    bnez    a2, .Lct_loop

.Lct_done:
    ret
    .size   pie_copy_tile_to_fb, . - pie_copy_tile_to_fb
