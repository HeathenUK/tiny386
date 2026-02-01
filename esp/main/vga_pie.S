/*
 * PIE-optimized VGA rendering functions for ESP32-P4
 * Uses 128-bit vector stores for writing 8 pixels at once
 */

#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP32P4

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

#endif /* CONFIG_IDF_TARGET_ESP32P4 */
