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

#endif /* CONFIG_IDF_TARGET_ESP32P4 */
