/*
 * i386_build_fast.c — Compile the fast CPU core with prefixed symbols.
 * All public symbols get a fast_ prefix to avoid clashing with the full core.
 */

/* cpui386_* public API (18 functions) */
#define cpui386_new              fast_cpui386_new
#define cpui386_delete           fast_cpui386_delete
#define cpui386_enable_fpu       fast_cpui386_enable_fpu
#define cpui386_reset            fast_cpui386_reset
#define cpui386_reset_pm         fast_cpui386_reset_pm
#define cpui386_step             fast_cpui386_step
#define cpui386_raise_irq        fast_cpui386_raise_irq
#define cpui386_set_gpr          fast_cpui386_set_gpr
#define cpui386_get_cycle        fast_cpui386_get_cycle
#define cpui386_set_a20          fast_cpui386_set_a20
#define cpui386_get_a20          fast_cpui386_get_a20
#define cpui386_set_diag         fast_cpui386_set_diag
#define cpui386_set_fast_mode    fast_cpui386_set_fast_mode
#define cpui386_is_halted        fast_cpui386_is_halted
#define cpui386_get_perf_counters  fast_cpui386_get_perf_counters
#define cpui386_get_detail_counters fast_cpui386_get_detail_counters
#define cpui386_snapshot         fast_cpui386_snapshot

/* cpu_* functions called by fpu.c (14 functions) */
#define cpu_load8    fast_cpu_load8
#define cpu_store8   fast_cpu_store8
#define cpu_load16   fast_cpu_load16
#define cpu_store16  fast_cpu_store16
#define cpu_load32   fast_cpu_load32
#define cpu_store32  fast_cpu_store32
#define cpu_load64   fast_cpu_load64
#define cpu_store64  fast_cpu_store64
#define cpu_setax    fast_cpu_setax
#define cpu_getax    fast_cpu_getax
#define cpu_setexc   fast_cpu_setexc
#define cpu_setflags fast_cpu_setflags
#define cpu_getflags fast_cpu_getflags
#define cpu_abort    fast_cpu_abort

/* fpu.h functions (called by i386, defined in fpu_build_fast.c) */
#define fpu_new      fast_fpu_new
#define fpu_delete   fast_fpu_delete
#define fpu_exec1    fast_fpu_exec1
#define fpu_exec2    fast_fpu_exec2

/* Standalone */
#define i386_reset_flags_tables_for_reinit fast_i386_reset_flags_tables_for_reinit
#define i386_init_flags_tables fast_i386_init_flags_tables

/* PIE SIMD externs (shared — no rename needed, defined once in vga_pie.S) */

#include "i386_fast.c"
