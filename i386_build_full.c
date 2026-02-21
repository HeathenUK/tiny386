/*
 * i386_build_full.c — Compile the full CPU core with prefixed symbols.
 * All public symbols get a full_ prefix to avoid clashing with the fast core.
 */

/* cpui386_* public API (18 functions) */
#define cpui386_new              full_cpui386_new
#define cpui386_delete           full_cpui386_delete
#define cpui386_enable_fpu       full_cpui386_enable_fpu
#define cpui386_reset            full_cpui386_reset
#define cpui386_reset_pm         full_cpui386_reset_pm
#define cpui386_step             full_cpui386_step
#define cpui386_raise_irq        full_cpui386_raise_irq
#define cpui386_set_gpr          full_cpui386_set_gpr
#define cpui386_get_cycle        full_cpui386_get_cycle
#define cpui386_set_a20          full_cpui386_set_a20
#define cpui386_get_a20          full_cpui386_get_a20
#define cpui386_set_diag         full_cpui386_set_diag
#define cpui386_set_fast_mode    full_cpui386_set_fast_mode
#define cpui386_is_halted        full_cpui386_is_halted
#define cpui386_get_perf_counters  full_cpui386_get_perf_counters
#define cpui386_get_detail_counters full_cpui386_get_detail_counters
#define cpui386_snapshot         full_cpui386_snapshot

/* cpu_* functions called by fpu.c (14 functions) */
#define cpu_load8    full_cpu_load8
#define cpu_store8   full_cpu_store8
#define cpu_load16   full_cpu_load16
#define cpu_store16  full_cpu_store16
#define cpu_load32   full_cpu_load32
#define cpu_store32  full_cpu_store32
#define cpu_load64   full_cpu_load64
#define cpu_store64  full_cpu_store64
#define cpu_setax    full_cpu_setax
#define cpu_getax    full_cpu_getax
#define cpu_setexc   full_cpu_setexc
#define cpu_setflags full_cpu_setflags
#define cpu_getflags full_cpu_getflags
#define cpu_abort    full_cpu_abort

/* fpu.h functions (called by i386, defined in fpu_build_full.c) */
#define fpu_new      full_fpu_new
#define fpu_delete   full_fpu_delete
#define fpu_exec1    full_fpu_exec1
#define fpu_exec2    full_fpu_exec2

/* Standalone */
#define i386_reset_flags_tables_for_reinit full_i386_reset_flags_tables_for_reinit
#define i386_init_flags_tables full_i386_init_flags_tables
#define cpui386_set_verbose full_cpui386_set_verbose

/* PIE SIMD externs (shared — no rename needed, defined once in vga_pie.S) */

#include "i386.c"
