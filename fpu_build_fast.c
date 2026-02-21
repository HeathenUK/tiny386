/*
 * fpu_build_fast.c — Compile FPU for fast core with prefixed symbols.
 */

/* FPU public functions (defined in fpu.c, called by i386_fast.c) */
#define fpu_new      fast_fpu_new
#define fpu_delete   fast_fpu_delete
#define fpu_exec1    fast_fpu_exec1
#define fpu_exec2    fast_fpu_exec2

/* cpu_* functions that fpu.c calls (defined in i386_fast.c via i386_build_fast.c) */
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

#include "fpu.c"
