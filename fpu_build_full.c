/*
 * fpu_build_full.c — Compile FPU for full core with prefixed symbols.
 */

/* FPU public functions (defined in fpu.c, called by i386.c) */
#define fpu_new      full_fpu_new
#define fpu_delete   full_fpu_delete
#define fpu_exec1    full_fpu_exec1
#define fpu_exec2    full_fpu_exec2

/* cpu_* functions that fpu.c calls (defined in i386.c via i386_build_full.c) */
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

#include "fpu.c"
