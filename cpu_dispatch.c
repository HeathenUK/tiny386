/*
 * cpu_dispatch.c — Provides real cpui386_* symbols, dispatches to active core.
 *
 * Only one core executes per session. cpu_select_core() is called once at boot
 * based on the "accuracy" INI setting. The inactive core's code stays in flash,
 * never fetched into the ESP32-P4's 16KB L1 I-cache.
 */

#include "i386.h"
#include "cpu_dispatch.h"

static bool use_fast_core;

void cpu_select_core(bool fast)
{
	use_fast_core = fast;
}

/* --- Forward declarations for both cores --- */

/* Fast core (i386_fast.c via i386_build_fast.c) */
extern CPUI386 *fast_cpui386_new(int gen, char *pm, long sz, CPU_CB **cb);
extern void fast_cpui386_delete(CPUI386 *cpu);
extern void fast_cpui386_enable_fpu(CPUI386 *cpu);
extern void fast_cpui386_reset(CPUI386 *cpu);
extern void fast_cpui386_reset_pm(CPUI386 *cpu, uint32_t start_addr);
extern void fast_cpui386_step(CPUI386 *cpu, int n);
extern void fast_cpui386_raise_irq(CPUI386 *cpu);
extern void fast_cpui386_set_gpr(CPUI386 *cpu, int i, u32 val);
extern long fast_cpui386_get_cycle(CPUI386 *cpu);
extern void fast_cpui386_set_a20(CPUI386 *cpu, int enabled);
extern int  fast_cpui386_get_a20(CPUI386 *cpu);
extern void fast_cpui386_set_diag(CPUI386 *cpu, bool enabled);
extern void fast_cpui386_set_fast_mode(CPUI386 *cpu, bool enabled);
extern bool fast_cpui386_is_halted(CPUI386 *cpu);
extern void fast_cpui386_get_perf_counters(CPUI386 *cpu, uint32_t *a, uint32_t *b,
                                           uint32_t *c, uint32_t *d, uint32_t *e);
extern void fast_cpui386_get_detail_counters(CPUI386 *cpu, CpuDetailCounters *out);
extern void fast_cpui386_snapshot(CPUI386 *cpu, CpuSnapshot *snap);
extern void fast_i386_reset_flags_tables_for_reinit(void);

/* Full core (i386.c via i386_build_full.c) */
extern CPUI386 *full_cpui386_new(int gen, char *pm, long sz, CPU_CB **cb);
extern void full_cpui386_delete(CPUI386 *cpu);
extern void full_cpui386_enable_fpu(CPUI386 *cpu);
extern void full_cpui386_reset(CPUI386 *cpu);
extern void full_cpui386_reset_pm(CPUI386 *cpu, uint32_t start_addr);
extern void full_cpui386_step(CPUI386 *cpu, int n);
extern void full_cpui386_raise_irq(CPUI386 *cpu);
extern void full_cpui386_set_gpr(CPUI386 *cpu, int i, u32 val);
extern long full_cpui386_get_cycle(CPUI386 *cpu);
extern void full_cpui386_set_a20(CPUI386 *cpu, int enabled);
extern int  full_cpui386_get_a20(CPUI386 *cpu);
extern void full_cpui386_set_diag(CPUI386 *cpu, bool enabled);
extern void full_cpui386_set_fast_mode(CPUI386 *cpu, bool enabled);
extern bool full_cpui386_is_halted(CPUI386 *cpu);
extern void full_cpui386_get_perf_counters(CPUI386 *cpu, uint32_t *a, uint32_t *b,
                                           uint32_t *c, uint32_t *d, uint32_t *e);
extern void full_cpui386_get_detail_counters(CPUI386 *cpu, CpuDetailCounters *out);
extern void full_cpui386_snapshot(CPUI386 *cpu, CpuSnapshot *snap);
extern void full_i386_reset_flags_tables_for_reinit(void);

/* --- Dispatch functions --- */

CPUI386 *cpui386_new(int gen, char *pm, long sz, CPU_CB **cb)
{
	if (use_fast_core) return fast_cpui386_new(gen, pm, sz, cb);
	return full_cpui386_new(gen, pm, sz, cb);
}

void cpui386_delete(CPUI386 *cpu)
{
	if (use_fast_core) fast_cpui386_delete(cpu);
	else full_cpui386_delete(cpu);
}

void cpui386_enable_fpu(CPUI386 *cpu)
{
	if (use_fast_core) fast_cpui386_enable_fpu(cpu);
	else full_cpui386_enable_fpu(cpu);
}

void cpui386_reset(CPUI386 *cpu)
{
	if (use_fast_core) fast_cpui386_reset(cpu);
	else full_cpui386_reset(cpu);
}

void cpui386_reset_pm(CPUI386 *cpu, uint32_t start_addr)
{
	if (use_fast_core) fast_cpui386_reset_pm(cpu, start_addr);
	else full_cpui386_reset_pm(cpu, start_addr);
}

void cpui386_step(CPUI386 *cpu, int n)
{
	if (use_fast_core) fast_cpui386_step(cpu, n);
	else full_cpui386_step(cpu, n);
}

void cpui386_raise_irq(CPUI386 *cpu)
{
	if (use_fast_core) fast_cpui386_raise_irq(cpu);
	else full_cpui386_raise_irq(cpu);
}

void cpui386_set_gpr(CPUI386 *cpu, int i, u32 val)
{
	if (use_fast_core) fast_cpui386_set_gpr(cpu, i, val);
	else full_cpui386_set_gpr(cpu, i, val);
}

long cpui386_get_cycle(CPUI386 *cpu)
{
	if (use_fast_core) return fast_cpui386_get_cycle(cpu);
	return full_cpui386_get_cycle(cpu);
}

void cpui386_set_a20(CPUI386 *cpu, int enabled)
{
	if (use_fast_core) fast_cpui386_set_a20(cpu, enabled);
	else full_cpui386_set_a20(cpu, enabled);
}

int cpui386_get_a20(CPUI386 *cpu)
{
	if (use_fast_core) return fast_cpui386_get_a20(cpu);
	return full_cpui386_get_a20(cpu);
}

void cpui386_set_diag(CPUI386 *cpu, bool enabled)
{
	if (use_fast_core) fast_cpui386_set_diag(cpu, enabled);
	else full_cpui386_set_diag(cpu, enabled);
}

void cpui386_set_fast_mode(CPUI386 *cpu, bool enabled)
{
	if (use_fast_core) fast_cpui386_set_fast_mode(cpu, enabled);
	else full_cpui386_set_fast_mode(cpu, enabled);
}

bool cpui386_is_halted(CPUI386 *cpu)
{
	if (use_fast_core) return fast_cpui386_is_halted(cpu);
	return full_cpui386_is_halted(cpu);
}

void cpui386_get_perf_counters(CPUI386 *cpu, uint32_t *a, uint32_t *b,
                               uint32_t *c, uint32_t *d, uint32_t *e)
{
	if (use_fast_core) fast_cpui386_get_perf_counters(cpu, a, b, c, d, e);
	else full_cpui386_get_perf_counters(cpu, a, b, c, d, e);
}

void cpui386_get_detail_counters(CPUI386 *cpu, CpuDetailCounters *out)
{
	if (use_fast_core) fast_cpui386_get_detail_counters(cpu, out);
	else full_cpui386_get_detail_counters(cpu, out);
}

void cpui386_snapshot(CPUI386 *cpu, CpuSnapshot *snap)
{
	if (use_fast_core) fast_cpui386_snapshot(cpu, snap);
	else full_cpui386_snapshot(cpu, snap);
}

/* Reset BOTH cores' flag tables — INI switch may change active core */
void i386_reset_flags_tables_for_reinit(void)
{
	fast_i386_reset_flags_tables_for_reinit();
	full_i386_reset_flags_tables_for_reinit();
}
