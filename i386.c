#include "i386.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "i8259.h"
#include "i8254.h"
#include "pc.h"
extern int pc_batch_size_setting;
extern int pc_last_batch_size;
extern int pc_pit_burst_setting;
extern void vga_dump_io_trace(void);
extern void vga_validate_current(int mode);
extern void vga_dump_regs_summary(void);
extern void vga_dump_screen_text(int max_lines);

/* PIE SIMD helpers for ESP32-P4 string operations */
extern void pie_memcpy_128(void *dst, const void *src, size_t n);
extern void pie_memset32_128(void *dst, uint32_t val, size_t n);
uint32_t get_uticks(void);

static inline void fast_memcpy(void *dst, const void *src, size_t n) {
	if (n >= 16 && !((uintptr_t)dst & 0xf) && !((uintptr_t)src & 0xf)) {
		size_t aligned = n & ~(size_t)0xf;
		pie_memcpy_128(dst, src, aligned);
		if (n > aligned)
			memcpy((char *)dst + aligned, (const char *)src + aligned, n - aligned);
	} else {
		memcpy(dst, src, n);
	}
}

static inline void fast_memset32(void *dst, uint32_t val, size_t count) {
	if (count >= 4 && !((uintptr_t)dst & 0xf)) {
		size_t bytes = count * 4;
		size_t aligned = bytes & ~(size_t)0xf;
		pie_memset32_128(dst, val, aligned);
		uint32_t *p = (uint32_t *)((char *)dst + aligned);
		for (size_t i = 0; i < count - aligned / 4; i++)
			p[i] = val;
	} else {
		uint32_t *p = (uint32_t *)dst;
		for (size_t i = 0; i < count; i++)
			p[i] = val;
	}
}

#define I386_OPT1
#define I386_OPT2
#ifdef I386_OPT2
#define I386_SEQ_FASTPATH
#endif
/* Fine-grained REP string fast-path toggles for Win3.x validation. */
#define I386_MOVS_FASTPATH 1
#define I386_STOS_FASTPATH 1

#ifdef I386_SEQ_FASTPATH
#define SEQ_INVALIDATE(cpu) do { (cpu)->seq_active = false; } while(0)
#else
#define SEQ_INVALIDATE(cpu)
#endif

#define I386_ENABLE_FPU

#ifdef I386_ENABLE_FPU
#include "fpu.h"
#else
#define fpu_new(...) NULL
#define fpu_exec1(...) false
#define fpu_exec2(...) false
typedef void FPU;
#endif

struct CPUI386 {
#ifdef I386_OPT1
	union {
		u32 r32;
		u16 r16;
		u8 r8[2];
	} gprx[8];
#else
	uword gpr[8];
#endif
	uword ip, next_ip;
	uword flags;
	uword flags_mask;
	int cpl;
	bool code16;
	uword sp_mask;
	bool halt;
	bool tf_trap_pending; /* TF single-step: #DB pending after previous insn */

	FPU *fpu;

	struct {
		uword sel;
		uword base;
		uword limit;
		uword flags;
	} seg[8];

	struct {
		uword base;
		uword limit;
	} idt, gdt;

	uword cr0, cr2, cr3, cr4;
	uint32_t a20_mask;  /* 0xFFFFFFFF when A20 enabled, 0xFFEFFFFF when disabled */

	/* Debug registers: storage only.  Hardware breakpoint matching (DR0-DR3
	 * address comparison on every memory access / instruction fetch) and #DB
	 * generation from DR7 enable bits are deliberately NOT implemented.
	 * The per-access cost is incompatible with the ESP32-P4 performance
	 * budget — it would add branches inside translate() / translate8r() and
	 * defeat the flat-segment fast paths.  MOV to/from DRn works so that
	 * guest OS context-switch code doesn't fault; the values are simply
	 * never acted upon.  TF-based single-step (#DB from EFLAGS.TF) is a
	 * separate issue tracked elsewhere and is NOT blocked by this decision. */
	uword dr[8];

	struct {
		unsigned long laddr;
		uword xaddr;
	} ifetch;

	/* Instruction prefetch pointer: points into phys_mem for current instruction.
	 * Eliminates repeated ifetch cache checks within a single instruction. */
	const uint8_t *pf_ptr;
	uint8_t pf_pos;
	uint8_t pf_avail;

	struct {
		int op;
		uword dst;
		uword dst2;
		uword src1;
		uword src2;
		uword mask;
	} cc;

	struct {
		int size;
		uint32_t generation;    /* incremented on CR3 write; entries must match */
		struct tlb_entry {
			uword lpgno;
			uint32_t generation;
			uword xaddr;
			int (*pte_lookup)[2];
			u8 *ppte;
		} *tab;
	} tlb;

	u8 *phys_mem;
	long phys_mem_size;

	long cycle;
	uint64_t tsc;        /* Time stamp counter for RDTSC (lazy-synced from cycle) */
	long tsc_sync_cycle; /* cycle value at last TSC sync point */

	int excno;
	uword excerr;

	bool intr;
	CPU_CB cb;

	/* Cached INT 8 (timer) vector for fast interrupt dispatch.
	 * Real-mode timer ISR address rarely changes after boot.
	 * We use a warmup counter to avoid caching during game setup
	 * when the vector might still be changing. */
	uint32_t cached_int8_vector;
	bool int8_cache_valid;
	uint32_t int8_warmup_counter;  /* Count INT 8 calls before trusting cache */

	int gen;
	struct {
		uword cs, eip, esp;
	} sysenter;

	int diag_gen; /* Diagnostic generation counter — incremented on reset */

	/* Flat memory optimization: bitmask of segments with base=0 and limit=4GB.
	 * When set, we can skip segment base addition and limit checks.
	 * Used by DPMI programs like DOOM that set up flat 4GB segments. */
	uint8_t seg_flat;
	/* True when ALL 6 data/code segments are flat (seg_flat == 0x3F).
	 * Allows a single branch to skip all segment translation overhead. */
	bool all_segs_flat;

#ifdef I386_SEQ_FASTPATH
	uword seq_phys;
	uword seq_prev_ip;
	bool seq_active;
	uint32_t seq_hits;
#endif

	/* Performance counters — read once per 2s reporting window, near-zero cost */
	uint32_t tlb_miss_count;   /* TLB refills (only when paging enabled) */
	uint32_t irq_count;        /* Hardware IRQ deliveries */
	uint32_t fusion_count;     /* CMP/TEST+Jcc macro-op fusions */
	uint32_t hle_hit_count;    /* BIOS HLE calls intercepted */
	uint32_t hle_call_count;   /* BIOS HLE calls attempted */
};

#define dolog(...) fprintf(stderr, __VA_ARGS__)
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#define wordmask ((uword) ((sword) -1))
#define TRY(f) if(!(f)) { return false; }
#define TRYL(f) if(unlikely(!(f))) { return false; }
#define TRY1(f) if(unlikely(!(f))) { dolog("@ %s %s %d\n", __FILE__, __FUNCTION__, __LINE__); cpu_abort(cpu, -1); }
#define THROW(ex, err) do { cpu->excno = (ex); cpu->excerr = (err); return false; } while(0)
#define THROW0(ex) do { cpu->excno = (ex); return false; } while(0)

// the second branchless version works better on gcc
//#define SET_BIT(w, f, m) ((w) ^= ((-(uword)(f)) ^ (w)) & (m))
#define SET_BIT(w, f, m) ((w) = ((w) & ~((uword)(m))) | ((-(uword)(f)) & (m)))
//#define SET_BIT(w, f, m) do { if (f) (w) |= (m); else (w) &= ~(m); } while (0)

enum {
	EX_DE,
	EX_DB,
	EX_NMI,
	EX_BP,
	EX_OF,
	EX_BR,
	EX_UD,
	EX_NM,
	EX_DF,
	EX_INT9,
	EX_TS,
	EX_NP,
	EX_SS,
	EX_GP,
	EX_PF,
};

enum {
	CF = 0x1,
	/* 1 0x2 */
	PF = 0x4,
	/* 0 0x8 */
	AF = 0x10,
	/* 0 0x20 */
	ZF = 0x40,
	SF = 0x80,
	TF = 0x100,
	IF = 0x200,
	DF = 0x400,
	OF = 0x800,
	IOPL = 0x3000,
	NT = 0x4000,
	/* 0 0x8000 */
	RF = 0x10000,
	VM = 0x20000,
};

enum {
	SEG_ES = 0,
	SEG_CS,
	SEG_SS,
	SEG_DS,
	SEG_FS,
	SEG_GS,
	SEG_LDT,
	SEG_TR,
};

enum {
	SEG_D_BIT = 1 << 14,
	SEG_B_BIT = 1 << 14,
};

#ifdef I386_OPT1
#define REGi(x) (cpu->gprx[x].r32)
#else
#define REGi(x) (cpu->gpr[x])
#endif
#define SEGi(x) (cpu->seg[x].sel)

/* Flat segment detection: base=0 and limit=0xFFFFFFFF (4GB) */
#define SEG_IS_FLAT(cpu, seg) ((cpu)->seg_flat & (1 << (seg)))

static void cpu_debug(CPUI386 *cpu);

void cpu_abort(CPUI386 *cpu, int code)
{
	dolog("abort: %d %x cycle %ld\n", code, code, cpu->cycle);
	cpu_debug(cpu);
	abort();
}

/* Intel 386 exception classification for double-fault escalation.
 * Per 386 Programmer's Reference Manual, Table 9-3:
 *   Contributory: #DE(0), #TS(10), #NP(11), #SS(12), #GP(13)
 *   Page Fault:   #PF(14)
 *   Benign:       everything else (#DB, #NMI, #BP, #OF, #BR, #UD, #NM, #MF)
 */
static inline int exc_class_386(int exc) {
	switch (exc) {
	case EX_DE: case EX_TS: case EX_NP: case EX_SS: case EX_GP:
		return 1; /* contributory */
	case EX_PF:
		return 2; /* page fault */
	default:
		return 0; /* benign */
	}
}

static inline bool exc_pushes_error_code(int exc) {
	switch (exc) {
	case EX_DF: case EX_TS: case EX_NP: case EX_SS: case EX_GP:
	case EX_PF: case 17:
		return true;
	default:
		return false;
	}
}

static uword sext8(u8 a)
{
	return (sword) (s8) a;
}

static uword sext16(u16 a)
{
	return (sword) (s16) a;
}

static uword sext32(u32 a)
{
	return (sword) (s32) a;
}

#ifdef I386_OPT1
/* only works on hosts that are little-endian and support unaligned access */
static inline u8 pload8(CPUI386 *cpu, uword addr)
{
	return cpu->phys_mem[addr];
}

static inline u16 pload16(CPUI386 *cpu, uword addr)
{
	return *(u16 *)&(cpu->phys_mem[addr]);
}

static inline u32 pload32(CPUI386 *cpu, uword addr)
{
	return *(u32 *)&(cpu->phys_mem[addr]);
}

static inline void pstore8(CPUI386 *cpu, uword addr, u8 val)
{
	cpu->phys_mem[addr] = val;
}

static inline void pstore16(CPUI386 *cpu, uword addr, u16 val)
{
	*(u16 *)&(cpu->phys_mem[addr]) = val;
}

static inline void pstore32(CPUI386 *cpu, uword addr, u32 val)
{
	*(u32 *)&(cpu->phys_mem[addr]) = val;
}
#else
static inline u8 pload8(CPUI386 *cpu, uword addr)
{
	return cpu->phys_mem[addr];
}

static inline u16 pload16(CPUI386 *cpu, uword addr)
{
	u8 *mem = (u8 *) cpu->phys_mem;
	return mem[addr] | (mem[addr + 1] << 8);
}

static inline u32 pload32(CPUI386 *cpu, uword addr)
{
	u8 *mem = (u8 *) cpu->phys_mem;
	return mem[addr] | (mem[addr + 1] << 8) |
		(mem[addr + 2] << 16) | (mem[addr + 3] << 24);
}

static inline void pstore8(CPUI386 *cpu, uword addr, u8 val)
{
	cpu->phys_mem[addr] = val;
}

static inline void pstore16(CPUI386 *cpu, uword addr, u16 val)
{
	cpu->phys_mem[addr] = val;
	cpu->phys_mem[addr + 1] = val >> 8;
}

static inline void pstore32(CPUI386 *cpu, uword addr, u32 val)
{
	cpu->phys_mem[addr] = val;
	cpu->phys_mem[addr + 1] = val >> 8;
	cpu->phys_mem[addr + 2] = val >> 16;
	cpu->phys_mem[addr + 3] = val >> 24;
}
#endif

/* PF handler diagnostic: watches the physical PTE address during handler execution.
 * Declared early so store functions can check the watchpoint.
 * Silent during normal operation — only produces output when the handler FAILS. */
static struct {
	bool active;            /* true while inside PF handler for watched fault */
	uword watch_phys;       /* physical address of the PTE being watched (4-byte aligned) */
	uword initial_pte;      /* PTE value when watchpoint was set up */
	uword cr2;              /* CR2 that triggered this watch */
	int write_count;        /* number of writes to watched PTE during handler */
	int nested_exc_count;   /* number of nested exceptions during handler */
	uword last_write_val;   /* last value written to watched PTE */
	uword last_write_eip;   /* EIP of last write instruction */
	uint16_t last_write_cs; /* CS of last write instruction */
	int cr3_write_count;    /* CR3 writes during handler */
	int invlpg_count;       /* INVLPG executions during handler */
	int nested_pf_count;    /* nested #PF during handler */
	uword nested_pf_last_cr2;  /* CR2 of last nested #PF */
	int oob_store_count;    /* stores to addr >= phys_mem_size during handler */
	uword oob_store_last_addr;  /* last out-of-bounds store address */
} pf_diag = {0};

/* Runtime diagnostic toggle — controlled via OSD "CPU Debug" setting.
 * When false, all diagnostic dolog output is suppressed. */
static bool cpu_diag_enabled = false;
/* Win3.x compatibility:
 * route VM86 INT n at IOPL=3 through IDT (historical emulator behavior)
 * instead of direct IVT dispatch. */
static bool vm86_iopl3_softint_via_idt = true;
/* Route VM86+IOPL=3 software INTs through IDT so VMM can consistently
 * mediate/virtualize services (Bochs/QEMU-compatible behavior). */
static inline bool vm86_iopl3_int_uses_idt(int intno, uint16_t ax)
{
	(void)ax;
	(void)intno;
	return true;
}
#define diaglog(...) do { if (cpu_diag_enabled) dolog(__VA_ARGS__); } while(0)

/* Per-generation, rate-limited diagnostic events for heavy CPU-debug traces. */
#define CPU_DIAG_EVENT_LIMIT 8
static struct {
	int gen;
	uint8_t vm86_int_iopl;
	uint8_t vm86_gp_decode;
	uint8_t vm86_ud_decode;
	uint8_t swint_dpl;
	uint8_t task_gate;
	uint8_t vm86_isr_entry;
	uint8_t iret_to_vm86;
	uint8_t setseg_gap;
	uint8_t hlt_exec;
	uint8_t halt_irq_wake;
} cpu_diag_events = { .gen = -1 };

/* Last VM86 trap seen before returning to DOS; helps correlate Win386 aborts. */
static struct {
	int gen;
	bool valid;
	uint8_t excno;
	uint16_t cs;
	uword ip;
	uword fl;
	uword err;
	char decoded[80];
} vm86_last_fault = { .gen = -1 };

/* Keep a tail of VM86 #GP/#UD traps for post-mortem on Win386 abort. */
#define VM86_FAULT_RING_SIZE 96
static struct {
	int gen;
	uint16_t count;
	uint16_t pos;
	struct {
		uint8_t excno;
		uint16_t cs;
		uint16_t ds;
		uint16_t es;
		uint16_t ss;
		uword ip;
		uword sp;
		uword fl;
		uword err;
		char decoded[40];
	} ring[VM86_FAULT_RING_SIZE];
} vm86_fault_ring = { .gen = -1 };

/* Keep a tail of actual VM86 IRET executions (IOPL=3 path). */
#define VM86_IRET_RING_SIZE 96
static struct {
	int gen;
	uint16_t count;
	uint16_t pos;
	struct {
		uint16_t from_cs;
		uint16_t from_ss;
		uint16_t to_cs;
		uword from_ip;
		uword from_sp;
		uword to_ip;
		uword to_fl;
		uint8_t opsz16;
	} ring[VM86_IRET_RING_SIZE];
} vm86_iret_ring = { .gen = -1 };

/* Dense VM86 tripwire ring: one compact block dumped on first fatal VM86 fault. */
#define VM86_TRIP_RING_SIZE 64
enum {
	VM86_TRIP_E = 'E', /* VM86 exception entry */
	VM86_TRIP_R = 'R', /* ring0 IRET -> VM86 */
	VM86_TRIP_V = 'V', /* VM86 -> ring0 gate entry */
	VM86_TRIP_I = 'I', /* I/O bitmap deny */
};
enum {
	VM86_TRIP_IO_BAD_TR = 1,
	VM86_TRIP_IO_BAD_TR_LIMIT = 2,
	VM86_TRIP_IO_BAD_IOBASE = 3,
	VM86_TRIP_IO_BAD_BYTE_OOR = 4,
	VM86_TRIP_IO_BAD_BIT = 5,
};
static struct {
	int gen;
	uint16_t count;
	uint16_t pos;
	bool frozen;
	bool has_first_bad;
	uint8_t first_bad_exc;
	uint16_t first_bad_cs;
	uint32_t first_bad_ip;
	char first_bad_decoded[24];
	struct {
		uint8_t type;   /* VM86_TRIP_* */
		uint8_t v0;     /* exc/vec/width/reason */
		uint8_t blen;
		uint16_t cs, ss;
		uint16_t cs2, ss2;
		uint32_t ip, sp;
		uint32_t fl;
		uint32_t a, b;
		uint8_t bytes[8];
	} ring[VM86_TRIP_RING_SIZE];
} vm86_trip = { .gen = -1 };

/* Focused Win386 diagnostics: enable narrow traces only while win386.exe runs. */
static struct {
	int gen;
	bool active;
	uint16_t code_cs;
	uint16_t int21_count;
	uint16_t int2f_count;
	uint16_t pm_exc_count;
} win386_diag = { .gen = -1 };

/* WfW 3.11 boot monitor: condensed diagnostics collected while win386 is active.
 * All counters gated behind cpu_diag_enabled && win386_diag.active — zero overhead
 * when not running Windows with debug enabled. Dumps ~20 line summary on exit. */
#define WFW_FLOW_RING_SIZE 96
static struct {
	/* Counters */
	uint32_t vm86_ints;        /* Total VM86 interrupts delivered */
	uint32_t pm_ints;          /* Total PM interrupts delivered */
	uint32_t mode_switches;    /* CR0 PE/PG transitions */
	struct {
		uint16_t cs;
		uint32_t ip;
		uint32_t old_cr0;
		uint32_t new_cr0;
	} cr0_tail[8];
	uint8_t cr0_tail_pos;
	uint8_t cr0_tail_count;
	uint32_t v86_to_ring0;     /* VM86->ring0 ISR entries */
	uint32_t isr_failures;     /* call_isr() returned false */
	uint32_t exc_counts[32];   /* Per-exception counters (0-31) */

	/* INT 10h VGA mode tracking */
	uint8_t last_int10h_ah;
	uint8_t last_int10h_al;
	uint16_t int10h_count;
	/* Capture teletype output (AH=0Eh) — the error message */
	char tty_buf[80];
	uint8_t tty_pos;

	/* INT 15h tracking */
	uint16_t int15h_count;
	uint8_t last_int15h_ah;

	/* A20 state */
	uint8_t a20_state;
	uint16_t a20_toggles;

	/* Last fatal context */
	uint16_t fatal_cs, fatal_ds, fatal_es, fatal_ss;
	uint32_t fatal_eip, fatal_esp, fatal_efl;
	uint8_t fatal_cpl;
	uint8_t fatal_vm;
	uint8_t fatal_excno;
	uint8_t fatal_ext;
	uint8_t fatal_pusherr;
	bool has_fatal;
	char fatal_decoded[24];
	uint8_t fatal_bytes[8];
	uint8_t fatal_blen;
	/* Software INT vectors < 20h (tracked separately from exceptions). */
	uint32_t lowvec_softint_total;
	uint16_t lowvec_softint_hist[32];
	/* Unified dense event flow (single-block causal timeline). */
	struct {
		uint8_t tag;    /* 'I' int, 'E' exc, 'D' dpl reject, 'H' compat hle, 'L' lowvec, 'R' return, 'A' abort-site */
		uint8_t mode;   /* 'R' real, 'P' protected, 'V' vm86 */
		uint8_t vec;    /* vector/exception */
		uint8_t cpl;
		uint16_t cs, ds;
		uint32_t ip;
		uint16_t ax, bx, cx, dx;
		uint32_t extra; /* tag-specific payload (see flow dump) */
	} flow_ring[WFW_FLOW_RING_SIZE];
	uint16_t flow_pos;
	uint16_t flow_count;

	/* V86 INT #GP histogram: which INT vectors cause #GP in V86 (IOPL<3) */
	uint16_t v86_int_gp_counts[256];

	/* Ring-3 PM tracking: nonzero means DPMI client entered PM */
	uint32_t pm_ring3_ints;
	uint32_t pm_ring3_irqs;         /* Hardware IRQs (exc >= 32) in ring-3 */
	uint32_t pm_ring3_faults;       /* CPU exceptions (exc < 32) in ring-3 */
	uint16_t pm_ring3_exc_hist[32]; /* Per-exception counter for ring-3 */
	uint16_t pm_r3_last_cs;         /* Last ring-3 fault context */
	uint32_t pm_r3_last_eip;
	uint8_t pm_r3_last_exc;
	/* Ring-3 #GP capture: first few ring-3 #GPs with CS:EIP */
	struct { uint16_t cs; uint32_t eip; uint32_t err; char dec[24]; } r3_gp_log[8];
	uint8_t r3_gp_count;
	struct { uint16_t cs; uint32_t eip; uint32_t err; uint32_t cr2; char dec[24]; } r3_pf_log[8];
	uint8_t r3_pf_count;
	/* Ring-3 high-address #PF page table snapshot (CR2 >= 0x80000000).
	 * Ring buffer — last 16 entries, so the failing PF is always captured. */
	struct {
		uint32_t cr2;
		uint32_t err;
		uint16_t cs;
		uint32_t eip;
		uint32_t cr3;
		uint32_t pde;    /* Raw page directory entry */
		uint32_t pte;    /* Raw page table entry (0 if PDE not present) */
	} r3_hipf[16];
	uint8_t r3_hipf_pos;
	uint8_t r3_hipf_count;
	/* PM ring-3 INT 21h/31h tracking (ring buffers for last-N) */
	uint16_t pm_r3_int21_count;
	uint8_t pm_r3_int21_ah_ring[16];  /* ring buffer of AH values */
	uint16_t pm_r3_int31_count;
	uint16_t pm_r3_int31_ax_ring[16]; /* ring buffer of AX values */
	/* File open tracking: capture filenames for AH=3Dh */
	struct { char name[32]; uint8_t result_ah; bool failed; } r3_fopen_log[8];
	uint8_t r3_fopen_count;

	/* BIOS privileged instruction detection */
	uint16_t bios_lgdt_count;	/* LGDT in V86 at f000/c000 */
	uint16_t bios_priv_count;	/* Other privileged insns in V86 BIOS */

	/* V86 INT 21h subfunction tracking */
	uint8_t v86_int21_ah_log[16];   /* First 16 INT 21h AH values */
	uint8_t v86_int21_ah_count;

	/* V86 IOPL=3 INT tracking (softints through IVT) */
	uint32_t v86_softint_total;	/* Total V86 INTs via IOPL=3 IVT path */
	uint16_t v86_int2f_count;	/* INT 2Fh calls in V86 */
	uint16_t v86_int2f_ax_hist[8];	/* Last 8 unique AX values for INT 2Fh */
	uint8_t v86_int2f_hist_pos;
	uint16_t v86_int21_count;	/* INT 21h calls in V86 */
	/* VM86 INT 13h request/return correlation (disk service health). */
	uint16_t v86_int13_count;
	uint16_t v86_int13_ret_count;
	uint16_t v86_int13_fail_count;
	uint8_t v86_int13_last_path; /* 1=IVT softint, 2=IDT-compat */
	bool pending_v86_int13;
	uint16_t pending_v86_int13_ret_cs;
	uint32_t pending_v86_int13_ret_ip;
	uint16_t v86_int13_req_ax, v86_int13_req_bx, v86_int13_req_cx, v86_int13_req_dx, v86_int13_req_es;
	uint16_t v86_int13_ret_ax, v86_int13_ret_bx, v86_int13_ret_cx, v86_int13_ret_dx, v86_int13_ret_es;
	uint8_t v86_int13_ret_cf;
	/* First VM86 INT events: compact path tuples for quick boot diffing */
	struct {
		uint8_t vec;   /* interrupt vector */
		uint8_t path;  /* 0=IOPL<3(#GP), 1=IOPL=3 softINT */
		uint8_t iopl;  /* IOPL at INT execution */
		uint16_t ax;   /* AX at INT execution */
		uint16_t cs;   /* CS at INT execution */
		uint16_t ip;   /* IP at INT execution */
	} vm86_int_evt[8];
	uint8_t vm86_int_evt_count;
	/* IVT[2Fh] snapshot at trace start (to detect if VMM patches it) */
	uint32_t ivt_2f_at_start;

	/* IOPL transition tracking: captures first PM→V86 IRET with IOPL=3 */
	bool iopl3_captured;        /* Already captured */
	uint32_t iopl3_iret_count;  /* PM→V86 IRETs before first IOPL=3 */
	uint16_t iopl3_from_cs;     /* Ring-0 CS performing the IRET */
	uint32_t iopl3_from_eip;    /* Ring-0 EIP performing the IRET */
	uint16_t iopl3_to_cs;       /* V86 target CS */
	uint32_t iopl3_to_ip;       /* V86 target IP */
	uint32_t iopl3_eflags;      /* Full EFLAGS being loaded */
	uint32_t iopl3_ivt_2f;      /* IVT[2Fh] at the moment of transition */
	/* First fault seen after first IOPL=3 VM86 entry */
	bool post_iopl3_fault_captured;
	uint8_t post_iopl3_excno;
	uint16_t post_iopl3_cs;
	uint32_t post_iopl3_eip;
	uint32_t post_iopl3_fl;
	uint32_t post_iopl3_err;
	int8_t post_iopl3_last_evt; /* vm86_int_evt index, -1 if none */
	char post_iopl3_decoded[24];
	uint8_t post_iopl3_bytes[8];
	uint8_t post_iopl3_blen;
	/* Hot VM86 BIOS fault PCs (recurring #GP/#UD) */
	struct {
		uint16_t cs;
		uint16_t ip;
		uint8_t exc; /* EX_GP/EX_UD */
		uint16_t count;
		uint16_t resume_same;  /* IRET returned to same CS:IP (reflected) */
		uint16_t resume_other; /* IRET returned elsewhere (emulated/advanced) */
		char decoded[24];
		uint8_t bytes[8];
		uint8_t blen;
	} bios_hot[6];
	uint8_t bios_hot_count;
	int8_t pending_bios_fault_idx; /* set on VM86 BIOS fault, consumed on VM86 IRET */
	/* Focused VM86 #UD opcode 63h tracker (ARPL callback loop signal). */
	struct {
		uint16_t cs;
		uint16_t ip;
		uint16_t count;
		uint16_t resume_same;  /* reflected: resumed at same CS:IP */
		uint16_t resume_next4; /* advanced to CS:IP+4 (after ARPL trap opcode) */
		uint16_t resume_next8; /* advanced to CS:IP+8 (past TEST/Jcc pair) */
		uint16_t resume_other; /* advanced/emulated elsewhere */
		uint16_t out_ax;       /* AX at resume (last observed) */
		uint32_t out_fl;       /* FLAGS at resume (last observed) */
		uint16_t cf_set;       /* Resume returns with CF=1 */
		uint16_t ax, bx, cx, dx, si, di;
		uint16_t ss;
		uint32_t sp;
		uint32_t fl;
		uint8_t bytes[8];
		uint8_t blen;
	} ud63_hot[6];
	uint8_t ud63_hot_count;
	int8_t pending_ud63_idx;
	uint16_t pending_ud63_cs;
	uint32_t pending_ud63_ip;
	/* Focused ARPL resize probe: #UD opcode 63h at FE95:1638 around sel 0127. */
	bool pending_ud63_fe95_0127;
	struct {
		uint16_t seen;       /* FE95:1638 #UD entries observed */
		uint16_t resumed;    /* IRET resumes observed */
		uint16_t desc_changed;
		uint16_t pre_cs, pre_ip;
		uint16_t post_cs, post_ip;
		uint16_t pre_ds;     /* DS at #UD entry */
		uint16_t post_ds;    /* DS at resume-note time */
		uint16_t resume_ds;  /* target DS on IRET path when available */
		uint16_t pre_ax, post_ax;
		uint32_t pre_fl, post_fl;
		uint32_t pre_w1, pre_w2;
		uint32_t post_w1, post_w2;
		uint32_t pre_near_w1[4], pre_near_w2[4];   /* 011c/0124/0127/012c */
		uint32_t post_near_w1[4], post_near_w2[4];
		uint8_t pre_near_ok, post_near_ok;         /* bitmask */
		uint16_t near_changed[4];                  /* per-slot pre->post changes */
		/* First-callback ring-0 micro-trace (FE95:1638 -> ring0 -> IRET). */
		bool trace_active;
		bool trace_done;
		uint16_t trace_entry_ax;
		uint16_t trace_prev_ax;
		bool trace_ax5_valid;
		uint16_t trace_ax5_prev;
		uint16_t trace_ax5_ax;
		uint16_t trace_ax5_cs, trace_ax5_ip;
		/* Path probes across FE95 callback sessions (not first-trace only). */
		uint16_t ret_ax5;
		uint16_t ret_ax_other;
		uint16_t ret_ax_last;
		/* 0028:6ca7 -> test byte [bp+2e],2 */
		uint16_t t6ca7_count;
		uint16_t t6ca7_bit_set;
		uint16_t t6ca7_bit_clear;
		uint8_t t6ca7_last_byte;
		uint16_t t6ca7_last_ss, t6ca7_last_bp;
		/* 0028:6cab -> je rel8 (branch outcome observed at next step). */
		bool j6cab_pending;
		uint16_t j6cab_count;
		uint16_t j6cab_taken;
		uint16_t j6cab_not_taken;
		uint16_t j6cab_other;
		uint16_t j6cab_last_from_cs, j6cab_last_from_ip;
		uint16_t j6cab_last_to_cs, j6cab_last_to_ip;
		uint16_t j6cab_last_tgt, j6cab_last_fall;
		uint8_t j6cab_last_zf;
		/* AX transition tracker: first point where callback path reaches AX=0005. */
		bool step_prev_valid;
		uint16_t step_prev_cs, step_prev_ip, step_prev_ax;
		uint8_t step_prev_bytes[8];
		uint8_t step_prev_blen;
		uint16_t ax5_transitions;
		uint16_t ax5_from_cs, ax5_from_ip, ax5_from_ax;
		uint16_t ax5_to_cs, ax5_to_ip, ax5_to_ax;
		uint8_t ax5_from_bytes[8];
		uint8_t ax5_from_blen;
		struct {
			uint16_t cs, ip;
			uint16_t ax, bx, cx, dx, si, di, bp, sp;
			uint32_t fl;
			uint8_t bytes[8];
			uint8_t blen;
		} trace[128];
		uint8_t trace_count;
	} ud63_fe95_0127;
	/* Software-INT DPL reject tracking (PM #GP from INT n with DPL<CPL). */
	struct {
		uint8_t vec;
		uint16_t count;
		uint16_t first_cs;
		uint32_t first_ip;
		uint16_t first_err;
		uint16_t resume_same; /* returned to faulting CS:IP */
		uint16_t resume_next; /* returned to CS:next_ip (emulated INT) */
		uint16_t resume_other;
		uint16_t in_ax, in_bx;
		uint16_t out_ax, out_bx;
		uint32_t out_fl;
		uint16_t cf_set;
	} swint_dpl[6];
	uint8_t swint_dpl_count;
	struct {
		uint8_t vec;
		uint8_t mode;
		uint8_t cpl;
		uint8_t rcls;      /* 1=same 2=next 3=other */
		uint8_t cf;
		uint16_t in_ax, in_bx;
		uint16_t out_ax, out_bx;
		uint16_t in_cs, out_cs;
		uint32_t in_ip, out_ip;
	} dpl_tail[16];
	uint8_t dpl_tail_pos;
	uint8_t dpl_tail_count;
	int8_t pending_swint_dpl_idx;
	uint16_t pending_swint_cs;
	uint32_t pending_swint_ip;
	uint32_t pending_swint_next_ip;

	/* INT 31h first-16 capture (init sequence) */
	uint16_t pm_r3_int31_ax_first[16];
	uint8_t pm_r3_int31_first_count;
	/* INT 31h histogram (compact: {ax, count} pairs) */
	struct { uint16_t ax; uint16_t count; } int31_hist[24];
	uint8_t int31_hist_len;
	/* DPMI/INT21h failure detection via IRET CF check */
	bool pending_dpmi;
	uint16_t pending_dpmi_ax;
	bool pending_int21;
	uint8_t pending_int21_ah;
	/* DPMI failure log (INT 31h returned CF=1) */
	struct { uint16_t ax; uint32_t ret_eip; } int31_fail_log[8];
	uint8_t int31_fail_count;
	/* INT 21h failure log (returned CF=1) */
	struct { uint8_t ah; uint32_t ret_eip; } int21_fail_log[8];
	uint8_t int21_fail_count;
	/* INT 21h first-16 capture */
	uint8_t pm_r3_int21_ah_first[16];
	uint8_t pm_r3_int21_first_count;
	/* INT 31h/0006 selector capture: BX values for first 16 Get Segment Base calls */
	uint16_t int31_0006_bx[16];
	uint8_t int31_0006_count;
	/* Initial PM ring-3 register state (first ring-3 event) */
	bool r3_first_captured;
	uint16_t r3_first_cs, r3_first_ds, r3_first_es, r3_first_ss;
	uint32_t r3_first_eip, r3_first_esp, r3_first_eax;
	/* PSP command tail from DS:0080h and environment from PSP:002Ch */
	uint32_t psp_ds_base;    /* Linear address of DS (PSP) */
	uint8_t psp_header[16];  /* First 16 bytes at DS_base (PSP signature check) */
	char psp_cmdtail[128];   /* Command tail (up to 127 chars) */
	uint8_t psp_cmdtail_len;
	char psp_env[256];       /* First 256 bytes of environment block */
	uint16_t psp_env_seg;    /* PSP:2Ch value (selector or segment) */
	uint32_t psp_env_lin;    /* Resolved linear address of environment */
	bool psp_env_is_sel;     /* true if resolved via LDT lookup */
	/* DPMI return value capture: first 8 returns from INT 31h */
	struct { uint16_t call_ax; uint16_t ret_ax; uint16_t ret_cx; uint16_t ret_dx; bool cf; } int31_ret_log[8];
	uint8_t int31_ret_count;
	/* DPMI 0501/0503 memory block allocation log */
	struct {
		uint16_t func;       /* 0501=alloc, 0503=resize */
		uint32_t req_size;   /* BX:CX on call (requested size in bytes) */
		uint32_t ret_addr;   /* BX:CX on return (linear address) */
		uint32_t ret_handle; /* SI:DI on return (memory block handle) */
		bool cf;             /* CF on return */
	} dpmi_alloc_log[8];
	uint8_t dpmi_alloc_count;
	uint32_t pending_alloc_size; /* BX:CX at call time for pending 0501/0503 */
	/* DPMI descriptor-mutating function log (first 32 calls):
	 * 0001=AllocLDT, 0007=SetBase, 0008=SetLimit, 0009=SetRights,
	 * 000A=CreateAlias, 000B=GetDesc, 000C=SetDesc */
	struct {
		uint16_t func;   /* DPMI function code */
		uint16_t sel;    /* BX = selector (or AX return for 0001) */
		uint32_t val;    /* type-specific value (see dump formatter) */
		uint32_t extra;  /* type-specific extra (e.g. SetDesc w2) */
		uint8_t ret_cf;  /* CF at IRET return (0/1) */
		uint8_t ret_valid; /* IRET return captured for this call */
	} dpmi_seg_log[32];
	uint8_t dpmi_seg_log_count;
	/* INT 21h return value capture: first 8 returns */
	struct { uint8_t call_ah; uint16_t ret_ax; bool cf; } int21_ret_log[8];
	uint8_t int21_ret_count;
	/* INT 31h/0006 return values: last-16 ring buffer {selector, base} */
	struct { uint16_t bx; uint32_t base; } int31_0006_ret_ring[16];
	uint16_t int31_0006_ret_count;
	/* INT 31h/0006 last-16 selector ring buffer */
	uint16_t int31_0006_bx_ring[16];
	/* Track pending 0006 BX for IRET return capture */
	uint16_t pending_dpmi_bx;
	/* PM ring-3 INT 2Fh tracking */
	uint16_t pm_r3_int2f_count;
	uint16_t pm_r3_int2f_ax[16]; /* First 16 AX values */
	/* INT 2Fh/168Ah specific: DS:SI string capture */
	uint16_t pm_r3_168a_count; /* Count of AX=16xx calls */
	char pm_r3_168a_str[16]; /* DS:SI string from first 168Ah call */
	/* INT 2Fh return capture via IRET */
	bool pending_int2f;
	uint16_t pending_int2f_ax;
	/* INT 2Fh return values: first 4 returns with full register state */
	struct {
		uint16_t call_ax;   /* AX on entry */
		uint16_t ret_ax;    /* AX on return */
		uint16_t ret_ds;    /* DS on return (168Ah changes DS:SI) */
		uint16_t ret_si;    /* SI on return */
		uint16_t ret_es;    /* ES on return */
		uint16_t ret_di;    /* DI on return (1687h uses ES:DI) */
		bool cf;            /* CF on return */
	} int2f_ret_log[4];
	uint8_t int2f_ret_count;
	/* Tail of PM ring-3 software-INT returns (in->out), newest last. */
	struct {
		uint8_t vec;         /* 21h / 31h / 2Fh */
		uint16_t in_ax;      /* AX at call site */
		uint16_t out_ax;     /* AX at return */
		uint16_t out_bx;
		uint16_t out_cx;
		uint16_t out_dx;
		uint16_t out_ds;
		uint16_t out_cs;     /* return target CS (newcs) */
		uint32_t out_ip;     /* return target EIP (newip) */
		uint8_t cf;          /* CF at return */
	} r3_ret_tail[16];
	uint8_t r3_ret_tail_pos;
	uint8_t r3_ret_tail_count;
	/* Compact abort trail: last INT21/INT2F call sites before WIN386 exit. */
	struct {
		uint8_t vec;   /* 21h or 2Fh */
		uint8_t mode;  /* 'R' real, 'P' protected, 'V' vm86 */
		uint8_t cpl;
		uint8_t ah;
		uint16_t cs;
		uint32_t ip;
		uint16_t ds;
		uint16_t ax, bx, cx, dx;
	} abort_tail[16];
	uint8_t abort_tail_pos;
	uint8_t abort_tail_count;
	/* Sliding window base addresses from SetSegBase(0x100F) */
	uint32_t seg100f_bases[8];    /* Unique base values seen */
	uint8_t seg100f_base_count;
	/* Full register snapshot for the last ring-3 PF with PTE=0 (unmapped) */
	struct {
		uint32_t cr2;
		uint16_t cs;
		uint32_t eip;
		uint32_t eax, ebx, ecx, edx, esi, edi, ebp, esp;
		uint16_t ds, es, ss, fs, gs;
		uint32_t ds_base, es_base, ss_base;
		uint32_t ds_limit, es_limit, cs_limit;
	} fatal_pf;
	bool fatal_pf_captured;
	/* Last PM ring-3 segcheck reject (architectural #GP/#SS source). */
	struct {
		bool valid;
		uint16_t cs;
		uint32_t eip;
		uint8_t seg;      /* SEG_* */
		uint8_t why;      /* 1=write-protect, 2=limit */
		uint8_t rwm;      /* translate access mode */
		uint8_t size;     /* bytes */
		uint16_t sel;
		uint16_t flags;
		uint32_t limit;
		uint32_t addr;    /* offset within segment */
		/* Context snapshot for DS-limit investigations at fault site. */
		uint16_t bx, di, bp;
		uint8_t bl;
		bool ds_word_ok;
		uint16_t ds_word; /* word at SEG:addr via linear read (if mapped) */
		bool ss_bp_m6_ok;
		uint16_t ss_bp_m6; /* word at SS:[BP-6] via linear read (if mapped) */
	} segchk_last;
	/* INT 41h module identification (LOADMODULE/LOADDLL notifications) */
	struct {
		uint16_t hmod;       /* Module handle (selector) */
		uint16_t ax;         /* INT 41h function (005Ah or 005Ch) */
		uint16_t call_cs;    /* CS of the INT 41h call */
		char name[16];       /* NE module name */
	} int41_modules[8];
	uint8_t int41_module_count;
	/* Selector 0x0127 provenance: snapshots when DS is loaded with 0x0127 */
	struct {
		uint8_t count;       /* Number of DS←0127 loads seen */
		/* Raw descriptor words at first load + at segcheck trigger */
		uint32_t first_w1, first_w2;   /* 0x0127 descriptor at first DS load */
		uint32_t first_adj_w1, first_adj_w2; /* 0x0117 descriptor at same time */
		uint16_t first_cs;
		uint32_t first_eip;
		uint32_t last_w1, last_w2;     /* 0x0127 descriptor at last DS load */
		uint32_t last_adj_w1, last_adj_w2; /* 0x0117 descriptor at same time */
		uint16_t last_cs;
		uint32_t last_eip;
	} sel0127_prov;
	/* Instruction bytes at segcheck fault site */
	uint8_t segchk_insn_bytes[16];
	uint8_t segchk_insn_len;    /* Number of bytes captured (up to 16) */
	bool segchk_insn_captured;
	/* LDT write watch around selector 0x0127:
	 * watch window = LDT offsets 0x118..0x12f (selectors 0x0117..0x0127). */
	struct {
		uint16_t cs;
		uint32_t eip;
		uint8_t cpl;
		uint8_t size;
		uint32_t phys;      /* physical write address (first overlapping byte) */
		uint32_t ldt_off;   /* linear offset from LDT base */
		uint16_t sel;       /* selector corresponding to (ldt_off & ~7), TI=1 */
		uint32_t val;       /* raw write value (low size bytes significant) */
	} ldt_watch_log[24];
	uint16_t ldt_watch_pos;
	uint8_t ldt_watch_count;
	/* DS=0127 load source trace (how selector value reached DS). */
	struct {
		uint16_t cs;
		uint32_t eip;
		uint8_t src_kind;   /* 1=mov_ds_reg 2=mov_ds_mem 3=pop_ds 4=lds 5=other */
		uint8_t src_reg;    /* GPR index if src_kind==1, else 0xff */
		uint16_t src_val;   /* source value if known */
		uint16_t bx, di, bp;
		uint16_t ss;
		uint32_t sp;
		bool bp_m6_ok;
		uint16_t bp_m6_val; /* SS:[BP-6] if source was mov ds,[bp-6] */
		uint8_t op;
		uint8_t modrm;
		uint8_t blen;
		uint8_t bytes[8];
	} ds0127_load_log[8];
	uint8_t ds0127_load_count;
} wfw_monitor;

/* One-shot DPMI trace: captures INT 2Fh/AX=1687h return value and entry point */
static struct {
	bool ret_active;	/* Waiting for return from 1687h handler */
	uint16_t ret_cs;	/* Return CS:IP (instruction after INT 2Fh) */
	uint32_t ret_ip;
	bool entry_active;	/* Watching for DPMI entry point call */
	uint16_t entry_cs;	/* Entry point ES:DI from 1687h return */
	uint16_t entry_ip;
	bool entry_called;	/* Entry point was reached */
	uint16_t entry_ax;	/* AX when entry point called */
	uint16_t entry_es;	/* ES when entry point called */
	/* IVT data */
	uint16_t ivt_phys_cs;	/* IVT[2Fh] from physical memory */
	uint16_t ivt_phys_ip;
	uint16_t ivt_paged_cs;	/* IVT[2Fh] as read through paging */
	uint16_t ivt_paged_ip;
	uint8_t target_first_byte;	/* First byte at handler (through paging) */
	uint8_t target_phys_byte;	/* First byte at handler (physical memory) */
	bool ivt_mismatch;	/* Paged != physical */
	/* Return values */
	bool returned;
	uint16_t ret_ax;
	uint16_t ret_bx;
	uint16_t ret_dx;
	uint16_t ret_si;
	uint16_t ret_es;
	uint16_t ret_di;
} dpmi_trace;

static inline void cpu_diag_events_sync(CPUI386 *cpu)
{
	if (cpu_diag_events.gen != cpu->diag_gen) {
		memset(&cpu_diag_events, 0, sizeof(cpu_diag_events));
		cpu_diag_events.gen = cpu->diag_gen;
	}
}

static inline bool cpu_diag_event_allow(CPUI386 *cpu, uint8_t *slot)
{
	cpu_diag_events_sync(cpu);
	if (*slot >= CPU_DIAG_EVENT_LIMIT)
		return false;
	(*slot)++;
	return true;
}

static inline void cpu_diag_win386_sync(CPUI386 *cpu)
{
	if (win386_diag.gen != cpu->diag_gen) {
		memset(&win386_diag, 0, sizeof(win386_diag));
		win386_diag.gen = cpu->diag_gen;
	}
}

static inline void vm86_trip_sync(CPUI386 *cpu)
{
	if (vm86_trip.gen != cpu->diag_gen) {
		memset(&vm86_trip, 0, sizeof(vm86_trip));
		vm86_trip.gen = cpu->diag_gen;
	}
}

static inline bool vm86_trip_enabled(CPUI386 *cpu)
{
	if (!cpu_diag_enabled)
		return false;
	cpu_diag_win386_sync(cpu);
	if (!win386_diag.active)
		return false;
	vm86_trip_sync(cpu);
	return true;
}

static inline bool vm86_trip_silence_live_logs(CPUI386 *cpu)
{
	return vm86_trip_enabled(cpu) && !vm86_trip.frozen;
}

static inline void vm86_trip_push(CPUI386 *cpu, uint8_t type, uint8_t v0,
				  uint16_t cs, uint32_t ip, uint16_t ss, uint32_t sp,
				  uint32_t fl, uint16_t cs2, uint32_t a, uint16_t ss2,
				  uint32_t b, const uint8_t *bytes, uint8_t blen)
{
	int idx;
	if (!vm86_trip_enabled(cpu) || vm86_trip.frozen)
		return;
	idx = vm86_trip.pos % VM86_TRIP_RING_SIZE;
	vm86_trip.ring[idx].type = type;
	vm86_trip.ring[idx].v0 = v0;
	vm86_trip.ring[idx].cs = cs;
	vm86_trip.ring[idx].ip = ip;
	vm86_trip.ring[idx].ss = ss;
	vm86_trip.ring[idx].sp = sp;
	vm86_trip.ring[idx].fl = fl;
	vm86_trip.ring[idx].cs2 = cs2;
	vm86_trip.ring[idx].ss2 = ss2;
	vm86_trip.ring[idx].a = a;
	vm86_trip.ring[idx].b = b;
	if (bytes && blen > 0) {
		if (blen > 8) blen = 8;
		memcpy(vm86_trip.ring[idx].bytes, bytes, blen);
		vm86_trip.ring[idx].blen = blen;
	} else {
		vm86_trip.ring[idx].blen = 0;
	}
	vm86_trip.pos++;
	if (vm86_trip.count < VM86_TRIP_RING_SIZE)
		vm86_trip.count++;
}

static void vm86_trip_dump(CPUI386 *cpu, const char *reason)
{
	int count, start;
	if (!vm86_trip_enabled(cpu))
		return;
	count = vm86_trip.count;
	start = vm86_trip.pos - count;
	dolog("=== VM86 Tripwire (%s) ===\n", reason);
	if (vm86_trip.has_first_bad) {
		dolog("tripwire: first_bad=#%02x@%04x:%08x dec=%s n=%d frozen=%u\n",
			vm86_trip.first_bad_exc, vm86_trip.first_bad_cs,
			vm86_trip.first_bad_ip, vm86_trip.first_bad_decoded,
			count, vm86_trip.frozen ? 1u : 0u);
	} else {
		dolog("tripwire: first_bad=<none> n=%d frozen=%u\n",
			count, vm86_trip.frozen ? 1u : 0u);
	}
	dolog("trip_evt:");
	for (int i = start; i < vm86_trip.pos; i++) {
		int j = i % VM86_TRIP_RING_SIZE;
		if (i != start && ((i - start) % 4) == 0)
			dolog("\n  ");
		switch (vm86_trip.ring[j].type) {
		case VM86_TRIP_E:
			dolog(" [E%02x %04x:%04x ss:%04x:%04x f=%08x e=%08x",
				vm86_trip.ring[j].v0,
				vm86_trip.ring[j].cs, vm86_trip.ring[j].ip & 0xffff,
				vm86_trip.ring[j].ss, vm86_trip.ring[j].sp & 0xffff,
				vm86_trip.ring[j].fl, vm86_trip.ring[j].a);
			if (vm86_trip.ring[j].blen > 0) {
				dolog(" b=");
				for (int k = 0; k < vm86_trip.ring[j].blen; k++)
					dolog("%02x", vm86_trip.ring[j].bytes[k]);
			}
			dolog("]");
			break;
		case VM86_TRIP_R:
			dolog(" [R %04x:%08x->%04x:%08x fl=%08x vmss:%04x vmsp:%08x es:%04x ds:%04x fs:%04x gs:%04x]",
				vm86_trip.ring[j].cs, vm86_trip.ring[j].ip,
				vm86_trip.ring[j].cs2, vm86_trip.ring[j].a,
				vm86_trip.ring[j].fl,
				vm86_trip.ring[j].ss, vm86_trip.ring[j].sp,
				vm86_trip.ring[j].bytes[0] | ((uint16_t)vm86_trip.ring[j].bytes[1] << 8),
				vm86_trip.ring[j].bytes[2] | ((uint16_t)vm86_trip.ring[j].bytes[3] << 8),
				vm86_trip.ring[j].bytes[4] | ((uint16_t)vm86_trip.ring[j].bytes[5] << 8),
				vm86_trip.ring[j].bytes[6] | ((uint16_t)vm86_trip.ring[j].bytes[7] << 8));
			break;
		case VM86_TRIP_V:
			dolog(" [V%02x %04x:%04x ss:%04x:%04x -> %04x:%08x r0:%04x:%08x]",
				vm86_trip.ring[j].v0,
				vm86_trip.ring[j].cs, vm86_trip.ring[j].ip & 0xffff,
				vm86_trip.ring[j].ss, vm86_trip.ring[j].sp & 0xffff,
				vm86_trip.ring[j].cs2, vm86_trip.ring[j].a,
				vm86_trip.ring[j].ss2, vm86_trip.ring[j].b);
			break;
		case VM86_TRIP_I:
			dolog(" [I p=%04x w=%u tr=%04x iob=%04x bo=%05x bv=%02x bit=%u r=%u at=%04x:%04x]",
				(uint16_t)vm86_trip.ring[j].sp,
				vm86_trip.ring[j].v0,
				vm86_trip.ring[j].cs,
				vm86_trip.ring[j].ss,
				vm86_trip.ring[j].a,
				(uint8_t)(vm86_trip.ring[j].b >> 8),
				(uint8_t)(vm86_trip.ring[j].b & 0xff),
				(uint8_t)(vm86_trip.ring[j].b >> 16),
				vm86_trip.ring[j].cs2,
				vm86_trip.ring[j].ip & 0xffff);
			break;
		default:
			dolog(" [?]");
			break;
		}
	}
	dolog("\n=== End VM86 Tripwire ===\n");
}

static inline void vm86_trip_mark_first_bad(CPUI386 *cpu, uint8_t exc, uint16_t cs,
					    uint32_t ip, const char *decoded)
{
	if (!vm86_trip_enabled(cpu) || vm86_trip.frozen)
		return;
	vm86_trip.frozen = true;
	vm86_trip.has_first_bad = true;
	vm86_trip.first_bad_exc = exc;
	vm86_trip.first_bad_cs = cs;
	vm86_trip.first_bad_ip = ip;
	snprintf(vm86_trip.first_bad_decoded, sizeof(vm86_trip.first_bad_decoded),
		"%s", decoded ? decoded : "");
	vm86_trip_dump(cpu, "first fatal");
}

static inline void cpu_diag_vm86_fault_ring_sync(CPUI386 *cpu)
{
	if (vm86_fault_ring.gen != cpu->diag_gen) {
		memset(&vm86_fault_ring, 0, sizeof(vm86_fault_ring));
		vm86_fault_ring.gen = cpu->diag_gen;
	}
}

static inline void cpu_diag_vm86_iret_ring_sync(CPUI386 *cpu)
{
	if (vm86_iret_ring.gen != cpu->diag_gen) {
		memset(&vm86_iret_ring, 0, sizeof(vm86_iret_ring));
		vm86_iret_ring.gen = cpu->diag_gen;
	}
}

static void cpu_diag_vm86_fault_ring_push(CPUI386 *cpu, int excno, bool pusherr, const char *decoded)
{
	uint16_t idx;
	cpu_diag_vm86_fault_ring_sync(cpu);
	idx = vm86_fault_ring.pos % VM86_FAULT_RING_SIZE;
	vm86_fault_ring.ring[idx].excno = (uint8_t)excno;
	vm86_fault_ring.ring[idx].cs = cpu->seg[SEG_CS].sel;
	vm86_fault_ring.ring[idx].ds = cpu->seg[SEG_DS].sel;
	vm86_fault_ring.ring[idx].es = cpu->seg[SEG_ES].sel;
	vm86_fault_ring.ring[idx].ss = cpu->seg[SEG_SS].sel;
	vm86_fault_ring.ring[idx].ip = cpu->ip;
	vm86_fault_ring.ring[idx].sp = REGi(4);
	vm86_fault_ring.ring[idx].fl = cpu->flags;
	vm86_fault_ring.ring[idx].err = pusherr ? cpu->excerr : 0;
	snprintf(vm86_fault_ring.ring[idx].decoded, sizeof(vm86_fault_ring.ring[idx].decoded),
		 "%s", decoded ? decoded : "?");
	vm86_fault_ring.pos++;
	if (vm86_fault_ring.count < VM86_FAULT_RING_SIZE)
		vm86_fault_ring.count++;
}

static void cpu_diag_vm86_iret_ring_push(CPUI386 *cpu,
					 uint16_t from_cs, uword from_ip,
					 uint16_t from_ss, uword from_sp,
					 uint16_t to_cs, uword to_ip,
					 uword to_fl, bool opsz16)
{
	uint16_t idx;
	cpu_diag_vm86_iret_ring_sync(cpu);
	idx = vm86_iret_ring.pos % VM86_IRET_RING_SIZE;
	vm86_iret_ring.ring[idx].from_cs = from_cs;
	vm86_iret_ring.ring[idx].from_ip = from_ip;
	vm86_iret_ring.ring[idx].from_ss = from_ss;
	vm86_iret_ring.ring[idx].from_sp = from_sp;
	vm86_iret_ring.ring[idx].to_cs = to_cs;
	vm86_iret_ring.ring[idx].to_ip = to_ip;
	vm86_iret_ring.ring[idx].to_fl = to_fl;
	vm86_iret_ring.ring[idx].opsz16 = opsz16 ? 1 : 0;
	vm86_iret_ring.pos++;
	if (vm86_iret_ring.count < VM86_IRET_RING_SIZE)
		vm86_iret_ring.count++;
}

static void cpu_diag_vm86_fault_ring_dump_tail(int n)
{
	int count = vm86_fault_ring.count;
	if (count <= 0)
		return;
	if (n > count)
		n = count;
	dolog("=== VM86 fault tail (%d of %d) ===\n", n, count);
	for (int i = count - n; i < count; i++) {
		int idx = (vm86_fault_ring.pos - count + i) % VM86_FAULT_RING_SIZE;
		if (idx < 0)
			idx += VM86_FAULT_RING_SIZE;
		dolog("  #%d exc=%02x %04x:%08x DS=%04x ES=%04x SS:ESP=%04x:%08x FL=%08x err=%08x %s\n",
			i + 1,
			vm86_fault_ring.ring[idx].excno,
			vm86_fault_ring.ring[idx].cs,
			vm86_fault_ring.ring[idx].ip,
			vm86_fault_ring.ring[idx].ds,
			vm86_fault_ring.ring[idx].es,
			vm86_fault_ring.ring[idx].ss,
			vm86_fault_ring.ring[idx].sp,
			vm86_fault_ring.ring[idx].fl,
			vm86_fault_ring.ring[idx].err,
			vm86_fault_ring.ring[idx].decoded);
	}
}

static void cpu_diag_vm86_iret_ring_dump_tail(int n)
{
	int count = vm86_iret_ring.count;
	if (count <= 0)
		return;
	if (n > count)
		n = count;
	dolog("=== VM86 IRET tail (%d of %d) ===\n", n, count);
	for (int i = count - n; i < count; i++) {
		int idx = (vm86_iret_ring.pos - count + i) % VM86_IRET_RING_SIZE;
		if (idx < 0)
			idx += VM86_IRET_RING_SIZE;
		dolog("  #%d %s from %04x:%08x SS:ESP=%04x:%08x -> %04x:%08x FL=%08x\n",
			i + 1,
			vm86_iret_ring.ring[idx].opsz16 ? "IRET16" : "IRET32",
			vm86_iret_ring.ring[idx].from_cs,
			vm86_iret_ring.ring[idx].from_ip,
			vm86_iret_ring.ring[idx].from_ss,
			vm86_iret_ring.ring[idx].from_sp,
			vm86_iret_ring.ring[idx].to_cs,
			vm86_iret_ring.ring[idx].to_ip,
			vm86_iret_ring.ring[idx].to_fl);
	}
}

static inline uint8_t cpu_diag_ascii_tolower(uint8_t c)
{
	if (c >= 'A' && c <= 'Z')
		return (uint8_t)(c + ('a' - 'A'));
	return c;
}

static bool cpu_diag_str_has_win386(const char *s)
{
	static const char needle[] = "win386.exe";
	size_t nlen = sizeof(needle) - 1;
	if (!s)
		return false;
	for (size_t i = 0; s[i]; i++) {
		size_t j = 0;
		while (j < nlen && s[i + j] &&
		       cpu_diag_ascii_tolower((uint8_t)s[i + j]) == (uint8_t)needle[j])
			j++;
		if (j == nlen)
			return true;
	}
	return false;
}

/* Captures where the most recent architectural HLT executed. */
static struct {
	int gen;
	uint32_t count;
	uint16_t cs;
	uword ip;
	uint16_t ss;
	uword sp;
	uword fl;
	int cpl;
	bool vm;
	uint32_t wake_count;
	int last_wake_vec;
	uword last_wake_eip;
	uint16_t last_wake_cs;
	uint16_t last_wake_ss;
	uword last_wake_sp;
	uword last_wake_fl;
	uint32_t last_wake_time_us;
} hlt_diag = { .gen = -1 };

static inline void hlt_diag_sync(CPUI386 *cpu)
{
	if (hlt_diag.gen != cpu->diag_gen) {
		memset(&hlt_diag, 0, sizeof(hlt_diag));
		hlt_diag.gen = cpu->diag_gen;
	}
}

/* lazy flags */
enum {
	CC_ADC, CC_ADD,	CC_SBB, CC_SUB,
	CC_NEG8, CC_NEG16, CC_NEG32,
	CC_DEC8, CC_DEC16, CC_DEC32,
	CC_INC8, CC_INC16, CC_INC32,
	CC_IMUL8, CC_IMUL16, CC_IMUL32,	CC_MUL8, CC_MUL16, CC_MUL32,
	CC_SAR, CC_SHL, CC_SHR,
	CC_SHLD, CC_SHRD, CC_BSF, CC_BSR,
	CC_AND, CC_OR, CC_XOR,
};

static int get_CF(CPUI386 *cpu)
{
	if (cpu->cc.mask & CF) {
		switch(cpu->cc.op) {
		case CC_ADC:
			return cpu->cc.dst <= cpu->cc.src2;
		case CC_ADD:
			return cpu->cc.dst < cpu->cc.src2;
		case CC_SBB:
			return cpu->cc.src1 <= cpu->cc.src2;
		case CC_SUB:
			return cpu->cc.src1 < cpu->cc.src2;
		case CC_NEG8: case CC_NEG16: case CC_NEG32:
			return cpu->cc.dst != 0;
		case CC_DEC8: case CC_DEC16: case CC_DEC32:
		case CC_INC8: case CC_INC16: case CC_INC32:
			assert(false); // should not happen
		case CC_IMUL8:
			return sext8(cpu->cc.dst) != cpu->cc.dst;
		case CC_IMUL16:
			return sext16(cpu->cc.dst) != cpu->cc.dst;
		case CC_IMUL32:
			return (((s32) cpu->cc.dst) >> 31) != cpu->cc.dst2;
		case CC_MUL8:
			return (cpu->cc.dst >> 8) != 0;
		case CC_MUL16:
			return (cpu->cc.dst >> 16) != 0;
		case CC_MUL32:
			return (cpu->cc.dst2) != 0;
		case CC_SHL:
		case CC_SHR:
		case CC_SAR:
			return cpu->cc.dst2 & 1;
		case CC_SHLD:
			return cpu->cc.dst2 >> 31;
		case CC_SHRD:
			return cpu->cc.dst2 & 1;
		case CC_BSF:
		case CC_BSR:
			return 0;
		case CC_AND:
		case CC_OR:
		case CC_XOR:
			return 0;
		}
	} else {
		return !!(cpu->flags & CF);
	}
	assert(false);
}

static const TCM_DRAM_ATTR u8 parity_tab[256] = {
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
  1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

/*
 * Pre-computed flags tables for 8-bit ALU operations.
 * Eliminates per-flag computation in hot paths.
 *
 * flags_logic8[result] - ZF|SF|PF for logical ops (AND, OR, XOR)
 * flags_add8[src1][src2] - CF|PF|AF|ZF|SF|OF for ADD
 * flags_sub8[src1][src2] - CF|PF|AF|ZF|SF|OF for SUB/CMP
 *
 * Tables are 256 + 128KB + 128KB = ~256KB, stored in PSRAM on ESP32.
 */
typedef uint16_t flags8_arith_t[256][256];
static TCM_DRAM_ATTR uint8_t flags_logic8_tcm[256];
static uint8_t *flags_logic8 = NULL;
static flags8_arith_t *flags_add8 = NULL;
static flags8_arith_t *flags_sub8 = NULL;
static bool flags_add8_from_heap = false;
static bool flags_sub8_from_heap = false;

/* Initialize 8-bit flags lookup tables */
void i386_init_flags_tables(void)
{
	extern void *psmalloc(long size);
	/* flags_logic8 in TCM — always available, just point to it */
	flags_logic8 = flags_logic8_tcm;

	if (flags_sub8 != NULL)
		return;  /* Arithmetic tables already initialized */
	/* Allocate flags_sub8 (CMP) first — it's hotter than flags_add8 (ADD).
	 * VGA glyph+font caches are forced to PSRAM to free ~144KB SRAM for these. */
	flags_sub8 = heap_caps_malloc(sizeof(flags8_arith_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	flags_sub8_from_heap = (flags_sub8 != NULL);
	if (!flags_sub8) flags_sub8 = psmalloc(sizeof(flags8_arith_t));
	flags_add8 = heap_caps_malloc(sizeof(flags8_arith_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	flags_add8_from_heap = (flags_add8 != NULL);
	if (!flags_add8) flags_add8 = psmalloc(sizeof(flags8_arith_t));
	fprintf(stderr, "flags_sub8: %s (%dKB), flags_add8: %s (%dKB)\n",
		flags_sub8_from_heap ? "SRAM" : "PSRAM", (int)(sizeof(flags8_arith_t)/1024),
		flags_add8_from_heap ? "SRAM" : "PSRAM", (int)(sizeof(flags8_arith_t)/1024));

	if (!flags_add8 || !flags_sub8) {
		/* Allocation failed - will fall back to computed flags */
		flags_add8 = NULL;
		flags_sub8 = NULL;
		return;
	}

	/* Build logical ops flags table (result-only flags) */
	for (int r = 0; r < 256; r++) {
		uint8_t f = 0;
		if (r == 0) f |= ZF;
		if (r & 0x80) f |= SF;
		if (parity_tab[r]) f |= PF;
		flags_logic8[r] = f;
	}

	/* Build ADD8 flags table */
	for (int s1 = 0; s1 < 256; s1++) {
		for (int s2 = 0; s2 < 256; s2++) {
			int r = (s1 + s2) & 0xff;
			uint16_t f = 0;
			/* CF: unsigned overflow */
			if ((s1 + s2) > 0xff) f |= CF;
			/* PF: parity of result */
			if (parity_tab[r]) f |= PF;
			/* AF: carry from bit 3 to bit 4 */
			if (((s1 ^ s2 ^ r) & 0x10)) f |= AF;
			/* ZF: result is zero */
			if (r == 0) f |= ZF;
			/* SF: sign of result */
			if (r & 0x80) f |= SF;
			/* OF: signed overflow */
			if (((s1 ^ r) & (s2 ^ r) & 0x80)) f |= OF;
			(*flags_add8)[s1][s2] = f;
		}
	}

	/* Build SUB8 flags table */
	for (int s1 = 0; s1 < 256; s1++) {
		for (int s2 = 0; s2 < 256; s2++) {
			int r = (s1 - s2) & 0xff;
			uint16_t f = 0;
			/* CF: borrow (unsigned s1 < s2) */
			if (s1 < s2) f |= CF;
			/* PF: parity of result */
			if (parity_tab[r]) f |= PF;
			/* AF: borrow from bit 4 */
			if (((s1 ^ s2 ^ r) & 0x10)) f |= AF;
			/* ZF: result is zero */
			if (r == 0) f |= ZF;
			/* SF: sign of result */
			if (r & 0x80) f |= SF;
			/* OF: signed overflow */
			if (((s1 ^ s2) & (s1 ^ r) & 0x80)) f |= OF;
			(*flags_sub8)[s1][s2] = f;
		}
	}
}

/* Reset flags table pointers so they'll be re-initialized on next cpui386_new().
 * Called during INI switch after PSRAM pool is zeroed — tables allocated from
 * psmalloc() are now dangling.  Heap-allocated tables are still valid. */
void i386_reset_flags_tables_for_reinit(void)
{
	/* flags_logic8 is in TCM (always valid). Only NULL-ify psram-allocated
	 * arithmetic tables so init recreates them on next use. */
	if (!flags_add8_from_heap) flags_add8 = NULL;
	if (!flags_sub8_from_heap) flags_sub8 = NULL;
}

static int get_PF(CPUI386 *cpu)
{
	if (cpu->cc.mask & PF) {
		return parity_tab[cpu->cc.dst & 0xff];
	} else {
		return !!(cpu->flags & PF);
	}
}

static int get_AF(CPUI386 *cpu)
{
	if (cpu->cc.mask & AF) {
		switch(cpu->cc.op) {
		case CC_ADC:
		case CC_ADD:
		case CC_SBB:
		case CC_SUB:
			return ((cpu->cc.src1 ^ cpu->cc.src2 ^ cpu->cc.dst) >> 4) & 1;
		case CC_NEG8: case CC_NEG16: case CC_NEG32:
			return (cpu->cc.dst & 0xf) != 0;
		case CC_DEC8: case CC_DEC16: case CC_DEC32:
			return (cpu->cc.dst & 0xf) == 0xf;
		case CC_INC8: case CC_INC16: case CC_INC32:
			return (cpu->cc.dst & 0xf) == 0;
		case CC_IMUL8: case CC_IMUL16: case CC_IMUL32:
		case CC_MUL8: case CC_MUL16: case CC_MUL32:
			return 0;
		case CC_SAR:
		case CC_SHL:
		case CC_SHR:
		case CC_SHLD:
		case CC_SHRD:
		case CC_BSF:
		case CC_BSR:
		case CC_AND:
		case CC_OR:
		case CC_XOR:
			return 0;
		}
	} else {
		return !!(cpu->flags & AF);
	}
	assert(false);
}

static int IRAM_ATTR get_ZF(CPUI386 *cpu)
{
	if (cpu->cc.mask & ZF) {
		return cpu->cc.dst == 0;
	} else {
		return !!(cpu->flags & ZF);
	}
}

static int IRAM_ATTR get_SF(CPUI386 *cpu)
{
	if (cpu->cc.mask & SF) {
		return cpu->cc.dst >> (sizeof(uword) * 8 - 1);
	} else {
		return !!(cpu->flags & SF);
	}
}

static int get_OF(CPUI386 *cpu)
{
	if (cpu->cc.mask & OF) {
		switch(cpu->cc.op) {
		case CC_ADC:
		case CC_ADD:
			return (~(cpu->cc.src1 ^ cpu->cc.src2) & (cpu->cc.dst ^ cpu->cc.src2)) >> (sizeof(uword) * 8 - 1);
		case CC_SBB:
		case CC_SUB:
			return ((cpu->cc.src1 ^ cpu->cc.src2) & (cpu->cc.dst ^ cpu->cc.src1)) >> (sizeof(uword) * 8 - 1);
		case CC_DEC8:
			return cpu->cc.dst == sext8((u8) ~(1u << 7));
		case CC_DEC16:
			return cpu->cc.dst == sext16((u16) ~(1u << 15));
		case CC_DEC32:
			return cpu->cc.dst == sext32((u32) ~(1u << 31));
		case CC_INC8: case CC_NEG8:
			return cpu->cc.dst == sext8(1u << 7);
		case CC_INC16: case CC_NEG16:
			return cpu->cc.dst == sext16(1u << 15);
		case CC_INC32: case CC_NEG32:
			return cpu->cc.dst == sext32(1u << 31);
		case CC_IMUL8: case CC_IMUL16: case CC_IMUL32:
		case CC_MUL8: case CC_MUL16: case CC_MUL32:
			return get_CF(cpu);
		case CC_SAR:
			return 0;
		case CC_SHL:
			return (cpu->cc.dst >> (sizeof(uword) * 8 - 1)) ^ (cpu->cc.dst2 & 1);
		case CC_SHR:
			return (cpu->cc.src1 >> (sizeof(uword) * 8 - 1));
		case CC_SHLD:
		case CC_SHRD:
			return (cpu->cc.src1 ^ cpu->cc.dst) >> (sizeof(uword) * 8 - 1);
		case CC_BSF:
		case CC_BSR:
			return 0;
		case CC_AND:
		case CC_OR:
		case CC_XOR:
			return 0;
		}
		assert(false);
	} else {
		return !!(cpu->flags & OF);
	}
	assert(false);
}

static void refresh_flags(CPUI386 *cpu)
{
	SET_BIT(cpu->flags, get_CF(cpu), CF);
	SET_BIT(cpu->flags, get_PF(cpu), PF);
	SET_BIT(cpu->flags, get_AF(cpu), AF);
	SET_BIT(cpu->flags, get_ZF(cpu), ZF);
	SET_BIT(cpu->flags, get_SF(cpu), SF);
	SET_BIT(cpu->flags, get_OF(cpu), OF);
}

static inline int get_IOPL(CPUI386 *cpu)
{
	return (cpu->flags & IOPL) >> 12;
}

static inline void wfw_monitor_log_vm86_int_event(CPUI386 *cpu, uint8_t vec, uint8_t path)
{
	if (unlikely(cpu_diag_enabled) && win386_diag.active &&
	    wfw_monitor.vm86_int_evt_count < 8) {
		uint8_t i = wfw_monitor.vm86_int_evt_count++;
		wfw_monitor.vm86_int_evt[i].vec = vec;
		wfw_monitor.vm86_int_evt[i].path = path;
		wfw_monitor.vm86_int_evt[i].iopl = (uint8_t)get_IOPL(cpu);
		wfw_monitor.vm86_int_evt[i].ax = cpu->gprx[0].r16;
		wfw_monitor.vm86_int_evt[i].cs = cpu->seg[SEG_CS].sel;
		wfw_monitor.vm86_int_evt[i].ip = cpu->ip & 0xffff;
	}
}

static inline uint8_t wfw_mode_char(CPUI386 *cpu)
{
	if (cpu->flags & VM)
		return 'V';
	if (cpu->cr0 & 1)
		return 'P';
	return 'R';
}

static inline uint8_t wfw_mode_from_state(uword flags, uword cr0)
{
	if (flags & VM)
		return 'V';
	if (cr0 & 1)
		return 'P';
	return 'R';
}

static inline void wfw_flow_push_raw(CPUI386 *cpu, uint8_t tag, uint8_t vec,
				     uint8_t mode, uint8_t cpl,
				     uint16_t cs, uint32_t ip, uint32_t extra)
{
	if (unlikely(!(cpu_diag_enabled && win386_diag.active)))
		return;
	uint16_t idx = wfw_monitor.flow_pos % WFW_FLOW_RING_SIZE;
	wfw_monitor.flow_ring[idx].tag = tag;
	wfw_monitor.flow_ring[idx].mode = mode;
	wfw_monitor.flow_ring[idx].vec = vec;
	wfw_monitor.flow_ring[idx].cpl = cpl & 3;
	wfw_monitor.flow_ring[idx].cs = cs;
	wfw_monitor.flow_ring[idx].ip = ip;
	wfw_monitor.flow_ring[idx].ds = cpu->seg[SEG_DS].sel;
	wfw_monitor.flow_ring[idx].ax = cpu->gprx[0].r16;
	wfw_monitor.flow_ring[idx].bx = cpu->gprx[3].r16;
	wfw_monitor.flow_ring[idx].cx = cpu->gprx[1].r16;
	wfw_monitor.flow_ring[idx].dx = cpu->gprx[2].r16;
	wfw_monitor.flow_ring[idx].extra = extra;
	wfw_monitor.flow_pos++;
	if (wfw_monitor.flow_count < WFW_FLOW_RING_SIZE)
		wfw_monitor.flow_count++;
}

static inline void wfw_flow_push_here(CPUI386 *cpu, uint8_t tag, uint8_t vec, uint32_t extra)
{
	wfw_flow_push_raw(cpu, tag, vec, wfw_mode_char(cpu), cpu->cpl,
			  cpu->seg[SEG_CS].sel, cpu->ip, extra);
}

static inline void wfw_abort_tail_push(CPUI386 *cpu, uint8_t vec,
				       uint16_t ax, uint16_t bx, uint16_t cx, uint16_t dx)
{
	if (unlikely(!(cpu_diag_enabled && win386_diag.active)))
		return;

	uint8_t idx = wfw_monitor.abort_tail_pos % 16;
	wfw_monitor.abort_tail[idx].vec = vec;
	wfw_monitor.abort_tail[idx].mode = wfw_mode_char(cpu);
	wfw_monitor.abort_tail[idx].cpl = cpu->cpl & 3;
	wfw_monitor.abort_tail[idx].ah = (uint8_t)(ax >> 8);
	wfw_monitor.abort_tail[idx].cs = cpu->seg[SEG_CS].sel;
	wfw_monitor.abort_tail[idx].ip = cpu->ip;
	wfw_monitor.abort_tail[idx].ds = cpu->seg[SEG_DS].sel;
	wfw_monitor.abort_tail[idx].ax = ax;
	wfw_monitor.abort_tail[idx].bx = bx;
	wfw_monitor.abort_tail[idx].cx = cx;
	wfw_monitor.abort_tail[idx].dx = dx;
	wfw_monitor.abort_tail_pos++;
	if (wfw_monitor.abort_tail_count < 16)
		wfw_monitor.abort_tail_count++;
	wfw_flow_push_here(cpu, 'A', vec, (uint32_t)((ax >> 8) & 0xff));
}

static inline void wfw_monitor_note_int13_return(CPUI386 *cpu, uint16_t ret_cs, uint16_t ret_ip, uword ret_flags)
{
	if (unlikely(!(cpu_diag_enabled && win386_diag.active)))
		return;
	if (!wfw_monitor.pending_v86_int13)
		return;
	if (ret_cs != wfw_monitor.pending_v86_int13_ret_cs ||
	    ret_ip != (uint16_t)wfw_monitor.pending_v86_int13_ret_ip)
		return;

	wfw_monitor.pending_v86_int13 = false;
	wfw_monitor.v86_int13_ret_count++;
	wfw_monitor.v86_int13_ret_ax = cpu->gprx[0].r16;
	wfw_monitor.v86_int13_ret_bx = cpu->gprx[3].r16;
	wfw_monitor.v86_int13_ret_cx = cpu->gprx[1].r16;
	wfw_monitor.v86_int13_ret_dx = cpu->gprx[2].r16;
	wfw_monitor.v86_int13_ret_es = cpu->seg[SEG_ES].sel;
	wfw_monitor.v86_int13_ret_cf = (ret_flags & CF) ? 1 : 0;
	if (ret_flags & CF)
		wfw_monitor.v86_int13_fail_count++;
	wfw_flow_push_raw(cpu, 'K', 0x13, wfw_mode_from_state(ret_flags, cpu->cr0),
			  ret_cs & 3, ret_cs, ret_ip,
			  ((uint32_t)wfw_monitor.v86_int13_req_ax << 16) |
			  ((ret_flags & CF) ? 1u : 0u));
}

/* MMU */
#define CR0_PG (1<<31)
#define CR0_WP (0x10000)
#define CR4_PSE (1<<4)
#define tlb_size 1024
#define tlb_mask (tlb_size - 1)  /* bitmask: faster than modulo on RISC-V */
typedef struct {
	enum {
		ADDR_OK1,
		ADDR_OK2,
	} res;
	uword addr1;
	uword addr2;
} OptAddr;

static void tlb_clear(CPUI386 *cpu)
{
	for (int i = 0; i < tlb_size; i++) {
		cpu->tlb.tab[i].lpgno = -1;
	}
	cpu->tlb.generation++;
	cpu->ifetch.laddr = -1;
}

/* Lightweight TLB flush: increment generation so all existing entries
 * become stale without touching them.  Used for CR3 writes and
 * task switches where only the page directory changes. */
static void IRAM_ATTR tlb_flush_generation(CPUI386 *cpu)
{
	cpu->tlb.generation++;
	cpu->ifetch.laddr = -1;
}

static int pte_lookup[2][4][2][2] = { //[wp != 0][(pte >> 1) & 3][cpl > 0][rwm > 1]
	{ // wp == 0
		{ {0, 0}, {1, 1} }, // s,r
		{ {0, 0}, {1, 1} }, // s,w
		{ {0, 0}, {0, 1} }, // u,r
		{ {0, 0}, {0, 0} }, // u,w
	},
	{ // wp == 1
		{ {0, 1}, {1, 1} }, // s,r
		{ {0, 0}, {1, 1} }, // s,w
		{ {0, 1}, {0, 1} }, // u,r
		{ {0, 0}, {0, 0} }, // u,w
	}
};

static bool translate_laddr(CPUI386 *cpu, OptAddr *res, int rwm, uword laddr, int size, int cpl);
static u32 load32(CPUI386 *cpu, OptAddr *res);
static bool translate(CPUI386 *cpu, OptAddr *res, int rwm, int seg, uword addr, int size, int cpl);
static u8 load8(CPUI386 *cpu, OptAddr *res);
static u16 load16(CPUI386 *cpu, OptAddr *res);

static bool IRAM_ATTR tlb_refill(CPUI386 *cpu, struct tlb_entry *ent, uword lpgno)
{
	cpu->tlb_miss_count++;
	uint32_t a20 = cpu->a20_mask;
	uword base_addr = (cpu->cr3 & ~0xfff) & a20;
	uword i = lpgno >> 10;
	uword j = lpgno & 1023;

	u8 *mem = (u8 *) cpu->phys_mem;
	uword pde_addr = base_addr + i * 4;
	if ((uint64_t)pde_addr + 4 > (uint64_t)cpu->phys_mem_size)
		return false;
	uword pde = pload32(cpu, pde_addr);
	if (!(pde & 1))
		return false;
	mem[pde_addr] |= 1 << 5; // accessed

	/* 4MB pages (PSE): map this linear 4KB page directly from PDE */
	if ((cpu->cr4 & CR4_PSE) && (pde & (1 << 7))) {
		uword phys_page = (pde & 0xffc00000) | ((lpgno & 0x3ff) << 12);
		ent->lpgno = lpgno;
		ent->generation = cpu->tlb.generation;
		ent->xaddr = (phys_page & a20) ^ (lpgno << 12);
		ent->pte_lookup = pte_lookup[!!(cpu->cr0 & CR0_WP)][(pde >> 1) & 3];
		ent->ppte = &(mem[pde_addr]); /* dirty bit lives in PDE for 4MB pages */
		return true;
	}

	uword base_addr2 = (pde & ~0xfff) & a20;
	uword pte_addr = base_addr2 + j * 4;
	if ((uint64_t)pte_addr + 4 > (uint64_t)cpu->phys_mem_size)
		return false;
	uword pte = pload32(cpu, pte_addr);
	if (!(pte & 1))
		return false;

	mem[pte_addr] |= 1 << 5; // accessed
//	mem[base_addr2 + j * 4] |= 1 << 6; // dirty

	ent->lpgno = lpgno;
	ent->generation = cpu->tlb.generation;
	ent->xaddr = ((pte & ~0xfff) & a20) ^ (lpgno << 12);
	pte = pte & ((pde & 7) | 0xfffffff8);
	ent->pte_lookup = pte_lookup[!!(cpu->cr0 & CR0_WP)][(pte >> 1) & 3];
	ent->ppte = &(mem[pte_addr]);
	return true;
}

/* One-shot diagnostic helper: dump raw page-walk state for a linear address. */
static void dump_pf_walk(CPUI386 *cpu, uword laddr, const char *tag)
{
	uword a20 = cpu->a20_mask;
	uword cr3_base = (cpu->cr3 & ~0xfff) & a20;
	uword pdi = (laddr >> 22) & 0x3ff;
	uword pti = (laddr >> 12) & 0x3ff;
	uword pde_addr = cr3_base + pdi * 4;

	diaglog("=== %s: PF walk LA=%08x CR2=%08x CR3=%08x CR4=%08x ===\n",
		tag, laddr, cpu->cr2, cpu->cr3, cpu->cr4);
	diaglog("  CR3.base=%08x A20=%08x PDI=%03x PTI=%03x\n",
		cr3_base, a20, pdi, pti);

	if ((uint64_t)pde_addr + 4 > (uint64_t)cpu->phys_mem_size) {
		diaglog("  PDE addr OOB: %08x (phys_mem_size=%u)\n",
			pde_addr, cpu->phys_mem_size);
		return;
	}

	uword pde = pload32(cpu, pde_addr);
	diaglog("  PDE[%03x] @ %08x = %08x P=%d RW=%d US=%d A=%d D=%d PS=%d\n",
		pdi, pde_addr, pde,
		!!(pde & 1), !!(pde & 2), !!(pde & 4),
		!!(pde & (1 << 5)), !!(pde & (1 << 6)), !!(pde & (1 << 7)));

	if (!(pde & 1))
		return;

	if ((cpu->cr4 & CR4_PSE) && (pde & (1 << 7))) {
		uword phys = (pde & 0xffc00000) | (laddr & 0x003fffff);
		diaglog("  4MB map: phys=%08x (from PDE)\n", phys & a20);
		return;
	}

	uword pt_base = (pde & ~0xfff) & a20;
	uword pte_addr = pt_base + pti * 4;
	if ((uint64_t)pte_addr + 4 > (uint64_t)cpu->phys_mem_size) {
		diaglog("  PTE addr OOB: %08x (pt_base=%08x phys_mem_size=%u)\n",
			pte_addr, pt_base, cpu->phys_mem_size);
		return;
	}

	uword pte = pload32(cpu, pte_addr);
	uword phys = ((pte & ~0xfff) & a20) | (laddr & 0xfff);
	diaglog("  PTE[%03x] @ %08x = %08x P=%d RW=%d US=%d A=%d D=%d -> phys=%08x\n",
		pti, pte_addr, pte,
		!!(pte & 1), !!(pte & 2), !!(pte & 4),
		!!(pte & (1 << 5)), !!(pte & (1 << 6)), phys);
}

static void dump_idt_vector(CPUI386 *cpu, int vec, const char *tag)
{
	uword off = (uword)vec << 3;
	if (off + 7 > cpu->idt.limit) {
		dolog("=== %s: IDT[%02x] OOB (limit=%x) ===\n", tag, vec, cpu->idt.limit);
		return;
	}

	OptAddr meml;
	if (!translate_laddr(cpu, &meml, 1, cpu->idt.base + off, 4, 0)) {
		dolog("=== %s: IDT[%02x] read fail @%08x ===\n", tag, vec, cpu->idt.base + off);
		return;
	}
	uword w1 = load32(cpu, &meml);
	if (!translate_laddr(cpu, &meml, 1, cpu->idt.base + off + 4, 4, 0)) {
		dolog("=== %s: IDT[%02x] read fail @%08x ===\n", tag, vec, cpu->idt.base + off + 4);
		return;
	}
	uword w2 = load32(cpu, &meml);

	int gt = (w2 >> 8) & 0xf;
	int dpl = (w2 >> 13) & 0x3;
	int p = (w2 >> 15) & 1;
	uword newcs = (w1 >> 16) & 0xffff;
	bool diag_gate16 = (gt == 6 || gt == 7 || gt == 4);
	uword newip = diag_gate16 ? (w1 & 0xffff)
				  : ((w1 & 0xffff) | (w2 & 0xffff0000));

	dolog("=== %s: IDT[%02x] raw=%08x %08x ===\n", tag, vec, w1, w2);
	dolog("  type=%x(%s) dpl=%d p=%d target=%04x:%08x cpl=%d CS=%04x EIP=%08x\n",
		gt, diag_gate16 ? "16" : "32", dpl, p, newcs, newip,
		cpu->cpl, cpu->seg[SEG_CS].sel, cpu->ip);
}

/* Side-effect-safe IDT gate read for diagnostics. */
static bool read_idt_vector_raw_noexcept(CPUI386 *cpu, int vec, uword *w1, uword *w2)
{
	uword saved_cr2 = cpu->cr2;
	uword saved_excno = cpu->excno;
	uword saved_excerr = cpu->excerr;
	OptAddr meml;
	uword off = (uword)vec << 3;
	bool ok = false;

	if (off + 7 > cpu->idt.limit)
		goto out;
	if (!translate_laddr(cpu, &meml, 1, cpu->idt.base + off, 4, 0))
		goto out;
	*w1 = load32(cpu, &meml);
	if (!translate_laddr(cpu, &meml, 1, cpu->idt.base + off + 4, 4, 0))
		goto out;
	*w2 = load32(cpu, &meml);
	ok = true;

out:
	cpu->cr2 = saved_cr2;
	cpu->excno = saved_excno;
	cpu->excerr = saved_excerr;
	return ok;
}

static void dump_cs_bytes(CPUI386 *cpu, const char *tag, int nbytes)
{
	dolog("%s", tag);
	for (int i = 0; i < nbytes; i++) {
		OptAddr b;
		if (translate(cpu, &b, 1, SEG_CS, cpu->ip + i, 1, cpu->cpl))
			dolog(" %02x", load8(cpu, &b));
		else {
			dolog(" ??");
			break;
		}
	}
	dolog("\n");
}

static bool read_seg_u8_noexcept(CPUI386 *cpu, int seg, uword off, int cpl, uint8_t *out)
{
	uword saved_cr2 = cpu->cr2;
	uword saved_excno = cpu->excno;
	uword saved_excerr = cpu->excerr;
	OptAddr b;
	bool ok = translate(cpu, &b, 1, seg, off, 1, cpl);
	if (ok)
		*out = load8(cpu, &b);
	cpu->cr2 = saved_cr2;
	cpu->excno = saved_excno;
	cpu->excerr = saved_excerr;
	return ok;
}

static bool read_seg_u16_noexcept(CPUI386 *cpu, int seg, uword off, int cpl, uint16_t *out)
{
	uword saved_cr2 = cpu->cr2;
	uword saved_excno = cpu->excno;
	uword saved_excerr = cpu->excerr;
	OptAddr b;
	bool ok = translate(cpu, &b, 1, seg, off, 2, cpl);
	if (ok)
		*out = load16(cpu, &b);
	cpu->cr2 = saved_cr2;
	cpu->excno = saved_excno;
	cpu->excerr = saved_excerr;
	return ok;
}

/* Read a byte from a linear address without throwing exceptions. */
static bool read_laddr_u8_noexcept(CPUI386 *cpu, uword laddr, int cpl, uint8_t *out)
{
	uword saved_cr2 = cpu->cr2;
	uword saved_excno = cpu->excno;
	uword saved_excerr = cpu->excerr;
	OptAddr b;
	bool ok = translate_laddr(cpu, &b, 1, laddr, 1, cpl);
	if (ok)
		*out = load8(cpu, &b);
	cpu->cr2 = saved_cr2;
	cpu->excno = saved_excno;
	cpu->excerr = saved_excerr;
	return ok;
}

static bool read_laddr_u16_noexcept(CPUI386 *cpu, uword laddr, int cpl, uint16_t *out)
{
	uword saved_cr2 = cpu->cr2;
	uword saved_excno = cpu->excno;
	uword saved_excerr = cpu->excerr;
	OptAddr b;
	bool ok = translate_laddr(cpu, &b, 1, laddr, 2, cpl);
	if (ok)
		*out = load16(cpu, &b);
	cpu->cr2 = saved_cr2;
	cpu->excno = saved_excno;
	cpu->excerr = saved_excerr;
	return ok;
}

/* Translate linear address to physical without throwing; returns false if unmapped. */
static bool laddr_to_paddr_noexcept(CPUI386 *cpu, uint32_t laddr, uint32_t *out_pa)
{
	if (!(cpu->cr0 & CR0_PG)) {
		uint32_t pa = laddr & cpu->a20_mask;
		if (pa >= (uint32_t)cpu->phys_mem_size)
			return false;
		*out_pa = pa;
		return true;
	}

	uint32_t pde_a = (cpu->cr3 & 0xfffff000u) | ((laddr >> 20) & 0xffcu);
	if (pde_a + 4 > (uint32_t)cpu->phys_mem_size)
		return false;
	uint32_t pde = *(uint32_t *)&cpu->phys_mem[pde_a];
	if (!(pde & 1))
		return false;

	uint32_t pte_a = (pde & 0xfffff000u) | ((laddr >> 10) & 0xffcu);
	if (pte_a + 4 > (uint32_t)cpu->phys_mem_size)
		return false;
	uint32_t pte = *(uint32_t *)&cpu->phys_mem[pte_a];
	if (!(pte & 1))
		return false;

	uint32_t pa = (pte & 0xfffff000u) | (laddr & 0xfffu);
	if (pa >= (uint32_t)cpu->phys_mem_size)
		return false;
	*out_pa = pa;
	return true;
}

static inline uint32_t wfw_desc_base_u32(uint32_t w1, uint32_t w2)
{
	return (w1 >> 16) | ((w2 & 0xffu) << 16) | (w2 & 0xff000000u);
}

static inline uint32_t wfw_desc_limit_u32(uint32_t w1, uint32_t w2)
{
	uint32_t lim = (w1 & 0xffffu) | (w2 & 0x000f0000u);
	if (w2 & 0x00800000u)
		lim = (lim << 12) | 0xfffu;
	return lim;
}

static inline uint16_t wfw_desc_flags_u16(uint32_t w2)
{
	return (uint16_t)((w2 >> 8) & 0xffffu);
}

/* Read a raw LDT descriptor without throwing exceptions. */
/* Side-effect-safe descriptor read — works for both GDT and LDT selectors. */
static bool read_desc_noexcept(CPUI386 *cpu, uint16_t sel, uint32_t *w1, uint32_t *w2)
{
	uint32_t off = (uint32_t)(sel & ~7u);
	uint32_t tbl_base, tbl_limit;
	if (sel & 4) {
		tbl_base = cpu->seg[SEG_LDT].base;
		tbl_limit = cpu->seg[SEG_LDT].limit;
	} else {
		tbl_base = cpu->gdt.base;
		tbl_limit = cpu->gdt.limit;
	}
	if (off + 7 > tbl_limit)
		return false;
	uint32_t la = tbl_base + off;
	uint8_t d[8];
	for (int i = 0; i < 8; i++) {
		if (!read_laddr_u8_noexcept(cpu, la + (uint32_t)i, 0, &d[i]))
			return false;
	}
	*w1 = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
	      ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
	*w2 = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
	      ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
	return true;
}

/* Legacy alias for LDT-only reads. */
static bool wfw_read_ldt_desc_noexcept(CPUI386 *cpu, uint16_t sel, uint32_t *w1, uint32_t *w2)
{
	if (!(sel & 4))
		return false; /* GDT selector */
	return read_desc_noexcept(cpu, sel, w1, w2);
}

static const uint16_t wfw_ud63_near_sels[4] = { 0x011c, 0x0124, 0x0127, 0x012c };

static inline void wfw_ud63_snapshot_near_desc(CPUI386 *cpu, uint32_t *w1, uint32_t *w2, uint8_t *ok_mask)
{
	uint8_t m = 0;
	for (int i = 0; i < 4; i++) {
		uint32_t a = 0xffffffffu, b = 0xffffffffu;
		if (wfw_read_ldt_desc_noexcept(cpu, wfw_ud63_near_sels[i], &a, &b))
			m |= (uint8_t)(1u << i);
		w1[i] = a;
		w2[i] = b;
	}
	*ok_mask = m;
}

/* Watch physical writes that overlap the LDT window around selector 0x0127. */
static inline void wfw_note_ldt_window_store(CPUI386 *cpu, uint32_t paddr, uint8_t size, uint32_t val)
{
	if (unlikely(!(cpu_diag_enabled && win386_diag.active)))
		return;
	if (unlikely(size == 0))
		return;

	uint32_t ldt_base = cpu->seg[SEG_LDT].base;
	uint32_t ldt_lim = cpu->seg[SEG_LDT].limit;
	const uint32_t win_lo = 0x118;
	const uint32_t win_hi = 0x12f;
	if (win_lo > ldt_lim)
		return;
	uint32_t off_hi = (win_hi <= ldt_lim) ? win_hi : ldt_lim;
	uint32_t la_lo = ldt_base + win_lo;
	uint32_t la_hi = ldt_base + off_hi;
	uint32_t wr_lo = paddr;
	uint32_t wr_hi = paddr + (uint32_t)size - 1;

	/* Walk window by linear page; per-page mapping is contiguous in physical page. */
	for (uint32_t page = la_lo & ~0xfffu; page <= (la_hi & ~0xfffu); page += 0x1000u) {
		uint32_t chunk_la_lo = (la_lo > page) ? la_lo : page;
		uint32_t chunk_la_hi = (la_hi < (page + 0xfffu)) ? la_hi : (page + 0xfffu);
		uint32_t chunk_pa_lo;
		if (!laddr_to_paddr_noexcept(cpu, chunk_la_lo, &chunk_pa_lo))
			continue;
		uint32_t chunk_len = chunk_la_hi - chunk_la_lo + 1;
		uint32_t chunk_pa_hi = chunk_pa_lo + chunk_len - 1;
		if (wr_hi < chunk_pa_lo || wr_lo > chunk_pa_hi)
			continue;

		uint32_t ov_pa = (wr_lo > chunk_pa_lo) ? wr_lo : chunk_pa_lo;
		uint32_t ov_la = chunk_la_lo + (ov_pa - chunk_pa_lo);
		uint32_t ldt_off = ov_la - ldt_base;
		uint16_t sel = (uint16_t)((ldt_off & ~7u) | 0x4u); /* TI=LDT */

		{
			int i = wfw_monitor.ldt_watch_pos % 24;
			wfw_monitor.ldt_watch_log[i].cs = cpu->seg[SEG_CS].sel;
			wfw_monitor.ldt_watch_log[i].eip = cpu->ip;
			wfw_monitor.ldt_watch_log[i].cpl = cpu->cpl & 3;
			wfw_monitor.ldt_watch_log[i].size = size;
			wfw_monitor.ldt_watch_log[i].phys = ov_pa;
			wfw_monitor.ldt_watch_log[i].ldt_off = ldt_off;
			wfw_monitor.ldt_watch_log[i].sel = sel;
			wfw_monitor.ldt_watch_log[i].val = val;
			wfw_monitor.ldt_watch_pos++;
			if (wfw_monitor.ldt_watch_count < 24)
				wfw_monitor.ldt_watch_count++;
		}
		/* One log entry per store is sufficient. */
		break;
	}
}

/* Read the base address of a selector from GDT/LDT without throwing. */
static uint32_t desc_base_noexcept(CPUI386 *cpu, uint16_t sel)
{
	uint32_t off = sel & ~7;
	uint32_t tbl_base, tbl_limit;
	if (sel & 4) {
		tbl_base = cpu->seg[SEG_LDT].base;
		tbl_limit = cpu->seg[SEG_LDT].limit;
	} else {
		tbl_base = cpu->gdt.base;
		tbl_limit = cpu->gdt.limit;
	}
	if (off + 7 > tbl_limit)
		return 0xFFFFFFFF;
	uint8_t d[8];
	for (int i = 0; i < 8; i++)
		if (!read_laddr_u8_noexcept(cpu, tbl_base + off + i, 0, &d[i]))
			return 0xFFFFFFFF;
	return (uint32_t)d[2] | ((uint32_t)d[3] << 8) |
	       ((uint32_t)d[4] << 16) | ((uint32_t)d[7] << 24);
}

/* Read the NE module name from a module handle selector.
 * The module handle in Windows 3.x is a selector pointing to the NE header.
 * The module name is the first entry in the Resident Name Table. */
static void read_ne_module_name(CPUI386 *cpu, uint16_t hmod, char *buf, int bufsize)
{
	buf[0] = '\0';
	if (bufsize < 2 || (hmod & ~7) == 0)
		return;
	uint32_t base = desc_base_noexcept(cpu, hmod);
	if (base == 0xFFFFFFFF)
		return;
	/* Verify NE signature */
	uint16_t sig;
	if (!read_laddr_u16_noexcept(cpu, base, 0, &sig))
		return;
	if (sig != 0x454E) /* 'NE' little-endian */
		return;
	/* Resident Name Table offset at NE+0x26 */
	uint16_t rnt_off;
	if (!read_laddr_u16_noexcept(cpu, base + 0x26, 0, &rnt_off))
		return;
	/* First entry: length byte followed by module name */
	uint8_t len;
	if (!read_laddr_u8_noexcept(cpu, base + rnt_off, 0, &len))
		return;
	if (len == 0 || len >= (uint8_t)bufsize)
		return;
	for (int i = 0; i < len; i++) {
		uint8_t c;
		if (!read_laddr_u8_noexcept(cpu, base + rnt_off + 1 + i, 0, &c)) {
			buf[0] = '\0';
			return;
		}
		buf[i] = (c >= 0x20 && c <= 0x7e) ? (char)c : '.';
	}
	buf[len] = '\0';
}

static void vm86_dump_stack_words(CPUI386 *cpu, int words)
{
	uword sp = REGi(4);
	uword sp_mask = (cpu->seg[SEG_SS].flags & SEG_B_BIT) ? 0xffffffff : 0xffff;
	dolog("  vm86 stack SS:SP=%04x:%08x", cpu->seg[SEG_SS].sel, sp);
	for (int i = 0; i < words; i++) {
		uint16_t w = 0;
		uword off = (sp + (uword)(i * 2)) & sp_mask;
		if (read_seg_u16_noexcept(cpu, SEG_SS, off, cpu->cpl, &w))
			dolog(" [%d]=%04x", i, w);
		else {
			dolog(" [%d]=????", i);
			break;
		}
	}
	dolog("\n");
}

static void vm86_log_ioperm_probe(CPUI386 *cpu, int port, int bits)
{
	bool need_iobitmap = false;
	if (cpu->cr0 & 1) {
		/* 80386 rule: VM86 IN/OUT always consults the TSS I/O bitmap.
		 * In PM, bitmap checks are needed only when CPL > IOPL. */
		int iopl = get_IOPL(cpu);
		if (cpu->flags & VM)
			need_iobitmap = true;
		else
			need_iobitmap = (cpu->cpl > iopl);
	}

	if (!need_iobitmap) {
		dolog("  io-perm: bypass (no bitmap check needed)\n");
		return;
	}

	{
		int tr_type = cpu->seg[SEG_TR].flags & 0xf;
		if (tr_type != 9 && tr_type != 11) {
			dolog("  io-perm: deny (TR type %x not 32-bit TSS)\n", tr_type);
			return;
		}
	}
	if (cpu->seg[SEG_TR].limit < 103) {
		dolog("  io-perm: deny (TR limit %x < 0x67)\n", cpu->seg[SEG_TR].limit);
		return;
	}

	uint16_t iobase16 = 0;
	if (!read_seg_u16_noexcept(cpu, SEG_TR, 102, 0, &iobase16)) {
		dolog("  io-perm: failed reading TSS iobase\n");
		return;
	}

	uword iobase = iobase16;
	dolog("  io-perm: port=%04x bits=%d TR.limit=%x iobase=%x\n",
		port & 0xffff, bits, cpu->seg[SEG_TR].limit, iobase);

	if (iobase > cpu->seg[SEG_TR].limit) {
		dolog("  io-perm: deny (iobase beyond TR limit)\n");
		return;
	}

	int denied_bit = -1;
	uword denied_byte_off = 0;
	uint8_t denied_byte = 0;
	for (int b = 0; b < bits; b++) {
		uword p = (uword)port + (uword)b;
		uword byte_off = iobase + (p >> 3);
		if (byte_off > cpu->seg[SEG_TR].limit) {
			denied_bit = b;
			denied_byte_off = byte_off;
			break;
		}
		uint8_t perm = 0;
		if (!read_seg_u8_noexcept(cpu, SEG_TR, byte_off, 0, &perm)) {
			dolog("  io-perm: failed reading bitmap byte @%x\n", byte_off);
			return;
		}
		if (perm & (1u << (p & 7))) {
			denied_bit = b;
			denied_byte_off = byte_off;
			denied_byte = perm;
			break;
		}
	}

	if (denied_bit >= 0) {
		if (denied_byte_off > cpu->seg[SEG_TR].limit)
			dolog("  io-perm: deny bit=%d (bitmap out-of-range)\n", denied_bit);
		else
			dolog("  io-perm: deny bit=%d byte[%x]=%02x\n",
				denied_bit, denied_byte_off, denied_byte);
	} else {
		dolog("  io-perm: allow\n");
	}
}

static void vm86_decode_fault_instruction(CPUI386 *cpu, char *buf, size_t buflen,
					  bool *is_io, int *io_port, int *io_bits,
					  bool *needs_stack_dump)
{
	int idx = 0;
	bool op32 = false;
	uint8_t op = 0;
	uint8_t imm = 0;

	*is_io = false;
	*io_port = -1;
	*io_bits = 0;
	*needs_stack_dump = false;

	while (idx < 6) {
		if (!read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx, cpu->cpl, &op)) {
			snprintf(buf, buflen, "<unreadable>");
			return;
		}
		switch (op) {
		case 0x66:
			op32 = !op32;
			idx++;
			continue;
		case 0x67: case 0xF2: case 0xF3:
		case 0x26: case 0x2E: case 0x36: case 0x3E: case 0x64: case 0x65:
			idx++;
			continue;
		default:
			break;
		}
		break;
	}

	switch (op) {
	case 0xFA:
		snprintf(buf, buflen, "CLI");
		break;
	case 0xFB:
		snprintf(buf, buflen, "STI");
		break;
	case 0xCF:
		snprintf(buf, buflen, "IRET");
		*needs_stack_dump = true;
		break;
	case 0x9C:
		snprintf(buf, buflen, op32 ? "PUSHFD" : "PUSHF");
		break;
	case 0x9D:
		snprintf(buf, buflen, op32 ? "POPFD" : "POPF");
		*needs_stack_dump = true;
		break;
	case 0xCD:
		if (read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 1, cpu->cpl, &imm))
			snprintf(buf, buflen, "INT %02Xh", imm);
		else
			snprintf(buf, buflen, "INT imm8");
		*needs_stack_dump = true;
		break;
	case 0xCE:
		snprintf(buf, buflen, "INTO");
		*needs_stack_dump = true;
		break;
	case 0xF4:
		snprintf(buf, buflen, "HLT");
		break;
	case 0xE4:
		*is_io = true; *io_bits = 8;
		if (read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 1, cpu->cpl, &imm)) {
			*io_port = imm;
			snprintf(buf, buflen, "IN AL,%02Xh", imm);
		} else {
			snprintf(buf, buflen, "IN AL,imm8");
		}
		break;
	case 0xE5:
		*is_io = true; *io_bits = op32 ? 32 : 16;
		if (read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 1, cpu->cpl, &imm)) {
			*io_port = imm;
			snprintf(buf, buflen, "IN %s,%02Xh", op32 ? "EAX" : "AX", imm);
		} else {
			snprintf(buf, buflen, "IN %s,imm8", op32 ? "EAX" : "AX");
		}
		break;
	case 0xE6:
		*is_io = true; *io_bits = 8;
		if (read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 1, cpu->cpl, &imm)) {
			*io_port = imm;
			snprintf(buf, buflen, "OUT %02Xh,AL", imm);
		} else {
			snprintf(buf, buflen, "OUT imm8,AL");
		}
		break;
	case 0xE7:
		*is_io = true; *io_bits = op32 ? 32 : 16;
		if (read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 1, cpu->cpl, &imm)) {
			*io_port = imm;
			snprintf(buf, buflen, "OUT %02Xh,%s", imm, op32 ? "EAX" : "AX");
		} else {
			snprintf(buf, buflen, "OUT imm8,%s", op32 ? "EAX" : "AX");
		}
		break;
	case 0xEC:
		*is_io = true; *io_bits = 8;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, "IN AL,DX");
		break;
	case 0xED:
		*is_io = true; *io_bits = op32 ? 32 : 16;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, "IN %s,DX", op32 ? "EAX" : "AX");
		break;
	case 0xEE:
		*is_io = true; *io_bits = 8;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, "OUT DX,AL");
		break;
	case 0xEF:
		*is_io = true; *io_bits = op32 ? 32 : 16;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, "OUT DX,%s", op32 ? "EAX" : "AX");
		break;
	case 0x6C:
		*is_io = true; *io_bits = 8;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, "INSB");
		break;
	case 0x6D:
		*is_io = true; *io_bits = op32 ? 32 : 16;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, op32 ? "INSD" : "INSW");
		break;
	case 0x6E:
		*is_io = true; *io_bits = 8;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, "OUTSB");
		break;
	case 0x6F:
		*is_io = true; *io_bits = op32 ? 32 : 16;
		*io_port = (u16)REGi(2);
		snprintf(buf, buflen, op32 ? "OUTSD" : "OUTSW");
		break;
	case 0x0F: {
		uint8_t op2 = 0, modrm = 0;
		if (!read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 1, cpu->cpl, &op2)) {
			snprintf(buf, buflen, "0F ??");
			break;
		}
		if (op2 == 0x01 &&
		    read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + idx + 2, cpu->cpl, &modrm)) {
			int grp = (modrm >> 3) & 7;
			static const char *grp7_names[8] = {
				"SGDT", "SIDT", "LGDT", "LIDT", "SMSW", "grp7/5", "LMSW", "INVLPG"
			};
			snprintf(buf, buflen, "0F 01 /%d (%s)", grp, grp7_names[grp]);
		} else {
			snprintf(buf, buflen, "0F %02Xh", op2);
		}
		break;
	}
	default:
		snprintf(buf, buflen, "opcode %02Xh", op);
		break;
	}
}

static uint8_t wfw_capture_cs_bytes(CPUI386 *cpu, uint8_t out[8])
{
	uint8_t n = 0, b = 0;
	for (int i = 0; i < 8; i++) {
		if (!read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + i, cpu->cpl, &b))
			break;
		out[n++] = b;
	}
	return n;
}

static void wfw_capture_decode_and_bytes(CPUI386 *cpu, char *decoded, size_t dlen,
					 uint8_t out[8], uint8_t *out_len)
{
	bool is_io = false, needs_stack_dump = false;
	int io_port = -1, io_bits = 0;
	vm86_decode_fault_instruction(cpu, decoded, dlen, &is_io, &io_port, &io_bits, &needs_stack_dump);
	*out_len = wfw_capture_cs_bytes(cpu, out);
}

static int wfw_bios_hot_find_or_add(uint16_t cs, uint16_t ip, uint8_t exc)
{
	for (int i = 0; i < wfw_monitor.bios_hot_count; i++) {
		if (wfw_monitor.bios_hot[i].cs == cs &&
		    wfw_monitor.bios_hot[i].ip == ip &&
		    wfw_monitor.bios_hot[i].exc == exc)
			return i;
	}
	if (wfw_monitor.bios_hot_count < 6) {
		int i = wfw_monitor.bios_hot_count++;
		wfw_monitor.bios_hot[i].cs = cs;
		wfw_monitor.bios_hot[i].ip = ip;
		wfw_monitor.bios_hot[i].exc = exc;
		return i;
	}
	/* If full, fold into slot 0 to keep output bounded. */
	return 0;
}

static int wfw_swint_dpl_find_or_add(uint8_t vec)
{
	for (int i = 0; i < wfw_monitor.swint_dpl_count; i++) {
		if (wfw_monitor.swint_dpl[i].vec == vec)
			return i;
	}
	if (wfw_monitor.swint_dpl_count < 6) {
		int i = wfw_monitor.swint_dpl_count++;
		wfw_monitor.swint_dpl[i].vec = vec;
		return i;
	}
	/* Keep output bounded. Fold overflow into slot 0. */
	return 0;
}

static inline void wfw_swint_dpl_note_resume(CPUI386 *cpu, uint16_t newcs, uword newip, uword newflags)
{
	if (!win386_diag.active)
		return;
	if (wfw_monitor.pending_swint_dpl_idx < 0 ||
	    wfw_monitor.pending_swint_dpl_idx >= (int8_t)wfw_monitor.swint_dpl_count)
		return;

	int i = wfw_monitor.pending_swint_dpl_idx;
	uint8_t rcls = 0;
	if (newcs == wfw_monitor.pending_swint_cs &&
	    newip == wfw_monitor.pending_swint_ip) {
		wfw_monitor.swint_dpl[i].resume_same++;
		rcls = 1;
	} else if (newcs == wfw_monitor.pending_swint_cs &&
		   newip == wfw_monitor.pending_swint_next_ip) {
		wfw_monitor.swint_dpl[i].resume_next++;
		rcls = 2;
	} else {
		wfw_monitor.swint_dpl[i].resume_other++;
		rcls = 3;
	}
	wfw_monitor.swint_dpl[i].out_ax = REGi(0) & 0xffff;
	wfw_monitor.swint_dpl[i].out_bx = REGi(3) & 0xffff;
	wfw_monitor.swint_dpl[i].out_fl = newflags;
	if (newflags & CF)
		wfw_monitor.swint_dpl[i].cf_set++;
	{
		int ti = wfw_monitor.dpl_tail_pos % 16;
		wfw_monitor.dpl_tail[ti].vec = wfw_monitor.swint_dpl[i].vec;
		wfw_monitor.dpl_tail[ti].mode = wfw_mode_from_state(newflags, cpu->cr0);
		wfw_monitor.dpl_tail[ti].cpl = newcs & 3;
		wfw_monitor.dpl_tail[ti].rcls = rcls;
		wfw_monitor.dpl_tail[ti].cf = (newflags & CF) ? 1 : 0;
		wfw_monitor.dpl_tail[ti].in_ax = wfw_monitor.swint_dpl[i].in_ax;
		wfw_monitor.dpl_tail[ti].in_bx = wfw_monitor.swint_dpl[i].in_bx;
		wfw_monitor.dpl_tail[ti].out_ax = wfw_monitor.swint_dpl[i].out_ax;
		wfw_monitor.dpl_tail[ti].out_bx = wfw_monitor.swint_dpl[i].out_bx;
		wfw_monitor.dpl_tail[ti].in_cs = wfw_monitor.pending_swint_cs;
		wfw_monitor.dpl_tail[ti].in_ip = wfw_monitor.pending_swint_ip;
		wfw_monitor.dpl_tail[ti].out_cs = newcs;
		wfw_monitor.dpl_tail[ti].out_ip = newip;
		wfw_monitor.dpl_tail_pos++;
		if (wfw_monitor.dpl_tail_count < 16)
			wfw_monitor.dpl_tail_count++;
	}
	wfw_flow_push_raw(cpu, 'Q', wfw_monitor.swint_dpl[i].vec,
			  wfw_mode_from_state(newflags, cpu->cr0),
			  newcs & 3, newcs, newip,
			  ((uint32_t)wfw_monitor.swint_dpl[i].in_ax << 16) |
			  ((uint32_t)rcls << 8) |
			  ((newflags & CF) ? 1u : 0u));
	wfw_monitor.pending_swint_dpl_idx = -1;
}

static int wfw_ud63_find_or_add(uint16_t cs, uint16_t ip)
{
	for (int i = 0; i < wfw_monitor.ud63_hot_count; i++) {
		if (wfw_monitor.ud63_hot[i].cs == cs &&
		    wfw_monitor.ud63_hot[i].ip == ip)
			return i;
	}
	if (wfw_monitor.ud63_hot_count < 6) {
		int i = wfw_monitor.ud63_hot_count++;
		wfw_monitor.ud63_hot[i].cs = cs;
		wfw_monitor.ud63_hot[i].ip = ip;
		return i;
	}
	/* Keep output bounded. Fold overflow into slot 0. */
	return 0;
}

/* Capture FE95 callback path details:
 * - bounded ring-0 instruction trace for the first callback
 * - branch predicate/outcome probes at 0028:6ca7/6cab for every callback
 * - AX transition-to-0005 writer site across callbacks */
static inline void wfw_ud63_trace_step(CPUI386 *cpu)
{
	if (unlikely(!(cpu_diag_enabled && win386_diag.active)))
		return;
	if (!(wfw_monitor.ud63_fe95_0127.trace_active ||
	      wfw_monitor.pending_ud63_fe95_0127))
		return;
	if (cpu->cpl != 0)
		return;
	bool in_cb = wfw_monitor.pending_ud63_fe95_0127;

	/* Resolve pending branch outcome from previous 0028:6cab JE step. */
	if (in_cb && wfw_monitor.ud63_fe95_0127.j6cab_pending) {
		uint16_t csn = cpu->seg[SEG_CS].sel;
		uint16_t ipn = cpu->ip & 0xffff;
		wfw_monitor.ud63_fe95_0127.j6cab_count++;
		wfw_monitor.ud63_fe95_0127.j6cab_last_to_cs = csn;
		wfw_monitor.ud63_fe95_0127.j6cab_last_to_ip = ipn;
		if (csn == wfw_monitor.ud63_fe95_0127.j6cab_last_from_cs &&
		    ipn == wfw_monitor.ud63_fe95_0127.j6cab_last_tgt) {
			wfw_monitor.ud63_fe95_0127.j6cab_taken++;
		} else if (csn == wfw_monitor.ud63_fe95_0127.j6cab_last_from_cs &&
			   ipn == wfw_monitor.ud63_fe95_0127.j6cab_last_fall) {
			wfw_monitor.ud63_fe95_0127.j6cab_not_taken++;
		} else {
			wfw_monitor.ud63_fe95_0127.j6cab_other++;
		}
		wfw_monitor.ud63_fe95_0127.j6cab_pending = false;
	}

	if (wfw_monitor.ud63_fe95_0127.trace_count < 128) {
		int ti = wfw_monitor.ud63_fe95_0127.trace_count++;
		wfw_monitor.ud63_fe95_0127.trace[ti].cs = cpu->seg[SEG_CS].sel;
		wfw_monitor.ud63_fe95_0127.trace[ti].ip = cpu->ip & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].ax = REGi(0) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].bx = REGi(3) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].cx = REGi(1) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].dx = REGi(2) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].si = REGi(6) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].di = REGi(7) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].bp = REGi(5) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].sp = REGi(4) & 0xffff;
		wfw_monitor.ud63_fe95_0127.trace[ti].fl = cpu->flags;
		wfw_monitor.ud63_fe95_0127.trace[ti].blen =
			wfw_capture_cs_bytes(cpu, wfw_monitor.ud63_fe95_0127.trace[ti].bytes);
	}

	if (in_cb) {
		uint16_t cs = cpu->seg[SEG_CS].sel;
		uint16_t ip = cpu->ip & 0xffff;
		uint16_t ax = REGi(0) & 0xffff;

		/* Track where AX first transitions to 0005 on callback path. */
		if (wfw_monitor.ud63_fe95_0127.step_prev_valid &&
		    wfw_monitor.ud63_fe95_0127.step_prev_ax != ax &&
		    ax == 0x0005) {
			wfw_monitor.ud63_fe95_0127.ax5_transitions++;
			wfw_monitor.ud63_fe95_0127.ax5_from_cs =
				wfw_monitor.ud63_fe95_0127.step_prev_cs;
			wfw_monitor.ud63_fe95_0127.ax5_from_ip =
				wfw_monitor.ud63_fe95_0127.step_prev_ip;
			wfw_monitor.ud63_fe95_0127.ax5_from_ax =
				wfw_monitor.ud63_fe95_0127.step_prev_ax;
			wfw_monitor.ud63_fe95_0127.ax5_to_cs = cs;
			wfw_monitor.ud63_fe95_0127.ax5_to_ip = ip;
			wfw_monitor.ud63_fe95_0127.ax5_to_ax = ax;
			wfw_monitor.ud63_fe95_0127.ax5_from_blen =
				wfw_monitor.ud63_fe95_0127.step_prev_blen;
			for (int b = 0; b < wfw_monitor.ud63_fe95_0127.step_prev_blen; b++)
				wfw_monitor.ud63_fe95_0127.ax5_from_bytes[b] =
					wfw_monitor.ud63_fe95_0127.step_prev_bytes[b];
		}

		/* 0028:6ca7: test byte [bp+2e],2 branch predicate source. */
		if (cs == 0x0028 && ip == 0x6ca7) {
			uint8_t v = 0;
			uint16_t ss = cpu->seg[SEG_SS].sel;
			uint32_t bp = REGi(5);
			uint32_t off = bp + 0x2e;
			if (read_seg_u8_noexcept(cpu, SEG_SS, off, cpu->cpl, &v)) {
				wfw_monitor.ud63_fe95_0127.t6ca7_count++;
				wfw_monitor.ud63_fe95_0127.t6ca7_last_byte = v;
				wfw_monitor.ud63_fe95_0127.t6ca7_last_ss = ss;
				wfw_monitor.ud63_fe95_0127.t6ca7_last_bp = (uint16_t)(bp & 0xffff);
				if (v & 0x02)
					wfw_monitor.ud63_fe95_0127.t6ca7_bit_set++;
				else
					wfw_monitor.ud63_fe95_0127.t6ca7_bit_clear++;
			}
		}

		/* 0028:6cab: JE rel8 — evaluate outcome on next traced step. */
		if (cs == 0x0028 && ip == 0x6cab) {
			uint8_t rel = 0;
			int8_t srel = 0;
			if (read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + 1, cpu->cpl, &rel))
				srel = (int8_t)rel;
			wfw_monitor.ud63_fe95_0127.j6cab_pending = true;
			wfw_monitor.ud63_fe95_0127.j6cab_last_from_cs = cs;
			wfw_monitor.ud63_fe95_0127.j6cab_last_from_ip = ip;
			wfw_monitor.ud63_fe95_0127.j6cab_last_zf = (cpu->flags & ZF) ? 1 : 0;
			wfw_monitor.ud63_fe95_0127.j6cab_last_fall = (uint16_t)(ip + 2);
			wfw_monitor.ud63_fe95_0127.j6cab_last_tgt =
				(uint16_t)(ip + 2 + srel);
		}

		/* Save pre-exec state for next-step transition analysis. */
		wfw_monitor.ud63_fe95_0127.step_prev_valid = true;
		wfw_monitor.ud63_fe95_0127.step_prev_cs = cs;
		wfw_monitor.ud63_fe95_0127.step_prev_ip = ip;
		wfw_monitor.ud63_fe95_0127.step_prev_ax = ax;
		wfw_monitor.ud63_fe95_0127.step_prev_blen = 0;
		for (int b = 0; b < 8; b++) {
			uint8_t v = 0;
			if (!read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + (uword)b, cpu->cpl, &v))
				break;
			wfw_monitor.ud63_fe95_0127.step_prev_bytes[
				wfw_monitor.ud63_fe95_0127.step_prev_blen++] = v;
		}
	}

	{
		uint16_t ax = REGi(0) & 0xffff;
		if (!wfw_monitor.ud63_fe95_0127.trace_ax5_valid &&
		    ax == 0x0005 &&
		    wfw_monitor.ud63_fe95_0127.trace_prev_ax != 0x0005) {
			wfw_monitor.ud63_fe95_0127.trace_ax5_valid = true;
			wfw_monitor.ud63_fe95_0127.trace_ax5_prev =
				wfw_monitor.ud63_fe95_0127.trace_prev_ax;
			wfw_monitor.ud63_fe95_0127.trace_ax5_ax = ax;
			wfw_monitor.ud63_fe95_0127.trace_ax5_cs = cpu->seg[SEG_CS].sel;
			wfw_monitor.ud63_fe95_0127.trace_ax5_ip = cpu->ip & 0xffff;
		}
		wfw_monitor.ud63_fe95_0127.trace_prev_ax = ax;
	}
}

static inline void wfw_ud63_note_resume(CPUI386 *cpu, uint16_t newcs, uword newip,
					uword newflags, uint16_t resume_ds)
{
	if (!win386_diag.active)
		return;
	if (wfw_monitor.pending_ud63_idx < 0 ||
	    wfw_monitor.pending_ud63_idx >= (int8_t)wfw_monitor.ud63_hot_count)
		return;

	int i = wfw_monitor.pending_ud63_idx;
	uint16_t old_ip16 = (uint16_t)(wfw_monitor.pending_ud63_ip & 0xffff);
	uint16_t new_ip16 = (uint16_t)(newip & 0xffff);
	uint16_t old_ip4 = (uint16_t)(old_ip16 + 4);
	uint16_t old_ip8 = (uint16_t)(old_ip16 + 8);
	if (newcs == wfw_monitor.pending_ud63_cs &&
	    new_ip16 == old_ip16)
		wfw_monitor.ud63_hot[i].resume_same++;
	else if (newcs == wfw_monitor.pending_ud63_cs &&
		 new_ip16 == old_ip4)
		wfw_monitor.ud63_hot[i].resume_next4++;
	else if (newcs == wfw_monitor.pending_ud63_cs &&
		 new_ip16 == old_ip8)
		wfw_monitor.ud63_hot[i].resume_next8++;
	else
		wfw_monitor.ud63_hot[i].resume_other++;
	wfw_monitor.ud63_hot[i].out_ax = REGi(0) & 0xffff;
	wfw_monitor.ud63_hot[i].out_fl = newflags;
	if (newflags & CF)
		wfw_monitor.ud63_hot[i].cf_set++;

	if (wfw_monitor.pending_ud63_fe95_0127) {
		uint32_t w1 = 0xffffffffu, w2 = 0xffffffffu;
		if (wfw_read_ldt_desc_noexcept(cpu, 0x0127, &w1, &w2)) {
			wfw_monitor.ud63_fe95_0127.post_w1 = w1;
			wfw_monitor.ud63_fe95_0127.post_w2 = w2;
		} else {
			wfw_monitor.ud63_fe95_0127.post_w1 = 0xffffffffu;
			wfw_monitor.ud63_fe95_0127.post_w2 = 0xffffffffu;
		}
		wfw_monitor.ud63_fe95_0127.resumed++;
		if (wfw_monitor.ud63_fe95_0127.pre_w1 != wfw_monitor.ud63_fe95_0127.post_w1 ||
		    wfw_monitor.ud63_fe95_0127.pre_w2 != wfw_monitor.ud63_fe95_0127.post_w2)
			wfw_monitor.ud63_fe95_0127.desc_changed++;
		wfw_monitor.ud63_fe95_0127.post_cs = newcs;
		wfw_monitor.ud63_fe95_0127.post_ip = (uint16_t)(newip & 0xffff);
		wfw_monitor.ud63_fe95_0127.post_ds = cpu->seg[SEG_DS].sel;
		wfw_monitor.ud63_fe95_0127.resume_ds = resume_ds;
		wfw_monitor.ud63_fe95_0127.post_ax = REGi(0) & 0xffff;
		wfw_monitor.ud63_fe95_0127.ret_ax_last = wfw_monitor.ud63_fe95_0127.post_ax;
		if (wfw_monitor.ud63_fe95_0127.post_ax == 0x0005)
			wfw_monitor.ud63_fe95_0127.ret_ax5++;
		else
			wfw_monitor.ud63_fe95_0127.ret_ax_other++;
		wfw_monitor.ud63_fe95_0127.post_fl = newflags;
		wfw_ud63_snapshot_near_desc(cpu,
					    wfw_monitor.ud63_fe95_0127.post_near_w1,
					    wfw_monitor.ud63_fe95_0127.post_near_w2,
					    &wfw_monitor.ud63_fe95_0127.post_near_ok);
		for (int k = 0; k < 4; k++) {
			uint8_t mk = (uint8_t)(1u << k);
			if ((wfw_monitor.ud63_fe95_0127.pre_near_ok & mk) &&
			    (wfw_monitor.ud63_fe95_0127.post_near_ok & mk)) {
				if (wfw_monitor.ud63_fe95_0127.pre_near_w1[k] !=
				    wfw_monitor.ud63_fe95_0127.post_near_w1[k] ||
				    wfw_monitor.ud63_fe95_0127.pre_near_w2[k] !=
				    wfw_monitor.ud63_fe95_0127.post_near_w2[k]) {
					wfw_monitor.ud63_fe95_0127.near_changed[k]++;
				}
			}
		}
		wfw_monitor.pending_ud63_fe95_0127 = false;
		wfw_monitor.ud63_fe95_0127.j6cab_pending = false;
		wfw_monitor.ud63_fe95_0127.step_prev_valid = false;
		if (wfw_monitor.ud63_fe95_0127.trace_active) {
			wfw_monitor.ud63_fe95_0127.trace_active = false;
			wfw_monitor.ud63_fe95_0127.trace_done = true;
		}
	}
	wfw_monitor.pending_ud63_idx = -1;
}

static const char *seg_name(int seg)
{
	switch (seg) {
	case SEG_ES: return "ES";
	case SEG_CS: return "CS";
	case SEG_SS: return "SS";
	case SEG_DS: return "DS";
	case SEG_FS: return "FS";
	case SEG_GS: return "GS";
	case SEG_LDT: return "LDT";
	case SEG_TR: return "TR";
	default: return "?";
	}
}

static bool IRAM_ATTR translate_lpgno(CPUI386 *cpu, int rwm, uword lpgno, uword laddr, int cpl, uword *paddr)
{
	struct tlb_entry *ent = &(cpu->tlb.tab[lpgno & tlb_mask]);
	if (ent->lpgno != lpgno || ent->generation != cpu->tlb.generation) {
		if (!tlb_refill(cpu, ent, lpgno)) {
			cpu->cr2 = laddr;
			cpu->excno = EX_PF;
			cpu->excerr = 0;
			if (rwm & 2)
				cpu->excerr |= 2;
			if (cpl)
				cpu->excerr |= 4;
			return false;
		}
	}
	if (ent->pte_lookup[cpl > 0][rwm > 1]) {
		cpu->cr2 = laddr;
		cpu->excno = EX_PF;
		cpu->excerr = 1;
		if (rwm & 2)
			cpu->excerr |= 2;
		if (cpl)
			cpu->excerr |= 4;
		ent->lpgno = -1;
		return false;
	}
	*paddr = ent->xaddr ^ laddr;
	if (rwm & 2) {
		*(ent->ppte) |= 1 << 6; // dirty
//		pstore8(cpu, ent->ppte,
//			pload8(cpu, ent->ppte) | (1 << 6)); // dirty
	}
	return true;
}

static bool IRAM_ATTR translate_laddr(CPUI386 *cpu, OptAddr *res, int rwm, uword laddr, int size, int cpl)
{
	if (cpu->cr0 & CR0_PG) {
		uword lpgno = laddr >> 12;
		uword paddr;
		TRY(translate_lpgno(cpu, rwm, lpgno, laddr, cpl, &paddr));
		res->res = ADDR_OK1;
		res->addr1 = paddr;
		if ((laddr & 0xfff) > 0x1000 - size) {
			lpgno++;
			TRY(translate_lpgno(cpu, rwm, lpgno, lpgno << 12, cpl, &paddr));
			res->res = ADDR_OK2;
			res->addr2 = paddr;
		}
	} else {
		res->res = ADDR_OK1;
		res->addr1 = laddr & cpu->a20_mask;
	}
	return true;
}

/* Non-throwing full-span segment check for REP string fast paths.
 * Returns false if taking a bulk memcpy/memset would bypass a #GP/#SS that
 * scalar accesses would have raised. */
static inline bool seg_span_fast_ok(CPUI386 *cpu, int seg, int rwm, uword addr, uword bytes)
{
	if (likely(bytes == 0))
		return true;
	/* Match segcheck(): only enforce in protected mode outside VM86. */
	if (!(cpu->cr0 & 1) || (cpu->flags & VM))
		return true;

	/* Null selector check. */
	if (cpu->seg[seg].limit == 0 && (cpu->seg[seg].sel & ~0x3) == 0)
		return false;

	uword flags = cpu->seg[seg].flags;
	uword limit = cpu->seg[seg].limit;
	bool is_code = !!(flags & 0x8);
	bool rw = !!(flags & 0x2);
	bool expand_down = !is_code && ((flags & 0xc) == 0x4);
	uword max_off = (flags & SEG_B_BIT) ? 0xffffffffu : 0xffffu;
	uword last = addr + bytes - 1;

	/* Writes require writable data segments. */
	if ((rwm & 2) && (is_code || !rw))
		return false;
	/* Overflow in end computation is invalid span. */
	if (last < addr)
		return false;

	if (!expand_down)
		return last <= limit;

	/* Expand-down: (limit..max_off] */
	if (addr <= limit || last > max_off)
		return false;
	return true;
}

static bool IRAM_ATTR segcheck(CPUI386 *cpu, int rwm, int seg, uword addr, int size)
{
	if ((cpu->cr0 & 1) && !(cpu->flags & VM)) {
		/* null selector check */
		if (cpu->seg[seg].limit == 0 && (cpu->seg[seg].sel & ~0x3) == 0) {
//			dolog("segcheck: seg %d is null %x\n", seg, cpu->seg[seg].sel);
			THROW(seg == SEG_SS ? EX_SS : EX_GP, 0);
		}
		/* Segment limit and writability checks (protected mode / VM86).
		 * This must run before paging translation so out-of-bounds selector
		 * accesses raise #GP/#SS rather than being misreported as #PF.
		 * Re-enabled: proven not the cause of DOSX regression. */
		{
			uword flags = cpu->seg[seg].flags;
			uword limit = cpu->seg[seg].limit;
			bool is_code = !!(flags & 0x8);
			bool rw = !!(flags & 0x2);
			bool expand_down = !is_code && ((flags & 0xc) == 0x4);
			uword max_off = (flags & SEG_B_BIT) ? 0xffffffffu : 0xffffu;
			uword last = addr + (uword)size - 1;
			bool over;

			if (unlikely(size <= 0))
				return true;

			/* Writes require writable data segments; code/read-only data trap. */
			if ((rwm & 2) && (is_code || !rw)) {
				if (unlikely(cpu_diag_enabled && win386_diag.active && cpu->cpl == 3)) {
					wfw_monitor.segchk_last.valid = true;
					wfw_monitor.segchk_last.cs = cpu->seg[SEG_CS].sel;
					wfw_monitor.segchk_last.eip = cpu->ip;
					wfw_monitor.segchk_last.seg = (uint8_t)seg;
					wfw_monitor.segchk_last.why = 1;
					wfw_monitor.segchk_last.rwm = (uint8_t)rwm;
					wfw_monitor.segchk_last.size = (uint8_t)size;
					wfw_monitor.segchk_last.sel = cpu->seg[seg].sel;
					wfw_monitor.segchk_last.flags = (uint16_t)flags;
					wfw_monitor.segchk_last.limit = limit;
					wfw_monitor.segchk_last.addr = addr;
					wfw_monitor.segchk_last.bx = REGi(3) & 0xffff;
					wfw_monitor.segchk_last.di = REGi(7) & 0xffff;
					wfw_monitor.segchk_last.bp = REGi(5) & 0xffff;
					wfw_monitor.segchk_last.bl = REGi(3) & 0xff;
					wfw_monitor.segchk_last.ds_word_ok = false;
					wfw_monitor.segchk_last.ds_word = 0;
					wfw_monitor.segchk_last.ss_bp_m6_ok = false;
					wfw_monitor.segchk_last.ss_bp_m6 = 0;
				}
				THROW(seg == SEG_SS ? EX_SS : EX_GP, 0);
			}

			if (!expand_down) {
				/* Expand-up: [0..limit] valid. */
				over = (last < addr) || (last > limit);
			} else {
				/* Expand-down: (limit..max_off] valid. */
				over = (last < addr) || (addr <= limit) || (last > max_off);
			}

			if (over) {
				if (unlikely(cpu_diag_enabled && win386_diag.active && cpu->cpl == 3)) {
					wfw_monitor.segchk_last.valid = true;
					wfw_monitor.segchk_last.cs = cpu->seg[SEG_CS].sel;
					wfw_monitor.segchk_last.eip = cpu->ip;
					wfw_monitor.segchk_last.seg = (uint8_t)seg;
					wfw_monitor.segchk_last.why = 2;
					wfw_monitor.segchk_last.rwm = (uint8_t)rwm;
					wfw_monitor.segchk_last.size = (uint8_t)size;
					wfw_monitor.segchk_last.sel = cpu->seg[seg].sel;
					wfw_monitor.segchk_last.flags = (uint16_t)flags;
					wfw_monitor.segchk_last.limit = limit;
					wfw_monitor.segchk_last.addr = addr;
					wfw_monitor.segchk_last.bx = REGi(3) & 0xffff;
					wfw_monitor.segchk_last.di = REGi(7) & 0xffff;
					wfw_monitor.segchk_last.bp = REGi(5) & 0xffff;
					wfw_monitor.segchk_last.bl = REGi(3) & 0xff;
					wfw_monitor.segchk_last.ds_word_ok = false;
					wfw_monitor.segchk_last.ds_word = 0;
					wfw_monitor.segchk_last.ss_bp_m6_ok = false;
					wfw_monitor.segchk_last.ss_bp_m6 = 0;
					{
						uint16_t w = 0;
						uint32_t la = cpu->seg[seg].base + addr;
						if (read_laddr_u16_noexcept(cpu, la, cpu->cpl, &w)) {
							wfw_monitor.segchk_last.ds_word_ok = true;
							wfw_monitor.segchk_last.ds_word = w;
						}
						{
							uint32_t spm = cpu->sp_mask;
							uint32_t bp = REGi(5) & spm;
							uint32_t off = (bp - 6) & spm;
							uint32_t ss_la = cpu->seg[SEG_SS].base + off;
							if (read_laddr_u16_noexcept(cpu, ss_la, cpu->cpl, &w)) {
								wfw_monitor.segchk_last.ss_bp_m6_ok = true;
								wfw_monitor.segchk_last.ss_bp_m6 = w;
							}
						}
					}
					/* Capture instruction bytes at fault site
					 * Need linear→physical translation if paging on */
					{
						uint32_t insn_la = cpu->seg[SEG_CS].base + cpu->ip;
						wfw_monitor.segchk_insn_len = 0;
						wfw_monitor.segchk_insn_captured = true;
						for (int ib = 0; ib < 16; ib++) {
							uint32_t la = insn_la + ib;
							uint32_t pa;
							if (cpu->cr0 & (1u << 31)) {
								/* Paging enabled: walk CR3→PDE→PTE */
								uint32_t pde_a = (cpu->cr3 & 0xfffff000) | ((la >> 20) & 0xffc);
								if (pde_a + 4 > (uint32_t)cpu->phys_mem_size) break;
								uint32_t pde = *(uint32_t *)&cpu->phys_mem[pde_a];
								if (!(pde & 1)) break;
								uint32_t pte_a = (pde & 0xfffff000) | ((la >> 10) & 0xffc);
								if (pte_a + 4 > (uint32_t)cpu->phys_mem_size) break;
								uint32_t pte = *(uint32_t *)&cpu->phys_mem[pte_a];
								if (!(pte & 1)) break;
								pa = (pte & 0xfffff000) | (la & 0xfff);
							} else {
								pa = la;
							}
							if (pa < (uint32_t)cpu->phys_mem_size) {
								wfw_monitor.segchk_insn_bytes[ib] = (uint8_t)cpu->phys_mem[pa];
								wfw_monitor.segchk_insn_len = ib + 1;
							} else break;
						}
					}
				}
				THROW(seg == SEG_SS ? EX_SS : EX_GP, 0);
			}
		}
		/* (segcheck limits re-enabled — not the cause) */
	}
	return true;
}

static bool IRAM_ATTR translate(CPUI386 *cpu, OptAddr *res, int rwm, int seg, uword addr, int size, int cpl)
{
	assert(seg != -1);
	uword laddr;

	/* Fast path: when ALL segments are flat, skip per-segment check entirely */
	if (likely(cpu->all_segs_flat)) {
		/* Null selectors are still invalid in protected mode even if every
		 * segment currently looks flat; keep architectural fault behavior. */
		TRYL(segcheck(cpu, rwm, seg, addr, size));
		return translate_laddr(cpu, res, rwm, addr, size, cpl);
	}

	if (likely(SEG_IS_FLAT(cpu, seg))) {
		laddr = addr;
	} else {
		laddr = cpu->seg[seg].base + addr;
		TRYL(segcheck(cpu, rwm, seg, addr, size));
	}

	return translate_laddr(cpu, res, rwm, laddr, size, cpl);
}

static bool IRAM_ATTR translate8r(CPUI386 *cpu, OptAddr *res, int seg, uword addr)
{
	assert(seg != -1);
	uword laddr;

	/* Fast path: all segments flat → addr IS the linear address */
	if (likely(cpu->all_segs_flat)) {
		TRYL(segcheck(cpu, 1, seg, addr, 1));
		laddr = addr;
	} else if (likely(SEG_IS_FLAT(cpu, seg))) {
		laddr = addr;
	} else {
		laddr = cpu->seg[seg].base + addr;
		TRYL(segcheck(cpu, 1, seg, addr, 1));
	}

	if (cpu->cr0 & CR0_PG) {
		uword lpgno = laddr >> 12;
		struct tlb_entry *ent = &(cpu->tlb.tab[lpgno & tlb_mask]);
		if (ent->lpgno != lpgno || ent->generation != cpu->tlb.generation) {
			if (!tlb_refill(cpu, ent, lpgno)) {
				cpu->cr2 = laddr;
				cpu->excno = EX_PF;
				cpu->excerr = 0;
				if (cpu->cpl)
					cpu->excerr |= 4;
				return false;
			}
		}
		if (ent->pte_lookup[cpu->cpl > 0][0]) {
			cpu->cr2 = laddr;
			cpu->excno = EX_PF;
			cpu->excerr = 1;
			if (cpu->cpl)
				cpu->excerr |= 4;
			ent->lpgno = -1;
			return false;
		}
		res->res = ADDR_OK1;
		res->addr1 = ent->xaddr ^ laddr;
	} else {
		res->res = ADDR_OK1;
		res->addr1 = laddr & cpu->a20_mask;
	}

	return true;
}

static inline bool translate8(CPUI386 *cpu, OptAddr *res, int rwm, int seg, uword addr)
{
	return translate(cpu, res, rwm, seg, addr, 1, cpu->cpl);
}

static inline bool translate16(CPUI386 *cpu, OptAddr *res, int rwm, int seg, uword addr)
{
	return translate(cpu, res, rwm, seg, addr, 2, cpu->cpl);
}

static inline bool translate32(CPUI386 *cpu, OptAddr *res, int rwm, int seg, uword addr)
{
	return translate(cpu, res, rwm, seg, addr, 4, cpu->cpl);
}

static inline bool in_iomem(uword addr)
{
	return (addr >= 0xa0000 && addr < 0xc0000) || addr >= 0xe0000000;
}

/* Check if address is in VGA direct-access range (0xA0000-0xAFFFF, 64KB) */
static inline bool in_vga_direct(uword addr)
{
	return addr >= 0xa0000 && addr < 0xb0000;
}

static u8 IRAM_ATTR load8(CPUI386 *cpu, OptAddr *res)
{
	uword addr = res->addr1;
	/* ~99% of loads are to normal RAM below VGA range */
	if (likely(addr < 0xa0000))
		return pload8(cpu, addr);
	/* Fast path for VGA direct access (chain-4 mode) */
	if (cpu->cb.vga_direct && in_vga_direct(addr))
		return cpu->cb.vga_direct[addr - 0xa0000];
	/* Fast path for Mode X reads (inline to avoid callback overhead) */
	if (cpu->cb.vga_modex_ram && in_vga_direct(addr)) {
		uword vga_addr = addr - 0xa0000;
		if (vga_addr * 4 < cpu->cb.vga_modex_ram_size) {
			/* Load latch and return selected plane */
			*cpu->cb.vga_modex_latch = ((u32 *)cpu->cb.vga_modex_ram)[vga_addr];
			return (u8)(*cpu->cb.vga_modex_latch >> (cpu->cb.vga_modex_read_plane * 8));
		}
	}
	/* Fast path for text mode reads (odd/even interleave) */
	if (cpu->cb.vga_text_ram && addr >= cpu->cb.vga_text_phys_base
	                         && addr < cpu->cb.vga_text_phys_end) {
		u32 off = addr - cpu->cb.vga_text_phys_base;
		u32 vram_off = ((off & ~1u) << 1) | (off & 1u);
		return cpu->cb.vga_text_ram[vram_off];
	}
	if (in_iomem(addr) && cpu->cb.iomem_read8)
		return cpu->cb.iomem_read8(cpu->cb.iomem, addr);
	if (unlikely(addr >= cpu->phys_mem_size)) {
		return 0;
	}
	return pload8(cpu, addr);
}

static u16 IRAM_ATTR load16(CPUI386 *cpu, OptAddr *res)
{
	if (likely(res->addr1 < 0xa0000)) {
		if (likely(res->res == ADDR_OK1))
			return pload16(cpu, res->addr1);
		return pload8(cpu, res->addr1) | (pload8(cpu, res->addr2) << 8);
	}
	/* Fast path for VGA direct access (chain-4 mode) */
	if (cpu->cb.vga_direct && in_vga_direct(res->addr1) && in_vga_direct(res->addr1 + 1))
		return *(u16 *)(cpu->cb.vga_direct + res->addr1 - 0xa0000);
	/* Fast path for text mode reads (odd/even interleave) */
	if (cpu->cb.vga_text_ram && res->addr1 >= cpu->cb.vga_text_phys_base
	                         && res->addr1 + 1 < cpu->cb.vga_text_phys_end) {
		u32 off = res->addr1 - cpu->cb.vga_text_phys_base;
		if (!(off & 1)) {
			/* Aligned char+attr read: adjacent in VRAM */
			return *(u16 *)(cpu->cb.vga_text_ram + (off << 1));
		} else {
			/* Unaligned: two separate byte reads */
			u32 v0 = ((off & ~1u) << 1) | 1u;
			u32 v1 = (((off + 1) & ~1u) << 1);
			return cpu->cb.vga_text_ram[v0] | ((u16)cpu->cb.vga_text_ram[v1] << 8);
		}
	}
	if (in_iomem(res->addr1) && cpu->cb.iomem_read16)
		return cpu->cb.iomem_read16(cpu->cb.iomem, res->addr1);
	if (unlikely(res->addr1 >= cpu->phys_mem_size)) {
		return 0;
	}
	if (likely(res->res == ADDR_OK1))
		return pload16(cpu, res->addr1);
	else
		return pload8(cpu, res->addr1) | (pload8(cpu, res->addr2) << 8);
}

static u32 IRAM_ATTR load32(CPUI386 *cpu, OptAddr *res)
{
	if (likely(res->addr1 < 0xa0000)) {
		if (likely(res->res == ADDR_OK1))
			return pload32(cpu, res->addr1);
		goto load32_slow;
	}
	/* Fast path for VGA direct access (chain-4 mode) */
	if (cpu->cb.vga_direct && in_vga_direct(res->addr1) && in_vga_direct(res->addr1 + 3))
		return *(u32 *)(cpu->cb.vga_direct + res->addr1 - 0xa0000);
	if (in_iomem(res->addr1) && cpu->cb.iomem_read32)
		return cpu->cb.iomem_read32(cpu->cb.iomem, res->addr1);
	if (unlikely(res->addr1 >= cpu->phys_mem_size)) {
		return 0;
	}
	if (likely(res->res == ADDR_OK1)) {
		return pload32(cpu, res->addr1);
	}
	load32_slow: {
		switch(res->addr1 & 0xf) {
		case 0xf:
			return pload8(cpu, res->addr1) | (pload16(cpu, res->addr2) << 8) |
				(pload8(cpu, res->addr2 + 2) << 24);
		case 0xe:
			return pload16(cpu, res->addr1) | (pload16(cpu, res->addr2) << 16);
		case 0xd:
			return pload8(cpu, res->addr1) | (pload16(cpu, res->addr1 + 1) << 8) |
				(pload8(cpu, res->addr2) << 24);
		}
	}
	assert(false);
}

static void IRAM_ATTR store8(CPUI386 *cpu, OptAddr *res, u8 val)
{
	uword addr = res->addr1;
	/* PTE watchpoint: detect byte writes overlapping watched PTE */
	if (unlikely(cpu_diag_enabled && pf_diag.active && addr >= pf_diag.watch_phys && addr < pf_diag.watch_phys + 4)) {
		pf_diag.write_count++;
		pf_diag.last_write_val = val;
		pf_diag.last_write_cs = cpu->seg[SEG_CS].sel;
		pf_diag.last_write_eip = cpu->ip;
	}
	if (unlikely(cpu_diag_enabled && win386_diag.active))
		wfw_note_ldt_window_store(cpu, addr, 1, val);
	if (likely(addr < 0xa0000)) {
		if (likely(addr < cpu->phys_mem_size))
			pstore8(cpu, addr, val);
		return;
	}
	/* Fast path for VGA direct access (chain-4 mode) */
	if (cpu->cb.vga_direct && in_vga_direct(addr)) {
		cpu->cb.vga_direct[addr - 0xa0000] = val;
		if (cpu->cb.vga_direct_write_notify)
			cpu->cb.vga_direct_write_notify(cpu->cb.iomem, addr, 1);
		return;
	}
	/* Fast path for Mode X writes (inline to avoid callback overhead) */
	if (cpu->cb.vga_modex_ram && in_vga_direct(addr)) {
		uword vga_addr = addr - 0xa0000;
		if (vga_addr * 4 < cpu->cb.vga_modex_ram_size) {
			u8 wm = cpu->cb.vga_modex_write_mode;
			u8 pmask = cpu->cb.vga_modex_plane_mask;
			u8 *vram = cpu->cb.vga_modex_ram;

			if (wm == 1) {
				/* Write mode 1: latch copy */
				u32 latch = *cpu->cb.vga_modex_latch;
				if (pmask == 0x0f) {
					((u32 *)vram)[vga_addr] = latch;
				} else {
					u32 wmask = ((pmask & 1) ? 0x000000ff : 0) |
					            ((pmask & 2) ? 0x0000ff00 : 0) |
					            ((pmask & 4) ? 0x00ff0000 : 0) |
					            ((pmask & 8) ? 0xff000000 : 0);
					((u32 *)vram)[vga_addr] = (((u32 *)vram)[vga_addr] & ~wmask) |
					                          (latch & wmask);
				}
				/* Notify for dirty page tracking (vga_addr is dword offset, convert to bytes) */
				if (cpu->cb.vga_direct_write_notify)
					cpu->cb.vga_direct_write_notify(cpu->cb.iomem, 0xa0000 + vga_addr * 4, 4);
				return;
			} else if (wm == 0) {
				/* Write mode 0: simple byte write */
				u32 val32 = val | (val << 8);
				val32 |= val32 << 16;
				if (pmask == 0x0f) {
					((u32 *)vram)[vga_addr] = val32;
				} else if ((pmask & (pmask - 1)) == 0 && pmask != 0) {
					/* Single plane: direct byte write */
					int plane = (pmask == 1) ? 0 : (pmask == 2) ? 1 :
					            (pmask == 4) ? 2 : 3;
					vram[vga_addr * 4 + plane] = val;
				} else {
					u32 wmask = ((pmask & 1) ? 0x000000ff : 0) |
					            ((pmask & 2) ? 0x0000ff00 : 0) |
					            ((pmask & 4) ? 0x00ff0000 : 0) |
					            ((pmask & 8) ? 0xff000000 : 0);
					((u32 *)vram)[vga_addr] = (((u32 *)vram)[vga_addr] & ~wmask) |
					                          (val32 & wmask);
				}
				/* Notify for dirty page tracking (vga_addr is dword offset, convert to bytes) */
				if (cpu->cb.vga_direct_write_notify)
					cpu->cb.vga_direct_write_notify(cpu->cb.iomem, 0xa0000 + vga_addr * 4, 4);
				return;
			}
			/* wm == 0xFF means use callback (complex VGA state) */
		}
	}
	/* Fast path for text mode writes (odd/even interleave) */
	if (cpu->cb.vga_text_ram && addr >= cpu->cb.vga_text_phys_base
	                         && addr < cpu->cb.vga_text_phys_end) {
		u32 off = addr - cpu->cb.vga_text_phys_base;
		u32 vram_off = ((off & ~1u) << 1) | (off & 1u);
		cpu->cb.vga_text_ram[vram_off] = val;
		*cpu->cb.vga_text_dirty_pages |= (1ULL << (vram_off >> 12));
		return;
	}
	if (in_iomem(addr) && cpu->cb.iomem_write8) {
		cpu->cb.iomem_write8(cpu->cb.iomem, addr, val);
		return;
	}
	if (unlikely(addr >= cpu->phys_mem_size)) {
		if (unlikely(cpu_diag_enabled && pf_diag.active)) {
			pf_diag.oob_store_count++;
			pf_diag.oob_store_last_addr = addr;
		}
		return;
	}
	pstore8(cpu, addr, val);
}

static void IRAM_ATTR store16(CPUI386 *cpu, OptAddr *res, u16 val)
{
	/* PTE watchpoint: detect word writes overlapping watched PTE */
	if (unlikely(cpu_diag_enabled && pf_diag.active && res->addr1 < pf_diag.watch_phys + 4 && res->addr1 + 2 > pf_diag.watch_phys)) {
		pf_diag.write_count++;
		pf_diag.last_write_val = val;
		pf_diag.last_write_cs = cpu->seg[SEG_CS].sel;
		pf_diag.last_write_eip = cpu->ip;
	}
	if (unlikely(cpu_diag_enabled && win386_diag.active))
		wfw_note_ldt_window_store(cpu, res->addr1, 2, val);
	if (likely(res->addr1 < 0xa0000)) {
		if (likely(res->res == ADDR_OK1))
			pstore16(cpu, res->addr1, val);
		else {
			pstore8(cpu, res->addr1, val);
			pstore8(cpu, res->addr2, val >> 8);
		}
		return;
	}
	/* Fast path for VGA direct access (chain-4 mode) */
	if (cpu->cb.vga_direct && in_vga_direct(res->addr1) && in_vga_direct(res->addr1 + 1)) {
		*(u16 *)(cpu->cb.vga_direct + res->addr1 - 0xa0000) = val;
		if (cpu->cb.vga_direct_write_notify)
			cpu->cb.vga_direct_write_notify(cpu->cb.iomem, res->addr1, 2);
		return;
	}
	/* Fast path for text mode writes (odd/even interleave) */
	if (cpu->cb.vga_text_ram && res->addr1 >= cpu->cb.vga_text_phys_base
	                         && res->addr1 + 1 < cpu->cb.vga_text_phys_end) {
		u32 off = res->addr1 - cpu->cb.vga_text_phys_base;
		if (!(off & 1)) {
			/* Aligned char+attr write: adjacent in VRAM */
			*(u16 *)(cpu->cb.vga_text_ram + (off << 1)) = val;
		} else {
			/* Unaligned: two separate byte writes */
			u32 v0 = ((off & ~1u) << 1) | 1u;
			u32 v1 = (((off + 1) & ~1u) << 1);
			cpu->cb.vga_text_ram[v0] = val & 0xFF;
			cpu->cb.vga_text_ram[v1] = val >> 8;
		}
		*cpu->cb.vga_text_dirty_pages |= (1ULL << ((off << 1) >> 12));
		return;
	}
	if (in_iomem(res->addr1) && cpu->cb.iomem_write16) {
		cpu->cb.iomem_write16(cpu->cb.iomem, res->addr1, val);
		return;
	}
	if (unlikely(res->addr1 >= cpu->phys_mem_size)) {
		if (unlikely(cpu_diag_enabled && pf_diag.active)) {
			pf_diag.oob_store_count++;
			pf_diag.oob_store_last_addr = res->addr1;
		}
		return;
	}
	if (likely(res->res == ADDR_OK1)) {
		pstore16(cpu, res->addr1, val);
	} else {
		pstore8(cpu, res->addr1, val);
		pstore8(cpu, res->addr2, val >> 8);
	}
}

static void IRAM_ATTR store32(CPUI386 *cpu, OptAddr *res, u32 val)
{
	/* PTE watchpoint: detect dword writes overlapping watched PTE */
	if (unlikely(cpu_diag_enabled && pf_diag.active && res->addr1 < pf_diag.watch_phys + 4 && res->addr1 + 4 > pf_diag.watch_phys)) {
		pf_diag.write_count++;
		pf_diag.last_write_val = val;
		pf_diag.last_write_cs = cpu->seg[SEG_CS].sel;
		pf_diag.last_write_eip = cpu->ip;
	}
	if (unlikely(cpu_diag_enabled && win386_diag.active))
		wfw_note_ldt_window_store(cpu, res->addr1, 4, val);
	if (likely(res->addr1 < 0xa0000)) {
		if (likely(res->res == ADDR_OK1))
			pstore32(cpu, res->addr1, val);
		else {
			switch(res->addr1 & 0xf) {
			case 0xf:
				pstore8(cpu, res->addr1, val);
				pstore16(cpu, res->addr2, val >> 8);
				pstore8(cpu, res->addr2 + 2, val >> 24);
				break;
			case 0xe:
				pstore16(cpu, res->addr1, val);
				pstore16(cpu, res->addr2, val >> 16);
				break;
			case 0xd:
				pstore8(cpu, res->addr1, val);
				pstore16(cpu, res->addr1 + 1, val >> 8);
				pstore8(cpu, res->addr2, val >> 24);
				break;
			}
		}
		return;
	}
	/* Fast path for VGA direct access (chain-4 mode) */
	if (cpu->cb.vga_direct && in_vga_direct(res->addr1) && in_vga_direct(res->addr1 + 3)) {
		*(u32 *)(cpu->cb.vga_direct + res->addr1 - 0xa0000) = val;
		if (cpu->cb.vga_direct_write_notify)
			cpu->cb.vga_direct_write_notify(cpu->cb.iomem, res->addr1, 4);
		return;
	}
	if (in_iomem(res->addr1) && cpu->cb.iomem_write32) {
		cpu->cb.iomem_write32(cpu->cb.iomem, res->addr1, val);
		return;
	}
	if (unlikely(res->addr1 >= cpu->phys_mem_size)) {
		if (unlikely(cpu_diag_enabled && pf_diag.active)) {
			pf_diag.oob_store_count++;
			pf_diag.oob_store_last_addr = res->addr1;
		}
		return;
	}
	if (likely(res->res == ADDR_OK1)) {
		pstore32(cpu, res->addr1, val);
	} else {
		switch(res->addr1 & 0xf) {
		case 0xf:
			pstore8(cpu, res->addr1, val);
			pstore16(cpu, res->addr2, val >> 8);
			pstore8(cpu, res->addr2 + 2, val >> 24);
			break;
		case 0xe:
			pstore16(cpu, res->addr1, val);
			pstore16(cpu, res->addr2, val >> 16);
			break;
		case 0xd:
			pstore8(cpu, res->addr1, val);
			pstore16(cpu, res->addr1 + 1, val >> 8);
			pstore8(cpu, res->addr2, val >> 24);
			break;
		}
	}
}

#define LOADSTORE(BIT) \
bool cpu_load ## BIT(CPUI386 *cpu, int seg, uword addr, u ## BIT *res) \
{ \
	OptAddr o; \
	TRY(translate ## BIT(cpu, &o, 1, seg, addr)); \
	*res = load ## BIT(cpu, &o); \
	return true; \
} \
\
bool cpu_store ## BIT(CPUI386 *cpu, int seg, uword addr, u ## BIT val) \
{ \
	OptAddr o; \
	TRY(translate ## BIT(cpu, &o, 2, seg, addr)); \
	store ## BIT(cpu, &o, val); \
	return true; \
} \

LOADSTORE(8)
LOADSTORE(16)
LOADSTORE(32)

/* Optimized 64-bit load: single translation when not crossing page boundary */
bool IRAM_ATTR cpu_load64(CPUI386 *cpu, int seg, uword addr, uint64_t *res)
{
	/* Fast path: if both dwords on same page, translate once */
	if (likely((addr & 0xfff) <= 0xff8)) {
		OptAddr o;
		TRY(translate32(cpu, &o, 1, seg, addr));
		u32 lo = load32(cpu, &o);
		/* Same page, just offset the physical address */
		o.addr1 += 4;
		if (o.res == ADDR_OK2) o.addr2 += 4;
		u32 hi = load32(cpu, &o);
		*res = ((uint64_t)hi << 32) | lo;
		return true;
	}
	/* Slow path: crosses page boundary */
	u32 lo, hi;
	TRY(cpu_load32(cpu, seg, addr, &lo));
	TRY(cpu_load32(cpu, seg, addr + 4, &hi));
	*res = ((uint64_t)hi << 32) | lo;
	return true;
}

/* Optimized 64-bit store: single translation when not crossing page boundary */
bool IRAM_ATTR cpu_store64(CPUI386 *cpu, int seg, uword addr, uint64_t val)
{
	/* Fast path: if both dwords on same page, translate once */
	if (likely((addr & 0xfff) <= 0xff8)) {
		OptAddr o;
		TRY(translate32(cpu, &o, 2, seg, addr));
		store32(cpu, &o, (u32)val);
		o.addr1 += 4;
		if (o.res == ADDR_OK2) o.addr2 += 4;
		store32(cpu, &o, (u32)(val >> 32));
		return true;
	}
	/* Slow path: crosses page boundary */
	TRY(cpu_store32(cpu, seg, addr, (u32)val));
	TRY(cpu_store32(cpu, seg, addr + 4, (u32)(val >> 32)));
	return true;
}

static bool IRAM_ATTR peek8(CPUI386 *cpu, u8 *val)
{
	/* Prefetch pointer fast path */
	if (likely(cpu->pf_pos < cpu->pf_avail)) {
		*val = cpu->pf_ptr[cpu->pf_pos];
		return true;
	}
	/* Fast path for flat CS: laddr = next_ip (skip base addition) */
	uword laddr = likely(SEG_IS_FLAT(cpu, SEG_CS))
		? cpu->next_ip
		: cpu->seg[SEG_CS].base + cpu->next_ip;
	if (likely((laddr ^ cpu->ifetch.laddr) < 4096)) {
		*val = pload8(cpu, cpu->ifetch.xaddr ^ laddr);
		return true;
	}
	OptAddr res;
	TRY(translate8r(cpu, &res, SEG_CS, cpu->next_ip));
	*val = load8(cpu, &res);
	cpu->ifetch.laddr = laddr & (~4095ul);
	cpu->ifetch.xaddr = res.addr1 ^ laddr;
	return true;
}

static bool IRAM_ATTR fetch8(CPUI386 *cpu, u8 *val)
{
	/* Prefetch buffer fast path */
	if (likely(cpu->pf_pos < cpu->pf_avail)) {
		*val = cpu->pf_ptr[cpu->pf_pos++];
		cpu->next_ip++;
		return true;
	}
	TRY(peek8(cpu, val));
	cpu->next_ip++;
	return true;
}

static bool IRAM_ATTR fetch16(CPUI386 *cpu, u16 *val)
{
	/* Prefetch buffer fast path */
	if (likely(cpu->pf_pos + 2 <= cpu->pf_avail)) {
		int p = cpu->pf_pos;
		*val = cpu->pf_ptr[p] | ((u16)cpu->pf_ptr[p + 1] << 8);
		cpu->pf_pos = p + 2;
		cpu->next_ip += 2;
		return true;
	}
	cpu->pf_avail = 0; /* Exceeded prefetch, disable */
	/* Fast path for flat CS */
	uword laddr = likely(SEG_IS_FLAT(cpu, SEG_CS))
		? cpu->next_ip
		: cpu->seg[SEG_CS].base + cpu->next_ip;
	if (likely((laddr ^ cpu->ifetch.laddr) < 4095)) {
		*val = pload16(cpu, cpu->ifetch.xaddr ^ laddr);
	} else {
		OptAddr res;
		TRY(translate16(cpu, &res, 1, SEG_CS, cpu->next_ip));
		*val = load16(cpu, &res);
	}
	cpu->next_ip += 2;
	return true;
}

static bool IRAM_ATTR fetch32(CPUI386 *cpu, u32 *val)
{
	/* Prefetch buffer fast path */
	if (likely(cpu->pf_pos + 4 <= cpu->pf_avail)) {
		int p = cpu->pf_pos;
		*val = cpu->pf_ptr[p] | ((u32)cpu->pf_ptr[p + 1] << 8)
		     | ((u32)cpu->pf_ptr[p + 2] << 16) | ((u32)cpu->pf_ptr[p + 3] << 24);
		cpu->pf_pos = p + 4;
		cpu->next_ip += 4;
		return true;
	}
	cpu->pf_avail = 0; /* Exceeded prefetch, disable */
	/* Fast path for flat CS */
	uword laddr = likely(SEG_IS_FLAT(cpu, SEG_CS))
		? cpu->next_ip
		: cpu->seg[SEG_CS].base + cpu->next_ip;
	if (likely((laddr ^ cpu->ifetch.laddr) < 4093)) {
		*val = pload32(cpu, cpu->ifetch.xaddr ^ laddr);
	} else {
		OptAddr res;
		TRY(translate32(cpu, &res, 1, SEG_CS, cpu->next_ip));
		*val = load32(cpu, &res);
	}
	cpu->next_ip += 4;
	return true;
}

/* insts decode && execute */
static inline bool modsib32(CPUI386 *cpu, int mod, int rm, uword *addr, int *seg)
{
	if (rm == 4) {
		u8 sib;
		TRY(fetch8(cpu, &sib));
		int b = sib & 7;
		if (b == 5 && mod == 0) {
			TRY(fetch32(cpu, addr));
		} else {
			*addr = REGi(b);
			// sp bp as base register
			if ((b == 4 || b == 5) && *seg == -1)
				*seg = SEG_SS;
		}
		int i = (sib >> 3) & 7;
		if (i != 4)
			*addr += REGi(i) << (sib >> 6);
	} else if (rm == 5 && mod == 0) {
		TRY(fetch32(cpu, addr));
	} else {
		*addr = REGi(rm);
		// bp as base register
		if (rm == 5 && *seg == -1)
			*seg = SEG_SS;
	}
	if (mod == 1) {
		u8 imm8;
		TRY(fetch8(cpu, &imm8));
		*addr += (s8) imm8;
	} else if (mod == 2) {
		u32 imm32;
		TRY(fetch32(cpu, &imm32));
		*addr += (s32) imm32;
	}
	if (*seg == -1)
		*seg = SEG_DS;
	return true;
}

static inline bool modsib16(CPUI386 *cpu, int mod, int rm, uword *addr, int *seg)
{
	if (rm == 6 && mod == 0) {
		u16 imm16;
		TRY(fetch16(cpu, &imm16));
		*addr = imm16;
	} else {
		switch(rm) {
		case 0: *addr = REGi(3) + REGi(6); break;
		case 1: *addr = REGi(3) + REGi(7); break;
		case 2: *addr = REGi(5) + REGi(6); break;
		case 3: *addr = REGi(5) + REGi(7); break;
		case 4: *addr = REGi(6); break;
		case 5: *addr = REGi(7); break;
		case 6: *addr = REGi(5); break;
		case 7: *addr = REGi(3); break;
		}
		if (mod == 1) {
			u8 imm8;
			TRY(fetch8(cpu, &imm8));
			*addr += (s8) imm8;
		} else if (mod == 2) {
			u16 imm16;
			TRY(fetch16(cpu, &imm16));
			*addr += imm16;
		}
		*addr &= 0xffff;
	}
	if (*seg == -1) {
		if (rm == 2 || rm == 3)
			*seg = SEG_SS;
		else if (mod != 0 && rm == 6)
			*seg = SEG_SS;
		else
			*seg = SEG_DS;
	}
	return true;
}

static bool IRAM_ATTR modsib(CPUI386 *cpu, int adsz16, int mod, int rm, uword *addr, int *seg)
{
	if (adsz16) return modsib16(cpu, mod, rm, addr, seg);
	else return modsib32(cpu, mod, rm, addr, seg);
}

static bool read_desc(CPUI386 *cpu, int sel, uword *w1, uword *w2)
{
	OptAddr meml;
	sel = sel & 0xffff;
	uword off = sel & ~0x7;
	uword base;
	uword limit;
	if (sel & 0x4) {
		base = cpu->seg[SEG_LDT].base;
		limit = cpu->seg[SEG_LDT].limit;
	} else {
		base = cpu->gdt.base;
		limit = cpu->gdt.limit;
	}

	if (off + 7 > limit) {
		static int rd_log_count = 0;
		if (rd_log_count < 5) {
			dolog("read_desc: sel %04x base %x limit %x off %x CS:EIP=%04x:%08x CPL=%d\n",
				sel, base, limit, off,
				cpu->seg[SEG_CS].sel, cpu->ip, cpu->cpl);
			rd_log_count++;
		}
		THROW(EX_GP, sel & ~0x3);
	}
	if (w1) {
		TRY(translate_laddr(cpu, &meml, 1, base + off, 4, 0));
		*w1 = load32(cpu, &meml);
	}
	TRY(translate_laddr(cpu, &meml, 1, base + off + 4, 4, 0));
	*w2 = load32(cpu, &meml);
	return true;
}

/* Update flat segment flag based on current segment state */
static inline void update_seg_flat(CPUI386 *cpu, int seg)
{
	/* Flat = base is 0 and limit is 4GB (0xFFFFFFFF) */
	if (cpu->seg[seg].base == 0 && cpu->seg[seg].limit == 0xFFFFFFFF) {
		cpu->seg_flat |= (1 << seg);
	} else {
		cpu->seg_flat &= ~(1 << seg);
	}
	/* 0x3F = bits 0-5 (ES,CS,SS,DS,FS,GS) all set */
	cpu->all_segs_flat = (cpu->seg_flat == 0x3F);
}

/* Capture source/class for DS<-0127 loads (e.g. MOV DS,r/m16 vs POP DS vs LDS). */
static inline void wfw_note_ds0127_load(CPUI386 *cpu, uint16_t sel_val)
{
	if (unlikely(!(cpu_diag_enabled && win386_diag.active)))
		return;
	if (sel_val != 0x0127)
		return;
	if (wfw_monitor.ds0127_load_count >= 8)
		return;

	int i = wfw_monitor.ds0127_load_count++;
	wfw_monitor.ds0127_load_log[i].cs = cpu->seg[SEG_CS].sel;
	wfw_monitor.ds0127_load_log[i].eip = cpu->ip;
	wfw_monitor.ds0127_load_log[i].src_kind = 5;
	wfw_monitor.ds0127_load_log[i].src_reg = 0xff;
	wfw_monitor.ds0127_load_log[i].src_val = sel_val;
	wfw_monitor.ds0127_load_log[i].bx = REGi(3) & 0xffff;
	wfw_monitor.ds0127_load_log[i].di = REGi(7) & 0xffff;
	wfw_monitor.ds0127_load_log[i].bp = REGi(5) & 0xffff;
	wfw_monitor.ds0127_load_log[i].ss = cpu->seg[SEG_SS].sel;
	wfw_monitor.ds0127_load_log[i].sp = REGi(4);
	wfw_monitor.ds0127_load_log[i].bp_m6_ok = false;
	wfw_monitor.ds0127_load_log[i].bp_m6_val = 0;
	wfw_monitor.ds0127_load_log[i].op = 0;
	wfw_monitor.ds0127_load_log[i].modrm = 0;
	wfw_monitor.ds0127_load_log[i].blen = 0;

	for (int b = 0; b < 8; b++) {
		uint8_t v;
		if (!read_seg_u8_noexcept(cpu, SEG_CS, cpu->ip + (uword)b, cpu->cpl, &v))
			break;
		wfw_monitor.ds0127_load_log[i].bytes[wfw_monitor.ds0127_load_log[i].blen++] = v;
	}
	if (wfw_monitor.ds0127_load_log[i].blen == 0)
		return;

	uint8_t op = wfw_monitor.ds0127_load_log[i].bytes[0];
	wfw_monitor.ds0127_load_log[i].op = op;
	switch (op) {
	case 0x1f: { /* POP DS */
		wfw_monitor.ds0127_load_log[i].src_kind = 3;
		uint16_t spm = REGi(4) & cpu->sp_mask;
		uint16_t top;
		if (read_seg_u16_noexcept(cpu, SEG_SS, spm, cpu->cpl, &top))
			wfw_monitor.ds0127_load_log[i].src_val = top;
		break;
	}
	case 0x8e: { /* MOV Sreg, r/m16 */
		if (wfw_monitor.ds0127_load_log[i].blen < 2)
			break;
		uint8_t modrm = wfw_monitor.ds0127_load_log[i].bytes[1];
		wfw_monitor.ds0127_load_log[i].modrm = modrm;
		uint8_t reg = (modrm >> 3) & 7;
		uint8_t mod = modrm >> 6;
		uint8_t rm = modrm & 7;
		if (reg != 3) /* not DS destination */
			break;
		if (mod == 3) {
			wfw_monitor.ds0127_load_log[i].src_kind = 1;
			wfw_monitor.ds0127_load_log[i].src_reg = rm;
			wfw_monitor.ds0127_load_log[i].src_val = REGi(rm) & 0xffff;
		} else {
			wfw_monitor.ds0127_load_log[i].src_kind = 2;
			/* Focused decode for MOV DS,[BP-6] pattern at 0117:7488. */
			if (modrm == 0x5e) {
				uint16_t bp = REGi(5) & cpu->sp_mask;
				uint16_t off = (bp - 6) & cpu->sp_mask;
				uint16_t w = 0;
				if (read_seg_u16_noexcept(cpu, SEG_SS, off, cpu->cpl, &w)) {
					wfw_monitor.ds0127_load_log[i].bp_m6_ok = true;
					wfw_monitor.ds0127_load_log[i].bp_m6_val = w;
				}
			}
		}
		break;
	}
	case 0xc5: /* LDS Gv,Mp */
		wfw_monitor.ds0127_load_log[i].src_kind = 4;
		break;
	default:
		break;
	}
}

static bool set_seg(CPUI386 *cpu, int seg, int sel)
{
	sel = sel & 0xffff;
	if (!(cpu->cr0 & 1) || (cpu->flags & VM)) {
		cpu->seg[seg].sel = sel;
		cpu->seg[seg].base = sel << 4;
		cpu->seg[seg].limit = 0xffff;
		cpu->seg[seg].flags = 0; // D_BIT is not set
		/* Real mode: never flat (base = sel << 4) */
		cpu->seg_flat &= ~(1 << seg);
		cpu->all_segs_flat = false;
		if (seg == SEG_CS) {
			cpu->cpl = cpu->flags & VM ? 3 : 0;
			cpu->code16 = true;
			SEQ_INVALIDATE(cpu);
		}
		if (seg == SEG_SS) {
			cpu->sp_mask = 0xffff;
		}
		return true;
	}

	/* Protected-mode data-segment null selector semantics:
	 * DS/ES/FS/GS accept null selectors and become unusable until reloaded.
	 * SS null is always invalid (#GP(0)).
	 * Re-enabled: testing if this is the cause. */
	if (!(sel & ~0x3)) {
		if (seg == SEG_DS || seg == SEG_ES || seg == SEG_FS || seg == SEG_GS) {
			cpu->seg[seg].sel = sel;
			cpu->seg[seg].base = 0;
			cpu->seg[seg].limit = 0;
			cpu->seg[seg].flags = 0;
			update_seg_flat(cpu, seg);
			return true;
		}
		if (seg == SEG_SS)
			THROW(EX_GP, 0);
	}

	uword w1, w2;
	TRY(read_desc(cpu, sel, &w1, &w2));

	bool s = (w2 >> 12) & 1;
	bool p = (w2 >> 15) & 1;
	bool code = (w2 >> 11) & 1;
	bool rw = (w2 >> 9) & 1;
	bool conforming = code && ((w2 >> 10) & 1);
	int dpl = (w2 >> 13) & 0x3;
	int rpl = sel & 0x3;
	int maxpl = cpu->cpl > rpl ? cpu->cpl : rpl;
	if (sel & ~0x3) {
		switch(seg) {
		case SEG_SS:
			/* SS must be a present writable data segment. Structural
			 * checks are enforced; DPL/RPL privilege checks are relaxed.
			 *
			 * On real 386, DPL/RPL mismatch #GPs and the OS handler
			 * trap-and-emulates the load.  Win95 DOSX does this for
			 * ring-3 code loading ring-0 SS (sel 0x0068 at CPL=3).
			 * Our DOSX handler can't process that #GP correctly, so
			 * we allow the load directly — same end result as the
			 * trap-and-emulate path on real hardware. */
			if (!s || code || !rw)
				THROW(EX_GP, sel & ~0x3);
			if (!p)
				THROW(EX_SS, sel & ~0x3);
			break;
		case SEG_DS: case SEG_ES: case SEG_FS: case SEG_GS:
			/* Data regs may load data or readable code descriptors. */
			if (!s)
				THROW(EX_GP, sel & ~0x3);
			if (code && !rw)
				THROW(EX_GP, sel & ~0x3);
			if ((!code || !conforming) && dpl < maxpl)
				THROW(EX_GP, sel & ~0x3);
			if (!p)
				THROW(EX_NP, sel & ~0x3);
			break;
		default:
			/* Preserve historical behavior for non-data seg registers. */
			if (!p)
				THROW((seg == SEG_SS ? EX_SS : EX_NP), sel & ~0x3);
			break;
		}
	}

	cpu->seg[seg].sel = sel;
	cpu->seg[seg].base = (w1 >> 16) | ((w2 & 0xff) << 16) | (w2 & 0xff000000);
	cpu->seg[seg].limit = (w2 & 0xf0000) | (w1 & 0xffff);
	if (w2 & 0x00800000)
		cpu->seg[seg].limit = (cpu->seg[seg].limit << 12) | 0xfff;
	cpu->seg[seg].flags = (w2 >> 8) & 0xffff; // (w2 >> 20) & 0xf;
	/* Update flat flag for this segment */
	update_seg_flat(cpu, seg);
	if (seg == SEG_CS) {
		cpu->cpl = sel & 3;
		cpu->code16 = !(cpu->seg[SEG_CS].flags & SEG_D_BIT);
		SEQ_INVALIDATE(cpu);
	}
	if (seg == SEG_SS) {
		cpu->sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
	}
	/* Decode source/class for DS<-0127 before provenance snapshot. */
	if (unlikely(seg == SEG_DS && (sel & 0xffff) == 0x0127))
		wfw_note_ds0127_load(cpu, (uint16_t)(sel & 0xffff));
	/* Selector 0x0127 provenance watch: log when DS is loaded with 0x0127.
	 * w1/w2 are from read_desc() above — already correctly paging-translated. */
	if (unlikely(cpu_diag_enabled && win386_diag.active &&
	             seg == SEG_DS && (sel & 0xffff) == 0x0127)) {
		uint32_t rw1 = w1, rw2 = w2; /* 0x0127's descriptor from read_desc */
		/* Read adjacent 0x0117 descriptor via read_desc path */
		uword aw1 = 0, aw2 = 0;
		/* Temporarily save/restore exception state — read_desc may throw */
		{
			uint32_t ldt_base = cpu->seg[SEG_LDT].base;
			uint32_t ldt_lim  = cpu->seg[SEG_LDT].limit;
			uint32_t off117 = 0x0117 & ~7;
			if (off117 + 7 <= ldt_lim) {
				/* Manual page-walk for LDT linear address */
				for (int dw = 0; dw < 2; dw++) {
					uint32_t la = ldt_base + off117 + dw * 4;
					uint32_t pa = la;
					if (cpu->cr0 & (1u << 31)) {
						uint32_t pde_a = (cpu->cr3 & 0xfffff000) | ((la >> 20) & 0xffc);
						if (pde_a + 4 > (uint32_t)cpu->phys_mem_size) break;
						uint32_t pde = *(uint32_t *)&cpu->phys_mem[pde_a];
						if (!(pde & 1)) break;
						uint32_t pte_a = (pde & 0xfffff000) | ((la >> 10) & 0xffc);
						if (pte_a + 4 > (uint32_t)cpu->phys_mem_size) break;
						uint32_t pte = *(uint32_t *)&cpu->phys_mem[pte_a];
						if (!(pte & 1)) break;
						pa = (pte & 0xfffff000) | (la & 0xfff);
					}
					if (pa + 4 <= (uint32_t)cpu->phys_mem_size) {
						if (dw == 0) aw1 = *(uint32_t *)&cpu->phys_mem[pa];
						else         aw2 = *(uint32_t *)&cpu->phys_mem[pa];
					}
				}
			}
		}
		wfw_monitor.sel0127_prov.last_w1 = rw1;
		wfw_monitor.sel0127_prov.last_w2 = rw2;
		wfw_monitor.sel0127_prov.last_adj_w1 = aw1;
		wfw_monitor.sel0127_prov.last_adj_w2 = aw2;
		wfw_monitor.sel0127_prov.last_cs = cpu->seg[SEG_CS].sel;
		wfw_monitor.sel0127_prov.last_eip = cpu->ip;
		if (wfw_monitor.sel0127_prov.count == 0) {
			wfw_monitor.sel0127_prov.first_w1 = rw1;
			wfw_monitor.sel0127_prov.first_w2 = rw2;
			wfw_monitor.sel0127_prov.first_adj_w1 = aw1;
			wfw_monitor.sel0127_prov.first_adj_w2 = aw2;
			wfw_monitor.sel0127_prov.first_cs = cpu->seg[SEG_CS].sel;
			wfw_monitor.sel0127_prov.first_eip = cpu->ip;
		}
		wfw_monitor.sel0127_prov.count++;
	}
	return true;
}

static inline void clear_segs(CPUI386 *cpu)
{
	int segs[] = { SEG_DS, SEG_ES, SEG_FS, SEG_GS };
	for (int i = 0; i < 4; i++) {
		uword w2 = cpu->seg[segs[i]].flags << 8;
		bool is_dataseg = !((w2 >> 11) & 1);
		int dpl = (w2 >> 13) & 0x3;
		bool conforming = (w2 >> 8) & 0x4;
		if (is_dataseg || !conforming) {
			if (dpl < cpu->cpl) {
				cpu->seg[segs[i]].sel = 0;
				cpu->seg[segs[i]].base = 0;
				cpu->seg[segs[i]].limit = 0;
				cpu->seg[segs[i]].flags = 0;
				/* Clear flat flag - segment is now null */
				cpu->seg_flat &= ~(1 << segs[i]);
				cpu->all_segs_flat = false;
			}
		}
	}
}

/*
 * addressing modes
 */
#define _(rwm, inst) inst()

#define E_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX(rm, lreg ## BIT, sreg ## BIT) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, laddr ## BIT, saddr ## BIT) \
	}

#define Eb(...) E_helper(8, , __VA_ARGS__)
#define Ev(...) if (opsz16) { E_helper(16, w, __VA_ARGS__) } else { E_helper(32, d, __VA_ARGS__) }

#define EG_helper(PM, BT, BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (PM && (!(cpu->cr0 & 1) || (cpu->flags & VM))) THROW0(EX_UD); \
	if (mod == 3) { \
		INST ## SUFFIX(rm, reg, lreg ## BIT, sreg ## BIT, lreg ## BIT, sreg ## BIT) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		if (BT) addr += lreg ## BIT(reg) / BIT * (BIT / 8); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, reg, laddr ## BIT, saddr ## BIT, lreg ## BIT, sreg ## BIT) \
	}

#define EbGb(...) EG_helper(false, false, 8, , __VA_ARGS__)
#define EwGw(...) EG_helper(false, false, 16, , __VA_ARGS__)
#define PMEwGw(...) EG_helper(true, false, 16, , __VA_ARGS__)
#define EvGv(...) if (opsz16) { EG_helper(false, false, 16, w, __VA_ARGS__) } else { EG_helper(false, false, 32, d, __VA_ARGS__) }
#define BTEvGv(...) if (opsz16) { EG_helper(false, true, 16, w, __VA_ARGS__) } else { EG_helper(false, true, 32, d, __VA_ARGS__) }

#define EGIb_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	u8 imm8; \
	if (mod == 3) { \
		TRY(fetch8(cpu, &imm8)); \
		INST ## SUFFIX(rm, reg, imm8, lreg ## BIT, sreg ## BIT, lreg ## BIT, sreg ## BIT, limm, 0) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(fetch8(cpu, &imm8)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, reg, imm8, laddr ## BIT, saddr ## BIT, lreg ## BIT, sreg ## BIT, limm, 0) \
	}

#define EvGvIb(...) if (opsz16) { EGIb_helper(16, w, __VA_ARGS__) } else { EGIb_helper(32, d, __VA_ARGS__) }

#define EGCL_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX(rm, reg, 1, lreg ## BIT, sreg ## BIT, lreg ## BIT, sreg ## BIT, lreg8, sreg8) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, reg, 1, laddr ## BIT, saddr ## BIT, lreg ## BIT, sreg ## BIT, lreg8, sreg8) \
	}

#define EvGvCL(...) if (opsz16) { EGCL_helper(16, w, __VA_ARGS__) } else { EGCL_helper(32, d, __VA_ARGS__) }

#define EI_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	u ## BIT imm ## BIT; \
	if (mod == 3) { \
		TRY(fetch ## BIT(cpu, &imm ## BIT)); \
		INST ## SUFFIX(rm, imm ## BIT, lreg ## BIT, sreg ## BIT, limm, 0) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(fetch ## BIT(cpu, &imm ## BIT)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, imm ## BIT, laddr ## BIT, saddr ## BIT, limm, 0) \
	}

#define EbIb(...) EI_helper(8, , __VA_ARGS__)
#define EvIv(...) if (opsz16) { EI_helper(16, w, __VA_ARGS__) } else { EI_helper(32, d, __VA_ARGS__) }

#define EIb_helper(BT, BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	u8 imm8; \
	u ## BIT imm ## BIT; \
	if (mod == 3) { \
		TRY(fetch8(cpu, &imm8)); \
		imm ## BIT = (s ## BIT) ((s8) imm8); \
		INST ## SUFFIX(rm, imm ## BIT, lreg ## BIT, sreg ## BIT, limm, 0) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(fetch8(cpu, &imm8)); \
		imm ## BIT = (s ## BIT) ((s8) imm8); \
		if (BT) addr += imm ## BIT / BIT * (BIT / 8); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, imm ## BIT, laddr ## BIT, saddr ## BIT, limm, 0) \
	}

#define EvIb(...) if (opsz16) { EIb_helper(false, 16, w, __VA_ARGS__) } else { EIb_helper(false, 32, d, __VA_ARGS__) }
#define BTEvIb(...) if (opsz16) { EIb_helper(true, 16, w, __VA_ARGS__) } else { EIb_helper(true, 32, d, __VA_ARGS__) }

#define E1_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX(rm, 1, lreg ## BIT, sreg ## BIT, limm, 0) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, 1, laddr ## BIT, saddr ## BIT, limm, 0) \
	}

#define Eb1(...) E1_helper(8, , __VA_ARGS__)
#define Ev1(...) if (opsz16) { E1_helper(16, w, __VA_ARGS__) } else { E1_helper(32, d, __VA_ARGS__) }

#define ECL_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX(rm, 1, lreg ## BIT, sreg ## BIT, lreg8, sreg8) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(&meml, 1, laddr ## BIT, saddr ## BIT, lreg8, sreg8) \
	}

#define EbCL(...) ECL_helper(8, , __VA_ARGS__)
#define EvCL(...) if (opsz16) { ECL_helper(16, w, __VA_ARGS__) } else { ECL_helper(32, d, __VA_ARGS__) }

#define GE_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX(reg, rm, lreg ## BIT, sreg ## BIT, lreg ## BIT, sreg ## BIT) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX(reg, &meml, lreg ## BIT, sreg ## BIT, laddr ## BIT, saddr ## BIT) \
	}

#define GbEb(...) GE_helper(8, , __VA_ARGS__)
#define GvEv(...) if (opsz16) { GE_helper(16, w, __VA_ARGS__) } else { GE_helper(32, d, __VA_ARGS__) }

#define GvM_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX(reg, rm, lreg ## BIT, sreg ## BIT, lreg ## BIT, sreg ## BIT) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		INST ## SUFFIX(reg, addr, lreg ## BIT, sreg ## BIT, limm, 0) \
	}
#define GvM(...) if (opsz16) { GvM_helper(16, w, __VA_ARGS__) } else { GvM_helper(32, d, __VA_ARGS__) }

#define GvMp_helper(BIT, SUFFIX, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) THROW0(EX_UD); \
	else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		INST ## SUFFIX(reg, addr, lreg ## BIT, sreg ## BIT, limm, 0) \
	}
#define GvMp(...) if (opsz16) { GvMp_helper(16, w, __VA_ARGS__) } else { GvMp_helper(32, d, __VA_ARGS__) }

#define GE_helper2(BIT, SUFFIX, BIT2, SUFFIX2, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST ## SUFFIX ## SUFFIX2(reg, rm, lreg ## BIT, sreg ## BIT, lreg ## BIT2, sreg ## BIT2) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate ## BIT2(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX ## SUFFIX2(reg, &meml, lreg ## BIT, sreg ## BIT, laddr ## BIT2, saddr ## BIT2) \
	}

#define GvEb(...) if (opsz16) { GE_helper2(16, w, 8, b, __VA_ARGS__) } else { GE_helper2(32, d, 8, b, __VA_ARGS__) }
#define GvEw(...) if (opsz16) { GE_helper2(16, w, 16, w, __VA_ARGS__) } else { GE_helper2(32, d, 16, w, __VA_ARGS__) }

#define GEI_helperI2(BIT, SUFFIX, BIT2, SUFFIX2, rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		u ## BIT2 imm ## BIT2; \
		TRY(fetch ## BIT2(cpu, &imm ## BIT2)); \
		INST ## SUFFIX ## I ## SUFFIX2(reg, rm, imm ## BIT2, lreg ## BIT, sreg ## BIT, lreg ## BIT, sreg ## BIT) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		u ## BIT2 imm ## BIT2; \
		TRY(fetch ## BIT2(cpu, &imm ## BIT2)); \
		TRY(translate ## BIT(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## SUFFIX ## I ## SUFFIX2(reg, &meml, imm ## BIT2, lreg ## BIT, sreg ## BIT, laddr ## BIT, saddr ## BIT) \
	}

#define GvEvIb(...) if (opsz16) { GEI_helperI2(16, w, 8, b, __VA_ARGS__) } else { GEI_helperI2(32, d, 8, b, __VA_ARGS__) }
#define GvEvIv(...) if (opsz16) { GEI_helperI2(16, w, 16, w, __VA_ARGS__) } else { GEI_helperI2(32, d, 32, d, __VA_ARGS__) }

#define ALIb(rwm, INST) \
	u8 imm8; \
	TRY(fetch8(cpu, &imm8)); \
	INST(0, imm8, lreg8, sreg8, limm, 0)

#define AXIb(rwm, INST) \
	if (opsz16) { \
		u8 imm8; \
		TRY(fetch8(cpu, &imm8)); \
		INST ## w(0, imm8, lreg16, sreg16, limm, 0) \
	} else { \
		u8 imm8; \
		TRY(fetch8(cpu, &imm8)); \
		INST ## d(0, imm8, lreg32, sreg32, limm, 0) \
	}

#define IbAL(rwm, INST) \
	u8 imm8; \
	TRY(fetch8(cpu, &imm8)); \
	INST(imm8, 0, limm, 0, lreg8, sreg8)

#define IbAX(rwm, INST) \
	if (opsz16) { \
		u8 imm8; \
		TRY(fetch8(cpu, &imm8)); \
		INST ## w(imm8, 0, limm, 0, lreg16, sreg16) \
	} else { \
		u8 imm8; \
		TRY(fetch8(cpu, &imm8)); \
		INST ## d(imm8, 0, limm, 0, lreg32, sreg32) \
	}

#define DXAL(rwm, INST) \
	INST(2, 0, lreg16, sreg16, lreg8, sreg8)

#define DXAX(rwm, INST) \
	if (opsz16) { \
		INST ## w(2, 0, lreg16, sreg16, lreg16, sreg16) \
	} else { \
		INST ## d(2, 0, lreg16, sreg16, lreg32, sreg32) \
	}

#define ALDX(rwm, INST) \
	INST(0, 2, lreg8, sreg8, lreg16, sreg16)

#define AXDX(rwm, INST) \
	if (opsz16) { \
		INST ## w(0, 2, lreg16, sreg16, lreg16, sreg16) \
	} else { \
		INST ## d(0, 2, lreg32, sreg32, lreg16, sreg16) \
	}

#define AXIv(rwm, INST) \
	if (opsz16) { \
		u16 imm16; \
		TRY(fetch16(cpu, &imm16)); \
		INST ## w(0, imm16, lreg16, sreg16, limm, 0) \
	} else { \
		u32 imm32; \
		TRY(fetch32(cpu, &imm32)); \
		INST ## d(0, imm32, lreg32, sreg32, limm, 0) \
	}

#define ALOb(rwm, INST) \
	if (adsz16) { \
		u16 addr16; \
		TRY(fetch16(cpu, &addr16)); \
		addr = addr16; \
	} else { \
		TRY(fetch32(cpu, &addr)); \
	} \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	TRY(translate8(cpu, &meml, rwm, curr_seg, addr)); \
	INST(0, &meml, lreg8, sreg8, laddr8, saddr8)

#define AXOv(rwm, INST) \
	if (adsz16) { \
		u16 addr16; \
		TRY(fetch16(cpu, &addr16)); \
		addr = addr16; \
	} else { \
		TRY(fetch32(cpu, &addr)); \
	} \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	if (opsz16) { \
		TRY(translate16(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## w(0, &meml, lreg16, sreg16, laddr16, saddr16) \
	} else { \
		TRY(translate32(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## d(0, &meml, lreg32, sreg32, laddr32, saddr32) \
	}

#define ObAL(rwm, INST) \
	if (adsz16) { \
		u16 addr16; \
		TRY(fetch16(cpu, &addr16)); \
		addr = addr16; \
	} else { \
		TRY(fetch32(cpu, &addr)); \
	} \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	TRY(translate8(cpu, &meml, rwm, curr_seg, addr)); \
	INST(&meml, 0, laddr8, saddr8, lreg8, sreg8)

#define OvAX(rwm, INST) \
	if (adsz16) { \
		u16 addr16; \
		TRY(fetch16(cpu, &addr16)); \
		addr = addr16; \
	} else { \
		TRY(fetch32(cpu, &addr)); \
	} \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	if (opsz16) { \
		TRY(translate16(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## w(&meml, 0, laddr16, saddr16, lreg16, sreg16) \
	} else { \
		TRY(translate32(cpu, &meml, rwm, curr_seg, addr)); \
		INST ## d(&meml, 0, laddr32, saddr32, lreg32, sreg32) \
	}

#define PlusRegv(rwm, INST) \
	if (opsz16) { \
		INST ## w((b1 & 7), lreg16, sreg16) \
	} else { \
		INST ## d((b1 & 7), lreg32, sreg32) \
	}

#define PlusRegIb(rwm, INST) \
	u8 imm8; \
	TRY(fetch8(cpu, &imm8)); \
	INST((b1 & 7), imm8, lreg8, sreg8, limm, 0)

#define PlusRegIv(rwm, INST) \
	if (opsz16) { \
		u16 imm16; \
		TRY(fetch16(cpu, &imm16)); \
		INST ## w((b1 & 7), imm16, lreg16, sreg16, limm, 0) \
	} else { \
		u32 imm32; \
		TRY(fetch32(cpu, &imm32)); \
		INST ## d((b1 & 7), imm32, lreg32, sreg32, limm, 0) \
	}

#define Ib(rwm, INST) \
	u8 imm8; \
	TRY(fetch8(cpu, &imm8)); \
	INST(imm8, limm, 0)
#define Jb Ib

#define Iw(rwm, INST) \
	u16 imm16; \
	TRY(fetch16(cpu, &imm16)); \
	INST(imm16, limm, 0)

#define IwIb(rwm, INST) \
	u16 imm16; \
	TRY(fetch16(cpu, &imm16)); \
	u8 imm8; \
	TRY(fetch8(cpu, &imm8)); \
	INST(imm16, imm8, limm, 0, limm, 0)

#define Iv(rwm, INST) \
	if (opsz16) { \
		u16 imm16; \
		TRY(fetch16(cpu, &imm16)); \
		INST ## w(imm16, limm, 0) \
	} else { \
		u32 imm32; \
		TRY(fetch32(cpu, &imm32)); \
		INST ## d(imm32, limm, 0) \
	}

#define Jv(rwm, INST) \
	/* Near branch displacement size follows operand size (66h), \
	 * not address size (67h). */ \
	if (opsz16) { \
		u16 imm16; \
		TRY(fetch16(cpu, &imm16)); \
		INST ## w(imm16, limm, 0); \
	} else { \
		u32 imm32; \
		TRY(fetch32(cpu, &imm32)); \
		INST ## d(imm32, limm, 0); \
	}
#define Av Iv

#define Ap(rwm, INST) \
	u16 seg; \
	if (opsz16) { \
		u16 addr16; \
		TRY(fetch16(cpu, &addr16)); \
		addr = addr16; \
	} else { \
		TRY(fetch32(cpu, &addr)); \
	} \
	TRY(fetch16(cpu, &seg)); \
	INST(addr, seg)

#define Ep(rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) THROW0(EX_UD); \
	else { \
		u16 seg; \
		u32 off; \
		OptAddr moff, mseg; \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		if (opsz16) { \
			TRY(translate16(cpu, &moff, rwm, curr_seg, addr)); \
			TRY(translate16(cpu, &mseg, rwm, curr_seg, addr + 2)); \
			off = laddr16(&moff); \
			seg = laddr16(&mseg); \
		} else { \
			TRY(translate32(cpu, &moff, rwm, curr_seg, addr)); \
			TRY(translate16(cpu, &mseg, rwm, curr_seg, addr + 4)); \
			off = laddr32(&moff); \
			seg = laddr16(&mseg); \
		} \
		INST(off, seg) \
	}

#define Ms(rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) THROW0(EX_UD); \
	else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		INST(addr) \
	}

#define Ew(rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		if (opsz16) { \
			INST(rm, lreg16, sreg16) \
		} else { \
			INST(rm, lreg32, sreg32) \
		} \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate16(cpu, &meml, rwm, curr_seg, addr)); \
		INST(&meml, laddr16, saddr16) \
	}

#define EwSw(rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		if (opsz16) { \
			INST(rm, reg, lreg16, sreg16, lseg, 0) \
		} else { \
			INST(rm, reg, lreg32, sreg32, lseg, 0) \
		} \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate16(cpu, &meml, rwm, curr_seg, addr)); \
		INST(&meml, reg, laddr16, saddr16, lseg, 0) \
	}

#define SwEw(rwm, INST) \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	if (mod == 3) { \
		INST(reg, rm, lseg, 0, lreg16, sreg16) \
	} else { \
		TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
		TRY(translate16(cpu, &meml, rwm, curr_seg, addr)); \
		INST(reg, &meml,lseg, 0, laddr16, saddr16) \
	}

#define limm(i) i
#ifdef I386_OPT1
#define lreg8(i) ((i) > 3 ? cpu->gprx[i - 4].r8[1] : cpu->gprx[i].r8[0])
#define sreg8(i, v) ((i) > 3 ? (cpu->gprx[i - 4].r8[1] = (v)) : (cpu->gprx[i].r8[0] = (v)))
#define lreg16(i) (cpu->gprx[i].r16)
#define sreg16(i, v) (cpu->gprx[i].r16 = (v))
#define lreg32(i) (REGi(i))
#define sreg32(i, v) ((REGi(i)) = (v))
#else
#define lreg8(i) ((u8) ((i) > 3 ? REGi((i) - 4) >> 8 : REGi((i))))
#define sreg8(i, v) ((i) > 3 ? \
		     (REGi((i) - 4) = (REGi((i) - 4) & (wordmask ^ 0xff00)) | (((v) & 0xff) << 8)) : \
		     (REGi((i)) = (REGi((i)) & (wordmask ^ 0xff)) | ((v) & 0xff)))
#define lreg16(i) ((u16) REGi((i)))
#define sreg16(i, v) (REGi((i)) = (REGi((i)) & (wordmask ^ 0xffff)) | ((v) & 0xffff))
#define lreg32(i) ((u32) REGi((i)))
#define sreg32(i, v) (REGi((i)) = (REGi((i)) & (wordmask ^ 0xffffffff)) | ((v) & 0xffffffff))
#endif
#define laddr8(addr) load8(cpu, addr)
#define saddr8(addr, v) store8(cpu, addr, v)
#define laddr16(addr) load16(cpu, addr)
#define saddr16(addr, v) store16(cpu, addr, v)
#define laddr32(addr) load32(cpu, addr)
#define saddr32(addr, v) store32(cpu, addr, v)
#define lseg(i) ((u16) SEGi((i)))
#define set_sp(v, mask) (sreg32(4, ((v) & mask) | (lreg32(4) & ~mask)))

static void cpu_diag_copy_seg_string(CPUI386 *cpu, int seg, uword off, int cpl,
	char term, char *out, size_t outsz)
{
	size_t i = 0;
	if (!out || outsz == 0)
		return;
	while (i + 1 < outsz) {
		uint8_t b = 0;
		if (!read_seg_u8_noexcept(cpu, seg, off + (uword)i, cpl, &b))
			break;
		if ((term == 0 && b == 0) || (term != 0 && b == (uint8_t)term))
			break;
		out[i++] = (b >= 0x20 && b <= 0x7e) ? (char)b : '.';
	}
	out[i] = '\0';
}

static const char *cpu_mode_name(uword flags, uword cr0)
{
	if (flags & VM)
		return "VM86";
	return (cr0 & 1) ? "PM" : "RM";
}

static void cpu_diag_log_cr0_transition(CPUI386 *cpu, uword old_cr0, uword new_cr0)
{
	static int diag_gen = -1;
	static uint16_t count = 0;
	if (!cpu_diag_enabled || old_cr0 == new_cr0)
		return;
	if (win386_diag.active) {
		wfw_monitor.mode_switches++;
		{
			int ti = wfw_monitor.cr0_tail_pos % 8;
			wfw_monitor.cr0_tail[ti].cs = cpu->seg[SEG_CS].sel;
			wfw_monitor.cr0_tail[ti].ip = cpu->ip;
			wfw_monitor.cr0_tail[ti].old_cr0 = old_cr0;
			wfw_monitor.cr0_tail[ti].new_cr0 = new_cr0;
			wfw_monitor.cr0_tail_pos++;
			if (wfw_monitor.cr0_tail_count < 8)
				wfw_monitor.cr0_tail_count++;
		}
	}
	if (diag_gen != cpu->diag_gen) {
		diag_gen = cpu->diag_gen;
		count = 0;
	}
	if (count >= 8)
		return;
	count++;
	dolog("CR0 #%u %08x->%08x PE:%d->%d PG:%d->%d at %04x:%08x\n",
		count, old_cr0, new_cr0,
		!!(old_cr0 & 1), !!(new_cr0 & 1),
		!!(old_cr0 & CR0_PG), !!(new_cr0 & CR0_PG),
		cpu->seg[SEG_CS].sel, cpu->ip);
}

/* Exception ring buffer — file-scope for post-mortem dump from cpui386_step */
#define EXC_RING_SIZE 1024
#define EXC_RING_MASK (EXC_RING_SIZE - 1)
static struct { uint8_t no; uint8_t mode; uint16_t cs; uint32_t ip; uint32_t err; uint32_t cr2; } exc_ring[EXC_RING_SIZE];
static int exc_ring_pos = 0;


static void cpu_monitor_dump(CPUI386 *cpu, const char *reason)
{
	uint32_t gp_total = 0;
	uint32_t dpmi_0006 = 0, dpmi_0703 = 0, dpmi_0007 = 0, dpmi_0008 = 0;
	uint32_t dpmi_0501 = 0, dpmi_0004 = 0;
	static const char *exc_names[] = {
		"#DE","#DB","NMI","#BP","#OF","#BR","#UD","#NM",
		"#DF","---","#TS","#NP","#SS","#GP","#PF","---",
		"#MF","#AC","#MC","#XF","---","---","---","---",
		"---","---","---","---","---","---","#SX","---"
	};

	for (int i = 0; i < 256; i++)
		gp_total += wfw_monitor.v86_int_gp_counts[i];
	for (int i = 0; i < wfw_monitor.int31_hist_len; i++) {
		switch (wfw_monitor.int31_hist[i].ax) {
		case 0x0006: dpmi_0006 = wfw_monitor.int31_hist[i].count; break;
		case 0x0703: dpmi_0703 = wfw_monitor.int31_hist[i].count; break;
		case 0x0007: dpmi_0007 = wfw_monitor.int31_hist[i].count; break;
		case 0x0008: dpmi_0008 = wfw_monitor.int31_hist[i].count; break;
		case 0x0501: dpmi_0501 = wfw_monitor.int31_hist[i].count; break;
		case 0x0004: dpmi_0004 = wfw_monitor.int31_hist[i].count; break;
		default: break;
		}
	}
	if (wfw_monitor.tty_pos > 0)
		wfw_monitor.tty_buf[wfw_monitor.tty_pos] = '\0';

	dolog("=== CPU Monitor (%s) ===\n", reason);
	dolog("core: pm_int=%u vm86_int=%u v86_to_r0=%u isr_fail=%u mode_sw=%u a20_toggles=%u(state=%u)\n",
		wfw_monitor.pm_ints, wfw_monitor.vm86_ints, wfw_monitor.v86_to_ring0,
		wfw_monitor.isr_failures, wfw_monitor.mode_switches, wfw_monitor.a20_toggles,
		wfw_monitor.a20_state);
	if (wfw_monitor.cr0_tail_count > 0) {
		int cnt = wfw_monitor.cr0_tail_count;
		dolog("cr0_tail:");
		for (int i = 0; i < cnt; i++) {
			int idx = (int)wfw_monitor.cr0_tail_pos - cnt + i;
			while (idx < 0)
				idx += 8;
			idx %= 8;
			dolog(" [%04x:%08x %08x>%08x]",
				wfw_monitor.cr0_tail[idx].cs,
				wfw_monitor.cr0_tail[idx].ip,
				wfw_monitor.cr0_tail[idx].old_cr0,
				wfw_monitor.cr0_tail[idx].new_cr0);
		}
		dolog("\n");
	}
	dolog("exc: #UD=%u #GP=%u #PF=%u #AC=%u #DF=%u #TS=%u #NP=%u #SS=%u\n",
		wfw_monitor.exc_counts[EX_UD], wfw_monitor.exc_counts[EX_GP],
		wfw_monitor.exc_counts[EX_PF], wfw_monitor.exc_counts[17],
		wfw_monitor.exc_counts[EX_DF], wfw_monitor.exc_counts[EX_TS],
		wfw_monitor.exc_counts[EX_NP], wfw_monitor.exc_counts[EX_SS]);
	dolog("int: 10h=%u(last=%02x/%02x tty_len=%u) 15h=%u(last=%02x)\n",
		wfw_monitor.int10h_count, wfw_monitor.last_int10h_ah, wfw_monitor.last_int10h_al,
		wfw_monitor.tty_pos, wfw_monitor.int15h_count, wfw_monitor.last_int15h_ah);
	{
		uint16_t bda_equip = 0;
		uint16_t bda_convk = 0;
		uint8_t bda_mode = 0;
		if (cpu->phys_mem_size > 0x411)
			bda_equip = cpu->phys_mem[0x410] | ((uint16_t)cpu->phys_mem[0x411] << 8);
		if (cpu->phys_mem_size > 0x414)
			bda_convk = cpu->phys_mem[0x413] | ((uint16_t)cpu->phys_mem[0x414] << 8);
		if (cpu->phys_mem_size > 0x449)
			bda_mode = cpu->phys_mem[0x449];
		dolog("bda: equip=%04x convkb=%04x mode=%02x\n", bda_equip, bda_convk, bda_mode);
	}
	dolog("r3: pm=%u irq=%u faults=%u gp=%u pf=%u ac=%u last=%s@%04x:%08x bios_lgdt=%u bios_priv=%u\n",
		wfw_monitor.pm_ring3_ints, wfw_monitor.pm_ring3_irqs, wfw_monitor.pm_ring3_faults,
		wfw_monitor.pm_ring3_exc_hist[EX_GP], wfw_monitor.pm_ring3_exc_hist[EX_PF],
		wfw_monitor.pm_ring3_exc_hist[17],
		(wfw_monitor.pm_r3_last_exc < 32) ? exc_names[wfw_monitor.pm_r3_last_exc] : "irq",
		wfw_monitor.pm_r3_last_cs, wfw_monitor.pm_r3_last_eip,
		wfw_monitor.bios_lgdt_count, wfw_monitor.bios_priv_count);
	dolog("r3_dos: int21=%u int31=%u int2f=%u open=%u fail21=%u fail31=%u\n",
		wfw_monitor.pm_r3_int21_count, wfw_monitor.pm_r3_int31_count,
		wfw_monitor.pm_r3_int2f_count, wfw_monitor.r3_fopen_count,
		wfw_monitor.int21_fail_count, wfw_monitor.int31_fail_count);
		if (wfw_monitor.r3_fopen_count > 0) {
			int n = wfw_monitor.r3_fopen_count;
			if (n > 8)
				n = 8;
			dolog("r3_open_files:");
			for (int i = 0; i < n; i++) {
				const char *name = wfw_monitor.r3_fopen_log[i].name[0] ?
					wfw_monitor.r3_fopen_log[i].name : "<unreadable>";
				dolog(" [%d:\"%s\"]", i, name);
			}
			dolog("\n");
		}
		if (wfw_monitor.int41_module_count > 0) {
			dolog("int41_modules:");
			for (int i = 0; i < wfw_monitor.int41_module_count; i++) {
				dolog(" [%04x %s h=%04x cs=%04x]",
					wfw_monitor.int41_modules[i].ax,
					wfw_monitor.int41_modules[i].name[0] ?
						wfw_monitor.int41_modules[i].name : "?",
					wfw_monitor.int41_modules[i].hmod,
					wfw_monitor.int41_modules[i].call_cs);
			}
			dolog("\n");
		}
	if (wfw_monitor.pm_r3_int2f_count > 0) {
		int n = wfw_monitor.pm_r3_int2f_count;
		if (n > 16)
			n = 16;
		dolog("r3_int2f:");
		for (int i = 0; i < n; i++)
			dolog(" %04x", wfw_monitor.pm_r3_int2f_ax[i]);
		dolog(" 16xx=%u", wfw_monitor.pm_r3_168a_count);
		if (wfw_monitor.pm_r3_168a_str[0])
			dolog(" 168a=\"%s\"", wfw_monitor.pm_r3_168a_str);
		dolog("\n");
	}
	if (wfw_monitor.int2f_ret_count > 0) {
		dolog("r3_int2f_ret:");
		for (int i = 0; i < wfw_monitor.int2f_ret_count; i++) {
			dolog(" [%04x>%04x ds:si=%04x:%04x es:di=%04x:%04x cf=%u]",
				wfw_monitor.int2f_ret_log[i].call_ax,
				wfw_monitor.int2f_ret_log[i].ret_ax,
				wfw_monitor.int2f_ret_log[i].ret_ds,
				wfw_monitor.int2f_ret_log[i].ret_si,
				wfw_monitor.int2f_ret_log[i].ret_es,
				wfw_monitor.int2f_ret_log[i].ret_di,
				wfw_monitor.int2f_ret_log[i].cf);
		}
		dolog("\n");
	}
	if (wfw_monitor.r3_gp_count > 0) {
		dolog("r3_gp:");
		for (int i = 0; i < wfw_monitor.r3_gp_count; i++) {
			dolog(" %04x:%08x/e=%04x",
				wfw_monitor.r3_gp_log[i].cs,
				wfw_monitor.r3_gp_log[i].eip,
				wfw_monitor.r3_gp_log[i].err);
			if (wfw_monitor.r3_gp_log[i].dec[0])
				dolog("/d=%s", wfw_monitor.r3_gp_log[i].dec);
		}
		dolog("\n");
	}
	if (wfw_monitor.r3_pf_count > 0) {
		dolog("r3_pf:");
		for (int i = 0; i < wfw_monitor.r3_pf_count; i++) {
			dolog(" %04x:%08x/e=%04x c2=%08x",
				wfw_monitor.r3_pf_log[i].cs,
				wfw_monitor.r3_pf_log[i].eip,
				wfw_monitor.r3_pf_log[i].err,
				wfw_monitor.r3_pf_log[i].cr2);
			if (wfw_monitor.r3_pf_log[i].dec[0])
				dolog("/d=%s", wfw_monitor.r3_pf_log[i].dec);
		}
		dolog("\n");
	}
	if (wfw_monitor.r3_hipf_count > 0) {
		dolog("r3_hipf: cr3=%08x (%u entries, last %u)\n",
			wfw_monitor.r3_hipf[((wfw_monitor.r3_hipf_pos - 1) % 16)].cr3,
			wfw_monitor.r3_hipf_count,
			(wfw_monitor.r3_hipf_pos > 16) ? 16u : wfw_monitor.r3_hipf_count);
		int start = (wfw_monitor.r3_hipf_count < 16) ? 0 :
			(int)(wfw_monitor.r3_hipf_pos % 16);
		for (int i = 0; i < wfw_monitor.r3_hipf_count; i++) {
			int idx = (start + i) % 16;
			dolog("  %04x:%08x cr2=%08x e=%04x pde=%08x pte=%08x%s\n",
				wfw_monitor.r3_hipf[idx].cs,
				wfw_monitor.r3_hipf[idx].eip,
				wfw_monitor.r3_hipf[idx].cr2,
				wfw_monitor.r3_hipf[idx].err,
				wfw_monitor.r3_hipf[idx].pde,
				wfw_monitor.r3_hipf[idx].pte,
				(wfw_monitor.r3_hipf[idx].pde & 1) ?
					((wfw_monitor.r3_hipf[idx].pte & 1) ? " P" : " pde=P pte=NP") :
					" pde=NP");
		}
	}
	dolog("dpmi: 0006=%u 0703=%u 0007=%u 0008=%u 0501=%u 0004=%u ret31=%u ret21=%u\n",
		dpmi_0006, dpmi_0703, dpmi_0007, dpmi_0008, dpmi_0501, dpmi_0004,
		wfw_monitor.int31_ret_count, wfw_monitor.int21_ret_count);
	if (wfw_monitor.int31_hist_len > 0) {
		dolog("int31_hist(%u funcs):", wfw_monitor.int31_hist_len);
		for (int i = 0; i < wfw_monitor.int31_hist_len; i++)
			dolog(" %04x=%u", wfw_monitor.int31_hist[i].ax,
				wfw_monitor.int31_hist[i].count);
		dolog("\n");
	}
	if (wfw_monitor.dpmi_alloc_count > 0) {
		/* Find the last faulting CR2 from r3_hipf for range check */
		uint32_t last_cr2 = 0;
		if (wfw_monitor.r3_hipf_count > 0) {
			int li = ((int)wfw_monitor.r3_hipf_pos - 1 + 16) % 16;
			last_cr2 = wfw_monitor.r3_hipf[li].cr2;
		}
		for (int i = 0; i < wfw_monitor.dpmi_alloc_count; i++) {
			uint32_t base = wfw_monitor.dpmi_alloc_log[i].ret_addr;
			uint32_t size = wfw_monitor.dpmi_alloc_log[i].req_size;
			uint32_t end = base + size;
			const char *contains = "";
			if (last_cr2 && last_cr2 >= base && last_cr2 < end)
				contains = " <<CR2-IN";
			else if (last_cr2 && !wfw_monitor.dpmi_alloc_log[i].cf)
				contains = " cr2-OUT";
			dolog("dpmi_alloc[%d]: %04x size=%08x addr=%08x-%08x h=%08x cf=%u%s\n",
				i, wfw_monitor.dpmi_alloc_log[i].func,
				size, base, end,
				wfw_monitor.dpmi_alloc_log[i].ret_handle,
				wfw_monitor.dpmi_alloc_log[i].cf ? 1u : 0u,
				contains);
		}
	}
	if (wfw_monitor.fatal_pf_captured) {
		dolog("fatal_pf: %04x:%08x cr2=%08x\n",
			wfw_monitor.fatal_pf.cs, wfw_monitor.fatal_pf.eip,
			wfw_monitor.fatal_pf.cr2);
		dolog("  eax=%08x ebx=%08x ecx=%08x edx=%08x\n",
			wfw_monitor.fatal_pf.eax, wfw_monitor.fatal_pf.ebx,
			wfw_monitor.fatal_pf.ecx, wfw_monitor.fatal_pf.edx);
		dolog("  esi=%08x edi=%08x ebp=%08x esp=%08x\n",
			wfw_monitor.fatal_pf.esi, wfw_monitor.fatal_pf.edi,
			wfw_monitor.fatal_pf.ebp, wfw_monitor.fatal_pf.esp);
		dolog("  ds=%04x(b=%08x l=%08x) es=%04x(b=%08x l=%08x) ss=%04x(b=%08x)\n",
			wfw_monitor.fatal_pf.ds, wfw_monitor.fatal_pf.ds_base,
			wfw_monitor.fatal_pf.ds_limit,
			wfw_monitor.fatal_pf.es, wfw_monitor.fatal_pf.es_base,
			wfw_monitor.fatal_pf.es_limit,
			wfw_monitor.fatal_pf.ss, wfw_monitor.fatal_pf.ss_base);
		dolog("  cs_lim=%08x fs=%04x gs=%04x\n",
			wfw_monitor.fatal_pf.cs_limit,
			wfw_monitor.fatal_pf.fs, wfw_monitor.fatal_pf.gs);
		/* Check if DS_base+DS_limit exceeds any DPMI allocation */
		uint32_t ds_end = wfw_monitor.fatal_pf.ds_base + wfw_monitor.fatal_pf.ds_limit;
		for (int i = 0; i < wfw_monitor.dpmi_alloc_count; i++) {
			uint32_t a_base = wfw_monitor.dpmi_alloc_log[i].ret_addr;
			uint32_t a_end = a_base + wfw_monitor.dpmi_alloc_log[i].req_size;
			if (wfw_monitor.fatal_pf.ds_base >= a_base &&
			    wfw_monitor.fatal_pf.ds_base < a_end) {
				dolog("  DS in alloc[%d]: base+%05x, seg_end=%08x alloc_end=%08x overshoot=%+d\n",
					i,
					(unsigned)(wfw_monitor.fatal_pf.ds_base - a_base),
					ds_end, a_end,
					(int)(ds_end - a_end));
				break;
			}
		}
	}
	if (wfw_monitor.segchk_last.valid) {
		dolog("segchk_last: %04x:%08x seg=%s rwm=%u size=%u why=%s sel=%04x flags=%04x lim=%08x off=%08x\n",
			wfw_monitor.segchk_last.cs, wfw_monitor.segchk_last.eip,
			seg_name(wfw_monitor.segchk_last.seg),
			(unsigned)wfw_monitor.segchk_last.rwm,
			(unsigned)wfw_monitor.segchk_last.size,
			(wfw_monitor.segchk_last.why == 1) ? "write" :
			((wfw_monitor.segchk_last.why == 2) ? "limit" : "?"),
			wfw_monitor.segchk_last.sel,
			wfw_monitor.segchk_last.flags,
			wfw_monitor.segchk_last.limit,
			wfw_monitor.segchk_last.addr);
		dolog("  ctx: bx=%04x(bl=%02x) di=%04x bp=%04x",
			wfw_monitor.segchk_last.bx,
			(unsigned)wfw_monitor.segchk_last.bl,
			wfw_monitor.segchk_last.di,
			wfw_monitor.segchk_last.bp);
		if (wfw_monitor.segchk_last.ds_word_ok)
			dolog(" seg:[off]=%04x", wfw_monitor.segchk_last.ds_word);
		if (wfw_monitor.segchk_last.ss_bp_m6_ok)
			dolog(" ss:[bp-6]=%04x", wfw_monitor.segchk_last.ss_bp_m6);
		dolog("\n");
	}
	if (wfw_monitor.dpmi_seg_log_count > 0) {
		dolog("dpmi_desc_ops: (%u entries)\n", wfw_monitor.dpmi_seg_log_count);
		for (int i = 0; i < wfw_monitor.dpmi_seg_log_count; i++) {
			uint16_t f = wfw_monitor.dpmi_seg_log[i].func;
			const char *rcf = !wfw_monitor.dpmi_seg_log[i].ret_valid ? "cf=?" :
				(wfw_monitor.dpmi_seg_log[i].ret_cf ? "cf=1" : "cf=0");
			const char *fn = f==1?"AllocLDT":f==7?"SetBase":f==8?"SetLim":
				f==9?"SetRights":f==0xa?"CreateAlias":f==0xb?"GetDesc":
				f==0xc?"SetDesc":"?";
			if (f == 0x000C)
				dolog("  [%2d] %04x(%s) sel=%04x w1=%08x w2=%08x %s\n",
					i, f, fn, wfw_monitor.dpmi_seg_log[i].sel,
					wfw_monitor.dpmi_seg_log[i].val,
					wfw_monitor.dpmi_seg_log[i].extra, rcf);
			else if (f == 0x0001)
				dolog("  [%2d] %04x(%s) cnt=%u ret_sel=%04x %s\n",
					i, f, fn,
					(unsigned)wfw_monitor.dpmi_seg_log[i].val,
					wfw_monitor.dpmi_seg_log[i].sel, rcf);
			else if (f == 0x000A)
				dolog("  [%2d] %04x(%s) src=%04x alias=%04x %s\n",
					i, f, fn,
					wfw_monitor.dpmi_seg_log[i].sel,
					(uint16_t)(wfw_monitor.dpmi_seg_log[i].val & 0xffff),
					rcf);
			else if (f == 0x000B) {
				if (wfw_monitor.dpmi_seg_log[i].ret_valid &&
				    !wfw_monitor.dpmi_seg_log[i].ret_cf)
					dolog("  [%2d] %04x(%s) sel=%04x w1=%08x w2=%08x %s\n",
						i, f, fn,
						wfw_monitor.dpmi_seg_log[i].sel,
						wfw_monitor.dpmi_seg_log[i].val,
						wfw_monitor.dpmi_seg_log[i].extra,
						rcf);
				else
					dolog("  [%2d] %04x(%s) sel=%04x buf=%08x %s\n",
						i, f, fn,
						wfw_monitor.dpmi_seg_log[i].sel,
						wfw_monitor.dpmi_seg_log[i].val,
						rcf);
			}
			else
				dolog("  [%2d] %04x(%s) sel=%04x val=%08x %s\n",
					i, f, fn, wfw_monitor.dpmi_seg_log[i].sel,
					wfw_monitor.dpmi_seg_log[i].val, rcf);
		}
	}
	if (wfw_monitor.sel0127_prov.count > 0) {
		dolog("sel0127_prov: DS<-0127 loaded %u times\n",
			(unsigned)wfw_monitor.sel0127_prov.count);
		/* Decode descriptor: base = w1[31:16] | w2[7:0]<<16 | w2[31:24]<<24
		 *                    limit = w2[19:16]<<16 | w1[15:0], G=w2[23] */
		#define DESC_BASE(w1,w2) (((w1)>>16)|(((w2)&0xff)<<16)|((w2)&0xff000000))
		#define DESC_LIMIT(w1,w2) ({ \
			uint32_t _l = (((w2)&0xf0000)|((w1)&0xffff)); \
			if ((w2) & 0x800000) _l = (_l << 12) | 0xfff; _l; })
		#define DESC_FLAGS(w2) (((w2) >> 8) & 0xffff)
		dolog("  first@%04x:%08x: 0127=[%08x %08x] b=%08x l=%08x fl=%04x\n",
			wfw_monitor.sel0127_prov.first_cs,
			wfw_monitor.sel0127_prov.first_eip,
			wfw_monitor.sel0127_prov.first_w1,
			wfw_monitor.sel0127_prov.first_w2,
			DESC_BASE(wfw_monitor.sel0127_prov.first_w1, wfw_monitor.sel0127_prov.first_w2),
			DESC_LIMIT(wfw_monitor.sel0127_prov.first_w1, wfw_monitor.sel0127_prov.first_w2),
			DESC_FLAGS(wfw_monitor.sel0127_prov.first_w2));
		dolog("  first@same:          0117=[%08x %08x] b=%08x l=%08x fl=%04x\n",
			wfw_monitor.sel0127_prov.first_adj_w1,
			wfw_monitor.sel0127_prov.first_adj_w2,
			DESC_BASE(wfw_monitor.sel0127_prov.first_adj_w1, wfw_monitor.sel0127_prov.first_adj_w2),
			DESC_LIMIT(wfw_monitor.sel0127_prov.first_adj_w1, wfw_monitor.sel0127_prov.first_adj_w2),
			DESC_FLAGS(wfw_monitor.sel0127_prov.first_adj_w2));
		if (wfw_monitor.sel0127_prov.count > 1) {
			dolog("  last@%04x:%08x:  0127=[%08x %08x] b=%08x l=%08x fl=%04x\n",
				wfw_monitor.sel0127_prov.last_cs,
				wfw_monitor.sel0127_prov.last_eip,
				wfw_monitor.sel0127_prov.last_w1,
				wfw_monitor.sel0127_prov.last_w2,
				DESC_BASE(wfw_monitor.sel0127_prov.last_w1, wfw_monitor.sel0127_prov.last_w2),
				DESC_LIMIT(wfw_monitor.sel0127_prov.last_w1, wfw_monitor.sel0127_prov.last_w2),
				DESC_FLAGS(wfw_monitor.sel0127_prov.last_w2));
			dolog("  last@same:           0117=[%08x %08x] b=%08x l=%08x fl=%04x\n",
				wfw_monitor.sel0127_prov.last_adj_w1,
				wfw_monitor.sel0127_prov.last_adj_w2,
				DESC_BASE(wfw_monitor.sel0127_prov.last_adj_w1, wfw_monitor.sel0127_prov.last_adj_w2),
				DESC_LIMIT(wfw_monitor.sel0127_prov.last_adj_w1, wfw_monitor.sel0127_prov.last_adj_w2),
				DESC_FLAGS(wfw_monitor.sel0127_prov.last_adj_w2));
		}
		#undef DESC_BASE
		#undef DESC_LIMIT
		#undef DESC_FLAGS
	}
	if (wfw_monitor.segchk_insn_captured) {
		dolog("segchk_insn @%04x:%08x bytes:",
			wfw_monitor.segchk_last.cs, wfw_monitor.segchk_last.eip);
		for (int i = 0; i < wfw_monitor.segchk_insn_len; i++)
			dolog(" %02x", wfw_monitor.segchk_insn_bytes[i]);
		dolog("\n");
	}
	if (wfw_monitor.ds0127_load_count > 0) {
		static const char *const srcn[6] = {
			"?", "mov_ds_reg", "mov_ds_mem", "pop_ds", "lds", "other"
		};
		static const char *const r16n[8] = {
			"AX", "CX", "DX", "BX", "SP", "BP", "SI", "DI"
		};
		dolog("ds0127_load: (%u entries)\n", wfw_monitor.ds0127_load_count);
		for (int i = 0; i < wfw_monitor.ds0127_load_count; i++) {
			uint8_t sk = wfw_monitor.ds0127_load_log[i].src_kind;
			dolog("  [%2d] %04x:%08x src=%s",
				i,
				wfw_monitor.ds0127_load_log[i].cs,
				wfw_monitor.ds0127_load_log[i].eip,
				(sk <= 5) ? srcn[sk] : "?");
			if (sk == 1 && wfw_monitor.ds0127_load_log[i].src_reg < 8) {
				dolog(" reg=%s val=%04x",
					r16n[wfw_monitor.ds0127_load_log[i].src_reg],
					wfw_monitor.ds0127_load_log[i].src_val);
			} else if (sk == 3) {
				dolog(" ss:sp=%04x:%08x val=%04x",
					wfw_monitor.ds0127_load_log[i].ss,
					wfw_monitor.ds0127_load_log[i].sp,
					wfw_monitor.ds0127_load_log[i].src_val);
			} else if (wfw_monitor.ds0127_load_log[i].src_val) {
				dolog(" val=%04x", wfw_monitor.ds0127_load_log[i].src_val);
			}
			dolog(" bx=%04x di=%04x bp=%04x",
				wfw_monitor.ds0127_load_log[i].bx,
				wfw_monitor.ds0127_load_log[i].di,
				wfw_monitor.ds0127_load_log[i].bp);
			if (wfw_monitor.ds0127_load_log[i].bp_m6_ok)
				dolog(" [ss:bp-6]=%04x", wfw_monitor.ds0127_load_log[i].bp_m6_val);
			dolog(" op=%02x", wfw_monitor.ds0127_load_log[i].op);
			if (wfw_monitor.ds0127_load_log[i].op == 0x8e)
				dolog(" modrm=%02x", wfw_monitor.ds0127_load_log[i].modrm);
			dolog("\n");
			dolog("      bytes:");
			for (int b = 0; b < wfw_monitor.ds0127_load_log[i].blen; b++)
				dolog(" %02x", wfw_monitor.ds0127_load_log[i].bytes[b]);
			dolog("\n");
		}
	}
	if (wfw_monitor.ldt_watch_count > 0) {
		dolog("ldt_watch: (%u writes off=0x118..0x12f)\n",
			wfw_monitor.ldt_watch_count);
		for (int i = 0; i < wfw_monitor.ldt_watch_count; i++) {
			int idx = (int)wfw_monitor.ldt_watch_pos - (int)wfw_monitor.ldt_watch_count + i;
			while (idx < 0)
				idx += 24;
			idx %= 24;
			dolog("  [%2d] r%u %04x:%08x p=%08x sz=%u val=%08x off=%03x sel=%04x\n",
				i,
				(unsigned)wfw_monitor.ldt_watch_log[idx].cpl,
				wfw_monitor.ldt_watch_log[idx].cs,
				wfw_monitor.ldt_watch_log[idx].eip,
				wfw_monitor.ldt_watch_log[idx].phys,
				(unsigned)wfw_monitor.ldt_watch_log[idx].size,
				wfw_monitor.ldt_watch_log[idx].val,
				(unsigned)wfw_monitor.ldt_watch_log[idx].ldt_off,
				wfw_monitor.ldt_watch_log[idx].sel);
		}
	}
	/* TR/TSS snapshot for VM86 I/O permission diagnosis. */
	{
		int tr_type = cpu->seg[SEG_TR].flags & 0xf;
		uint16_t tr_sel = cpu->seg[SEG_TR].sel;
		uint32_t tr_base = cpu->seg[SEG_TR].base;
		uint32_t tr_lim = cpu->seg[SEG_TR].limit;
		bool tr32 = (tr_type == 9 || tr_type == 11);
		bool iob_ok = false;
		bool irb_ok = false;
		uint16_t iobase = 0xffff;
		char p20 = '?', p21 = '?', pa0 = '?', pa1 = '?';
		char ir21 = '?', ir2f = '?';

		if ((cpu->cr0 & 1) && tr32 && tr_lim >= 103 &&
		    read_seg_u16_noexcept(cpu, SEG_TR, 102, 0, &iobase)) {
			iob_ok = (iobase <= tr_lim);
			if (iob_ok) {
				const uint16_t ports[4] = { 0x20, 0x21, 0xA0, 0xA1 };
				char *outs[4] = { &p20, &p21, &pa0, &pa1 };
				for (int i = 0; i < 4; i++) {
					uint32_t byte_off = (uint32_t)iobase + (ports[i] >> 3);
					uint8_t perm;
					if (byte_off > tr_lim) {
						*outs[i] = 'X';
					} else if (!read_seg_u8_noexcept(cpu, SEG_TR, byte_off, 0, &perm)) {
						*outs[i] = '?';
					} else {
						*outs[i] = (perm & (1u << (ports[i] & 7))) ? 'D' : 'A';
					}
				}
			}
			/* IRB occupies 32 bytes immediately before I/O bitmap. */
			if (iobase >= 32) {
				uint32_t irb_base = (uint32_t)iobase - 32;
				uint32_t b21 = irb_base + (0x21 >> 3);
				uint32_t b2f = irb_base + (0x2F >> 3);
				if (b21 <= tr_lim && b2f <= tr_lim) {
					uint8_t v21, v2f;
					if (read_seg_u8_noexcept(cpu, SEG_TR, b21, 0, &v21) &&
					    read_seg_u8_noexcept(cpu, SEG_TR, b2f, 0, &v2f)) {
						irb_ok = true;
						ir21 = ((v21 >> (0x21 & 7)) & 1) ? '1' : '0';
						ir2f = ((v2f >> (0x2F & 7)) & 1) ? '1' : '0';
					}
				}
			}
		}

		dolog("tss: tr=%04x b=%08x l=%05x ty=%02x iob=%04x(%c) bm20=%c bm21=%c bma0=%c bma1=%c ir21=%c ir2f=%c\n",
			tr_sel, tr_base, tr_lim, tr_type, iobase, iob_ok ? 'Y' : 'N',
			p20, p21, pa0, pa1, irb_ok ? ir21 : '?', irb_ok ? ir2f : '?');
	}
	/* Descriptor tables and key IDT vectors — always useful. */
	dolog("gdt: base=%08x limit=%04x  idt: base=%08x limit=%04x  ldt: sel=%04x base=%08x limit=%04x\n",
		cpu->gdt.base, cpu->gdt.limit, cpu->idt.base, cpu->idt.limit,
		cpu->seg[SEG_LDT].sel, cpu->seg[SEG_LDT].base, cpu->seg[SEG_LDT].limit);
	dolog("cr: cr0=%08x cr2=%08x cr3=%08x  efl=%08x cpl=%u\n",
		cpu->cr0, cpu->cr2, cpu->cr3, cpu->flags, cpu->cpl);
	dolog("seg: cs=%04x(b=%08x l=%08x f=%04x) ss=%04x(b=%08x l=%08x f=%04x)\n",
		cpu->seg[SEG_CS].sel, cpu->seg[SEG_CS].base, cpu->seg[SEG_CS].limit,
		cpu->seg[SEG_CS].flags,
		cpu->seg[SEG_SS].sel, cpu->seg[SEG_SS].base, cpu->seg[SEG_SS].limit,
		cpu->seg[SEG_SS].flags);
	dolog("     ds=%04x(b=%08x) es=%04x(b=%08x) fs=%04x gs=%04x\n",
		cpu->seg[SEG_DS].sel, cpu->seg[SEG_DS].base,
		cpu->seg[SEG_ES].sel, cpu->seg[SEG_ES].base,
		cpu->seg[SEG_FS].sel, cpu->seg[SEG_GS].sel);
	dolog("reg: eax=%08x ebx=%08x ecx=%08x edx=%08x esi=%08x edi=%08x ebp=%08x esp=%08x eip=%08x\n",
		REGi(0), REGi(3), REGi(1), REGi(2),
		REGi(6), REGi(7), REGi(5), REGi(4), cpu->ip);
	/* Dump key IDT vectors (#DF=8, #TS=10, #GP=13, #PF=14) */
	if (cpu->cr0 & 1) {
		static const int key_vecs[] = { 8, 10, 13, 14 };
		static const char *key_names[] = { "#DF", "#TS", "#GP", "#PF" };
		for (int ki = 0; ki < 4; ki++) {
			uword iw1, iw2;
			if (read_idt_vector_raw_noexcept(cpu, key_vecs[ki], &iw1, &iw2)) {
				int igt = (iw2 >> 8) & 0xf;
				int idpl = (iw2 >> 13) & 3;
				int ip_ = (iw2 >> 15) & 1;
				bool ig16 = (igt == 6 || igt == 7);
				uword ioff = ig16 ? (iw1 & 0xffff)
					         : ((iw1 & 0xffff) | (iw2 & 0xffff0000));
				dolog("idt[%02x](%s): sel=%04x off=%08x ty=%x(%s) dpl=%d p=%d\n",
					key_vecs[ki], key_names[ki],
					(iw1 >> 16) & 0xffff, ioff,
					igt, ig16 ? "16" : "32", idpl, ip_);
			}
		}
	}
	if (wfw_monitor.r3_first_captured) {
		bool psp_valid = (wfw_monitor.psp_header[0] == 0xCD && wfw_monitor.psp_header[1] == 0x20);
		dolog("psp: valid=%u es_base=%08x cmdtail_len=%u env=%04x->%08x src=%s\n",
			psp_valid ? 1u : 0u, wfw_monitor.psp_ds_base, wfw_monitor.psp_cmdtail_len,
			wfw_monitor.psp_env_seg, wfw_monitor.psp_env_lin,
			wfw_monitor.psp_env_is_sel ? "sel" : "seg");
	}
	dolog("vm86: gp_total=%u soft_total=%u int21_soft=%u int2f_soft=%u int21_gp_log=%u\n",
		gp_total, wfw_monitor.v86_softint_total, wfw_monitor.v86_int21_count,
		wfw_monitor.v86_int2f_count, wfw_monitor.v86_int21_ah_count);
	if (wfw_monitor.v86_int13_count > 0) {
		dolog("int13: calls=%u ret=%u fail=%u p%u req=ax=%04x bx=%04x cx=%04x dx=%04x es=%04x ret=ax=%04x bx=%04x cx=%04x dx=%04x es=%04x cf=%u\n",
			wfw_monitor.v86_int13_count, wfw_monitor.v86_int13_ret_count,
			wfw_monitor.v86_int13_fail_count, wfw_monitor.v86_int13_last_path,
			wfw_monitor.v86_int13_req_ax, wfw_monitor.v86_int13_req_bx,
			wfw_monitor.v86_int13_req_cx, wfw_monitor.v86_int13_req_dx,
			wfw_monitor.v86_int13_req_es, wfw_monitor.v86_int13_ret_ax,
			wfw_monitor.v86_int13_ret_bx, wfw_monitor.v86_int13_ret_cx,
			wfw_monitor.v86_int13_ret_dx, wfw_monitor.v86_int13_ret_es,
			wfw_monitor.v86_int13_ret_cf);
	}
	if (wfw_monitor.abort_tail_count > 0) {
		int cnt = wfw_monitor.abort_tail_count;
		if (cnt > 10)
			cnt = 10;
		dolog("abort_trail:");
		for (int i = 0; i < cnt; i++) {
			int idx = (int)wfw_monitor.abort_tail_pos - cnt + i;
			while (idx < 0)
				idx += 16;
			idx %= 16;
			dolog(" [%02x/%c%d ah=%02x %04x:%08x ax=%04x bx=%04x cx=%04x dx=%04x ds=%04x]",
				wfw_monitor.abort_tail[idx].vec,
				wfw_monitor.abort_tail[idx].mode,
				wfw_monitor.abort_tail[idx].cpl,
				wfw_monitor.abort_tail[idx].ah,
				wfw_monitor.abort_tail[idx].cs,
				wfw_monitor.abort_tail[idx].ip,
				wfw_monitor.abort_tail[idx].ax,
				wfw_monitor.abort_tail[idx].bx,
				wfw_monitor.abort_tail[idx].cx,
				wfw_monitor.abort_tail[idx].dx,
				wfw_monitor.abort_tail[idx].ds);
		}
		dolog("\n");
	}
	if (wfw_monitor.r3_ret_tail_count > 0) {
		int cnt = wfw_monitor.r3_ret_tail_count;
		if (cnt > 10)
			cnt = 10;
		dolog("r3_tail:");
		for (int i = 0; i < cnt; i++) {
			int idx = (int)wfw_monitor.r3_ret_tail_pos - cnt + i;
			while (idx < 0)
				idx += 16;
			idx %= 16;
			dolog(" [%02x %04x>%04x cf=%u %04x:%08x bx=%04x cx=%04x dx=%04x ds=%04x]",
				wfw_monitor.r3_ret_tail[idx].vec,
				wfw_monitor.r3_ret_tail[idx].in_ax,
				wfw_monitor.r3_ret_tail[idx].out_ax,
				wfw_monitor.r3_ret_tail[idx].cf,
				wfw_monitor.r3_ret_tail[idx].out_cs,
				wfw_monitor.r3_ret_tail[idx].out_ip,
				wfw_monitor.r3_ret_tail[idx].out_bx,
				wfw_monitor.r3_ret_tail[idx].out_cx,
				wfw_monitor.r3_ret_tail[idx].out_dx,
				wfw_monitor.r3_ret_tail[idx].out_ds);
		}
		dolog("\n");
	}
	if (wfw_monitor.flow_count > 0) {
		int ordered[WFW_FLOW_RING_SIZE];
		int pick_core[48];
		int pick_abort[16];
		int ncore = 0;
		int nabort = 0;
		int scan = wfw_monitor.flow_count;
		if (scan > WFW_FLOW_RING_SIZE)
			scan = WFW_FLOW_RING_SIZE;
		for (int i = 0; i < scan; i++) {
			int idx = (int)wfw_monitor.flow_pos - scan + i;
			while (idx < 0)
				idx += WFW_FLOW_RING_SIZE;
			idx %= WFW_FLOW_RING_SIZE;
			ordered[i] = idx;
		}

		for (int k = 0; k < scan && (ncore < 48 || nabort < 16); k++) {
			int idx = (int)wfw_monitor.flow_pos - 1 - k;
			while (idx < 0)
				idx += WFW_FLOW_RING_SIZE;
			idx %= WFW_FLOW_RING_SIZE;
			if (wfw_monitor.flow_ring[idx].tag == 'A') {
				if (nabort < 16)
					pick_abort[nabort++] = idx;
			} else {
				if (ncore < 48)
					pick_core[ncore++] = idx;
			}
		}

		if (ncore > 0) {
			dolog("flow_core:");
			for (int pi = ncore - 1; pi >= 0; pi--) {
				int idx = pick_core[pi];
				switch (wfw_monitor.flow_ring[idx].tag) {
				case 'R':
					dolog(" [R%c%u/%02x %04x:%08x in=%04x ax=%04x bx=%04x cf=%u]",
					      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
					      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
					      wfw_monitor.flow_ring[idx].ip,
					      (uint16_t)(wfw_monitor.flow_ring[idx].extra >> 16),
					      wfw_monitor.flow_ring[idx].ax, wfw_monitor.flow_ring[idx].bx,
					      (unsigned)(wfw_monitor.flow_ring[idx].extra & 1));
					break;
				case 'E':
					if (wfw_monitor.flow_ring[idx].vec == EX_PF)
						dolog(" [E%c%u/%02x %04x:%08x c2=%08x ax=%04x bx=%04x]",
						      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
						      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
						      wfw_monitor.flow_ring[idx].ip, wfw_monitor.flow_ring[idx].extra,
						      wfw_monitor.flow_ring[idx].ax, wfw_monitor.flow_ring[idx].bx);
					else
						dolog(" [E%c%u/%02x %04x:%08x err=%x ax=%04x bx=%04x]",
						      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
						      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
						      wfw_monitor.flow_ring[idx].ip, wfw_monitor.flow_ring[idx].extra,
						      wfw_monitor.flow_ring[idx].ax, wfw_monitor.flow_ring[idx].bx);
					break;
				case 'D':
					dolog(" [D%c%u/%02x %04x:%08x err=%x ax=%04x bx=%04x]",
					      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
					      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
					      wfw_monitor.flow_ring[idx].ip, wfw_monitor.flow_ring[idx].extra,
					      wfw_monitor.flow_ring[idx].ax, wfw_monitor.flow_ring[idx].bx);
					break;
				case 'H':
					dolog(" [H%c%u/%02x %04x:%08x ax=%04x src=%04x]",
					      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
					      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
					      wfw_monitor.flow_ring[idx].ip, wfw_monitor.flow_ring[idx].ax,
					      (uint16_t)wfw_monitor.flow_ring[idx].extra);
					break;
				case 'K':
					dolog(" [K%c%u/%02x %04x:%08x req=%04x ret=%04x bx=%04x cf=%u]",
					      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
					      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
					      wfw_monitor.flow_ring[idx].ip,
					      (uint16_t)(wfw_monitor.flow_ring[idx].extra >> 16),
					      wfw_monitor.flow_ring[idx].ax,
					      wfw_monitor.flow_ring[idx].bx,
					      (unsigned)(wfw_monitor.flow_ring[idx].extra & 1));
					break;
				case 'Q':
					dolog(" [Q%c%u/%02x %04x:%08x in=%04x out=%04x bx=%04x r=%u cf=%u]",
					      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
					      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
					      wfw_monitor.flow_ring[idx].ip,
					      (uint16_t)(wfw_monitor.flow_ring[idx].extra >> 16),
					      wfw_monitor.flow_ring[idx].ax,
					      wfw_monitor.flow_ring[idx].bx,
					      (unsigned)((wfw_monitor.flow_ring[idx].extra >> 8) & 0xff),
					      (unsigned)(wfw_monitor.flow_ring[idx].extra & 1));
					break;
				default:
					dolog(" [%c%c%u/%02x %04x:%08x ax=%04x bx=%04x ex=%x]",
					      wfw_monitor.flow_ring[idx].tag, wfw_monitor.flow_ring[idx].mode,
					      wfw_monitor.flow_ring[idx].cpl, wfw_monitor.flow_ring[idx].vec,
					      wfw_monitor.flow_ring[idx].cs, wfw_monitor.flow_ring[idx].ip,
					      wfw_monitor.flow_ring[idx].ax, wfw_monitor.flow_ring[idx].bx,
					      wfw_monitor.flow_ring[idx].extra);
					break;
				}
			}
			dolog("\n");
		}
		if (nabort > 0) {
			dolog("flow_abort:");
			for (int pi = nabort - 1; pi >= 0; pi--) {
				int idx = pick_abort[pi];
				dolog(" [A%c%u/%02x %04x:%08x ah=%02x ax=%04x bx=%04x]",
				      wfw_monitor.flow_ring[idx].mode, wfw_monitor.flow_ring[idx].cpl,
				      wfw_monitor.flow_ring[idx].vec, wfw_monitor.flow_ring[idx].cs,
				      wfw_monitor.flow_ring[idx].ip,
				      (unsigned)(wfw_monitor.flow_ring[idx].extra & 0xff),
				      wfw_monitor.flow_ring[idx].ax, wfw_monitor.flow_ring[idx].bx);
			}
			dolog("\n");
		}
			/* Event immediately preceding the first abort-site DOS call.
			 * Skip contiguous abort-site entries and print the nearest
			 * non-abort event so the ring-0 origin is always visible. */
			{
				int first_abort_pos = -1;
				for (int i = 0; i < scan; i++) {
					if (wfw_monitor.flow_ring[ordered[i]].tag == 'A') {
						first_abort_pos = i;
						break;
					}
				}
				if (first_abort_pos >= 0) {
					int prev = first_abort_pos - 1;
					while (prev >= 0 && wfw_monitor.flow_ring[ordered[prev]].tag == 'A')
						prev--;
					if (prev >= 0) {
						int idx = ordered[prev];
						dolog("pre_abort_origin: [%c%c%u/%02x %04x:%08x ax=%04x bx=%04x ex=%x]\n",
						      wfw_monitor.flow_ring[idx].tag,
						      wfw_monitor.flow_ring[idx].mode,
						      wfw_monitor.flow_ring[idx].cpl,
						      wfw_monitor.flow_ring[idx].vec,
						      wfw_monitor.flow_ring[idx].cs,
						      wfw_monitor.flow_ring[idx].ip,
						      wfw_monitor.flow_ring[idx].ax,
						      wfw_monitor.flow_ring[idx].bx,
						      wfw_monitor.flow_ring[idx].extra);
					}
				}
			}
		}
		dolog("vm86_gpvec:13=%u 16=%u 1a=%u 1c=%u 21=%u 28=%u 2a=%u 2f=%u 4b=%u\n",
			wfw_monitor.v86_int_gp_counts[0x13], wfw_monitor.v86_int_gp_counts[0x16],
			wfw_monitor.v86_int_gp_counts[0x1a], wfw_monitor.v86_int_gp_counts[0x1c],
			wfw_monitor.v86_int_gp_counts[0x21], wfw_monitor.v86_int_gp_counts[0x28],
			wfw_monitor.v86_int_gp_counts[0x2a], wfw_monitor.v86_int_gp_counts[0x2f],
			wfw_monitor.v86_int_gp_counts[0x4b]);
		if (wfw_monitor.lowvec_softint_total > 0) {
			dolog("lowvec_softint: total=%u 11=%u 13=%u 16=%u 19=%u 1a=%u 1c=%u\n",
				wfw_monitor.lowvec_softint_total,
				wfw_monitor.lowvec_softint_hist[0x11],
				wfw_monitor.lowvec_softint_hist[0x13],
				wfw_monitor.lowvec_softint_hist[0x16],
				wfw_monitor.lowvec_softint_hist[0x19],
				wfw_monitor.lowvec_softint_hist[0x1a],
				wfw_monitor.lowvec_softint_hist[0x1c]);
		}
		if (wfw_monitor.vm86_int_evt_count > 0) {
			dolog("vm86_evt:");
			for (int i = 0; i < wfw_monitor.vm86_int_evt_count; i++) {
				dolog(" [%02x p%u i%u ax=%04x %04x:%04x]",
					wfw_monitor.vm86_int_evt[i].vec,
					wfw_monitor.vm86_int_evt[i].path,
					wfw_monitor.vm86_int_evt[i].iopl,
					wfw_monitor.vm86_int_evt[i].ax,
					wfw_monitor.vm86_int_evt[i].cs,
					wfw_monitor.vm86_int_evt[i].ip);
			}
			dolog("\n");
		}
	if (wfw_monitor.iopl3_captured) {
		dolog("iopl3: after_irets=%u from=%04x:%08x to=%04x:%08x fl=%08x ivt2f=%04x:%04x\n",
			wfw_monitor.iopl3_iret_count,
			wfw_monitor.iopl3_from_cs, wfw_monitor.iopl3_from_eip,
			wfw_monitor.iopl3_to_cs, wfw_monitor.iopl3_to_ip,
			wfw_monitor.iopl3_eflags,
			wfw_monitor.iopl3_ivt_2f >> 16, wfw_monitor.iopl3_ivt_2f & 0xffff);
	} else {
		dolog("iopl3: never_reached (pm_to_vm86_irets=%u)\n", wfw_monitor.iopl3_iret_count);
	}
	if (wfw_monitor.post_iopl3_fault_captured) {
		dolog("post_iopl3: exc=%02x@%04x:%08x fl=%08x err=%08x",
			wfw_monitor.post_iopl3_excno, wfw_monitor.post_iopl3_cs,
			wfw_monitor.post_iopl3_eip, wfw_monitor.post_iopl3_fl,
			wfw_monitor.post_iopl3_err);
		if (wfw_monitor.post_iopl3_decoded[0])
			dolog(" dec=%s", wfw_monitor.post_iopl3_decoded);
		if (wfw_monitor.post_iopl3_blen > 0) {
			dolog(" b=");
			for (int i = 0; i < wfw_monitor.post_iopl3_blen; i++)
				dolog("%02x", wfw_monitor.post_iopl3_bytes[i]);
		}
		if (wfw_monitor.post_iopl3_last_evt >= 0 &&
		    wfw_monitor.post_iopl3_last_evt < (int8_t)wfw_monitor.vm86_int_evt_count) {
			int k = wfw_monitor.post_iopl3_last_evt;
			dolog(" prev=[%02x p%u i%u ax=%04x %04x:%04x]",
				wfw_monitor.vm86_int_evt[k].vec,
				wfw_monitor.vm86_int_evt[k].path,
				wfw_monitor.vm86_int_evt[k].iopl,
				wfw_monitor.vm86_int_evt[k].ax,
				wfw_monitor.vm86_int_evt[k].cs,
				wfw_monitor.vm86_int_evt[k].ip);
		}
		dolog("\n");
	}
	if (vm86_trip.gen == cpu->diag_gen) {
		if (vm86_trip.has_first_bad) {
			dolog("trip: n=%u frozen=%u first=#%02x@%04x:%08x dec=%s\n",
				vm86_trip.count, vm86_trip.frozen ? 1u : 0u,
				vm86_trip.first_bad_exc, vm86_trip.first_bad_cs,
				vm86_trip.first_bad_ip, vm86_trip.first_bad_decoded);
		} else {
			dolog("trip: n=%u frozen=%u first=<none>\n",
				vm86_trip.count, vm86_trip.frozen ? 1u : 0u);
		}
	}
	if (wfw_monitor.bios_hot_count > 0) {
		dolog("bios_hot:");
		for (int i = 0; i < wfw_monitor.bios_hot_count; i++) {
			dolog(" [%04x:%04x %s x%u refl=%u adv=%u",
				wfw_monitor.bios_hot[i].cs, wfw_monitor.bios_hot[i].ip,
				(wfw_monitor.bios_hot[i].exc == EX_GP) ? "#GP" : "#UD",
				wfw_monitor.bios_hot[i].count,
				wfw_monitor.bios_hot[i].resume_same,
				wfw_monitor.bios_hot[i].resume_other);
			if (wfw_monitor.bios_hot[i].decoded[0])
				dolog(" dec=%s", wfw_monitor.bios_hot[i].decoded);
			if (wfw_monitor.bios_hot[i].blen > 0) {
				dolog(" b=");
				for (int j = 0; j < wfw_monitor.bios_hot[i].blen; j++)
					dolog("%02x", wfw_monitor.bios_hot[i].bytes[j]);
			}
			dolog("]");
		}
		dolog("\n");
	}
		if (wfw_monitor.ud63_hot_count > 0) {
			dolog("op63:");
		for (int i = 0; i < wfw_monitor.ud63_hot_count; i++) {
			uint32_t adv = (uint32_t)wfw_monitor.ud63_hot[i].resume_next4 +
				       (uint32_t)wfw_monitor.ud63_hot[i].resume_next8 +
				       (uint32_t)wfw_monitor.ud63_hot[i].resume_other;
			dolog(" [%04x:%04x x%u refl=%u adv=%u(+4=%u +8=%u ro=%u) ax=%04x>%04x bx=%04x cx=%04x dx=%04x si=%04x di=%04x ss:sp=%04x:%04x fl=%08x rfl=%08x cf=%u",
				wfw_monitor.ud63_hot[i].cs, wfw_monitor.ud63_hot[i].ip,
				wfw_monitor.ud63_hot[i].count,
				wfw_monitor.ud63_hot[i].resume_same,
				adv,
				wfw_monitor.ud63_hot[i].resume_next4,
				wfw_monitor.ud63_hot[i].resume_next8,
				wfw_monitor.ud63_hot[i].resume_other,
				wfw_monitor.ud63_hot[i].ax,
				wfw_monitor.ud63_hot[i].out_ax,
				wfw_monitor.ud63_hot[i].bx,
				wfw_monitor.ud63_hot[i].cx, wfw_monitor.ud63_hot[i].dx,
				wfw_monitor.ud63_hot[i].si, wfw_monitor.ud63_hot[i].di,
				wfw_monitor.ud63_hot[i].ss,
				(uint16_t)(wfw_monitor.ud63_hot[i].sp & 0xffff),
				wfw_monitor.ud63_hot[i].fl,
				wfw_monitor.ud63_hot[i].out_fl,
				wfw_monitor.ud63_hot[i].cf_set);
			if (wfw_monitor.ud63_hot[i].blen > 0) {
				dolog(" b=");
				for (int j = 0; j < wfw_monitor.ud63_hot[i].blen; j++)
					dolog("%02x", wfw_monitor.ud63_hot[i].bytes[j]);
			}
			dolog("]");
			}
			dolog("\n");
		}
		if (wfw_monitor.ud63_fe95_0127.seen > 0) {
			dolog("op63_fe95_0127: seen=%u resumed=%u changed=%u pending=%u\n",
				(unsigned)wfw_monitor.ud63_fe95_0127.seen,
				(unsigned)wfw_monitor.ud63_fe95_0127.resumed,
				(unsigned)wfw_monitor.ud63_fe95_0127.desc_changed,
				wfw_monitor.pending_ud63_fe95_0127 ? 1u : 0u);
			dolog("  pre@%04x:%04x ds=%04x ax=%04x fl=%08x 0127=[%08x %08x]",
				wfw_monitor.ud63_fe95_0127.pre_cs,
				wfw_monitor.ud63_fe95_0127.pre_ip,
				wfw_monitor.ud63_fe95_0127.pre_ds,
				wfw_monitor.ud63_fe95_0127.pre_ax,
				wfw_monitor.ud63_fe95_0127.pre_fl,
				wfw_monitor.ud63_fe95_0127.pre_w1,
				wfw_monitor.ud63_fe95_0127.pre_w2);
			if (wfw_monitor.ud63_fe95_0127.pre_w1 != 0xffffffffu ||
			    wfw_monitor.ud63_fe95_0127.pre_w2 != 0xffffffffu) {
				dolog(" b=%08x l=%08x fl=%04x",
					wfw_desc_base_u32(wfw_monitor.ud63_fe95_0127.pre_w1,
							wfw_monitor.ud63_fe95_0127.pre_w2),
					wfw_desc_limit_u32(wfw_monitor.ud63_fe95_0127.pre_w1,
							 wfw_monitor.ud63_fe95_0127.pre_w2),
					wfw_desc_flags_u16(wfw_monitor.ud63_fe95_0127.pre_w2));
			}
			dolog("\n");
			if (wfw_monitor.ud63_fe95_0127.resumed > 0) {
				dolog("  post->%04x:%04x ds_now=%04x ds_tgt=%04x ax=%04x fl=%08x 0127=[%08x %08x]",
					wfw_monitor.ud63_fe95_0127.post_cs,
					wfw_monitor.ud63_fe95_0127.post_ip,
					wfw_monitor.ud63_fe95_0127.post_ds,
					wfw_monitor.ud63_fe95_0127.resume_ds,
					wfw_monitor.ud63_fe95_0127.post_ax,
					wfw_monitor.ud63_fe95_0127.post_fl,
					wfw_monitor.ud63_fe95_0127.post_w1,
					wfw_monitor.ud63_fe95_0127.post_w2);
				if (wfw_monitor.ud63_fe95_0127.post_w1 != 0xffffffffu ||
				    wfw_monitor.ud63_fe95_0127.post_w2 != 0xffffffffu) {
					dolog(" b=%08x l=%08x fl=%04x",
						wfw_desc_base_u32(wfw_monitor.ud63_fe95_0127.post_w1,
								wfw_monitor.ud63_fe95_0127.post_w2),
						wfw_desc_limit_u32(wfw_monitor.ud63_fe95_0127.post_w1,
								 wfw_monitor.ud63_fe95_0127.post_w2),
						wfw_desc_flags_u16(wfw_monitor.ud63_fe95_0127.post_w2));
				}
				dolog("\n");
			}
			dolog("  near_changed: %04x=%u %04x=%u %04x=%u %04x=%u\n",
				wfw_ud63_near_sels[0], (unsigned)wfw_monitor.ud63_fe95_0127.near_changed[0],
				wfw_ud63_near_sels[1], (unsigned)wfw_monitor.ud63_fe95_0127.near_changed[1],
				wfw_ud63_near_sels[2], (unsigned)wfw_monitor.ud63_fe95_0127.near_changed[2],
				wfw_ud63_near_sels[3], (unsigned)wfw_monitor.ud63_fe95_0127.near_changed[3]);
			dolog("  near_pre: ");
			for (int k = 0; k < 4; k++) {
				if (k)
					dolog(" ");
				if (wfw_monitor.ud63_fe95_0127.pre_near_ok & (1u << k)) {
					dolog("%04x=[%08x %08x]",
						wfw_ud63_near_sels[k],
						wfw_monitor.ud63_fe95_0127.pre_near_w1[k],
						wfw_monitor.ud63_fe95_0127.pre_near_w2[k]);
				} else {
					dolog("%04x=[???????? ????????]", wfw_ud63_near_sels[k]);
				}
			}
			dolog("\n");
			if (wfw_monitor.ud63_fe95_0127.resumed > 0) {
				dolog("  near_post:");
				for (int k = 0; k < 4; k++) {
					if (k)
						dolog(" ");
					if (wfw_monitor.ud63_fe95_0127.post_near_ok & (1u << k)) {
						dolog("%04x=[%08x %08x]",
							wfw_ud63_near_sels[k],
							wfw_monitor.ud63_fe95_0127.post_near_w1[k],
							wfw_monitor.ud63_fe95_0127.post_near_w2[k]);
					} else {
						dolog("%04x=[???????? ????????]", wfw_ud63_near_sels[k]);
					}
				}
				dolog("\n");
			}
			dolog("  cb_retax: ax5=%u other=%u last=%04x\n",
				(unsigned)wfw_monitor.ud63_fe95_0127.ret_ax5,
				(unsigned)wfw_monitor.ud63_fe95_0127.ret_ax_other,
				wfw_monitor.ud63_fe95_0127.ret_ax_last);
			if (wfw_monitor.ud63_fe95_0127.t6ca7_count > 0) {
				dolog("  cb_t6ca7: count=%u bit2_set=%u bit2_clear=%u last=%02x ss:bp=%04x:%04x\n",
					(unsigned)wfw_monitor.ud63_fe95_0127.t6ca7_count,
					(unsigned)wfw_monitor.ud63_fe95_0127.t6ca7_bit_set,
					(unsigned)wfw_monitor.ud63_fe95_0127.t6ca7_bit_clear,
					(unsigned)wfw_monitor.ud63_fe95_0127.t6ca7_last_byte,
					wfw_monitor.ud63_fe95_0127.t6ca7_last_ss,
					wfw_monitor.ud63_fe95_0127.t6ca7_last_bp);
			}
			if (wfw_monitor.ud63_fe95_0127.j6cab_count > 0 ||
			    wfw_monitor.ud63_fe95_0127.j6cab_pending) {
				dolog("  cb_j6cab: count=%u taken=%u not=%u other=%u pending=%u zf=%u from=%04x:%04x tgt=%04x fall=%04x to=%04x:%04x\n",
					(unsigned)wfw_monitor.ud63_fe95_0127.j6cab_count,
					(unsigned)wfw_monitor.ud63_fe95_0127.j6cab_taken,
					(unsigned)wfw_monitor.ud63_fe95_0127.j6cab_not_taken,
					(unsigned)wfw_monitor.ud63_fe95_0127.j6cab_other,
					wfw_monitor.ud63_fe95_0127.j6cab_pending ? 1u : 0u,
					(unsigned)wfw_monitor.ud63_fe95_0127.j6cab_last_zf,
					wfw_monitor.ud63_fe95_0127.j6cab_last_from_cs,
					wfw_monitor.ud63_fe95_0127.j6cab_last_from_ip,
					wfw_monitor.ud63_fe95_0127.j6cab_last_tgt,
					wfw_monitor.ud63_fe95_0127.j6cab_last_fall,
					wfw_monitor.ud63_fe95_0127.j6cab_last_to_cs,
					wfw_monitor.ud63_fe95_0127.j6cab_last_to_ip);
			}
			if (wfw_monitor.ud63_fe95_0127.ax5_transitions > 0) {
				dolog("  cb_ax5: trans=%u from=%04x:%04x ax=%04x to=%04x:%04x ax=%04x b=",
					(unsigned)wfw_monitor.ud63_fe95_0127.ax5_transitions,
					wfw_monitor.ud63_fe95_0127.ax5_from_cs,
					wfw_monitor.ud63_fe95_0127.ax5_from_ip,
					wfw_monitor.ud63_fe95_0127.ax5_from_ax,
					wfw_monitor.ud63_fe95_0127.ax5_to_cs,
					wfw_monitor.ud63_fe95_0127.ax5_to_ip,
					wfw_monitor.ud63_fe95_0127.ax5_to_ax);
				if (wfw_monitor.ud63_fe95_0127.ax5_from_blen == 0) {
					dolog("??");
				} else {
					for (int j = 0; j < wfw_monitor.ud63_fe95_0127.ax5_from_blen; j++)
						dolog("%02x", wfw_monitor.ud63_fe95_0127.ax5_from_bytes[j]);
				}
				dolog("\n");
			}
			if (wfw_monitor.ud63_fe95_0127.trace_done ||
			    wfw_monitor.ud63_fe95_0127.trace_active) {
				dolog("  cb_trace: active=%u done=%u steps=%u entry_ax=%04x",
					wfw_monitor.ud63_fe95_0127.trace_active ? 1u : 0u,
					wfw_monitor.ud63_fe95_0127.trace_done ? 1u : 0u,
					(unsigned)wfw_monitor.ud63_fe95_0127.trace_count,
					wfw_monitor.ud63_fe95_0127.trace_entry_ax);
				if (wfw_monitor.ud63_fe95_0127.trace_ax5_valid)
					dolog(" ax5@%04x:%04x %04x>%04x",
						wfw_monitor.ud63_fe95_0127.trace_ax5_cs,
						wfw_monitor.ud63_fe95_0127.trace_ax5_ip,
						wfw_monitor.ud63_fe95_0127.trace_ax5_prev,
						wfw_monitor.ud63_fe95_0127.trace_ax5_ax);
				dolog("\n");
				for (int t = 0; t < wfw_monitor.ud63_fe95_0127.trace_count; t++) {
					dolog("    [%2d] %04x:%04x ax=%04x bx=%04x cx=%04x dx=%04x si=%04x di=%04x bp=%04x sp=%04x fl=%08x b=",
						t,
						wfw_monitor.ud63_fe95_0127.trace[t].cs,
						wfw_monitor.ud63_fe95_0127.trace[t].ip,
						wfw_monitor.ud63_fe95_0127.trace[t].ax,
						wfw_monitor.ud63_fe95_0127.trace[t].bx,
						wfw_monitor.ud63_fe95_0127.trace[t].cx,
						wfw_monitor.ud63_fe95_0127.trace[t].dx,
						wfw_monitor.ud63_fe95_0127.trace[t].si,
						wfw_monitor.ud63_fe95_0127.trace[t].di,
						wfw_monitor.ud63_fe95_0127.trace[t].bp,
						wfw_monitor.ud63_fe95_0127.trace[t].sp,
						wfw_monitor.ud63_fe95_0127.trace[t].fl);
					if (wfw_monitor.ud63_fe95_0127.trace[t].blen == 0)
						dolog("??");
					else
						for (int j = 0; j < wfw_monitor.ud63_fe95_0127.trace[t].blen; j++)
							dolog("%02x", wfw_monitor.ud63_fe95_0127.trace[t].bytes[j]);
					dolog("\n");
				}
			}
		}
	if (wfw_monitor.swint_dpl_count > 0) {
			dolog("swint_dpl:");
		for (int i = 0; i < wfw_monitor.swint_dpl_count; i++) {
			dolog(" [%02x x%u first=%04x:%08x/e=%x rs=%u rn=%u ro=%u ax=%04x>%04x bx=%04x>%04x cf=%u fl=%08x]",
				wfw_monitor.swint_dpl[i].vec,
				wfw_monitor.swint_dpl[i].count,
				wfw_monitor.swint_dpl[i].first_cs,
				wfw_monitor.swint_dpl[i].first_ip,
				wfw_monitor.swint_dpl[i].first_err,
				wfw_monitor.swint_dpl[i].resume_same,
				wfw_monitor.swint_dpl[i].resume_next,
				wfw_monitor.swint_dpl[i].resume_other,
				wfw_monitor.swint_dpl[i].in_ax,
				wfw_monitor.swint_dpl[i].out_ax,
				wfw_monitor.swint_dpl[i].in_bx,
				wfw_monitor.swint_dpl[i].out_bx,
				wfw_monitor.swint_dpl[i].cf_set,
				wfw_monitor.swint_dpl[i].out_fl);
		}
		dolog("\n");
	}
	if (wfw_monitor.dpl_tail_count > 0) {
		int cnt = wfw_monitor.dpl_tail_count;
		if (cnt > 12)
			cnt = 12;
		dolog("dpl_tail:");
		for (int i = 0; i < cnt; i++) {
			int idx = (int)wfw_monitor.dpl_tail_pos - cnt + i;
			while (idx < 0)
				idx += 16;
			idx %= 16;
			dolog(" [%02x %04x>%04x bx=%04x>%04x r=%u cf=%u %04x:%08x=>%04x:%08x]",
				wfw_monitor.dpl_tail[idx].vec,
				wfw_monitor.dpl_tail[idx].in_ax,
				wfw_monitor.dpl_tail[idx].out_ax,
				wfw_monitor.dpl_tail[idx].in_bx,
				wfw_monitor.dpl_tail[idx].out_bx,
				wfw_monitor.dpl_tail[idx].rcls,
				wfw_monitor.dpl_tail[idx].cf,
				wfw_monitor.dpl_tail[idx].in_cs,
				wfw_monitor.dpl_tail[idx].in_ip,
				wfw_monitor.dpl_tail[idx].out_cs,
				wfw_monitor.dpl_tail[idx].out_ip);
		}
		dolog("\n");
	}
	if (wfw_monitor.seg100f_base_count > 0) {
		dolog("sel100f:");
		for (int i = 0; i < wfw_monitor.seg100f_base_count; i++)
			dolog(" %05x", wfw_monitor.seg100f_bases[i]);
		dolog("\n");
	}
	if (wfw_monitor.has_fatal) {
		dolog("fatal: exc=%02x@%04x:%08x ds=%04x es=%04x ss:esp=%04x:%08x fl=%08x cpl=%u vm=%u ext=%u pe=%u",
			wfw_monitor.fatal_excno, wfw_monitor.fatal_cs, wfw_monitor.fatal_eip,
			wfw_monitor.fatal_ds, wfw_monitor.fatal_es,
			wfw_monitor.fatal_ss, wfw_monitor.fatal_esp, wfw_monitor.fatal_efl,
			wfw_monitor.fatal_cpl, wfw_monitor.fatal_vm,
			wfw_monitor.fatal_ext, wfw_monitor.fatal_pusherr);
		if (wfw_monitor.fatal_decoded[0])
			dolog(" dec=%s", wfw_monitor.fatal_decoded);
		if (wfw_monitor.fatal_blen > 0) {
			dolog(" b=");
			for (int i = 0; i < wfw_monitor.fatal_blen; i++)
				dolog("%02x", wfw_monitor.fatal_bytes[i]);
		}
		dolog("\n");
	}
	{
		int n = exc_ring_pos > 32 ? 32 : exc_ring_pos;
		if (n > 0) {
			int start = exc_ring_pos - n;
			dolog("exc_tail: last %d of %d exceptions\n", n, exc_ring_pos);
			for (int i = start; i < exc_ring_pos; i++) {
				int j = i & EXC_RING_MASK;
				const char *ename = (exc_ring[j].no < 32) ? exc_names[exc_ring[j].no] : "IRQ";
				if (exc_ring[j].no == EX_PF)
					dolog("  [%4d] %c %s @%04x:%08x err=%04x cr2=%08x\n",
						i, exc_ring[j].mode, ename,
						exc_ring[j].cs, exc_ring[j].ip,
						exc_ring[j].err, exc_ring[j].cr2);
				else
					dolog("  [%4d] %c %s @%04x:%08x err=%04x\n",
						i, exc_ring[j].mode, ename,
						exc_ring[j].cs, exc_ring[j].ip,
						exc_ring[j].err);
			}
		}
	}
	dolog("=== End CPU Monitor ===\n");
}

static void cpu_diag_log_int21(CPUI386 *cpu)
{
	static int diag_gen = -1;
	static uint16_t print_count = 0;
	static uint16_t write_count = 0;
	char text[128];
	uint8_t ah, al;
	u16 ax, bx, cx, dx;

	if (!cpu_diag_enabled)
		return;
	cpu_diag_win386_sync(cpu);
	if (diag_gen != cpu->diag_gen) {
		diag_gen = cpu->diag_gen;
		print_count = 0;
		write_count = 0;
	}

	ah = lreg8(4);
	al = lreg8(0);
	ax = lreg16(0);
	bx = lreg16(3);
	cx = lreg16(1);
	dx = lreg16(2);

	if (ah == 0x4b) {
		cpu_diag_copy_seg_string(cpu, SEG_DS, dx, cpu->cpl, 0, text, sizeof(text));
		dolog("=== INT 21h EXEC ===\n");
		dolog("  AX=%04x BX=%04x CX=%04x DX=%04x DS:DX=%04x:%04x path=\"%s\"\n",
			ax, bx, cx, dx, cpu->seg[SEG_DS].sel, dx, text);
		dolog("  at CS:EIP=%04x:%08x %s FL=%08x\n",
			cpu->seg[SEG_CS].sel, cpu->ip,
			cpu_mode_name(cpu->flags, cpu->cr0), cpu->flags);
			if (cpu_diag_str_has_win386(text)) {
				win386_diag.active = true;
				win386_diag.code_cs = 0;
				win386_diag.int21_count = 0;
				win386_diag.int2f_count = 0;
				win386_diag.pm_exc_count = 0;
				memset(&wfw_monitor, 0, sizeof(wfw_monitor));
				memset(&vm86_trip, 0, sizeof(vm86_trip));
				vm86_trip.gen = cpu->diag_gen;
				memset(&dpmi_trace, 0, sizeof(dpmi_trace));
				wfw_monitor.pending_bios_fault_idx = -1;
				wfw_monitor.pending_ud63_idx = -1;
				wfw_monitor.pending_swint_dpl_idx = -1;
				wfw_monitor.post_iopl3_last_evt = -1;
				wfw_monitor.a20_state = (cpu->a20_mask == 0xFFFFFFFF) ? 1 : 0;
			wfw_monitor.ivt_2f_at_start = *(uint32_t*)(cpu->phys_mem + 0x2f * 4);
			dolog("=== WIN386 trace start ===\n");
		}
		return;
	}

	if (win386_diag.active && ah != 0x40 && ah != 0x09) {
		win386_diag.int21_count++;
	}
	if (win386_diag.active)
		wfw_abort_tail_push(cpu, 0x21, ax, bx, cx, dx);

	if (ah == 0x4c || ah == 0x31 || ah == 0x00 || ah == 0x27) {
		dolog("=== INT 21h TERMINATE ===\n");
		dolog("  AH=%02x AL=%02x AX=%04x BX=%04x CX=%04x DX=%04x DS=%04x\n",
			ah, al, ax, bx, cx, dx, cpu->seg[SEG_DS].sel);
		dolog("  at CS:EIP=%04x:%08x %s FL=%08x\n",
			cpu->seg[SEG_CS].sel, cpu->ip,
			cpu_mode_name(cpu->flags, cpu->cr0), cpu->flags);
		if (al == 0xff && vm86_last_fault.valid && vm86_last_fault.gen == cpu->diag_gen) {
			dolog("  last VM86 fault: exc=%02x CS:EIP=%04x:%08x FL=%08x err=%08x %s\n",
				vm86_last_fault.excno, vm86_last_fault.cs, vm86_last_fault.ip,
				vm86_last_fault.fl, vm86_last_fault.err, vm86_last_fault.decoded);
			/* Only dump full tail during WIN386 session to avoid
			 * repetitive dumps from parent process exits */
			if (win386_diag.active) {
				cpu_diag_vm86_fault_ring_dump_tail(8);
				cpu_diag_vm86_iret_ring_dump_tail(8);
			}
		}
		if (win386_diag.active) {
			if (win386_diag.code_cs == 0)
				win386_diag.code_cs = cpu->seg[SEG_CS].sel;
			if (cpu->seg[SEG_CS].sel == win386_diag.code_cs) {
				cpu_monitor_dump(cpu, "WIN386 exit");
				dolog("=== WIN386 trace stop ===\n");
				dolog("  exit code AL=%02x CS=%04x int21=%u int2f=%u pm_exc=%u\n",
					al, cpu->seg[SEG_CS].sel, win386_diag.int21_count,
					win386_diag.int2f_count, win386_diag.pm_exc_count);
				win386_diag.active = false;
			}
		}
		return;
	}

	if (ah == 0x09 && print_count < 24) {
		print_count++;
		cpu_diag_copy_seg_string(cpu, SEG_DS, dx, cpu->cpl, '$', text, sizeof(text));
		dolog("=== INT 21h PRINT #%u === \"%s\"\n", print_count, text);
		return;
	}

	if (ah == 0x40 && (bx == 1 || bx == 2) && cx > 0 && write_count < 40) {
		size_t n = (size_t)cx;
		if (n >= sizeof(text))
			n = sizeof(text) - 1;
		size_t i = 0;
		for (; i < n; i++) {
			uint8_t b = 0;
			if (!read_seg_u8_noexcept(cpu, SEG_DS, dx + (uword)i, cpu->cpl, &b))
				break;
			text[i] = (b >= 0x20 && b <= 0x7e) ? (char)b : '.';
		}
		text[i] = '\0';
		write_count++;
		dolog("=== INT 21h WRITE #%u h=%u len=%u === \"%s\"\n",
			write_count, bx, cx, text);
		return;
	}

	if (ah == 0x59) {
		dolog("=== INT 21h GET EXTENDED ERROR ===\n");
		dolog("  AX=%04x BX=%04x CX=%04x DX=%04x DS=%04x at %04x:%08x\n",
			ax, bx, cx, dx, cpu->seg[SEG_DS].sel, cpu->seg[SEG_CS].sel, cpu->ip);
		return;
	}

	if (ah == 0x4d) {
		dolog("=== INT 21h GET RETURN CODE === AX=%04x BX=%04x CX=%04x DX=%04x at %04x:%08x\n",
			ax, bx, cx, dx, cpu->seg[SEG_CS].sel, cpu->ip);
	}
}

static void cpu_diag_log_int2f(CPUI386 *cpu)
{
	if (!cpu_diag_enabled)
		return;
	cpu_diag_win386_sync(cpu);
	if (!win386_diag.active)
		return;
	{
		uint16_t ax = lreg16(0);
		uint16_t bx = lreg16(3);
		uint16_t cx = lreg16(1);
		uint16_t dx = lreg16(2);
		wfw_abort_tail_push(cpu, 0x2f, ax, bx, cx, dx);
	}
	win386_diag.int2f_count++;
}

/*
 * instructions
 */

/*
 * Optimized 8-bit ALU helpers using pre-computed flags tables.
 * These directly store all flags from table lookup, avoiding lazy computation.
 */
#define AOP8_ADD_helper(a, b, la, sa, lb, sb) \
	{ \
		u8 _s1 = la(a), _s2 = lb(b); \
		u8 _r = _s1 + _s2; \
		if (flags_add8) { \
			cpu->flags = (cpu->flags & ~(CF|PF|AF|ZF|SF|OF)) | (*flags_add8)[_s1][_s2]; \
			cpu->cc.mask = 0; \
		} else { \
			cpu->cc.src1 = sext8(_s1); \
			cpu->cc.src2 = sext8(_s2); \
			cpu->cc.dst = sext8(_r); \
			cpu->cc.op = CC_ADD; \
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
		} \
		sa(a, _r); \
	}

#define AOP8_SUB_helper(a, b, la, sa, lb, sb) \
	{ \
		u8 _s1 = la(a), _s2 = lb(b); \
		u8 _r = _s1 - _s2; \
		if (flags_sub8) { \
			cpu->flags = (cpu->flags & ~(CF|PF|AF|ZF|SF|OF)) | (*flags_sub8)[_s1][_s2]; \
			cpu->cc.mask = 0; \
		} else { \
			cpu->cc.src1 = sext8(_s1); \
			cpu->cc.src2 = sext8(_s2); \
			cpu->cc.dst = sext8(_r); \
			cpu->cc.op = CC_SUB; \
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
		} \
		sa(a, _r); \
	}

#define AOP8_CMP_helper(a, b, la, sa, lb, sb) \
	{ \
		u8 _s1 = la(a), _s2 = lb(b); \
		if (flags_sub8) { \
			cpu->flags = (cpu->flags & ~(CF|PF|AF|ZF|SF|OF)) | (*flags_sub8)[_s1][_s2]; \
			cpu->cc.mask = 0; \
		} else { \
			cpu->cc.src1 = sext8(_s1); \
			cpu->cc.src2 = sext8(_s2); \
			cpu->cc.dst = sext8((u8)(_s1 - _s2)); \
			cpu->cc.op = CC_SUB; \
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
		} \
	}

#define LOP8_helper(OP, a, b, la, sa, lb, sb) \
	{ \
		u8 _r = la(a) OP lb(b); \
		if (flags_logic8) { \
			cpu->flags = (cpu->flags & ~(CF|PF|ZF|SF|OF)) | flags_logic8[_r]; \
			cpu->cc.mask = 0; \
		} else { \
			cpu->cc.dst = sext8(_r); \
			cpu->cc.op = CC_AND; \
			cpu->cc.mask = CF | PF | ZF | SF | OF; \
		} \
		sa(a, _r); \
	}

#define LOP8_TEST_helper(OP, a, b, la, sa, lb, sb) \
	{ \
		u8 _r = la(a) OP lb(b); \
		if (flags_logic8) { \
			cpu->flags = (cpu->flags & ~(CF|PF|ZF|SF|OF)) | flags_logic8[_r]; \
			cpu->cc.mask = 0; \
		} else { \
			cpu->cc.dst = sext8(_r); \
			cpu->cc.op = CC_AND; \
			cpu->cc.mask = CF | PF | ZF | SF | OF; \
		} \
	}

#define ACOP_helper(NAME1, NAME2, BIT, OP, a, b, la, sa, lb, sb) \
	int cf = get_CF(cpu); \
	cpu->cc.src1 = sext ## BIT(la(a)); \
	cpu->cc.src2 = sext ## BIT(lb(b)); \
	cpu->cc.dst = sext ## BIT(cpu->cc.src1 OP cpu->cc.src2 OP cf); \
	cpu->cc.op = cf ? CC_ ## NAME1 : CC_ ## NAME2; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define AOP0_helper(NAME, BIT, OP, a, b, la, sa, lb, sb) \
	cpu->cc.src1 = sext ## BIT(la(a)); \
	cpu->cc.src2 = sext ## BIT(lb(b)); \
	cpu->cc.dst = sext ## BIT(cpu->cc.src1 OP cpu->cc.src2); \
	cpu->cc.op = CC_ ## NAME; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF;

#define LOP0_helper(NAME, BIT, OP, a, b, la, sa, lb, sb) \
	cpu->cc.dst = sext ## BIT(la(a) OP lb(b)); \
	cpu->cc.op = CC_ ## NAME; \
	cpu->cc.mask = CF | PF | ZF | SF | OF;

#define AOP_helper(NAME1, BIT, OP, a, b, la, sa, lb, sb) \
	AOP0_helper(NAME1, BIT, OP, a, b, la, sa, lb, sb) \
	sa(a, cpu->cc.dst);

#define LOP_helper(NAME1, BIT, OP, a, b, la, sa, lb, sb) \
	LOP0_helper(NAME1, BIT, OP, a, b, la, sa, lb, sb) \
	sa(a, cpu->cc.dst);

#define INCDEC_helper(NAME, BIT, OP, a, la, sa) \
	int cf = get_CF(cpu); \
	cpu->cc.dst = sext ## BIT(sext ## BIT(la(a)) OP 1); \
	cpu->cc.op = CC_ ## NAME ## BIT; \
	SET_BIT(cpu->flags, cf, CF); \
	cpu->cc.mask = PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define NEG_helper(BIT, a, la, sa) \
	cpu->cc.src1 = sext ## BIT(la(a)); \
	cpu->cc.dst = sext ## BIT(-cpu->cc.src1); \
	cpu->cc.op = CC_NEG ## BIT; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define ADCb(...) ACOP_helper(ADC, ADD,  8, +, __VA_ARGS__)
#define ADCw(...) ACOP_helper(ADC, ADD, 16, +, __VA_ARGS__)
#define ADCd(...) ACOP_helper(ADC, ADD, 32, +, __VA_ARGS__)
#define SBBb(...) ACOP_helper(SBB, SUB,  8, -, __VA_ARGS__)
#define SBBw(...) ACOP_helper(SBB, SUB, 16, -, __VA_ARGS__)
#define SBBd(...) ACOP_helper(SBB, SUB, 32, -, __VA_ARGS__)
#define ADDb(...) AOP8_ADD_helper(__VA_ARGS__)
#define ADDw(...) AOP_helper(ADD, 16, +, __VA_ARGS__)
#define ADDd(...) AOP_helper(ADD, 32, +, __VA_ARGS__)
#define SUBb(...) AOP8_SUB_helper(__VA_ARGS__)
#define SUBw(...) AOP_helper(SUB, 16, -, __VA_ARGS__)
#define SUBd(...) AOP_helper(SUB, 32, -, __VA_ARGS__)
#define ORb(...)  LOP8_helper(|, __VA_ARGS__)
#define ORw(...)  LOP_helper(OR,  16, |, __VA_ARGS__)
#define ORd(...)  LOP_helper(OR,  32, |, __VA_ARGS__)
#define ANDb(...) LOP8_helper(&, __VA_ARGS__)
#define ANDw(...) LOP_helper(AND, 16, &, __VA_ARGS__)
#define ANDd(...) LOP_helper(AND, 32, &, __VA_ARGS__)
#define XORb(...) LOP8_helper(^, __VA_ARGS__)
#define XORw(...) LOP_helper(XOR, 16, ^, __VA_ARGS__)
#define XORd(...) LOP_helper(XOR, 32, ^, __VA_ARGS__)
#define CMPb(...)  AOP8_CMP_helper(__VA_ARGS__)
#define CMPw(...)  AOP0_helper(SUB, 16, -, __VA_ARGS__)
#define CMPd(...)  AOP0_helper(SUB, 32, -, __VA_ARGS__)
#define TESTb(...) LOP8_TEST_helper(&, __VA_ARGS__)
#define TESTw(...) LOP0_helper(AND, 16, &, __VA_ARGS__)
#define TESTd(...) LOP0_helper(AND, 32, &, __VA_ARGS__)
#define INCb(...) INCDEC_helper(INC,  8, +, __VA_ARGS__)
#define INCw(...) INCDEC_helper(INC, 16, +, __VA_ARGS__)
#define INCd(...) INCDEC_helper(INC, 32, +, __VA_ARGS__)
#define DECb(...) INCDEC_helper(DEC,  8, -, __VA_ARGS__)
#define DECw(...) INCDEC_helper(DEC, 16, -, __VA_ARGS__)
#define DECd(...) INCDEC_helper(DEC, 32, -, __VA_ARGS__)
#define NOTb(a, la, sa) sa(a, ~la(a));
#define NOTw(a, la, sa) sa(a, ~la(a));
#define NOTd(a, la, sa) sa(a, ~la(a));
#define NEGb(...) NEG_helper(8,  __VA_ARGS__)
#define NEGw(...) NEG_helper(16, __VA_ARGS__)
#define NEGd(...) NEG_helper(32, __VA_ARGS__)

#define SHL_helper(BIT, a, b, la, sa, lb, sb) \
	uword x = la(a); \
	uword y = (lb(b)) & 0x1f; \
	if (y) { \
		cpu->cc.dst = sext ## BIT(x << y); \
		cpu->cc.dst2 = ((x >> (BIT - y)) & 1); \
		cpu->cc.op = CC_SHL; \
		cpu->cc.mask = CF | PF | ZF | SF | OF; \
		sa(a, cpu->cc.dst); \
	}

#define SHLb(...) SHL_helper(8, __VA_ARGS__)
#define SHLw(...) SHL_helper(16, __VA_ARGS__)
#define SHLd(...) SHL_helper(32, __VA_ARGS__)

#define ROL_helper(BIT, a, b, la, sa, lb, sb) \
	uword x = la(a); \
	uword y0 = lb(b); \
	uword y = y0 & (BIT - 1); \
	uword res = x; \
	if (y) { \
		res = sext ## BIT((x << y) | (x >> (BIT - y))); \
		sa(a, res); \
	} \
	if (y0) { \
		int cf1 = res & 1; \
		int of1 = (res >> (sizeof(uword) * 8 - 1)) ^ cf1; \
		SET_BIT(cpu->flags, cf1, CF); \
		SET_BIT(cpu->flags, of1, OF); \
		cpu->cc.mask &= ~(CF | OF); \
	}

#define ROLb(...) ROL_helper(8, __VA_ARGS__)
#define ROLw(...) ROL_helper(16, __VA_ARGS__)
#define ROLd(...) ROL_helper(32, __VA_ARGS__)

#define RCL_helper(BIT, a, b, la, sa, lb, sb) \
	uword x = la(a); \
	uword y = ((lb(b)) & 0x1f) % (BIT + 1); \
	if (y) { \
		uword cf = get_CF(cpu); \
		uword res = sext ## BIT((x << y) | (cf << (y - 1)) | (y != 1 ? (x >> (BIT + 1 - y)) : 0)); \
		int cf1 = (x >> (BIT - y)) & 1; \
		int of1 = (res >> (sizeof(uword) * 8 - 1)) ^ cf1; \
		SET_BIT(cpu->flags, cf1, CF); \
		SET_BIT(cpu->flags, of1, OF); \
		cpu->cc.mask &= ~(CF | OF); \
		sa(a, res); \
	}

#define RCLb(...) RCL_helper(8, __VA_ARGS__)
#define RCLw(...) RCL_helper(16, __VA_ARGS__)
#define RCLd(...) RCL_helper(32, __VA_ARGS__)

#define RCR_helper(BIT, a, b, la, sa, lb, sb) \
	uword x = la(a); \
	uword y = ((lb(b)) & 0x1f) % (BIT + 1); \
	if (y) { \
		uword cf = get_CF(cpu); \
		uword res = sext ## BIT((x >> y) | (cf << (BIT - y)) | (y != 1 ? (x << (BIT + 1 - y)) : 0)); \
		int cf1 = (sext ## BIT(x << (BIT - y)) >> (BIT - 1)) & 1; \
		int of1 = ((res ^ (res << 1)) >> (BIT - 1)) & 1; \
		SET_BIT(cpu->flags, cf1, CF); \
		SET_BIT(cpu->flags, of1, OF); \
		cpu->cc.mask &= ~(CF | OF); \
		sa(a, res); \
	}

#define RCRb(...) RCR_helper(8, __VA_ARGS__)
#define RCRw(...) RCR_helper(16, __VA_ARGS__)
#define RCRd(...) RCR_helper(32, __VA_ARGS__)

#define ROR_helper(BIT, a, b, la, sa, lb, sb) \
	uword x = la(a); \
	uword y0 = lb(b); \
	uword y = y0 & (BIT - 1); \
	uword res = x; \
	if (y) { \
		res = sext ## BIT((x >> y) | (x << (BIT - y))); \
		sa(a, res); \
	} \
	if (y0) { \
		int cf1 = (res >> (BIT - 1)) & 1; \
		int of1 = ((res ^ (res << 1)) >> (BIT - 1)) & 1; \
		SET_BIT(cpu->flags, cf1, CF); \
		SET_BIT(cpu->flags, of1, OF); \
		cpu->cc.mask &= ~(CF | OF); \
	}

#define RORb(...) ROR_helper(8, __VA_ARGS__)
#define RORw(...) ROR_helper(16, __VA_ARGS__)
#define RORd(...) ROR_helper(32, __VA_ARGS__)

#define SHR_helper(BIT, a, b, la, sa, lb, sb) \
	uword x = la(a); \
	uword y = (lb(b)) & 0x1f; \
	if (y) { \
		cpu->cc.src1 = sext ## BIT(x); \
		cpu->cc.dst = sext ## BIT(x >> y); \
		cpu->cc.dst2 = (x >> (y - 1)) & 1; \
		cpu->cc.op = CC_SHR; \
		cpu->cc.mask = CF | PF | ZF | SF | OF; \
		sa(a, cpu->cc.dst); \
	}

#define SHRb(...) SHR_helper(8, __VA_ARGS__)
#define SHRw(...) SHR_helper(16, __VA_ARGS__)
#define SHRd(...) SHR_helper(32, __VA_ARGS__)

#define SHLD_helper(BIT, a, b, c, la, sa, lb, sb, lc, sc) \
	int count = (lc(c)) & 0x1f; \
	uword x = la(a); \
	uword y = lb(b); \
	if (count) { \
		cpu->cc.src1 = sext ## BIT(x); \
		if (BIT < count) {  /* undocumented */ \
			uword z = x; x = y; y = z; \
			count -= BIT; \
		} \
		cpu->cc.dst = sext ## BIT((x << count) | (y >> (BIT - count))); \
		if (count == 1) { \
			cpu->cc.dst2 = sext ## BIT(x); \
		} else { \
			cpu->cc.dst2 = sext ## BIT((x << (count - 1)) | (count == 1 ? 0 : (y >> (BIT - (count - 1))))); \
		} \
		cpu->cc.op = CC_SHLD; \
		cpu->cc.mask = CF | PF | ZF | SF | OF; \
		sa(a, cpu->cc.dst); \
	}

#define SHLDw(...) SHLD_helper(16, __VA_ARGS__)
#define SHLDd(...) SHLD_helper(32, __VA_ARGS__)

#define SHRD_helper(BIT, a, b, c, la, sa, lb, sb, lc, sc) \
	int count = (lc(c)) & 0x1f; \
	uword x = la(a); \
	uword y = lb(b); \
	if (count) { \
		if (BIT < count) {  /* undocumented */ \
			uword z = x; x = y; y = z; \
			count -= BIT; \
		} \
		cpu->cc.src1 = sext ## BIT(x); \
		cpu->cc.dst = sext ## BIT((x >> count) | (y << (BIT - count))); \
		if (count == 1) { \
			cpu->cc.dst2 = sext ## BIT(x); \
		} else { \
			cpu->cc.dst2 = sext ## BIT((x >> (count - 1)) | (y << (BIT - (count - 1)))); \
		} \
		cpu->cc.op = CC_SHRD; \
		cpu->cc.mask = CF | PF | ZF | SF | OF; \
		sa(a, cpu->cc.dst); \
	}

#define SHRDw(...) SHRD_helper(16, __VA_ARGS__)
#define SHRDd(...) SHRD_helper(32, __VA_ARGS__)

// ">>"
#define SAR_helper(BIT, a, b, la, sa, lb, sb) \
	sword x = sext ## BIT(la(a)); \
	sword y = (lb(b)) & 0x1f; \
	if (y) { \
		cpu->cc.dst = x >> y; \
		cpu->cc.dst2 = (x >> (y - 1)) & 1; \
		cpu->cc.op = CC_SAR; \
		cpu->cc.mask = CF | PF | ZF | SF | OF; \
		sa(a, cpu->cc.dst); \
	}

#define SARb(...) SAR_helper(8, __VA_ARGS__)
#define SARw(...) SAR_helper(16, __VA_ARGS__)
#define SARd(...) SAR_helper(32, __VA_ARGS__)

#define IMUL2w(a, b, la, sa, lb, sb) \
	cpu->cc.src1 = sext16(la(a)); \
	cpu->cc.src2 = sext16(lb(b)); \
	cpu->cc.dst = cpu->cc.src1 * cpu->cc.src2; \
	cpu->cc.op = CC_IMUL16; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define IMUL2d(a, b, la, sa, lb, sb) \
	cpu->cc.src1 = sext32(la(a)); \
	cpu->cc.src2 = sext32(lb(b)); \
	int64_t res = (int64_t) (s32) cpu->cc.src1 * (int64_t) (s32) cpu->cc.src2; \
	cpu->cc.dst = res; \
	cpu->cc.dst2 = res >> 32; \
	cpu->cc.op = CC_IMUL32; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define IMUL2wI_helper(BIT, BITI, a, b, c, la, sa, lb, sb) \
	cpu->cc.src1 = sext ## BIT(lb(b)); \
	cpu->cc.src2 = sext ## BITI(c); \
	cpu->cc.dst = cpu->cc.src1 * cpu->cc.src2; \
	cpu->cc.op = CC_IMUL ## BIT; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define IMUL2dI_helper(BIT, BITI, a, b, c, la, sa, lb, sb) \
	cpu->cc.src1 = sext ## BIT(lb(b)); \
	cpu->cc.src2 = sext ## BITI(c); \
	int64_t res = (int64_t) (s32) cpu->cc.src1 * (int64_t) (s32) cpu->cc.src2; \
	cpu->cc.dst = res; \
	cpu->cc.dst2 = res >> 32; \
	cpu->cc.op = CC_IMUL ## BIT; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sa(a, cpu->cc.dst);

#define IMUL2wIb(...) IMUL2wI_helper(16, 8, __VA_ARGS__)
#define IMUL2wIw(...) IMUL2wI_helper(16, 16, __VA_ARGS__)
#define IMUL2dIb(...) IMUL2dI_helper(32, 8, __VA_ARGS__)
#define IMUL2dId(...) IMUL2dI_helper(32, 32, __VA_ARGS__)

#define IMULb(a, la, sa) \
	cpu->cc.src1 = sext8(lreg8(0)); \
	cpu->cc.src2 = sext8(la(a)); \
	cpu->cc.dst = cpu->cc.src1 * cpu->cc.src2; \
	cpu->cc.op = CC_IMUL8; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sreg16(0, cpu->cc.dst);

#define IMULw(a, la, sa) \
	cpu->cc.src1 = sext16(lreg16(0)); \
	cpu->cc.src2 = sext16(la(a)); \
	cpu->cc.dst = cpu->cc.src1 * cpu->cc.src2; \
	cpu->cc.op = CC_IMUL16; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sreg16(0, cpu->cc.dst); \
	sreg16(2, (cpu->cc.dst >> 16));

#define IMULd(a, la, sa) \
	cpu->cc.src1 = sext32(lreg32(0)); \
	cpu->cc.src2 = sext32(la(a)); \
	int64_t res = (int64_t) (s32) cpu->cc.src1 * (int64_t) (s32) cpu->cc.src2; \
	cpu->cc.dst = res; \
	cpu->cc.dst2 = res >> 32; \
	cpu->cc.op = CC_IMUL32; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sreg32(0, cpu->cc.dst); \
	sreg32(2, cpu->cc.dst2);

#define MULb(a, la, sa) \
	cpu->cc.src1 = lreg8(0); \
	cpu->cc.src2 = la(a); \
	cpu->cc.dst = sext16(cpu->cc.src1 * cpu->cc.src2); \
	cpu->cc.op = CC_MUL8; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sreg16(0, cpu->cc.dst);

#define MULw(a, la, sa) \
	cpu->cc.src1 = lreg16(0); \
	cpu->cc.src2 = la(a); \
	cpu->cc.dst = cpu->cc.src1 * cpu->cc.src2; \
	cpu->cc.op = CC_MUL16; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sreg16(0, cpu->cc.dst); \
	sreg16(2, (cpu->cc.dst >> 16));

#define MULd(a, la, sa) \
	cpu->cc.src1 = lreg32(0); \
	cpu->cc.src2 = la(a); \
	uint64_t res = (uint64_t) cpu->cc.src1 * (uint64_t) cpu->cc.src2; \
	cpu->cc.dst = res; \
	cpu->cc.dst2 = res >> 32; \
	cpu->cc.op = CC_MUL32; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sreg32(0, cpu->cc.dst); \
	sreg32(2, cpu->cc.dst2);

#define IDIVb(a, la, sa) \
	sword src1 = sext16(lreg16(0)); \
	sword src2 = sext8(la(a)); \
	if (src2 == 0) THROW0(EX_DE); \
	sword res = src1 / src2; \
	if (res > 127 || res < -128) THROW0(EX_DE); \
	sreg8(0, res); \
	sreg8(4, src1 % src2);

#define IDIVw(a, la, sa) \
	sword src1 = sext32(lreg16(0) | (lreg16(2)<< 16)); \
	sword src2 = sext16(la(a)); \
	if (src2 == 0) THROW0(EX_DE); \
	sword res = src1 / src2; \
	if (res > 32767 || res < -32768) THROW0(EX_DE); \
	sreg16(0, res); \
	sreg16(2, src1 % src2);

#define IDIVd(a, la, sa) \
	int64_t src1 = (((uint64_t) lreg32(2)) << 32) | lreg32(0); \
	int64_t src2 = (sword) (la(a));	\
	if (src2 == 0) THROW0(EX_DE); \
	int64_t res = src1 / src2; \
	if (res > 2147483647 || res < -2147483648) THROW0(EX_DE); \
	sreg32(0, res); \
	sreg32(2, src1 % src2);

#define DIVb(a, la, sa) \
	uword src1 = lreg16(0); \
	uword src2 = la(a); \
	if (src2 == 0) THROW0(EX_DE); \
	uword res = src1 / src2; \
	if (res > 0xff) THROW0(EX_DE); \
	/* bypass the Cyrix 5/2 test */ \
	if (src1 == 0x5 && src2 == 0x2) { cpu->cc.mask &= ~ZF; cpu->flags |= ZF; } \
	sreg8(0, res); \
	sreg8(4, src1 % src2);

#define DIVw(a, la, sa) \
	uword src1 = lreg16(0) | (lreg16(2)<< 16); \
	uword src2 = la(a); \
	if (src2 == 0) THROW0(EX_DE); \
	uword res = src1 / src2; \
	if (res > 0xffff) THROW0(EX_DE); \
	/* bypass the NexGen 0x5555/2 test */ \
	if (src1 == 0x5555 && src2 == 0x2) { cpu->cc.mask &= ~ZF; cpu->flags &= ~ZF; } \
	sreg16(0, res); \
	sreg16(2, src1 % src2);

#define DIVd(a, la, sa) \
	uint64_t src1 = (((uint64_t) lreg32(2)) << 32) | lreg32(0); \
	uint64_t src2 = la(a); \
	if (src2 == 0) THROW0(EX_DE); \
	uint64_t res = src1 / src2; \
	if (res > 0xffffffff) THROW0(EX_DE); \
	sreg32(0, res); \
	sreg32(2, src1 % src2);

#define BT_helper(BIT, a, b, la, sa, lb, sb) \
	int bb = lb(b) % BIT; \
	bool bit = (la(a) >> bb) & 1; \
	cpu->cc.mask &= ~CF; \
	SET_BIT(cpu->flags, bit, CF);

#define BTw(...) BT_helper(16, __VA_ARGS__)
#define BTd(...) BT_helper(32, __VA_ARGS__)

#define BTX_helper(BIT, OP, a, b, la, sa, lb, sb) \
	int bb = lb(b) % BIT; \
	bool bit = (la(a) >> bb) & 1; \
	sa(a, la(a) OP (1 << bb)); \
	cpu->cc.mask &= ~CF; \
	SET_BIT(cpu->flags, bit, CF);

#define BTSw(...) BTX_helper(16, |, __VA_ARGS__)
#define BTSd(...) BTX_helper(32, |, __VA_ARGS__)
#define BTRw(...) BTX_helper(16, & ~, __VA_ARGS__)
#define BTRd(...) BTX_helper(32, & ~, __VA_ARGS__)
#define BTCw(...) BTX_helper(16, ^, __VA_ARGS__)
#define BTCd(...) BTX_helper(32, ^, __VA_ARGS__)

#define BSF_helper(BIT, a, b, la, sa, lb, sb) \
	u ## BIT src = lb(b); \
	u ## BIT temp = 0; \
	cpu->cc.mask = 0; \
	if (src == 0) { \
		cpu->flags |= ZF; \
	} else { \
		cpu->flags &= ~ZF; \
		while ((src & 1) == 0) { \
			temp++; \
			src >>= 1; \
		} \
		sa(a, temp); \
	}

#define BSFw(...) BSF_helper(16, __VA_ARGS__)
#define BSFd(...) BSF_helper(32, __VA_ARGS__)

#define BSR_helper(BIT, a, b, la, sa, lb, sb) \
	s ## BIT src = lb(b); \
	u ## BIT temp = BIT - 1; \
	cpu->cc.mask = 0; \
	if (src == 0) { \
		cpu->flags |= ZF; \
	} else { \
		cpu->flags &= ~ZF; \
		while (src >= 0) { \
			temp--; \
			src <<= 1; \
		} \
		sa(a, temp); \
	}

#define BSRw(...) BSR_helper(16, __VA_ARGS__)
#define BSRd(...) BSR_helper(32, __VA_ARGS__)

#define MOVb(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVw(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVd(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVSeg(a, b, la, sa, lb, sb) \
	if (a == SEG_CS) THROW0(EX_UD); \
	if (a == SEG_SS) stepcount++; \
	TRY(set_seg(cpu, a, lb(b)));
#define MOVZXdb(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVZXwb(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVZXww(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVZXdw(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVSXdb(a, b, la, sa, lb, sb) sa(a, sext8(lb(b)));
#define MOVSXwb(a, b, la, sa, lb, sb) sa(a, sext8(lb(b)));
#define MOVSXww(a, b, la, sa, lb, sb) sa(a, lb(b));
#define MOVSXdw(a, b, la, sa, lb, sb) sa(a, sext16(lb(b)));

#define XCHG(a, b, la, sa, lb, sb) \
	uword tmp = lb(b); \
	sb(b, la(a)); \
	sa(a, tmp);
#define XCHGb(...) XCHG(__VA_ARGS__)
#define XCHGw(...) XCHG(__VA_ARGS__)
#define XCHGd(...) XCHG(__VA_ARGS__)

#define XCHGAX() \
	if (opsz16) { \
		int reg = b1 & 7; \
		uword tmp = lreg16(reg); \
		sreg16(reg, lreg16(0)); \
		sreg16(0, tmp); \
	} else { \
		int reg = b1 & 7; \
		uword tmp = lreg32(reg); \
		sreg32(reg, lreg32(0)); \
		sreg32(0, tmp); \
	}

#define LEAd(a, b, la, sa, lb, sb) \
	if (mod == 3) THROW0(EX_UD); \
	sa(a, lb(b));
#define LEAw LEAd

#define CBW_CWDE() \
	if (opsz16) sreg16(0, sext8(lreg8(0))); \
	else sreg32(0, sext16(lreg16(0)));

#define CWD_CDQ() \
	if (opsz16) sreg16(2, sext16(-(sext16(lreg16(0)) >> 31))); \
	else sreg32(2, sext32(-(sext32(lreg32(0)) >> 31)));

#define MOVFC() \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int rm = modrm & 7; \
	if (reg == 0) { \
		sreg32(rm, cpu->cr0); \
	} else if (reg == 2) { \
		sreg32(rm, cpu->cr2); \
	} else if (reg == 3) { \
		sreg32(rm, cpu->cr3); \
	} else if (reg == 4) { \
		/* CR4 exists on Pentium+ (gen >= 5), not 486. */ \
		if (cpu->gen < 5) THROW0(EX_UD); \
		sreg32(rm, cpu->cr4); \
	} else THROW0(EX_UD);

#define MOVTC() \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int rm = modrm & 7; \
	if (reg == 0) { \
		u32 old_cr0 = cpu->cr0; \
		u32 new_cr0 = lreg32(rm); \
		if ((new_cr0 ^ cpu->cr0) & (CR0_PG | CR0_WP | 1)) { \
			tlb_clear(cpu); \
			SEQ_INVALIDATE(cpu); \
			if ((new_cr0 ^ cpu->cr0) & 1) { \
				cpu->int8_cache_valid = false; \
				cpu->int8_warmup_counter = 0; \
			} \
		} \
		if (cpu->fpu) new_cr0 |= 0x10; \
		if (unlikely(cpu_diag_enabled) && new_cr0 != old_cr0) \
			cpu_diag_log_cr0_transition(cpu, old_cr0, new_cr0); \
		cpu->cr0 = new_cr0; \
	} else if (reg == 2) { \
		cpu->cr2 = lreg32(rm); \
	} else if (reg == 3) { \
		cpu->cr3 = lreg32(rm); \
		tlb_flush_generation(cpu); \
		SEQ_INVALIDATE(cpu); \
		if (unlikely(cpu_diag_enabled && pf_diag.active)) pf_diag.cr3_write_count++; \
	} else if (reg == 4) { \
		/* CR4 exists on Pentium+ (gen >= 5), not 486. */ \
		if (cpu->gen < 5) THROW0(EX_UD); \
		u32 new_cr4 = lreg32(rm) & CR4_PSE; \
		if ((new_cr4 ^ cpu->cr4) & CR4_PSE) { \
			tlb_clear(cpu); \
			SEQ_INVALIDATE(cpu); \
		} \
		cpu->cr4 = new_cr4; \
	} else THROW0(EX_UD);

#define INT3() \
	cpu->ip = cpu->next_ip; \
	THROW0(EX_BP);

#define INTO() \
	if (get_OF(cpu)) { \
		cpu->ip = cpu->next_ip; \
		THROW0(EX_OF); \
	}

static bool call_isr(CPUI386 *cpu, int no, bool pusherr, int ext);
static bool call_isr_vm86_softint(CPUI386 *cpu, int no);

/* BIOS High-Level Emulation: intercept INT 15h calls that use problematic
 * GCC -m16 encodings.  SeaBIOS compiles handle_1587() with -m16, producing
 * 67h-prefixed LGDT that Win386/Win95 VMM can't decode in V86 mode.
 * Also faster than letting the BIOS do PM transitions for block copies. */
static bool bios_hle_int(CPUI386 *cpu, int intno)
{
	cpu->hle_call_count++;
	if (intno == 0x11) {
		/* INT 11h: Equipment list from BDA (40:10).
		 * Normalize noisy high bits for Win3.x compatibility probes. */
		u16 val = 0;
		if (0x411u < (u32)cpu->phys_mem_size)
			val = cpu->phys_mem[0x410] | ((u16)cpu->phys_mem[0x411] << 8);
		val &= 0x0fff;
		if ((val & 0x0030) == 0)
			val = (u16)((val & ~0x0030) | 0x0020);
		sreg16(0, val);
		return true;
	}

	if (intno == 0x12) {
		/* INT 12h: Conventional memory size in KB from BDA (40:13). */
		u16 val = 0;
		if (0x414u < (u32)cpu->phys_mem_size)
			val = cpu->phys_mem[0x413] | ((u16)cpu->phys_mem[0x414] << 8);
		sreg16(0, val);
		return true;
	}

	/* BDA pointer for convenience — all HLE reads/writes use physical memory */
	u8 *bda = (u8 *)cpu->phys_mem + 0x400;

	/* In V86 mode with IOPL<3, the VMM intercepts INTs via #GP.
	 * Don't HLE INT 10h/16h/1Ah there — let the VMM handle them.
	 * V86 with IOPL=3 (EMM386-style) is safe — INTs go through IVT normally. */
	bool v86_vmm = (cpu->flags & VM) && get_IOPL(cpu) < 3;

	/* ---- INT 16h: Keyboard ---- */
	if (intno == 0x16 && !v86_vmm) {
		u8 ah16 = (lreg32(0) >> 8) & 0xFF;
		u16 buf_head = bda[0x1A] | (bda[0x1B] << 8); /* offset from seg 0x40 */
		u16 buf_tail = bda[0x1C] | (bda[0x1D] << 8);
		u16 buf_start = bda[0x80] | (bda[0x81] << 8);
		u16 buf_end = bda[0x82] | (bda[0x83] << 8);
		if (buf_start == 0) buf_start = 0x1E;
		if (buf_end == 0) buf_end = 0x3E;

		if (ah16 == 0x01 || ah16 == 0x11) {
			/* Check keystroke (non-blocking).
			 * ZF=1 if empty, ZF=0 + AX=key if available. */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			if (buf_head == buf_tail) {
				cpu->flags |= ZF;
			} else {
				cpu->flags &= ~ZF;
				u16 key = bda[buf_head] | (bda[buf_head + 1] << 8);
				sreg16(0, key);
			}
			return true;
		}

		if (ah16 == 0x00 || ah16 == 0x10) {
			/* Wait for keystroke (blocking).
			 * If buffer empty, fall through to BIOS (which does HLT loop). */
			if (buf_head == buf_tail)
				return false;
			u16 key = bda[buf_head] | (bda[buf_head + 1] << 8);
			sreg16(0, key);
			/* Advance head with wrap */
			buf_head += 2;
			if (buf_head >= buf_end)
				buf_head = buf_start;
			bda[0x1A] = buf_head & 0xFF;
			bda[0x1B] = buf_head >> 8;
			return true;
		}

		if (ah16 == 0x02) {
			/* Get shift flags → AL = BDA[0x417] */
			sreg8(0, bda[0x17]);
			return true;
		}

		return false;
	}

	/* ---- INT 1Ah: Timer ---- */
	if (intno == 0x1A && !v86_vmm) {
		u8 ah1a = (lreg32(0) >> 8) & 0xFF;
		if (ah1a == 0x00) {
			/* Read system timer tick count from BDA 0x46C (32-bit) */
			u32 ticks = bda[0x6C] | (bda[0x6D] << 8) |
			            (bda[0x6E] << 16) | (bda[0x6F] << 24);
			u8 midnight = bda[0x70];
			bda[0x70] = 0; /* clear midnight flag */
			sreg16(1, ticks >> 16);   /* CX = high word */
			sreg16(2, ticks & 0xFFFF); /* DX = low word */
			sreg8(0, midnight);       /* AL = midnight flag */
			return true;
		}
		return false;
	}

	/* ---- INT 10h: Video ---- */
	if (intno == 0x10 && !v86_vmm) {
		/* Don't HLE INT 10h before BIOS has initialized the BDA.
		 * After pc_reset(), RAM is zeroed — BDA mode=0/cols=0 means
		 * the BIOS hasn't done AH=00h mode set yet. Let the real
		 * BIOS POST run unimpeded to initialize everything. */
		if (bda[0x49] == 0 && bda[0x4A] == 0)
			return false;

		u8 ah10 = (lreg32(0) >> 8) & 0xFF;
		u8 al10 = lreg32(0) & 0xFF;

		if (ah10 == 0x0F) {
			/* Get video mode: AL=mode, AH=cols, BH=page */
			u8 mode = bda[0x49];
			u8 cols = bda[0x4A];
			u8 page = bda[0x62];
			sreg16(0, mode | ((u16)cols << 8)); /* AL=mode, AH=cols */
			sreg16(3, (lreg16(3) & 0x00FF) | ((u16)page << 8)); /* BH=page */
			return true;
		}

		if (ah10 == 0x03) {
			/* Get cursor position: DH=row, DL=col, CH:CL=cursor shape */
			u8 page = (lreg16(3) >> 8) & 0x07;
			u16 pos_off = 0x50 + page * 2;
			u8 col = bda[pos_off];
			u8 row = bda[pos_off + 1];
			u16 shape = bda[0x60] | (bda[0x61] << 8);
			sreg16(2, col | ((u16)row << 8)); /* DL=col, DH=row */
			sreg16(1, shape);                 /* CX=cursor shape */
			return true;
		}

		if (ah10 == 0x02) {
			/* Set cursor position: DH=row, DL=col, BH=page */
			u8 page = (lreg16(3) >> 8) & 0x07;
			u8 col = lreg16(2) & 0xFF;
			u8 row = (lreg16(2) >> 8) & 0xFF;
			u8 cols = bda[0x4A];
			if (cols == 0) cols = 80;
			u16 pos_off = 0x50 + page * 2;
			bda[pos_off] = col;
			bda[pos_off + 1] = row;
			/* Update CRTC cursor if this is the active page */
			u8 active_page = bda[0x62];
			if (page == active_page && cpu->cb.vga_set_cursor) {
				u16 page_off = bda[0x4E] | (bda[0x4F] << 8);
				u16 cursor_addr = page_off / 2 + (u16)row * cols + col;
				cpu->cb.vga_set_cursor(cpu->cb.vga_state, cursor_addr);
			}
			return true;
		}

		/* AH=01h (Set Cursor Shape) intentionally NOT HLE'd.
		 * The BIOS performs CGA-to-VGA scan line conversion that
		 * we'd need to replicate. Not worth it for a rare call. */

		/* AH=06h/07h: Scroll window up/down — biggest single HLE win */
		if ((ah10 == 0x06 || ah10 == 0x07) && cpu->cb.vga_text_ram) {
			u8 lines = al10;
			u8 attr = (lreg16(3) >> 8) & 0xFF; /* BH = fill attribute */
			u8 top = (lreg16(1) >> 8) & 0xFF;  /* CH = top row */
			u8 left = lreg16(1) & 0xFF;         /* CL = left col */
			u8 bottom = (lreg16(2) >> 8) & 0xFF; /* DH = bottom row */
			u8 right = lreg16(2) & 0xFF;          /* DL = right col */
			u8 cols = bda[0x4A];
			if (cols == 0) cols = 80;
			u8 *vram = cpu->cb.vga_text_ram;
			u16 page_off = bda[0x4E] | (bda[0x4F] << 8);
			/* page_off is byte offset in CPU address space,
			 * VRAM uses odd/even interleave: multiply by 2 */
			u8 *base = vram + (u32)page_off * 2;
			int row_stride = cols * 4; /* 4 bytes per char cell in VRAM */
			int win_height = bottom - top + 1;
			int win_width = right - left + 1;
			if (win_height <= 0 || win_width <= 0)
				return true;
			bool full_width = (left == 0 && right == cols - 1);

			/* Build fill pattern: char=0x20 (space), attr=BH */
			/* VRAM layout: [char_byte, attr_byte, ?, ?] per cell */
			/* For 16-bit fill: char | (attr << 8) */
			u16 fill16 = 0x20 | ((u16)attr << 8);

			if (lines == 0 || lines >= win_height) {
				/* Clear entire window */
				if (full_width) {
					u8 *p = base + top * row_stride;
					for (int r = 0; r < win_height; r++) {
						for (int c = 0; c < cols; c++)
							*(u16 *)(p + c * 4) = fill16;
						p += row_stride;
					}
				} else {
					for (int r = top; r <= bottom; r++) {
						u8 *p = base + r * row_stride + left * 4;
						for (int c = 0; c < win_width; c++)
							*(u16 *)(p + c * 4) = fill16;
					}
				}
			} else if (ah10 == 0x06) {
				/* Scroll up */
				if (full_width) {
					memmove(base + top * row_stride,
					        base + (top + lines) * row_stride,
					        (win_height - lines) * row_stride);
					/* Fill blank lines at bottom */
					u8 *p = base + (bottom - lines + 1) * row_stride;
					for (int r = 0; r < lines; r++) {
						for (int c = 0; c < cols; c++)
							*(u16 *)(p + c * 4) = fill16;
						p += row_stride;
					}
				} else {
					for (int r = top; r <= bottom - lines; r++) {
						u8 *src = base + (r + lines) * row_stride + left * 4;
						u8 *dst = base + r * row_stride + left * 4;
						memcpy(dst, src, win_width * 4);
					}
					for (int r = bottom - lines + 1; r <= bottom; r++) {
						u8 *p = base + r * row_stride + left * 4;
						for (int c = 0; c < win_width; c++)
							*(u16 *)(p + c * 4) = fill16;
					}
				}
			} else {
				/* Scroll down (AH=07h) */
				if (full_width) {
					memmove(base + (top + lines) * row_stride,
					        base + top * row_stride,
					        (win_height - lines) * row_stride);
					/* Fill blank lines at top */
					u8 *p = base + top * row_stride;
					for (int r = 0; r < lines; r++) {
						for (int c = 0; c < cols; c++)
							*(u16 *)(p + c * 4) = fill16;
						p += row_stride;
					}
				} else {
					for (int r = bottom; r >= top + lines; r--) {
						u8 *src = base + (r - lines) * row_stride + left * 4;
						u8 *dst = base + r * row_stride + left * 4;
						memcpy(dst, src, win_width * 4);
					}
					for (int r = top; r < top + lines; r++) {
						u8 *p = base + r * row_stride + left * 4;
						for (int c = 0; c < win_width; c++)
							*(u16 *)(p + c * 4) = fill16;
					}
				}
			}
			/* Mark all VRAM dirty */
			if (cpu->cb.vga_text_dirty_pages)
				*cpu->cb.vga_text_dirty_pages = ~(uint64_t)0;
			return true;
		}

		if (ah10 == 0x08 && cpu->cb.vga_text_ram) {
			/* Read char+attr at cursor → AH=attr, AL=char */
			u8 page = (lreg16(3) >> 8) & 0x07;
			u8 cols = bda[0x4A];
			if (cols == 0) cols = 80;
			u16 pos_off = 0x50 + page * 2;
			u8 col = bda[pos_off];
			u8 row = bda[pos_off + 1];
			u16 page_size = bda[0x4C] | (bda[0x4D] << 8);
			if (page_size == 0) page_size = 4000;
			u8 *vram = cpu->cb.vga_text_ram;
			u32 voff = (u32)page * page_size * 2 +
			           ((u32)row * cols + col) * 4;
			u8 ch = vram[voff];
			u8 at = vram[voff + 1];
			sreg16(0, ch | ((u16)at << 8));
			return true;
		}

		if (ah10 == 0x09 && cpu->cb.vga_text_ram) {
			/* Write char+attr at cursor × CX copies (no cursor advance) */
			u8 page = (lreg16(3) >> 8) & 0x07;
			u8 attr = lreg16(3) & 0xFF; /* BL = attribute */
			u16 count = lreg16(1); /* CX = repeat count */
			u8 ch = al10;
			u8 cols = bda[0x4A];
			if (cols == 0) cols = 80;
			u16 pos_off = 0x50 + page * 2;
			u8 col = bda[pos_off];
			u8 row = bda[pos_off + 1];
			u16 page_size = bda[0x4C] | (bda[0x4D] << 8);
			if (page_size == 0) page_size = 4000;
			u8 *vram = cpu->cb.vga_text_ram;
			u32 voff = (u32)page * page_size * 2 +
			           ((u32)row * cols + col) * 4;
			u16 fill = ch | ((u16)attr << 8);
			for (u16 i = 0; i < count; i++) {
				*(u16 *)(vram + voff) = fill;
				voff += 4;
			}
			if (cpu->cb.vga_text_dirty_pages)
				*cpu->cb.vga_text_dirty_pages = ~(uint64_t)0;
			return true;
		}

		if (ah10 == 0x0A && cpu->cb.vga_text_ram) {
			/* Write char at cursor × CX copies (preserve attr, no advance) */
			u8 page = (lreg16(3) >> 8) & 0x07;
			u16 count = lreg16(1); /* CX */
			u8 ch = al10;
			u8 cols = bda[0x4A];
			if (cols == 0) cols = 80;
			u16 pos_off = 0x50 + page * 2;
			u8 col = bda[pos_off];
			u8 row = bda[pos_off + 1];
			u16 page_size = bda[0x4C] | (bda[0x4D] << 8);
			if (page_size == 0) page_size = 4000;
			u8 *vram = cpu->cb.vga_text_ram;
			u32 voff = (u32)page * page_size * 2 +
			           ((u32)row * cols + col) * 4;
			for (u16 i = 0; i < count; i++) {
				vram[voff] = ch; /* char byte only */
				voff += 4;
			}
			if (cpu->cb.vga_text_dirty_pages)
				*cpu->cb.vga_text_dirty_pages = ~(uint64_t)0;
			return true;
		}

		if (ah10 == 0x0E && cpu->cb.vga_text_ram) {
			/* Teletype output: write char + advance cursor + handle specials */
			u8 ch = al10;
			u8 page = bda[0x62]; /* active page */
			u8 cols = bda[0x4A];
			if (cols == 0) cols = 80;
			u8 rows = bda[0x84];
			if (rows == 0) rows = 24; /* BDA 0x84 stores max row (0-based) */
			u16 pos_off = 0x50 + page * 2;
			u8 col = bda[pos_off];
			u8 row = bda[pos_off + 1];
			u16 page_byte_off = bda[0x4E] | (bda[0x4F] << 8);
			u8 *vram = cpu->cb.vga_text_ram;

			if (ch == 0x0D) {
				/* CR: move to beginning of line */
				col = 0;
			} else if (ch == 0x0A) {
				/* LF: move down one line */
				row++;
			} else if (ch == 0x08) {
				/* BS: move back one column */
				if (col > 0) col--;
			} else if (ch == 0x07) {
				/* BEL: beep (no-op) */
			} else {
				/* Normal character: write and advance */
				u32 voff = (u32)page_byte_off * 2 +
				           ((u32)row * cols + col) * 4;
				vram[voff] = ch;
				/* Attribute is preserved (teletype doesn't change it) */
				col++;
				if (col >= cols) {
					col = 0;
					row++;
				}
			}

			/* Scroll if past bottom row */
			if (row > rows) {
				/* Scroll up 1 line, full width */
				int row_stride = cols * 4;
				u8 *base = vram + (u32)page_byte_off * 2;
				memmove(base, base + row_stride,
				        rows * row_stride);
				/* Clear last line with attr 0x07 (light gray on black) */
				u8 *p = base + rows * row_stride;
				u16 fill16 = 0x20 | (0x07 << 8);
				for (int c = 0; c < cols; c++)
					*(u16 *)(p + c * 4) = fill16;
				row = rows;
			}

			/* Update BDA cursor */
			bda[pos_off] = col;
			bda[pos_off + 1] = row;
			/* Update CRTC cursor */
			if (cpu->cb.vga_set_cursor) {
				u16 cursor_addr = page_byte_off / 2 +
				                  (u16)row * cols + col;
				cpu->cb.vga_set_cursor(cpu->cb.vga_state, cursor_addr);
			}
			if (cpu->cb.vga_text_dirty_pages)
				*cpu->cb.vga_text_dirty_pages = ~(uint64_t)0;
			return true;
		}

		return false;
	}

	/* ---- INT 13h: Disk ---- */
	if (intno == 0x13 && !v86_vmm) {
		u8 ah13 = (lreg32(0) >> 8) & 0xFF;
		u8 dl = lreg32(2) & 0xFF; /* drive number */

		/* Only handle hard drives (DL >= 0x80). Floppies use emulink. */
		if (dl < 0x80)
			return false;

		/* Map DL to IDE controller + drive:
		 * 0x80 = ide->drives[0], 0x81 = ide->drives[1],
		 * 0x82 = ide2->drives[0], 0x83 = ide2->drives[1] */
		PC *pc = (PC *)cpu->cb.io;
		int drv_idx = dl - 0x80;
		IDEIFState *ide_if;
		int ide_drive;
		if (drv_idx < 2) {
			ide_if = pc->ide;
			ide_drive = drv_idx;
		} else if (drv_idx < 4) {
			ide_if = pc->ide2;
			ide_drive = drv_idx - 2;
		} else {
			return false;
		}

		BlockDevice *bs = ide_get_drive_bs(ide_if, ide_drive);
		/* Don't HLE CD-ROMs — they have different sector sizes */
		if (!bs || ide_is_drive_cd(ide_if, ide_drive))
			return false;

		int heads = ide_get_drive_heads(ide_if, ide_drive);
		int spt = ide_get_drive_sectors(ide_if, ide_drive);
		int cyls = ide_get_drive_cylinders(ide_if, ide_drive);
		int64_t total_sectors = ide_get_drive_nb_sectors(ide_if, ide_drive);

		/* Count hard drives present for AH=08h/15h */
		int drive_count = 0;
		for (int i = 0; i < 2; i++) {
			if (ide_get_drive_bs(pc->ide, i) &&
			    !ide_is_drive_cd(pc->ide, i))
				drive_count++;
		}
		for (int i = 0; i < 2; i++) {
			if (ide_get_drive_bs(pc->ide2, i) &&
			    !ide_is_drive_cd(pc->ide2, i))
				drive_count++;
		}

		if (ah13 == 0x00) {
			/* Reset disk — noop for HLE */
			sreg16(0, lreg16(0) & 0x00FF); /* AH = 0 */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		if (ah13 == 0x02 || ah13 == 0x03) {
			/* AH=02h: Read sectors (CHS), AH=03h: Write sectors (CHS)
			 * AL=count, CH=cyl_lo, CL=sec(0-5)|cyl_hi(6-7), DH=head
			 * ES:BX = buffer */
			u8 count = lreg32(0) & 0xFF;
			u16 cx = lreg16(1);
			u8 dh = (lreg32(2) >> 8) & 0xFF;
			int cyl = ((cx >> 8) & 0xFF) | ((cx & 0xC0) << 2);
			int sec = cx & 0x3F;
			int head = dh;

			if (count == 0 || sec == 0 || spt == 0 || heads == 0) {
				/* Invalid params — set error */
				sreg16(0, (lreg16(0) & 0x00FF) | 0x0100); /* AH=1 */
				refresh_flags(cpu);
				cpu->cc.mask = 0;
				cpu->flags |= CF;
				return true;
			}

			/* Limit to 128 sectors per call (64KB) */
			if (count > 128) count = 128;

			uint64_t lba = ((uint64_t)cyl * heads + head) * spt + (sec - 1);
			u32 buf_phys = (cpu->seg[SEG_ES].base + (lreg16(3) & 0xFFFF)) & cpu->a20_mask;

			/* Bounds check */
			if ((uint64_t)buf_phys + (uint64_t)count * 512 > (uint64_t)cpu->phys_mem_size ||
			    lba + count > (uint64_t)total_sectors) {
				sreg16(0, (lreg16(0) & 0x00FF) | 0x0400); /* AH=4, sector not found */
				refresh_flags(cpu);
				cpu->cc.mask = 0;
				cpu->flags |= CF;
				return true;
			}

			int ret;
			if (ah13 == 0x02)
				ret = ide_block_read(bs, lba, (u8 *)&cpu->phys_mem[buf_phys], count);
			else
				ret = ide_block_write(bs, lba, (const u8 *)&cpu->phys_mem[buf_phys], count);

			if (ret < 0) {
				sreg16(0, (lreg16(0) & 0x00FF) | 0x0400); /* AH=4 */
				refresh_flags(cpu);
				cpu->cc.mask = 0;
				cpu->flags |= CF;
				return true;
			}

			sreg16(0, count); /* AH=0, AL=sectors transferred */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		if (ah13 == 0x08) {
			/* Get drive parameters.
			 * Returns: CH=max cyl low 8, CL=max sec | (max cyl hi << 6),
			 *          DH=max head, DL=drive count, AH=0, CF=0 */
			int max_cyl = cyls - 1;
			int max_head = heads - 1;
			int max_sec = spt;
			sreg16(0, lreg16(0) & 0x00FF); /* AH=0 */
			sreg16(1, (max_cyl & 0xFF) << 8 | (max_sec & 0x3F) | ((max_cyl >> 2) & 0xC0));
			sreg16(2, (drive_count & 0xFF) | ((max_head & 0xFF) << 8)); /* DL=drive count, DH=max_head */
			sreg16(3, (lreg16(3) & 0xFF00)); /* BL=0 (floppy type, N/A for HD) */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		if (ah13 == 0x15) {
			/* Get disk type. Returns AH=3 (hard disk) for fixed disks. */
			/* CX:DX = total sectors */
			u32 ts = (total_sectors > 0xFFFFFFFF) ? 0xFFFFFFFF : (u32)total_sectors;
			sreg16(0, (lreg16(0) & 0x00FF) | 0x0300); /* AH=3 */
			sreg16(1, (ts >> 16) & 0xFFFF); /* CX = high word */
			sreg16(2, ts & 0xFFFF);         /* DX = low word */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		if (ah13 == 0x41) {
			/* Check extensions present.
			 * BX=0x55AA on entry → BX=0xAA55, AH=version, CX=API bitmap */
			if (lreg16(3) != 0x55AA)
				return false;
			sreg16(3, 0xAA55); /* BX = signature */
			sreg16(0, (lreg16(0) & 0x00FF) | 0x2100); /* AH = 0x21 (version 2.1) */
			sreg16(1, 0x0001); /* CX = API subset: extended disk access */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		if (ah13 == 0x42 || ah13 == 0x43) {
			/* Extended read/write. DAP at DS:SI:
			 * [0] u8 size, [1] u8 0, [2] u16 count,
			 * [4] u16 buf_off, [6] u16 buf_seg, [8] u64 lba */
			u32 dap_phys = (cpu->seg[SEG_DS].base + (lreg16(6) & 0xFFFF)) & cpu->a20_mask;
			if (dap_phys + 16 > (u32)cpu->phys_mem_size)
				return false;

			u8 *dap = (u8 *)&cpu->phys_mem[dap_phys];
			u16 count = dap[2] | (dap[3] << 8);
			u16 buf_off = dap[4] | (dap[5] << 8);
			u16 buf_seg = dap[6] | (dap[7] << 8);
			uint64_t lba = dap[8] | ((uint64_t)dap[9] << 8) |
			               ((uint64_t)dap[10] << 16) | ((uint64_t)dap[11] << 24) |
			               ((uint64_t)dap[12] << 32) | ((uint64_t)dap[13] << 40) |
			               ((uint64_t)dap[14] << 48) | ((uint64_t)dap[15] << 56);

			if (count == 0 || count > 128) {
				sreg16(0, (lreg16(0) & 0x00FF) | 0x0100); /* AH=1 */
				refresh_flags(cpu);
				cpu->cc.mask = 0;
				cpu->flags |= CF;
				return true;
			}

			u32 buf_phys = (((u32)buf_seg << 4) + buf_off) & cpu->a20_mask;
			if ((uint64_t)buf_phys + (uint64_t)count * 512 > (uint64_t)cpu->phys_mem_size ||
			    lba + count > (uint64_t)total_sectors) {
				sreg16(0, (lreg16(0) & 0x00FF) | 0x0400); /* AH=4 */
				refresh_flags(cpu);
				cpu->cc.mask = 0;
				cpu->flags |= CF;
				return true;
			}

			int ret;
			if (ah13 == 0x42)
				ret = ide_block_read(bs, lba, (u8 *)&cpu->phys_mem[buf_phys], count);
			else
				ret = ide_block_write(bs, lba, (const u8 *)&cpu->phys_mem[buf_phys], count);

			if (ret < 0) {
				sreg16(0, (lreg16(0) & 0x00FF) | 0x0400); /* AH=4 */
				refresh_flags(cpu);
				cpu->cc.mask = 0;
				cpu->flags |= CF;
				return true;
			}

			sreg16(0, lreg16(0) & 0x00FF); /* AH=0 */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		if (ah13 == 0x48) {
			/* Get extended drive parameters. Result buffer at DS:SI.
			 * Minimum 26 bytes: [0] u16 size, [2] u16 flags,
			 * [4] u32 cyls, [8] u32 heads, [12] u32 spt,
			 * [16] u64 total_sectors, [24] u16 bytes_per_sector */
			u32 res_phys = (cpu->seg[SEG_DS].base + (lreg16(6) & 0xFFFF)) & cpu->a20_mask;
			if (res_phys + 26 > (u32)cpu->phys_mem_size)
				return false;

			u8 *res = (u8 *)&cpu->phys_mem[res_phys];
			/* Size field: caller sets minimum, we write 26 */
			res[0] = 26; res[1] = 0;
			/* Flags: bit 1 = CHS info valid */
			res[2] = 0x02; res[3] = 0;
			/* Cylinders (u32) */
			res[4] = cyls & 0xFF; res[5] = (cyls >> 8) & 0xFF;
			res[6] = (cyls >> 16) & 0xFF; res[7] = 0;
			/* Heads (u32) */
			res[8] = heads & 0xFF; res[9] = (heads >> 8) & 0xFF;
			res[10] = 0; res[11] = 0;
			/* Sectors per track (u32) */
			res[12] = spt & 0xFF; res[13] = (spt >> 8) & 0xFF;
			res[14] = 0; res[15] = 0;
			/* Total sectors (u64) */
			for (int i = 0; i < 8; i++)
				res[16 + i] = (total_sectors >> (i * 8)) & 0xFF;
			/* Bytes per sector */
			res[24] = 0x00; res[25] = 0x02; /* 512 */

			sreg16(0, lreg16(0) & 0x00FF); /* AH=0 */
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			cpu->flags &= ~CF;
			return true;
		}

		/* Unknown INT 13h function — fall through to BIOS */
		return false;
	}

	if (intno != 0x15) return false;

	u8 ah = (lreg32(0) >> 8) & 0xFF;

	if (ah == 0x87) {
		/* INT 15h/AH=87h — Block Move (copy extended memory)
		 * ES:SI -> 6-entry GDT: [2]=source, [3]=dest descriptors
		 * CX = number of 16-bit words to copy
		 * Returns: AH=0/CF=0 on success */
		u32 cx = lreg16(1);
		u32 si = lreg16(6);
		u32 gdt_lin = (cpu->seg[SEG_ES].base + si) & cpu->a20_mask;
		u32 byte_count = (u32)cx * 2;

		/* Bounds check GDT access (need 48 bytes: 6 descriptors x 8) */
		if (gdt_lin + 48 > (u32)cpu->phys_mem_size)
			return false; /* fall through to BIOS */

		if (byte_count == 0) goto hle87_ok;

		/* Extract 32-bit base addresses from segment descriptors */
		u8 *sd = &cpu->phys_mem[gdt_lin + 0x10]; /* source = descriptor 2 */
		u8 *dd = &cpu->phys_mem[gdt_lin + 0x18]; /* dest   = descriptor 3 */
		u32 src = sd[2] | (sd[3] << 8) | (sd[4] << 16) | (sd[7] << 24);
		u32 dst = dd[2] | (dd[3] << 8) | (dd[4] << 16) | (dd[7] << 24);

		/* Bounds check source and destination */
		if ((uint64_t)src + byte_count > (uint64_t)cpu->phys_mem_size ||
		    (uint64_t)dst + byte_count > (uint64_t)cpu->phys_mem_size)
			return false; /* fall through to BIOS */

		memmove(&cpu->phys_mem[dst], &cpu->phys_mem[src], byte_count);

	hle87_ok:
		sreg16(0, lreg16(0) & 0x00FF); /* AH = 0 */
		refresh_flags(cpu);
		cpu->cc.mask = 0;
		cpu->flags &= ~CF;
		return true;
	}

	if (ah == 0x89) {
		/* INT 15h/AH=89h — Switch to Protected Mode.
		 * Same -m16 encoding issue.  Rarely used; return error. */
		sreg16(0, (lreg16(0) & 0x00FF) | 0xFF00); /* AH = 0xFF */
		refresh_flags(cpu);
		cpu->cc.mask = 0;
		cpu->flags |= CF;
		return true;
	}

	if (ah == 0x88) {
		/* INT 15h/AH=88h — Get Extended Memory Size.
		 * Returns extended memory in KB (above 1MB) in AX, max 63MB.
		 * HLE avoids any BIOS code path issues. */
		long ext_kb = (cpu->phys_mem_size - 1024 * 1024) / 1024;
		if (ext_kb > 0xFC00) ext_kb = 0xFC00;
		if (ext_kb < 0) ext_kb = 0;
		sreg16(0, (u16)ext_kb);
		refresh_flags(cpu);
		cpu->cc.mask = 0;
		cpu->flags &= ~CF;
		return true;
	}

	return false;
}

#define INT(i, li, _) \
	/*dolog("int %02x %08x %04x:%08x\n", li(i), REGi[0], SEGi(SEG_CS), cpu->ip);*/ \
	if (unlikely(cpu_diag_enabled) && li(i) == 0x21) { \
		cpu_diag_log_int21(cpu); \
	} else if (unlikely(cpu_diag_enabled) && li(i) == 0x2f) { \
		cpu_diag_log_int2f(cpu); \
	} \
	if (unlikely(cpu_diag_enabled) && li(i) == 0x10 && \
		((cpu->flags & VM) || !(cpu->cr0 & 1))) { \
		if (win386_diag.active) { \
			uint8_t _ah10 = (lreg16(0) >> 8) & 0xff; \
			uint8_t _al10 = lreg16(0) & 0xff; \
			wfw_monitor.int10h_count++; \
			wfw_monitor.last_int10h_ah = _ah10; \
			wfw_monitor.last_int10h_al = _al10; \
			if (_ah10 == 0x0e && wfw_monitor.tty_pos < sizeof(wfw_monitor.tty_buf)-1) \
				wfw_monitor.tty_buf[wfw_monitor.tty_pos++] = _al10; \
		} \
	} \
	if (unlikely(cpu_diag_enabled) && win386_diag.active && \
		li(i) == 0x15 && ((cpu->flags & VM) || !(cpu->cr0 & 1))) { \
		wfw_monitor.int15h_count++; \
		wfw_monitor.last_int15h_ah = (lreg16(0) >> 8) & 0xff; \
	} \
		if (((cpu->flags & VM) || !(cpu->cr0 & 1)) && bios_hle_int(cpu, li(i))) { \
			cpu->hle_hit_count++; \
			cpu->ip = cpu->next_ip; \
		} else if ((cpu->flags & VM)) { \
			if(get_IOPL(cpu) < 3) { \
				wfw_monitor_log_vm86_int_event(cpu, (uint8_t)li(i), 0); \
				if (unlikely(cpu_diag_enabled) && win386_diag.active) { \
					wfw_monitor.v86_int_gp_counts[(uint8_t)li(i)]++; \
					if (li(i) == 0x21 && wfw_monitor.v86_int21_ah_count < 16) \
						wfw_monitor.v86_int21_ah_log[wfw_monitor.v86_int21_ah_count++] = \
							(lreg16(0) >> 8) & 0xff; \
			} \
			if (unlikely(cpu_diag_enabled) && cpu_diag_event_allow(cpu, &cpu_diag_events.vm86_int_iopl)) { \
				dolog("V86 INT %02x blocked IOPL #%u at %04x:%04x\n", \
					li(i), cpu_diag_events.vm86_int_iopl, \
					cpu->seg[SEG_CS].sel, cpu->ip & 0xffff); \
				} \
				THROW(EX_GP, 0); \
			} \
			/* With VM86+IOPL=3, consult the Interrupt Redirection Bitmap. \
			 * If redirected, this will raise #GP for the VMM to intercept. */ \
			TRY(check_v86_int_redirect(cpu, li(i))); \
			uint16_t _ax_int = lreg16(0); \
			bool _use_idt = vm86_iopl3_softint_via_idt && \
				vm86_iopl3_int_uses_idt(li(i), _ax_int); \
			uint8_t _v86_int_path = _use_idt ? 2 : 1; \
			wfw_monitor_log_vm86_int_event(cpu, (uint8_t)li(i), _v86_int_path); \
			if (unlikely(cpu_diag_enabled) && win386_diag.active) { \
				wfw_monitor.v86_softint_total++; \
				if (li(i) == 0x13) { \
					wfw_monitor.v86_int13_count++; \
					wfw_monitor.v86_int13_last_path = _v86_int_path; \
					wfw_monitor.v86_int13_req_ax = lreg16(0); \
					wfw_monitor.v86_int13_req_bx = lreg16(3); \
					wfw_monitor.v86_int13_req_cx = lreg16(1); \
					wfw_monitor.v86_int13_req_dx = lreg16(2); \
					wfw_monitor.v86_int13_req_es = cpu->seg[SEG_ES].sel; \
					wfw_monitor.pending_v86_int13 = true; \
					wfw_monitor.pending_v86_int13_ret_cs = cpu->seg[SEG_CS].sel; \
					wfw_monitor.pending_v86_int13_ret_ip = cpu->next_ip & 0xffff; \
				} \
				if (li(i) == 0x2f) { \
				wfw_monitor.v86_int2f_count++; \
				uint16_t _ax = lreg16(0); \
				uint8_t _hp = wfw_monitor.v86_int2f_hist_pos; \
				if (_hp < 8) { \
					bool _dup = false; \
					for (int _j = 0; _j < _hp; _j++) \
						if (wfw_monitor.v86_int2f_ax_hist[_j] == _ax) { _dup = true; break; } \
					if (!_dup) wfw_monitor.v86_int2f_ax_hist[wfw_monitor.v86_int2f_hist_pos++] = _ax; \
				} \
				if (_ax == 0x1687) { \
					/* DPMI detection — save IVT data, set return trace */ \
					uint32_t _ivt = *(uint32_t*)(cpu->phys_mem + 0x2f * 4); \
					dpmi_trace.ivt_phys_cs = _ivt >> 16; \
					dpmi_trace.ivt_phys_ip = _ivt & 0xffff; \
					dpmi_trace.ret_cs = cpu->seg[SEG_CS].sel; \
					dpmi_trace.ret_ip = cpu->next_ip & 0xffff; \
					dpmi_trace.ret_active = true; \
				} \
			} else if (li(i) == 0x21) { \
				wfw_monitor.v86_int21_count++; \
			} \
		} \
		uword oldip = cpu->ip; \
		cpu->ip = cpu->next_ip; \
		if (_use_idt) { \
			/* VM86 IDT-compat path uses normal software-INT semantics
			 * (including DPL checks) to preserve VMM trap behavior. */ \
			if (!call_isr(cpu, li(i), false, 0)) { \
				cpu->ip = oldip; \
				return false; \
			} \
		} else { \
			if (!call_isr_vm86_softint(cpu, li(i))) { \
				cpu->ip = oldip; \
				return false; \
			} \
		} \
	} else { \
		uword oldip = cpu->ip; \
		cpu->ip = cpu->next_ip; \
		if (!call_isr(cpu, li(i), false, 0)) { \
			cpu->ip = oldip; \
			return false; \
		} \
	}

#define IRET() \
	if ((cpu->cr0 & 1) && (!(cpu->flags & VM) || get_IOPL(cpu) < 3)) { \
		TRY(pmret(cpu, opsz16, 0, true)); \
	} else if (opsz16) { \
		/* 16-bit IRET in real/V86 mode */ \
		uword sp = lreg32(4); \
		uword newip, oldflags; \
		int newcs; \
		uint16_t newflags_val; \
		bool vm_iret_exec = !!(cpu->flags & VM); \
		uint16_t vm_from_cs = cpu->seg[SEG_CS].sel; \
		uint16_t vm_from_ss = cpu->seg[SEG_SS].sel; \
		uword vm_from_ip = cpu->ip; \
		/* Fast path: direct memory read if 6 bytes won't cross page. \
		 * MUST NOT use in V86 mode — paging maps linear != physical. */ \
		if (likely(((sp & 0xfff) <= 0xffa) && !(cpu->flags & VM) && \
			   !(cpu_diag_enabled && win386_diag.active))) { \
			uint8_t *stack = cpu->phys_mem + cpu->seg[SEG_SS].base + (sp & sp_mask); \
			newip = *(uint16_t *)(stack + 0); \
			newcs = *(uint16_t *)(stack + 2); \
			newflags_val = *(uint16_t *)(stack + 4); \
		} else { \
			OptAddr meml1, meml2, meml3; \
			/* ip */ TRY(translate16(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
			newip = laddr16(&meml1); \
			/* cs */ TRY(translate16(cpu, &meml2, 1, SEG_SS, (sp + 2) & sp_mask)); \
			newcs = laddr16(&meml2); \
			/* flags */ TRY(translate16(cpu, &meml3, 1, SEG_SS, (sp + 4) & sp_mask)); \
			newflags_val = laddr16(&meml3); \
		} \
		oldflags = cpu->flags; \
		if (cpu->flags & VM) cpu->flags = (cpu->flags & (0xffff0000 | IOPL)) | (newflags_val & ~IOPL); \
		else cpu->flags = (cpu->flags & 0xffff0000) | newflags_val; \
		cpu->flags &= EFLAGS_MASK; \
		cpu->flags |= 0x2; \
		if (!set_seg(cpu, SEG_CS, newcs)) { cpu->flags = oldflags; return false; } \
		cpu->cc.mask = 0; \
		set_sp(sp + 6, sp_mask); \
		cpu->next_ip = newip; \
		if (unlikely(cpu_diag_enabled && vm_iret_exec)) { \
			cpu_diag_vm86_iret_ring_push(cpu, vm_from_cs, vm_from_ip, vm_from_ss, sp, \
						     newcs, newip, cpu->flags, true); \
		} \
			if (unlikely(cpu_diag_enabled && vm_iret_exec)) \
				wfw_monitor_note_int13_return(cpu, (uint16_t)newcs, (uint16_t)newip, cpu->flags); \
	} else { \
		/* 32-bit IRETD in real/V86 mode */ \
		uword sp = lreg32(4); \
		uword newip, oldflags; \
		int newcs; \
		uint32_t newflags_val; \
		bool vm_iret_exec = !!(cpu->flags & VM); \
		uint16_t vm_from_cs = cpu->seg[SEG_CS].sel; \
		uint16_t vm_from_ss = cpu->seg[SEG_SS].sel; \
		uword vm_from_ip = cpu->ip; \
		/* Fast path: direct memory read if 12 bytes won't cross page. \
		 * MUST NOT use in V86 mode — paging maps linear != physical. */ \
		if (likely(((sp & 0xfff) <= 0xff4) && !(cpu->flags & VM) && \
			   !(cpu_diag_enabled && win386_diag.active))) { \
			uint8_t *stack = cpu->phys_mem + cpu->seg[SEG_SS].base + (sp & sp_mask); \
			newip = *(uint32_t *)(stack + 0); \
			newcs = *(uint32_t *)(stack + 4) & 0xffff; \
			newflags_val = *(uint32_t *)(stack + 8); \
		} else { \
			OptAddr meml1, meml2, meml3; \
			/* eip */ TRY(translate32(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
			newip = laddr32(&meml1); \
			/* cs */ TRY(translate32(cpu, &meml2, 1, SEG_SS, (sp + 4) & sp_mask)); \
			newcs = laddr32(&meml2) & 0xffff; \
			/* eflags */ TRY(translate32(cpu, &meml3, 1, SEG_SS, (sp + 8) & sp_mask)); \
			newflags_val = laddr32(&meml3); \
		} \
		oldflags = cpu->flags; \
		if (cpu->flags & VM) cpu->flags = (cpu->flags & (IOPL | VM)) | (newflags_val & ~(IOPL | VM)); \
		else cpu->flags = newflags_val; \
		cpu->flags &= EFLAGS_MASK; \
		cpu->flags |= 0x2; \
		if (!set_seg(cpu, SEG_CS, newcs)) { cpu->flags = oldflags; return false; } \
		cpu->cc.mask = 0; \
		set_sp(sp + 12, sp_mask); \
		cpu->next_ip = newip; \
		if (unlikely(cpu_diag_enabled && vm_iret_exec)) { \
			cpu_diag_vm86_iret_ring_push(cpu, vm_from_cs, vm_from_ip, vm_from_ss, sp, \
						     newcs, newip, cpu->flags, false); \
		} \
			if (unlikely(cpu_diag_enabled && vm_iret_exec)) \
				wfw_monitor_note_int13_return(cpu, (uint16_t)newcs, (uint16_t)newip, cpu->flags); \
	} \
	if (cpu->intr && (cpu->flags & IF)) return true;

#define RETFARw(i, li, _) \
	if ((cpu->cr0 & 1) && !(cpu->flags & VM)) { \
		TRY(pmret(cpu, opsz16, li(i), false)); \
	} else { \
		if (opsz16) { \
			OptAddr meml1, meml2; \
			uword sp = lreg32(4); \
			/* ip */ TRY(translate16(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
			uword newip = laddr16(&meml1); \
			/* cs */ TRY(translate16(cpu, &meml2, 1, SEG_SS, (sp + 2) & sp_mask)); \
			int newcs = laddr16(&meml2); \
			TRY(set_seg(cpu, SEG_CS, newcs)); \
			set_sp(sp + 4 + li(i), sp_mask); \
			cpu->next_ip = newip; \
		} else { \
			OptAddr meml1, meml2; \
			uword sp = lreg32(4); \
			/* ip */ TRY(translate32(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
			uword newip = laddr32(&meml1); \
			/* cs */ TRY(translate32(cpu, &meml2, 1, SEG_SS, (sp + 4) & sp_mask)); \
			int newcs = laddr32(&meml2); \
			TRY(set_seg(cpu, SEG_CS, newcs)); \
			set_sp(sp + 8 + li(i), sp_mask); \
			cpu->next_ip = newip; \
		} \
	}

#define RETFAR() RETFARw(0, limm, 0)

#define HLT() \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	hlt_diag_sync(cpu); \
	refresh_flags(cpu); \
	hlt_diag.count++; \
	hlt_diag.cs = cpu->seg[SEG_CS].sel; \
	hlt_diag.ip = cpu->ip; \
	hlt_diag.ss = cpu->seg[SEG_SS].sel; \
	hlt_diag.sp = REGi(4); \
	hlt_diag.fl = cpu->flags; \
	hlt_diag.cpl = cpu->cpl; \
	hlt_diag.vm = !!(cpu->flags & VM); \
	if (unlikely(cpu_diag_enabled) && cpu_diag_event_allow(cpu, &cpu_diag_events.hlt_exec)) { \
		dolog("=== HLT executed #%u ===\n", cpu_diag_events.hlt_exec); \
		dolog("  at CS:EIP=%04x:%08x SS:ESP=%04x:%08x FL=%08x\n", \
			cpu->seg[SEG_CS].sel, cpu->ip, cpu->seg[SEG_SS].sel, REGi(4), cpu->flags); \
	} \
	cpu->halt = true; return true;
#define NOP()

#define LAHF() \
	refresh_flags(cpu); \
	cpu->cc.mask = 0; \
	sreg8(4, cpu->flags);

#define SAHF() \
	cpu->cc.mask &= OF; \
	cpu->flags = (cpu->flags & (wordmask ^ 0xff)) | lreg8(4); \
	cpu->flags &= EFLAGS_MASK; \
	cpu->flags |= 0x2;

#define CMC() \
	int cf = get_CF(cpu); \
	cpu->cc.mask &= ~CF; \
	SET_BIT(cpu->flags, !cf, CF);

#define CLC() \
	cpu->cc.mask &= ~CF; \
	cpu->flags &= ~CF;

#define STC() \
	cpu->cc.mask &= ~CF; \
	cpu->flags |= CF;

#define CLI() \
	if (get_IOPL(cpu) < cpu->cpl) THROW(EX_GP, 0); \
	cpu->flags &= ~IF;

/* STI: interrupts enabled at the end of the **next** instruction */
#define STI() \
	if (get_IOPL(cpu) < cpu->cpl) THROW(EX_GP, 0); \
	cpu->flags |= IF; \
	if (cpu->intr || stepcount < 2) stepcount = 2;

#define CLD() \
	cpu->flags &= ~DF;

#define STD() \
	cpu->flags |= DF;

#define PUSHb(a, la, sa) \
	OptAddr meml1; \
	uword sp = lreg32(4); \
	uword val = sext8(la(a)); \
	if (opsz16) { \
		TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask)); \
		set_sp(sp - 2, sp_mask); \
		saddr16(&meml1, val); \
	} else { \
		TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask)); \
		set_sp(sp - 4, sp_mask); \
		saddr32(&meml1, val); \
	}

#define PUSHw(a, la, sa) \
	OptAddr meml1; \
	uword sp = lreg32(4); \
	uword val = sext16(la(a)); \
	TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask)); \
	set_sp(sp - 2, sp_mask); \
	saddr16(&meml1, val);

#define PUSHd(a, la, sa) \
	OptAddr meml1; \
	uword sp = lreg32(4); \
	uword val = sext32(la(a)); \
	TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask)); \
	set_sp(sp - 4, sp_mask); \
	saddr32(&meml1, val);

#define POPRegw(a, la, sa) \
	OptAddr meml1; \
	uword sp = lreg32(4); \
	TRY(translate16(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
	u16 src = laddr16(&meml1); \
	set_sp(sp + 2, sp_mask); \
	sa(a, src);

#define POPRegd(a, la, sa) \
	OptAddr meml1; \
	uword sp = lreg32(4); \
	TRY(translate32(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
	u32 src = laddr32(&meml1); \
	set_sp(sp + 4, sp_mask); \
	sa(a, src);

#define POP_helper(BIT) \
	OptAddr meml1; \
	TRY(fetch8(cpu, &modrm)); \
	int mod = modrm >> 6; \
	int rm = modrm & 7; \
	uword sp = lreg32(4); \
	TRY(translate ## BIT(cpu, &meml1, 1, SEG_SS, sp & sp_mask)); \
	u ## BIT src = laddr ## BIT(&meml1); \
	set_sp(sp + BIT / 8, sp_mask); \
	if (mod == 3) { \
		sreg ## BIT(rm, src); \
	} else { \
		if (!modsib(cpu, adsz16, mod, rm, &addr, &curr_seg) || \
		    !translate ## BIT(cpu, &meml, 2, curr_seg, addr)) { \
			set_sp(sp, sp_mask); \
			return false; \
		} \
		saddr ## BIT(&meml, src); \
	}

#define POP() if (opsz16) { POP_helper(16) } else { POP_helper(32) }

#define PUSHF() \
	if ((cpu->flags & VM) && get_IOPL(cpu) < 3) THROW(EX_GP, 0); \
	if (opsz16) { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 2, SEG_SS, (sp - 2) & sp_mask)); \
		refresh_flags(cpu); \
		cpu->cc.mask = 0; \
		set_sp(sp - 2, sp_mask); \
		saddr16(&meml, cpu->flags); \
	} else { \
		uword sp = lreg32(4); \
		TRY(translate32(cpu, &meml, 2, SEG_SS, (sp - 4) & sp_mask)); \
		refresh_flags(cpu); \
		cpu->cc.mask = 0; \
		set_sp(sp - 4, sp_mask); \
		saddr32(&meml, cpu->flags & ~(RF | VM)); \
	}

#define EFLAGS_MASK_386 0x37fd7
#define EFLAGS_MASK_486 0x77fd7
#define EFLAGS_MASK_586 0x277fd7
#define EFLAGS_MASK (cpu->flags_mask)

#define POPF() \
	if ((cpu->flags & VM) && get_IOPL(cpu) < 3) THROW(EX_GP, 0); \
	uword mask = VM; \
	if (cpu->cr0 & 1) { \
		if (cpu->cpl > 0) mask |= IOPL; \
		if (get_IOPL(cpu) < cpu->cpl) mask |= IF; \
	} \
	if (opsz16) { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		set_sp(sp + 2, sp_mask); \
		cpu->flags = (cpu->flags & (0xffff0000 | mask)) | (laddr16(&meml) & ~mask); \
	} else { \
		uword sp = lreg32(4); \
		TRY(translate32(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		set_sp(sp + 4, sp_mask); \
		cpu->flags = (cpu->flags & mask) | (laddr32(&meml) & ~mask); \
	} \
	cpu->flags &= EFLAGS_MASK; \
	cpu->flags |= 0x2; \
	cpu->cc.mask = 0; \
	if (cpu->intr && (cpu->flags & IF)) return true;

#define PUSHSeg(seg) \
	if (opsz16) { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 2, SEG_SS, (sp - 2) & sp_mask)); \
		set_sp(sp - 2, sp_mask); \
		saddr16(&meml, lseg(seg)); \
	} else { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 2, SEG_SS, (sp - 4) & sp_mask)); \
		set_sp(sp - 4, sp_mask); \
		saddr16(&meml, lseg(seg)); \
	}
#define PUSH_ES() PUSHSeg(SEG_ES)
#define PUSH_CS() PUSHSeg(SEG_CS)
#define PUSH_SS() PUSHSeg(SEG_SS)
#define PUSH_DS() PUSHSeg(SEG_DS)
#define PUSH_FS() PUSHSeg(SEG_FS)
#define PUSH_GS() PUSHSeg(SEG_GS)

#define POPSeg(seg) \
	if (opsz16) { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		TRY(set_seg(cpu, seg, laddr16(&meml))); \
		set_sp(sp + 2, sp_mask); \
	} else { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		TRY(set_seg(cpu, seg, laddr16(&meml))); \
		set_sp(sp + 4, sp_mask); \
	}
#define POP_ES() POPSeg(SEG_ES)
#define POP_SS() POPSeg(SEG_SS) stepcount++;
#define POP_DS() POPSeg(SEG_DS)
#define POP_FS() POPSeg(SEG_FS)
#define POP_GS() POPSeg(SEG_GS)

#define PUSHA_helper(BIT, BYTE) \
	uword sp = lreg32(4); \
	OptAddr meml1, meml2, meml3, meml4; \
	OptAddr meml5, meml6, meml7, meml8; \
	TRY(translate ## BIT(cpu, &meml1, 2, SEG_SS, (sp - BYTE * 1) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml2, 2, SEG_SS, (sp - BYTE * 2) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml3, 2, SEG_SS, (sp - BYTE * 3) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml4, 2, SEG_SS, (sp - BYTE * 4) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml5, 2, SEG_SS, (sp - BYTE * 5) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml6, 2, SEG_SS, (sp - BYTE * 6) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml7, 2, SEG_SS, (sp - BYTE * 7) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml8, 2, SEG_SS, (sp - BYTE * 8) & sp_mask)); \
	saddr ## BIT(&meml1, lreg ## BIT(0)); \
	saddr ## BIT(&meml2, lreg ## BIT(1)); \
	saddr ## BIT(&meml3, lreg ## BIT(2)); \
	saddr ## BIT(&meml4, lreg ## BIT(3)); \
	saddr ## BIT(&meml5, sp); \
	saddr ## BIT(&meml6, lreg ## BIT(5)); \
	saddr ## BIT(&meml7, lreg ## BIT(6)); \
	saddr ## BIT(&meml8, lreg ## BIT(7)); \
	set_sp(sp - BYTE * 8, sp_mask);
#define PUSHA() if (opsz16) { PUSHA_helper(16, 2) } else { PUSHA_helper(32, 4) }

#define POPA_helper(BIT, BYTE) \
	uword sp = lreg32(4); \
	OptAddr meml1, meml2, meml3, meml4; \
	OptAddr meml5, meml6, meml7; \
	TRY(translate ## BIT(cpu, &meml1, 1, SEG_SS, (sp + BYTE * 0) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml2, 1, SEG_SS, (sp + BYTE * 1) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml3, 1, SEG_SS, (sp + BYTE * 2) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml4, 1, SEG_SS, (sp + BYTE * 4) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml5, 1, SEG_SS, (sp + BYTE * 5) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml6, 1, SEG_SS, (sp + BYTE * 6) & sp_mask)); \
	TRY(translate ## BIT(cpu, &meml7, 1, SEG_SS, (sp + BYTE * 7) & sp_mask)); \
	sreg ## BIT(7, laddr ## BIT(&meml1)); \
	sreg ## BIT(6, laddr ## BIT(&meml2)); \
	sreg ## BIT(5, laddr ## BIT(&meml3)); \
	sreg ## BIT(3, laddr ## BIT(&meml4)); \
	sreg ## BIT(2, laddr ## BIT(&meml5)); \
	sreg ## BIT(1, laddr ## BIT(&meml6)); \
	sreg ## BIT(0, laddr ## BIT(&meml7)); \
	set_sp(sp + BYTE * 8, sp_mask);
#define POPA() if (opsz16) { POPA_helper(16, 2) } else { POPA_helper(32, 4) }

// string operations
#define stdi(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 2, SEG_ES, lreg ## ABIT(7))); \
	saddr ## BIT(&meml, ax); \
	sreg ## ABIT(7, lreg ## ABIT(7) + dir);

#define ldsi(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 1, curr_seg, lreg ## ABIT(6))); \
	ax = laddr ## BIT(&meml); \
	sreg ## ABIT(6, lreg ## ABIT(6) + dir);

#define lddi(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 1, SEG_ES, lreg ## ABIT(7))); \
	ax = laddr ## BIT(&meml); \
	sreg ## ABIT(7, lreg ## ABIT(7) + dir);

#define ldsistdi(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 1, curr_seg, lreg ## ABIT(6))); \
	ax = laddr ## BIT(&meml); \
	TRY(translate ## BIT(cpu, &meml, 2, SEG_ES, lreg ## ABIT(7))); \
	saddr ## BIT(&meml, ax); \
	sreg ## ABIT(6, lreg ## ABIT(6) + dir); \
	sreg ## ABIT(7, lreg ## ABIT(7) + dir);

#define ldsilddi(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 1, curr_seg, lreg ## ABIT(6))); \
	ax0 = laddr ## BIT(&meml); \
	TRY(translate ## BIT(cpu, &meml, 1, SEG_ES, lreg ## ABIT(7))); \
	ax = laddr ## BIT(&meml); \
	sreg ## ABIT(6, lreg ## ABIT(6) + dir); \
	sreg ## ABIT(7, lreg ## ABIT(7) + dir);

#define xdir8 int dir = (cpu->flags & DF) ? -1 : 1;
#define xdir16 int dir = (cpu->flags & DF) ? -2 : 2;
#define xdir32 int dir = (cpu->flags & DF) ? -4 : 4;

#define STOS_helper2(BIT, ABIT) \
	OptAddr memld; \
	uword cx = lreg ## ABIT(1); \
	/* Fast path: entire REP fits in single LINEAR page, forward, RAM-only */ \
	if (I386_STOS_FASTPATH && ABIT == 32 && cpu->a20_mask == 0xFFFFFFFFu && \
	    cx > 0 && dir > 0 && (cpu->cr0 & 1) && !(cpu->flags & VM)) { \
		uword dst_log = lreg ## ABIT(7); \
		uword dst_lin = cpu->seg[SEG_ES].base + dst_log; \
		uword bytes = cx * (BIT / 8); \
		if (bytes <= 4096 && \
		    (dst_lin >> 12) == ((dst_lin + bytes - 1) >> 12)) { \
			TRY(translate ## BIT(cpu, &memld, 2, SEG_ES, dst_log)); \
			if (!in_iomem(memld.addr1) && \
			    memld.addr1 + bytes <= cpu->phys_mem_size && \
			    seg_span_fast_ok(cpu, SEG_ES, 2, dst_log, bytes)) { \
				u8 *dest = cpu->phys_mem + memld.addr1; \
				if (BIT == 8) { \
					memset(dest, ax, cx); \
				} else if (BIT == 32) { \
					fast_memset32(dest, ax, cx); \
				} else { \
					for (uword i = 0; i < cx; i++) { \
						*(u16 *)(dest + i * 2) = ax; \
					} \
				} \
				sreg ## ABIT(7, dst_log + bytes); \
				sreg ## ABIT(1, 0); \
				cx = 0; \
			} \
		} \
	} \
	while (cx) { \
		TRY(translate ## BIT(cpu, &memld, 2, SEG_ES, lreg ## ABIT(7))); \
		if (memld.addr1 % (BIT / 8)) { \
			/* slow path */ \
			while (lreg ## ABIT(1)) { \
				stdi(BIT, ABIT) \
				sreg ## ABIT(1, lreg ## ABIT(1) - 1); \
			} \
			break; \
		} \
		uword count = cx; \
		int countd; \
		if (dir > 0) countd = (4096 - (memld.addr1 & 4095)) / (BIT / 8); \
		else countd = 1 + (memld.addr1 & 4095) / (BIT / 8); \
		if (countd < count) \
			count = countd; \
		for (uword i = 0; i <= count - 1; i++) { \
			saddr ## BIT(&memld, ax); \
			memld.addr1 += dir; \
		} \
		sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
		sreg ## ABIT(1, cx - count); \
		cx = lreg ## ABIT(1); \
	}

#define STOS_helper(BIT) \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	xdir ## BIT \
	u ## BIT ax = REGi(0); \
	if (rep == 0) { \
		if (adsz16) { stdi(BIT, 16) } else { stdi(BIT, 32) } \
	} else { \
		if (adsz16) { STOS_helper2(BIT, 16) } else { STOS_helper2(BIT, 32) } \
	}

#define LODS_helper(BIT) \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	xdir ## BIT \
	u ## BIT ax; \
	if (rep == 0) { \
		if (adsz16) { ldsi(BIT, 16) } else { ldsi(BIT, 32) } \
		sreg ## BIT(0, ax); \
	} else { \
		if (adsz16) { \
			while (lreg16(1)) { \
				ldsi(BIT, 16) \
				sreg ## BIT(0, ax); \
				sreg16(1, lreg16(1) - 1); \
			} \
		} else { \
			while (lreg32(1)) { \
				ldsi(BIT, 32) \
				sreg ## BIT(0, ax); \
				sreg32(1, lreg32(1) - 1); \
			} \
		} \
	}

#define SCAS_helper(BIT) \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	xdir ## BIT \
	u ## BIT ax0 = REGi(0); \
	u ## BIT ax; \
	if (rep == 0) { \
		if (adsz16) { lddi(BIT, 16) } else { lddi(BIT, 32) } \
		cpu->cc.src1 = sext ## BIT(ax0); \
		cpu->cc.src2 = sext ## BIT(ax); \
		cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
		cpu->cc.op = CC_SUB; \
		cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	} else { \
		if (adsz16) { \
			while (lreg16(1)) { \
				lddi(BIT, 16) \
				sreg16(1, lreg16(1) - 1); \
				cpu->cc.src1 = sext ## BIT(ax0); \
				cpu->cc.src2 = sext ## BIT(ax); \
				cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
				cpu->cc.op = CC_SUB; \
				cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
				bool zf = get_ZF(cpu); \
				if ((zf && rep == 2) || (!zf && rep == 1)) break; \
			} \
		} else { \
			while (lreg32(1)) { \
				lddi(BIT, 32) \
				sreg32(1, lreg32(1) - 1); \
				cpu->cc.src1 = sext ## BIT(ax0); \
				cpu->cc.src2 = sext ## BIT(ax); \
				cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
				cpu->cc.op = CC_SUB; \
				cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
				bool zf = get_ZF(cpu); \
				if ((zf && rep == 2) || (!zf && rep == 1)) break; \
			} \
		} \
	}

#define MOVS_helper2(BIT, ABIT) \
	OptAddr memls, memld; \
	uword cx = lreg ## ABIT(1); \
	/* Fast path: entire REP fits in single LINEAR pages, forward, RAM-only, non-overlapping */ \
	if (I386_MOVS_FASTPATH && ABIT == 32 && cpu->a20_mask == 0xFFFFFFFFu && \
	    cx > 0 && dir > 0 && curr_seg == SEG_DS && (cpu->cr0 & 1) && !(cpu->flags & VM)) { \
		uword src_log = lreg ## ABIT(6); \
		uword dst_log = lreg ## ABIT(7); \
		uword src_lin = cpu->seg[curr_seg].base + src_log; \
		uword dst_lin = cpu->seg[SEG_ES].base + dst_log; \
		uword bytes = cx * (BIT / 8); \
		/* Check LINEAR addresses stay within same pages */ \
		if (bytes <= 4096 && \
		    (src_lin >> 12) == ((src_lin + bytes - 1) >> 12) && \
		    (dst_lin >> 12) == ((dst_lin + bytes - 1) >> 12) && \
		    seg_span_fast_ok(cpu, curr_seg, 1, src_log, bytes) && \
		    seg_span_fast_ok(cpu, SEG_ES, 2, dst_log, bytes)) { \
			TRY(translate ## BIT(cpu, &memls, 1, curr_seg, src_log)); \
			TRY(translate ## BIT(cpu, &memld, 2, SEG_ES, dst_log)); \
			if (!in_iomem(memls.addr1) && !in_iomem(memld.addr1) && \
			    memls.addr1 + bytes <= cpu->phys_mem_size && \
			    memld.addr1 + bytes <= cpu->phys_mem_size) { \
				/* Check for overlapping forward copy (dst > src but dst < src+bytes). \
				 * x86 REP MOVSB forward with overlap creates a "fill" pattern - \
				 * memmove would incorrectly preserve source data by copying backward. \
				 * Only use fast path for non-overlapping copies. */ \
				if (memld.addr1 >= memls.addr1 + bytes || memls.addr1 >= memld.addr1 + bytes) { \
					fast_memcpy(cpu->phys_mem + memld.addr1, \
					            cpu->phys_mem + memls.addr1, bytes); \
					sreg ## ABIT(6, src_log + bytes); \
					sreg ## ABIT(7, dst_log + bytes); \
					sreg ## ABIT(1, 0); \
					cx = 0; \
				} \
				/* If overlapping, fall through to byte-by-byte loop */ \
			} \
		} \
	} \
	while (cx) { \
		TRY(translate ## BIT(cpu, &memls, 1, curr_seg, lreg ## ABIT(6))); \
		TRY(translate ## BIT(cpu, &memld, 2, SEG_ES, lreg ## ABIT(7))); \
		if (memls.addr1 % (BIT / 8) || memld.addr1 % (BIT / 8)) { \
			/* slow path */ \
			while (lreg ## ABIT(1)) { \
				ldsistdi(BIT, ABIT) \
				sreg ## ABIT(1, lreg ## ABIT(1) - 1); \
			} \
			break; \
		} \
		uword count = cx; \
		int counts, countd; \
		if (dir > 0) { \
			counts = (4096 - (memls.addr1 & 4095)) / (BIT / 8); \
			countd = (4096 - (memld.addr1 & 4095)) / (BIT / 8); \
		} else { \
			counts = 1 + (memls.addr1 & 4095) / (BIT / 8); \
			countd = 1 + (memld.addr1 & 4095) / (BIT / 8); \
		} \
		if (counts < count) \
			count = counts; \
		if (countd < count) \
			count = countd; \
		if (cpu->cb.iomem_write_string && in_iomem(memld.addr1) && \
		    dir > 0  && in_iomem(memld.addr1 + count - 1) && \
		    (memls.addr1 | 4095) < cpu->phys_mem_size && \
		    !in_iomem(memls.addr1) && !in_iomem(memls.addr1 | 4095)) { \
			if (cpu->cb.iomem_write_string( \
				    cpu->cb.iomem, memld.addr1, \
				    cpu->phys_mem + memls.addr1, count * dir)) { \
				sreg ## ABIT(6, lreg ## ABIT(6) + count * dir); \
				sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
				sreg ## ABIT(1, cx - count); \
				cx = lreg ## ABIT(1); \
				continue; \
			} \
		} \
		/* Fast path for VGA-to-RAM copy (e.g., collision detection) */ \
		if (cpu->cb.iomem_read_string && in_iomem(memls.addr1) && \
		    dir > 0 && in_iomem(memls.addr1 + count - 1) && \
		    (memld.addr1 | 4095) < cpu->phys_mem_size && \
		    !in_iomem(memld.addr1) && !in_iomem(memld.addr1 | 4095)) { \
			if (cpu->cb.iomem_read_string( \
				    cpu->cb.iomem, memls.addr1, \
				    cpu->phys_mem + memld.addr1, count * dir)) { \
				sreg ## ABIT(6, lreg ## ABIT(6) + count * dir); \
				sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
				sreg ## ABIT(1, cx - count); \
				cx = lreg ## ABIT(1); \
				continue; \
			} \
		} \
		/* Fast path for VGA-to-VGA copy (Mode X scrolling/blitting) */ \
		if (cpu->cb.iomem_copy_string && \
		    in_iomem(memls.addr1) && in_iomem(memld.addr1) && \
		    dir > 0 && \
		    in_iomem(memls.addr1 + count - 1) && \
		    in_iomem(memld.addr1 + count - 1)) { \
			if (cpu->cb.iomem_copy_string( \
				    cpu->cb.iomem, memld.addr1, memls.addr1, count)) { \
				sreg ## ABIT(6, lreg ## ABIT(6) + count * dir); \
				sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
				sreg ## ABIT(1, cx - count); \
				cx = lreg ## ABIT(1); \
				continue; \
			} \
		} \
		for (uword i = 0; i <= count - 1; i++) { \
			store ## BIT(cpu, &memld, load ## BIT(cpu, &memls)); \
			memld.addr1 += dir; \
			memls.addr1 += dir; \
		} \
		sreg ## ABIT(6, lreg ## ABIT(6) + count * dir); \
		sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
		sreg ## ABIT(1, cx - count); \
		cx = lreg ## ABIT(1); \
	}

#define MOVS_helper(BIT) \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	xdir ## BIT \
	u ## BIT ax; \
	if (rep == 0) { \
		if (adsz16) { ldsistdi(BIT, 16) } else { ldsistdi(BIT, 32) } \
	} else { \
		if (adsz16) { MOVS_helper2(BIT, 16) } else { MOVS_helper2(BIT, 32) } \
	}

#define CMPS_helper(BIT) \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	xdir ## BIT \
	u ## BIT ax0, ax; \
	if (rep == 0) { \
		if (adsz16) { ldsilddi(BIT, 16) } else { ldsilddi(BIT, 32) } \
		cpu->cc.src1 = sext ## BIT(ax0); \
		cpu->cc.src2 = sext ## BIT(ax); \
		cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
		cpu->cc.op = CC_SUB; \
		cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	} else { \
		if (adsz16) { \
			while (lreg16(1)) { \
				ldsilddi(BIT, 16) \
				sreg16(1, lreg16(1) - 1); \
				cpu->cc.src1 = sext ## BIT(ax0); \
				cpu->cc.src2 = sext ## BIT(ax); \
				cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
				cpu->cc.op = CC_SUB; \
				cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
				bool zf = get_ZF(cpu); \
				if ((zf && rep == 2) || (!zf && rep == 1)) break; \
			} \
		} else { \
			while (lreg32(1)) { \
				ldsilddi(BIT, 32) \
				sreg32(1, lreg32(1) - 1); \
				cpu->cc.src1 = sext ## BIT(ax0); \
				cpu->cc.src2 = sext ## BIT(ax); \
				cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
				cpu->cc.op = CC_SUB; \
				cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
				bool zf = get_ZF(cpu); \
				if ((zf && rep == 2) || (!zf && rep == 1)) break; \
			} \
		} \
	}

#define STOSb() STOS_helper(8)
#define LODSb() LODS_helper(8)
#define SCASb() SCAS_helper(8)
#define MOVSb() MOVS_helper(8)
#define CMPSb() CMPS_helper(8)
#define STOS() if (opsz16) { STOS_helper(16) } else { STOS_helper(32) }
#define LODS() if (opsz16) { LODS_helper(16) } else { LODS_helper(32) }
#define SCAS() if (opsz16) { SCAS_helper(16) } else { SCAS_helper(32) }
#define MOVS() if (opsz16) { MOVS_helper(16) } else { MOVS_helper(32) }
#define CMPS() if (opsz16) { CMPS_helper(16) } else { CMPS_helper(32) }

#define indxstdi(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 2, SEG_ES, lreg ## ABIT(7))); \
	ax = cpu->cb.io_read ## BIT(cpu->cb.io, lreg16(2)); \
	saddr ## BIT(&meml, ax); \
	sreg ## ABIT(7, lreg ## ABIT(7) + dir);

#define INS_helper2(BIT, ABIT) \
	OptAddr memld; \
	uword cx = lreg ## ABIT(1); \
	while (cx) { \
		TRY(translate ## BIT(cpu, &memld, 2, SEG_ES, lreg ## ABIT(7))); \
		if (memld.addr1 % (BIT / 8)) { \
			/* slow path */ \
			while (lreg ## ABIT(1)) { \
				indxstdi(BIT, ABIT) \
				sreg ## ABIT(1, lreg ## ABIT(1) - 1); \
			} \
			break; \
		} \
		uword count = cx; \
		int countd; \
		if (dir > 0) countd = (4096 - (memld.addr1 & 4095)) / (BIT / 8); \
		else countd = 1 + (memld.addr1 & 4095) / (BIT / 8); \
		if (countd < count) \
			count = countd; \
		if (cpu->cb.io_read_string && dir > 0 && \
		    (memld.addr1 | 4095) < cpu->phys_mem_size && \
		    !in_iomem(memld.addr1) && !in_iomem(memld.addr1 | 4095)) { \
			int count1 = cpu->cb.io_read_string( \
				cpu->cb.io, lreg16(2), \
				cpu->phys_mem + memld.addr1, dir, count); \
			if (count1 > 0) { \
				count = count1; \
				sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
				sreg ## ABIT(1, cx - count); \
				cx = lreg ## ABIT(1); \
				continue; \
			} \
		} \
		for (uword i = 0; i <= count - 1; i++) { \
			ax = cpu->cb.io_read ## BIT(cpu->cb.io, lreg16(2)); \
			saddr ## BIT(&memld, ax); \
			memld.addr1 += dir; \
		} \
		sreg ## ABIT(7, lreg ## ABIT(7) + count * dir); \
		sreg ## ABIT(1, cx - count); \
		cx = lreg ## ABIT(1); \
	}

#define INS_helper(BIT) \
	TRY(check_ioperm(cpu, lreg16(2), BIT)); \
	xdir ## BIT \
	u ## BIT ax; \
	if (rep == 0) { \
		if (adsz16) { indxstdi(BIT, 16) } else { indxstdi(BIT, 32) } \
	} else { \
		if (rep != 1) THROW0(EX_UD); \
		if (adsz16) { INS_helper2(BIT, 16) } else { INS_helper2(BIT, 32) } \
	}

#define INSb() INS_helper(8)
#define INS() if (opsz16) { INS_helper(16) } else { INS_helper(32) }

#define ldsioutdx(BIT, ABIT) \
	TRY(translate ## BIT(cpu, &meml, 1, curr_seg, lreg ## ABIT(6))); \
	ax = laddr ## BIT(&meml); \
	cpu->cb.io_write ## BIT(cpu->cb.io, lreg16(2), ax); \
	sreg ## ABIT(6, lreg ## ABIT(6) + dir);

#define OUTS_helper2(BIT, ABIT) \
	OptAddr memls; \
	uword cx = lreg ## ABIT(1); \
	while (cx) { \
		TRY(translate ## BIT(cpu, &memls, 1, curr_seg, lreg ## ABIT(6))); \
		if (memls.addr1 % (BIT / 8)) { \
			/* slow path */ \
			while (lreg ## ABIT(1)) { \
				ldsioutdx(BIT, ABIT) \
				sreg ## ABIT(1, lreg ## ABIT(1) - 1); \
			} \
			break; \
		} \
		uword count = cx; \
		int counts; \
		if (dir > 0) counts = (4096 - (memls.addr1 & 4095)) / (BIT / 8); \
		else counts = 1 + (memls.addr1 & 4095) / (BIT / 8); \
		if (counts < count) \
			count = counts; \
		if (cpu->cb.io_write_string && dir > 0 && \
		    (memls.addr1 | 4095) < cpu->phys_mem_size && \
		    !in_iomem(memls.addr1) && !in_iomem(memls.addr1 | 4095)) { \
			int count1 = cpu->cb.io_write_string( \
				cpu->cb.io, lreg16(2), \
				cpu->phys_mem + memls.addr1, dir, count); \
			if (count1 > 0) { \
				count = count1; \
				sreg ## ABIT(6, lreg ## ABIT(6) + count * dir); \
				sreg ## ABIT(1, cx - count); \
				cx = lreg ## ABIT(1); \
				continue; \
			} \
		} \
		for (uword i = 0; i <= count - 1; i++) { \
			ax = laddr ## BIT(&memls); \
			cpu->cb.io_write ## BIT(cpu->cb.io, lreg16(2), ax); \
			memls.addr1 += dir; \
		} \
		sreg ## ABIT(6, lreg ## ABIT(6) + count * dir); \
		sreg ## ABIT(1, cx - count); \
		cx = lreg ## ABIT(1); \
	}

#define OUTS_helper(BIT) \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	TRY(check_ioperm(cpu, lreg16(2), BIT)); \
	xdir ## BIT \
	u ## BIT ax; \
	if (rep == 0) { \
		if (adsz16) { ldsioutdx(BIT, 16) } else { ldsioutdx(BIT, 32) } \
	} else { \
		if (rep != 1) THROW0(EX_UD); \
		if (adsz16) { \
			OUTS_helper2(BIT, 16) \
		} else { \
			OUTS_helper2(BIT, 32) \
		} \
	}

#define OUTSb() OUTS_helper(8)
#define OUTS() if (opsz16) { OUTS_helper(16) } else { OUTS_helper(32) }

#define JCXZb(i, li, _) \
	sword d = sext8(li(i)); \
	if (adsz16) { \
		if (lreg16(1) == 0) cpu->next_ip += d; \
	} else { \
		if (lreg32(1) == 0) cpu->next_ip += d; \
	}

#define LOOPb(i, li, _) \
	sword d = sext8(li(i)); \
	if (adsz16) { \
		sreg16(1, lreg16(1) - 1); \
		if (lreg16(1)) cpu->next_ip += d; \
	} else { \
		sreg32(1, lreg32(1) - 1); \
		if (lreg32(1)) cpu->next_ip += d; \
	}

#define LOOPEb(i, li, _) \
	sword d = sext8(li(i)); \
	if (adsz16) { \
		sreg16(1, lreg16(1) - 1); \
		if (lreg16(1) && get_ZF(cpu)) cpu->next_ip += d; \
	} else { \
		sreg32(1, lreg32(1) - 1); \
		if (lreg32(1) && get_ZF(cpu)) cpu->next_ip += d; \
	}

#define LOOPNEb(i, li, _) \
	sword d = sext8(li(i)); \
	if (adsz16) { \
		sreg16(1, lreg16(1) - 1); \
		if (lreg16(1) && !get_ZF(cpu)) cpu->next_ip += d; \
	} else { \
		sreg32(1, lreg32(1) - 1); \
		if (lreg32(1) && !get_ZF(cpu)) cpu->next_ip += d; \
	}

#define COND() \
	int cond; \
	switch(b1 & 0xf) { \
	case 0x0: cond =  get_OF(cpu); break; \
	case 0x1: cond = !get_OF(cpu); break; \
	case 0x2: cond =  get_CF(cpu); break; \
	case 0x3: cond = !get_CF(cpu); break; \
	case 0x4: cond =  get_ZF(cpu); break; \
	case 0x5: cond = !get_ZF(cpu); break; \
	case 0x6: cond =  get_ZF(cpu) ||  get_CF(cpu); break; \
	case 0x7: cond = !get_ZF(cpu) && !get_CF(cpu); break; \
	case 0x8: cond =  get_SF(cpu); break; \
	case 0x9: cond = !get_SF(cpu); break; \
	case 0xa: cond =  get_PF(cpu); break; \
	case 0xb: cond = !get_PF(cpu); break; \
	case 0xc: cond =  get_SF(cpu) != get_OF(cpu); break; \
	case 0xd: cond =  get_SF(cpu) == get_OF(cpu); break; \
	case 0xe: cond =  get_ZF(cpu) || get_SF(cpu) != get_OF(cpu); break; \
	case 0xf: cond = !get_ZF(cpu) && get_SF(cpu) == get_OF(cpu); break; \
	}

#define JCC_common(d) \
	COND() \
	if (cond) cpu->next_ip += d;

#define SETCCb(a, la, sa) \
	COND() \
	sa(a, cond);

#define JCCb(i, li, _) \
	sword d = sext8(li(i)); \
	JCC_common(d)

#define JCCw(i, li, _) \
	sword d = sext16(li(i)); \
	JCC_common(d)

#define JCCd(i, li, _) \
	sword d = sext32(li(i)); \
	JCC_common(d)

#define JMPb(i, li, _) \
	sword d = sext8(li(i)); \
	cpu->next_ip += d;

#define JMPw(i, li, _) \
	sword d = sext16(li(i)); \
	cpu->next_ip += d;

#define JMPd(i, li, _) \
	sword d = sext32(li(i)); \
	cpu->next_ip += d;

#define JMPABSw(i, li, _) \
	cpu->next_ip = li(i);

#define JMPABSd(i, li, _) \
	cpu->next_ip = li(i);

#define JMPFAR(addr, seg) \
	if ((cpu->cr0 & 1) && !(cpu->flags & VM)) { \
		TRY(pmcall(cpu, opsz16, addr, seg, true)); \
	} else { \
	TRY(set_seg(cpu, SEG_CS, seg)); \
	cpu->next_ip = addr; \
	}

#define CALLFAR(addr, seg) \
	if ((cpu->cr0 & 1) && !(cpu->flags & VM)) { \
		TRY(pmcall(cpu, opsz16, addr, seg, false)); \
	} else { \
	OptAddr meml1, meml2; \
	uword sp = lreg32(4); \
	if (opsz16) { \
		TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask)); \
		TRY(translate16(cpu, &meml2, 2, SEG_SS, (sp - 4) & sp_mask)); \
		set_sp(sp - 4, sp_mask); \
		saddr16(&meml1, cpu->seg[SEG_CS].sel); \
		saddr16(&meml2, cpu->next_ip); \
	} else { \
		TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask)); \
		TRY(translate32(cpu, &meml2, 2, SEG_SS, (sp - 8) & sp_mask)); \
		set_sp(sp - 8, sp_mask); \
		saddr32(&meml1, cpu->seg[SEG_CS].sel); \
		saddr32(&meml2, cpu->next_ip); \
	} \
	TRY(set_seg(cpu, SEG_CS, seg)); \
	cpu->next_ip = addr; \
	}

#define CALLw(i, li, _) \
	sword d = sext16(li(i)); \
	uword sp = lreg32(4); \
	TRY(translate16(cpu, &meml, 2, SEG_SS, (sp - 2) & sp_mask)); \
	set_sp(sp - 2, sp_mask); \
	saddr16(&meml, cpu->next_ip); \
	cpu->next_ip += d;

#define CALLd(i, li, _) \
	sword d = sext32(li(i)); \
	uword sp = lreg32(4); \
	TRY(translate32(cpu, &meml, 2, SEG_SS, (sp - 4) & sp_mask)); \
	set_sp(sp - 4, sp_mask); \
	saddr32(&meml, cpu->next_ip); \
	cpu->next_ip += d;

#define CALLABSw(i, li, _) \
	uword nip = li(i); \
	uword sp = lreg32(4); \
	TRY(translate16(cpu, &meml, 2, SEG_SS, (sp - 2) & sp_mask)); \
	set_sp(sp - 2, sp_mask); \
	saddr16(&meml, cpu->next_ip); \
	cpu->next_ip = nip;

#define CALLABSd(i, li, _) \
	uword nip = li(i); \
	uword sp = lreg32(4); \
	TRY(translate32(cpu, &meml, 2, SEG_SS, (sp - 4) & sp_mask)); \
	set_sp(sp - 4, sp_mask); \
	saddr32(&meml, cpu->next_ip); \
	cpu->next_ip = nip;

#define RETw(i, li, _) \
	if (opsz16) { \
		uword sp = lreg32(4); \
		TRY(translate16(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		set_sp(sp + 2 + li(i), sp_mask); \
		cpu->next_ip = laddr16(&meml); \
	} else { \
		uword sp = lreg32(4); \
		TRY(translate32(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		set_sp(sp + 4 + li(i), sp_mask); \
		cpu->next_ip = laddr32(&meml); \
	}

#define RET() RETw(0, limm, 0)

static bool enter_helper(CPUI386 *cpu, bool opsz16, uword sp_mask,
			 int level, int allocsz)
{
	assert(level != 0);
	uword temp;
	OptAddr meml1;

	uword sp = lreg32(4);
	if (opsz16) {
		TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask));
		set_sp(sp - 2, sp_mask);
		saddr16(&meml1, lreg16(5));
		temp = lreg16(4);
	} else {
		TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask));
		set_sp(sp - 4, sp_mask);
		saddr32(&meml1, lreg32(5));
		temp = lreg32(4);
	}

	for (int i = 0; i < level - 1; i++) {
		if (opsz16) {
			if (sp_mask == 0xffff) {
				sreg16(5, lreg16(5) - 2);
			} else {
				sreg32(5, lreg32(5) - 2);
			}
			TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask));
			set_sp(sp - 2, sp_mask);
			saddr16(&meml1, lreg16(5));
		} else {
			if (sp_mask == 0xffff) {
				sreg16(5, lreg16(5) - 4);
			} else {
				sreg32(5, lreg32(5) - 4);
			}
			TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask));
			set_sp(sp - 4, sp_mask);
			saddr32(&meml1, lreg32(5));
		}
	}

	if (opsz16) {
		TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask));
		set_sp(sp - 2 - allocsz, sp_mask);
		saddr16(&meml1, temp);
		sreg16(5, temp);
	} else {
		TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask));
		set_sp(sp - 4 - allocsz, sp_mask);
		saddr32(&meml1, temp);
		sreg32(5, temp);
	}
	return true;
}

#define ENTER(i16, i8, l16, s16, l8, s8) \
	OptAddr meml1; \
	int level = l8(i8) % 32; \
	if (level == 0) { \
	uword sp = lreg32(4); \
	if (opsz16) { \
		TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask)); \
		set_sp(sp - 2 - l16(i16), sp_mask); \
		saddr16(&meml1, lreg16(5)); \
		sreg16(5, (sp - 2) & sp_mask); \
	} else { \
		TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask)); \
		set_sp(sp - 4 - l16(i16), sp_mask); \
		saddr32(&meml1, lreg32(5)); \
		sreg32(5, (sp - 4) & sp_mask); \
	} \
	} else { \
		TRY(enter_helper(cpu, opsz16, sp_mask, level, l16(i16))); \
	}

#define LEAVE() \
	uword sp = lreg32(5); \
	if (opsz16) { \
		TRY(translate16(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		set_sp(sp + 2, sp_mask); \
		sreg16(5, laddr16(&meml)); \
	} else { \
		TRY(translate32(cpu, &meml, 1, SEG_SS, sp & sp_mask)); \
		set_sp(sp + 4, sp_mask); \
		sreg32(5, laddr32(&meml)); \
	}

#define SXXX(addr) \
	OptAddr meml1, meml2; \
	TRY(translate16(cpu, &meml1, 2, curr_seg, addr)); \
	TRY(translate32(cpu, &meml2, 2, curr_seg, addr + 2)); \

#define SGDT(addr) \
	SXXX(addr) \
	store16(cpu, &meml1, cpu->gdt.limit); \
	store32(cpu, &meml2, cpu->gdt.base);

#define SIDT(addr) \
	SXXX(addr) \
	store16(cpu, &meml1, cpu->idt.limit); \
	store32(cpu, &meml2, cpu->idt.base);

#define LXXX(addr) \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	OptAddr meml1, meml2; \
	TRY(translate16(cpu, &meml1, 1, curr_seg, addr)); \
	TRY(translate32(cpu, &meml2, 1, curr_seg, addr + 2)); \
	u16 limit = load16(cpu, &meml1); \
	u32 base = load32(cpu, &meml2); \
	if (opsz16) base &= 0xffffff;

#define LGDT(addr) \
	LXXX(addr) \
	cpu->gdt.base = base; \
	cpu->gdt.limit = limit;

#define LIDT(addr) \
	LXXX(addr) \
	cpu->idt.base = base; \
	cpu->idt.limit = limit; \
	cpu->int8_cache_valid = false; \
	cpu->int8_warmup_counter = 0;  /* Reset INT 8 cache */

#define LLDT(a, la, sa) \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	TRY(set_seg(cpu, SEG_LDT, la(a)));

#define SLDT(a, la, sa) \
	sa(a, cpu->seg[SEG_LDT].sel);

#define LTR(a, la, sa) \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	{ \
		uword __ltr_sel = la(a) & 0xffff; \
		uword __ltr_w1, __ltr_w2; \
		bool __ltr_s, __ltr_p; \
		int __ltr_type; \
		if ((__ltr_sel & ~0x3) == 0) THROW(EX_GP, 0); \
		if (__ltr_sel & 0x4) THROW(EX_GP, __ltr_sel & ~0x3); \
		TRY(read_desc(cpu, __ltr_sel, &__ltr_w1, &__ltr_w2)); \
		__ltr_s = (__ltr_w2 >> 12) & 1; \
		__ltr_p = (__ltr_w2 >> 15) & 1; \
		__ltr_type = (__ltr_w2 >> 8) & 0xf; \
		/* LTR accepts available 16-bit TSS (type 1) or 32-bit TSS (type 9). */ \
		if (__ltr_s || (__ltr_type != 1 && __ltr_type != 9)) THROW(EX_GP, __ltr_sel & ~0x3); \
		if (!__ltr_p) THROW(EX_NP, __ltr_sel & ~0x3); \
		TRY(set_seg(cpu, SEG_TR, __ltr_sel)); \
		/* Mark descriptor busy (type 1->3 or 9->0xB). */ \
		{ \
			uword __ltr_desc = cpu->gdt.base + (__ltr_sel & ~0x7); \
			OptAddr __ltr_meml; \
			TRY(translate_laddr(cpu, &__ltr_meml, 3, __ltr_desc + 4, 4, 0)); \
			uword __ltr_hi = load32(cpu, &__ltr_meml); \
			__ltr_hi |= 0x200; \
			store32(cpu, &__ltr_meml, __ltr_hi); \
			cpu->seg[SEG_TR].flags |= 2; \
		} \
	}

#define STR(a, la, sa) \
	sa(a, cpu->seg[SEG_TR].sel);

#define MOVFD() \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int rm = modrm & 7; \
	sreg32(rm, cpu->dr[reg]);

#define MOVTD() \
	TRY(fetch8(cpu, &modrm)); \
	int reg = (modrm >> 3) & 7; \
	int rm = modrm & 7; \
	cpu->dr[reg] = lreg32(rm);

#define MOVFT() \
	TRY(fetch8(cpu, &modrm));
#define MOVTT() \
	TRY(fetch8(cpu, &modrm));

#define SMSW(addr, laddr, saddr) \
	saddr(addr, cpu->cr0 & 0xffff);

#define LMSW(addr, laddr, saddr) \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	{ u32 __lmsw_old = cpu->cr0; \
	u32 __lmsw_val = (cpu->cr0 & ((~0xf) | 1)) | (laddr(addr) & 0xf); \
	if ((__lmsw_val ^ cpu->cr0) & 1) { cpu->int8_cache_valid = false; cpu->int8_warmup_counter = 0; \
	tlb_clear(cpu); SEQ_INVALIDATE(cpu); } \
	if (unlikely(cpu_diag_enabled) && __lmsw_val != __lmsw_old) \
		cpu_diag_log_cr0_transition(cpu, __lmsw_old, __lmsw_val); \
	cpu->cr0 = __lmsw_val; }

#define LSEGd(NAME, reg, addr, lreg32, sreg32, laddr32, saddr32) \
	OptAddr meml1, meml2; \
	if (adsz16) addr = addr & 0xffff; \
	TRY(translate32(cpu, &meml1, 1, curr_seg, addr)); \
	TRY(translate16(cpu, &meml2, 1, curr_seg, addr + 4)); \
	u32 r = load32(cpu, &meml1); \
	u32 s = load16(cpu, &meml2); \
	TRY(set_seg(cpu, SEG_ ## NAME, s)); \
	sreg32(reg, r);

#define LSEGw(NAME, reg, addr, lreg16, sreg16, laddr16, saddr16) \
	OptAddr meml1, meml2; \
	if (adsz16) addr = addr & 0xffff; \
	TRY(translate16(cpu, &meml1, 1, curr_seg, addr)); \
	TRY(translate16(cpu, &meml2, 1, curr_seg, addr + 2)); \
	u32 r = load16(cpu, &meml1); \
	u32 s = load16(cpu, &meml2); \
	TRY(set_seg(cpu, SEG_ ## NAME, s)); \
	sreg16(reg, r);

#define LESd(...) LSEGd(ES, __VA_ARGS__)
#define LSSd(...) LSEGd(SS, __VA_ARGS__)
#define LDSd(...) LSEGd(DS, __VA_ARGS__)
#define LFSd(...) LSEGd(FS, __VA_ARGS__)
#define LGSd(...) LSEGd(GS, __VA_ARGS__)
#define LESw(...) LSEGw(ES, __VA_ARGS__)
#define LSSw(...) LSEGw(SS, __VA_ARGS__)
#define LDSw(...) LSEGw(DS, __VA_ARGS__)
#define LFSw(...) LSEGw(FS, __VA_ARGS__)
#define LGSw(...) LSEGw(GS, __VA_ARGS__)

static bool check_ioperm(CPUI386 *cpu, int port, int bit_width)
{
	/* Locked rule (Intel 80386 PRM + IA-32 SDM):
	 * - VM86: IN/OUT always consult the TSS I/O bitmap (independent of IOPL).
	 * - PM non-VM86: consult bitmap only when CPL > IOPL.
	 * References:
	 *   80386 PRM ch.8 (I/O protection): "In virtual 8086 mode, the processor
	 *   consults the map without regard for IOPL."
	 *   IA-32 SDM vol.1, I/O permission bitmap semantics (checks in VM86). */
	bool need_iobitmap = false;
	if (cpu->cr0 & 1) {
		int iopl = get_IOPL(cpu);
		if (cpu->flags & VM)
			need_iobitmap = true;
		else
			need_iobitmap = (cpu->cpl > iopl);
	}
	if (!need_iobitmap)
		return true;

	/* No usable 32-bit TSS I/O bitmap metadata -> deny. */
	{
		int tr_type = cpu->seg[SEG_TR].flags & 0xf;
		if (tr_type != 9 && tr_type != 11) {
			vm86_trip_push(cpu, VM86_TRIP_I, (uint8_t)bit_width,
				       cpu->seg[SEG_TR].sel, cpu->ip,
				       0xffff, (uint16_t)port, cpu->flags,
				       cpu->seg[SEG_CS].sel, 0xffffffffu, 0xffff,
				       ((uint32_t)VM86_TRIP_IO_BAD_TR << 16), NULL, 0);
			THROW(EX_GP, 0);
		}
	}
	if (cpu->seg[SEG_TR].limit < 103) {
		vm86_trip_push(cpu, VM86_TRIP_I, (uint8_t)bit_width,
			       cpu->seg[SEG_TR].sel, cpu->ip,
			       0xffff, (uint16_t)port, cpu->flags,
			       cpu->seg[SEG_CS].sel, cpu->seg[SEG_TR].limit, 0xffff,
			       ((uint32_t)VM86_TRIP_IO_BAD_TR_LIMIT << 16), NULL, 0);
		THROW(EX_GP, 0);
	}

	OptAddr meml;
	TRY(translate(cpu, &meml, 1, SEG_TR, 102, 2, 0));
	u32 iobase = load16(cpu, &meml);
	if (iobase > cpu->seg[SEG_TR].limit) {
		vm86_trip_push(cpu, VM86_TRIP_I, (uint8_t)bit_width,
			       cpu->seg[SEG_TR].sel, cpu->ip,
			       (uint16_t)iobase, (uint16_t)port, cpu->flags,
			       cpu->seg[SEG_CS].sel, iobase, 0xffff,
			       ((uint32_t)VM86_TRIP_IO_BAD_IOBASE << 16), NULL, 0);
		THROW(EX_GP, 0);
	}

	/* x86 I/O bitmap is indexed by I/O PORT bytes, not operand bits.
	 * IN/OUT 8/16/32 access 1/2/4 consecutive ports respectively. */
	int port_bytes = (bit_width + 7) >> 3;
	for (int b = 0; b < port_bytes; b++) {
		u32 p = (u32)port + (u32)b;
		u32 byte_off = iobase + (p >> 3);
		if (byte_off > cpu->seg[SEG_TR].limit) {
			vm86_trip_push(cpu, VM86_TRIP_I, (uint8_t)bit_width,
				       cpu->seg[SEG_TR].sel, cpu->ip,
				       (uint16_t)iobase, (uint16_t)port, cpu->flags,
				       cpu->seg[SEG_CS].sel, byte_off, 0xffff,
				       ((uint32_t)VM86_TRIP_IO_BAD_BYTE_OOR << 16), NULL, 0);
			THROW(EX_GP, 0);
		}
		TRY(translate(cpu, &meml, 1, SEG_TR, byte_off, 1, 0));
		u8 perm = load8(cpu, &meml);
		if (perm & (1u << (p & 7))) {
			vm86_trip_push(cpu, VM86_TRIP_I, (uint8_t)bit_width,
				       cpu->seg[SEG_TR].sel, cpu->ip,
				       (uint16_t)iobase, (uint16_t)port, cpu->flags,
				       cpu->seg[SEG_CS].sel, byte_off, 0xffff,
				       ((uint32_t)VM86_TRIP_IO_BAD_BIT << 16) |
				       ((uint32_t)perm << 8) | (p & 7), NULL, 0);
			THROW(EX_GP, 0);
		}
	}
	return true;
}

/* V86 Interrupt Redirection Bitmap check (Intel SDM Vol 3, Section 20.3.3).
 * With IOPL=3 in V86 mode, the IRB in the TSS determines per-vector whether
 * INT n goes through the real-mode IVT (bit=0) or causes #GP (bit=1).
 * Returns true if INT should proceed through IVT, false/#GP if redirected. */
static bool check_v86_int_redirect(CPUI386 *cpu, int intno)
{
	(void)cpu;
	(void)intno;
	/* 80386 target behavior: VM86 INT n with IOPL=3 uses real-mode IVT.
	 * The Interrupt Redirection Bitmap is a VME-era mechanism and should
	 * not alter INT dispatch in this emulator's 386 mode. */
	return true;
}

#define INb(a, b, la, sa, lb, sb) \
	int port = lb(b); \
	TRY(check_ioperm(cpu, port, 8)); \
	sa(a, cpu->cb.io_read8(cpu->cb.io, port));

#define INw(a, b, la, sa, lb, sb) \
	int port = lb(b); \
	TRY(check_ioperm(cpu, port, 16)); \
	sa(a, cpu->cb.io_read16(cpu->cb.io, port));

#define INd(a, b, la, sa, lb, sb) \
	int port = lb(b); \
	TRY(check_ioperm(cpu, port, 32)); \
	sa(a, cpu->cb.io_read32(cpu->cb.io, port));

#define OUTb(a, b, la, sa, lb, sb) \
	int port = la(a); \
	TRY(check_ioperm(cpu, port, 8)); \
	cpu->cb.io_write8(cpu->cb.io, port, lb(b));

#define OUTw(a, b, la, sa, lb, sb) \
	int port = la(a); \
	TRY(check_ioperm(cpu, port, 16)); \
	cpu->cb.io_write16(cpu->cb.io, port, lb(b));

#define OUTd(a, b, la, sa, lb, sb) \
	int port = la(a); \
	TRY(check_ioperm(cpu, port, 32)); \
	cpu->cb.io_write32(cpu->cb.io, port, lb(b));

#define CLTS() \
	cpu->cr0 &= ~(1 << 3);

#define ESC() \
	if (cpu->cr0 & 0xc) THROW0(EX_NM); \
	else { \
		TRY(fetch8(cpu, &modrm)); \
		int mod = modrm >> 6; \
		int rm = modrm & 7; \
		int op = b1 - 0xd8; \
		int group = (modrm >> 3) & 7; \
		if (mod != 3) { \
			TRY(modsib(cpu, adsz16, mod, rm, &addr, &curr_seg)); \
			if (cpu->fpu) { \
				TRY(fpu_exec2(cpu->fpu, cpu, opsz16, op, group, curr_seg, addr)); \
				cpu->tsc += 3;  /* FPU memory operations ~3-10 cycles */ \
			} \
		} else { \
			int reg = modrm & 7; \
			if (cpu->fpu) { \
				TRY(fpu_exec1(cpu->fpu, cpu, op, group, reg)); \
				cpu->tsc += 2;  /* FPU register operations ~2-8 cycles */ \
			} \
		} \
	}

#define WAIT() \
	if ((cpu->cr0 & 0xa) == 0xa) THROW0(EX_NM);

// ...
#define AAD(i, li, _) \
	u8 al = lreg8(0); \
	u8 ah = lreg8(4); \
	u8 imm = li(i); \
	u8 res = al + ah * imm; \
	sreg8(0, res); \
	sreg8(4, 0); \
	cpu->flags &= ~(OF | AF | CF); /* undocumented */ \
	cpu->cc.dst = sext8(res); \
	cpu->cc.mask = ZF | SF | PF;

#define AAM(i, li, _) \
	u8 al = lreg8(0); \
	u8 imm = li(i); \
	u8 res = al % imm; \
	sreg8(4, al / imm); \
	sreg8(0, res); \
	cpu->flags &= ~(OF | AF | CF); /* undocumented */ \
	cpu->cc.dst = sext8(res); \
	cpu->cc.mask = ZF | SF | PF;

#define SALC() \
	if (get_CF(cpu)) sreg8(0, 0xff); else sreg8(0, 0x00);

#define XLAT() \
	if (curr_seg == -1) curr_seg = SEG_DS; \
	if (adsz16) { \
		addr = lreg16(3) + lreg8(0); \
		addr &= 0xffff; \
		TRY(translate8(cpu, &meml, 1, curr_seg, addr)); \
		sreg8(0, laddr8(&meml)); \
	} else { \
		addr = lreg32(3) + lreg8(0); \
		TRY(translate8(cpu, &meml, 1, curr_seg, addr)); \
		sreg8(0, laddr8(&meml)); \
	}

#define DAA() \
	u8 al = lreg8(0); \
	int cf = get_CF(cpu); \
	cpu->flags &= ~CF; \
	if ((al & 0xf) > 9 || get_AF(cpu)) { \
		sreg8(0, al + 6); \
		if (cf || al > 0xff - 6) cpu->flags |= CF; \
		cpu->flags |= AF; \
	} else { \
		cpu->flags &= ~AF; \
	} \
	if (al > 0x99 || cf) { \
		sreg8(0, lreg8(0) + 0x60); \
		cpu->flags |= CF; \
	} \
	cpu->cc.dst = sext8(lreg8(0)); \
	cpu->cc.mask = ZF | SF | PF;

#define DAS() \
	u8 al = lreg8(0); \
	int cf = get_CF(cpu); \
	cpu->flags &= ~CF; \
	if ((al & 0xf) > 9 || get_AF(cpu)) { \
		sreg8(0, al - 6); \
		if (cf || al < 6) cpu->flags |= CF; \
		cpu->flags |= AF; \
	} else { \
		cpu->flags &= ~AF; \
	} \
	if (al > 0x99 || cf) { \
		sreg8(0, lreg8(0) - 0x60); \
		cpu->flags |= CF; \
	} \
	cpu->cc.dst = sext8(lreg8(0)); \
	cpu->cc.mask = ZF | SF | PF;

#define AAA() \
	if ((lreg8(0) & 0xf) > 9 || get_AF(cpu)) { \
		sreg16(0, lreg16(0) + 0x106); \
		cpu->flags |= AF | CF; \
	} else { \
		cpu->flags &= ~(AF | CF); \
	} \
	cpu->cc.mask = ZF | SF | PF; \
	sreg8(0, lreg8(0) & 0xf);

#define AAS() \
	if ((lreg8(0) & 0xf) > 9 || get_AF(cpu)) { \
		sreg16(0, lreg16(0) - 6); \
		sreg8(4, lreg8(4) - 1); \
		cpu->flags |= AF | CF; \
	} else { \
		cpu->flags &= ~(AF | CF); \
	} \
	cpu->cc.mask = ZF | SF | PF; \
	sreg8(0, lreg8(0) & 0xf);

static bool larsl_helper(CPUI386 *cpu, int sel, uword *ar, uword *sl, int *zf)
{
	sel = sel & 0xffff;

	if (!(cpu->cr0 & 1) || (cpu->flags & VM))
		THROW0(EX_UD);

	if ((sel & ~0x3) == 0) {
		*zf = 0;
		return true;
	}

	uword w1, w2;
	if (!read_desc(cpu, sel, &w1, &w2)) {
		*zf = 0;
		return true;
	}

	int dpl = (w2 >> 13) & 0x3;
	int rpl = sel & 0x3;
	int maxpl = cpu->cpl > rpl ? cpu->cpl : rpl;
	if ((w2 >> 12) & 1) {
		bool code = (w2 >> 11) & 1;
		bool conforming = code && ((w2 >> 10) & 1);
		if ((!code || !conforming) && dpl < maxpl) {
			*zf = 0;
			return true;
		}
	} else {
		int type = (w2 >> 8) & 0xf;
		if (ar) {
			switch (type) {
			case 0: case 6: case 7: case 8: case 10:
			case 13: case 14: case 15:
				*zf = 0;
				return true;
			}
		}
		if (sl) {
			switch (type) {
			case 0: case 4: case 5: case 6: case 7: case 8:
			case 10: case 12: case 13: case 14: case 15:
				*zf = 0;
				return true;
			}
		}
		if (dpl < maxpl) {
			*zf = 0;
			return true;
		}
	}

	if (ar)
		*ar = w2 & 0x00ffff00;
	if (sl) {
		*sl = (w2 & 0xf0000) | (w1 & 0xffff);
		if (w2 & 0x00800000)
			*sl = (*sl << 12) | 0xfff;
	}

	*zf = 1;
	return true;
}

static bool verrw_helper(CPUI386 *cpu, int sel, int wr, int *zf)
{
	sel = sel & 0xffff;

	if (!(cpu->cr0 & 1) || (cpu->flags & VM))
		THROW0(EX_UD);

	if ((sel & ~0x3) == 0) {
		*zf = 0;
		return true;
	}

	uword w1, w2;
	if (!read_desc(cpu, sel, &w1, &w2)) {
		*zf = 0;
		return true;
	}

	if (((w2 >> 12) & 0x1) == 0) {
		*zf = 0;
		return true;
	}

	int dpl = (w2 >> 13) & 0x3;
	int rpl = sel & 0x3;
	int maxpl = cpu->cpl > rpl ? cpu->cpl : rpl;
	bool code = (w2 >> 11) & 0x1;
	bool rw = (w2 >> 9) & 0x1;
	bool conforming = code && ((w2 >> 10) & 0x1);
	if ((!code || !conforming) && dpl < maxpl) {
		*zf = 0;
		return true;
	}

	if (!code) {
		/* Data segments are always readable; VERW additionally requires writable. */
		if (wr && !rw) {
			*zf = 0;
			return true;
		}
	} else {
		/* Code segments are never writable for VERW. */
		if (wr || !rw) {
			*zf = 0;
			return true;
		}
	}

	*zf = 1;
	return true;
}

#define LARdw(a, b, la, sa, lb, sb) \
	uword res; \
	int zf; \
	TRY(larsl_helper(cpu, lb(b), &res, NULL, &zf)); \
	if (zf) { \
		sa(a, res); \
		cpu->flags |= ZF; \
	} else { \
		cpu->flags &= ~ZF; \
	} \
	cpu->cc.mask &= ~ZF;
#define LARww LARdw

#define LSLdw(a, b, la, sa, lb, sb) \
	uword res; \
	int zf; \
	TRY(larsl_helper(cpu, lb(b), NULL, &res, &zf)); \
	if (zf) { \
		sa(a, res); \
		cpu->flags |= ZF; \
	} else { \
		cpu->flags &= ~ZF; \
	} \
	cpu->cc.mask &= ~ZF;
#define LSLww LSLdw

#define VERR(a, la, sa) \
	int zf; \
	TRY(verrw_helper(cpu, la(a), 0, &zf)); \
	cpu->cc.mask &= ~ZF; \
	SET_BIT(cpu->flags, zf, ZF);

#define VERW(a, la, sa) \
	int zf; \
	TRY(verrw_helper(cpu, la(a), 1, &zf)); \
	cpu->cc.mask &= ~ZF; \
	SET_BIT(cpu->flags, zf, ZF);

#define ARPL(a, b, la, sa, lb, sb) \
	if (!(cpu->cr0 & 1) || (cpu->flags & VM)) THROW0(EX_UD); \
	u16 dst = la(a); \
	u16 src = lb(b); \
	if ((dst & 3) < (src & 3)) { \
		cpu->flags |= ZF; \
		sa(a, ((dst & ~3) | (src & 3))); \
	} else { \
		cpu->flags &= ~ZF; \
	} \
	cpu->cc.mask &= ~ZF;

#define GvMa GvMp
#define BOUND_helper(BIT, a, b, la, sa, lb, sb) \
	OptAddr meml1, meml2; \
	s ## BIT idx = la(a); \
	uword addr1 = lb(b); \
	TRY(translate ## BIT(cpu, &meml1, 1, curr_seg, addr1)); \
	TRY(translate ## BIT(cpu, &meml2, 1, curr_seg, addr1 + BIT / 8)); \
	s ## BIT lo = load ## BIT(cpu, &meml1); \
	s ## BIT hi = load ## BIT(cpu, &meml2); \
	if (idx < lo || idx > hi) { \
		static int bound_log_count = 0; \
		if (bound_log_count < 5) { \
			dolog("BOUND%d fail: idx=%d lo=%d hi=%d reg=%d addr=%08x CS:IP=%04x:%04x\n", \
				BIT, (int)idx, (int)lo, (int)hi, a, (unsigned)addr1, \
				cpu->seg[SEG_CS].sel, cpu->ip & 0xffff); \
			bound_log_count++; \
		} \
		THROW0(EX_BR); \
	}
#define BOUNDw(...) BOUND_helper(16, __VA_ARGS__)
#define BOUNDd(...) BOUND_helper(32, __VA_ARGS__)

// 486...
#define CMPXCH_helper(BIT, a, b, la, sa, lb, sb) \
	cpu->cc.src2 = sext ## BIT(la(a)); \
	cpu->cc.src1 = sext ## BIT(lreg ## BIT(0)); \
	cpu->cc.dst = sext ## BIT(cpu->cc.src1 - cpu->cc.src2); \
	cpu->cc.op = CC_SUB; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	if (cpu->cc.dst == 0) sa(a, lb(b)); else sreg ## BIT(0, cpu->cc.src2);

#define XADD_helper(BIT, a, b, la, sa, lb, sb) \
	u ## BIT dst = la(a); \
	cpu->cc.src1 = sext ## BIT(la(a)); \
	cpu->cc.src2 = sext ## BIT(lb(b)); \
	cpu->cc.dst = sext ## BIT(cpu->cc.src1 + cpu->cc.src2); \
	cpu->cc.op = CC_ADD; \
	cpu->cc.mask = CF | PF | AF | ZF | SF | OF; \
	sb(b, dst); \
	sa(a, cpu->cc.dst);

#define CMPXCHb(...) do { if (cpu->gen < 4) THROW0(EX_UD); CMPXCH_helper(8, __VA_ARGS__); } while (0);
#define CMPXCHw(...) do { if (cpu->gen < 4) THROW0(EX_UD); CMPXCH_helper(16, __VA_ARGS__); } while (0);
#define CMPXCHd(...) do { if (cpu->gen < 4) THROW0(EX_UD); CMPXCH_helper(32, __VA_ARGS__); } while (0);
#define XADDb(...) do { if (cpu->gen < 4) THROW0(EX_UD); XADD_helper(8, __VA_ARGS__); } while (0);
#define XADDw(...) do { if (cpu->gen < 4) THROW0(EX_UD); XADD_helper(16, __VA_ARGS__); } while (0);
#define XADDd(...) do { if (cpu->gen < 4) THROW0(EX_UD); XADD_helper(32, __VA_ARGS__); } while (0);

#define INVLPG(addr) do { if (cpu->gen < 4) THROW0(EX_UD); \
	{ int __seg = (curr_seg == -1) ? SEG_DS : curr_seg; \
	  uword __inv_laddr = cpu->seg[__seg].base + (addr); \
	  uword __inv_lpgno = __inv_laddr >> 12; \
	  struct tlb_entry *__inv_ent = &(cpu->tlb.tab[__inv_lpgno & tlb_mask]); \
	  if (__inv_ent->lpgno == __inv_lpgno) __inv_ent->lpgno = -1; \
	  if ((cpu->ifetch.laddr >> 12) == __inv_lpgno) cpu->ifetch.laddr = -1; } \
	SEQ_INVALIDATE(cpu); \
	if (unlikely(cpu_diag_enabled && pf_diag.active)) pf_diag.invlpg_count++; } while(0);

#define BSWAPw(a, la, sa) THROW0(EX_UD);

#define BSWAPd(a, la, sa) \
	if (cpu->gen < 4) THROW0(EX_UD); \
	u32 src = la(a); \
	u32 dst = ((src & 0xff) << 24) | (((src >> 8) & 0xff) << 16) | (((src >> 16) & 0xff) << 8) | ((src >> 24) & 0xff); \
	sa(a, dst);

#define INVD() do { if (cpu->gen < 4) THROW0(EX_UD); } while (0)    // NOP for emulator
#define WBINVD() do { if (cpu->gen < 4) THROW0(EX_UD); } while (0)  // NOP for emulator

// 586 and later...
#define UD0() THROW0(EX_UD);
#define UD2() THROW0(EX_UD);  // Explicit undefined instruction

#if defined(I386_ENABLE_SSE)
#define CPUID_SIMD_FEATURE 0x3800000
#elif defined(I386_ENABLE_MMX)
#define CPUID_SIMD_FEATURE 0x800000
#else
#define CPUID_SIMD_FEATURE 0x0
#endif

/* CPUID model/stepping table per CPU generation
 * Format: stepping | (model << 4) | (family << 8) | (type << 12)
 * Type 0 = OEM, 1 = OverDrive, 2 = Dual
 */
static const u32 cpuid_signature[] = {
	[3] = 0x0303,  /* 386DX stepping C (family 3, model 0, stepping 3) */
	[4] = 0x0414,  /* 486DX-33 (family 4, model 1, stepping 4) */
	[5] = 0x0543,  /* Pentium MMX (family 5, model 4, stepping 3) */
	[6] = 0x0686,  /* Pentium III Coppermine (family 6, model 8, stepping 6) */
};

#define CPUID() \
	if (cpu->gen < 5) THROW0(EX_UD); \
	switch (REGi(0)) { \
	case 0: \
		REGi(0) = 1; \
		/* "TINY 386 CPU" — non-Intel vendor avoids Win95 Intel-specific code paths \
		 * that assume MSR/TSC/Pentium errata handling we don't fully implement. \
		 * "GenuineIntel" triggers Win95 RDTSC-based calibration that fails with \
		 * our emulated cycle-count TSC. */ \
		REGi(3) = 0x594e4954;  /* "TINY" */ \
		REGi(2) = 0x20363833;  /* " 386" */ \
		REGi(1) = 0x20555043;  /* " CPU" */ \
		break; \
	case 1: { \
		int gen = cpu->gen < 3 ? 3 : (cpu->gen > 6 ? 6 : cpu->gen); \
		REGi(0) = cpuid_signature[gen]; \
		REGi(3) = 0; \
		/* EDX feature flags */ \
		REGi(2) = 0x100;  /* CMPXCHG8B */ \
		if (cpu->fpu) REGi(2) |= 1;  /* FPU on-chip */ \
		if (cpu->gen > 5) REGi(2) |= 0x8830;  /* RDTSC, CMOV, FXSR, FXSAVE */ \
		if (cpu->gen > 5 && cpu->fpu) REGi(2) |= CPUID_SIMD_FEATURE; \
		REGi(1) = 0; \
		break; \
	} \
	default: \
		REGi(0) = 0; \
		REGi(3) = 0; \
		REGi(2) = 0; \
		REGi(1) = 0; \
		break; \
	}

/* RDTSC returns emulated cycle count, scaled to approximate real timing.
 * This provides more realistic timing for DOS-era benchmarks that use RDTSC.
 */
#define RDTSC() \
	if (cpu->gen < 5) THROW0(EX_UD); \
	/* Lazy-sync TSC from cycle counter (avoids 64-bit increment per instruction) */ \
	cpu->tsc += (uint32_t)((uint32_t)cpu->cycle - (uint32_t)cpu->tsc_sync_cycle); \
	cpu->tsc_sync_cycle = cpu->cycle; \
	REGi(0) = (u32)cpu->tsc; \
	REGi(2) = (u32)(cpu->tsc >> 32);

#define Mq Ms
#define CMPXCH8B(addr) \
	if (cpu->gen < 5) THROW0(EX_UD); \
	OptAddr meml1, meml2; \
	TRY(translate32(cpu, &meml1, 3, curr_seg, addr)); \
	TRY(translate32(cpu, &meml2, 3, curr_seg, addr + 4)); \
	uword lo = load32(cpu, &meml1); \
	uword hi = load32(cpu, &meml2); \
	if (REGi(0) == lo && REGi(2) == hi) { \
		cpu->flags |= ZF; \
		store32(cpu, &meml1, REGi(3)); \
		store32(cpu, &meml2, REGi(1)); \
	} else { \
		cpu->flags &= ~ZF; \
		REGi(0) = lo; \
		REGi(2) = hi; \
	} \
	cpu->cc.mask &= ~ZF;

#define CMOVw(a, b, la, sa, lb, sb) \
	if (cpu->gen < 6) THROW0(EX_UD); \
	COND() \
	if (cond) sa(a, lb(b));

#define CMOVd(a, b, la, sa, lb, sb) \
	if (cpu->gen < 6) THROW0(EX_UD); \
	COND() \
	if (cond) sa(a, lb(b));

#define WRMSR() \
	if (cpu->gen < 5) THROW0(EX_UD); \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	switch (REGi(1)) { \
	case 0x174: cpu->sysenter.cs = REGi(0); break; \
	case 0x176: cpu->sysenter.eip = REGi(0); break; \
	case 0x175: cpu->sysenter.esp = REGi(0); break; \
	default: cpu_debug(cpu); THROW(EX_GP, 0); \
	}

#define RDMSR() \
	if (cpu->gen < 5) THROW0(EX_UD); \
	if (cpu->cpl != 0) THROW(EX_GP, 0); \
	switch (REGi(1)) { \
	case 0x174: REGi(0) = cpu->sysenter.cs; REGi(2) = 0; break; \
	case 0x176: REGi(0) = cpu->sysenter.eip; REGi(2) = 0; break; \
	case 0x175: REGi(0) = cpu->sysenter.esp; REGi(2) = 0; break; \
	default: cpu_debug(cpu); THROW(EX_GP, 0); \
	}

static void __sysenter(CPUI386 *cpu, int pl, int cs)
{
	cpu->seg[SEG_CS].sel = (cs & 0xfffc) | pl;
	cpu->seg[SEG_CS].base = 0;
	cpu->seg[SEG_CS].limit = 0xffffffff;
	cpu->seg[SEG_CS].flags = SEG_D_BIT | 0x5b | (pl << 5);
	cpu->cpl = pl;
	cpu->code16 = false;
	SEQ_INVALIDATE(cpu);
	cpu->sp_mask = 0xffffffff;
	cpu->seg[SEG_SS].sel = ((cs + 8) & 0xfffc) | pl;
	cpu->seg[SEG_SS].base = 0;
	cpu->seg[SEG_SS].limit = 0xffffffff;
	cpu->seg[SEG_SS].flags = SEG_B_BIT | 0x53 | (pl << 5);
}

#define SYSENTER() \
	if (cpu->gen < 6) THROW0(EX_UD); \
	if (!(cpu->cr0 & 1) || (cpu->sysenter.cs & ~0x3) == 0) THROW(EX_GP, 0); \
	cpu->flags &= ~(VM | IF); \
	__sysenter(cpu, 0, cpu->sysenter.cs); \
	REGi(4) = cpu->sysenter.esp; \
	cpu->next_ip = cpu->sysenter.eip;

#define SYSEXIT() \
	if (cpu->gen < 6) THROW0(EX_UD); \
	if (!(cpu->cr0 & 1) || (cpu->sysenter.cs & ~0x3) == 0 || cpu->cpl) THROW(EX_GP, 0); \
	__sysenter(cpu, 3, cpu->sysenter.cs + 16); \
	REGi(4) = REGi(1); \
	cpu->next_ip = REGi(2);

#if defined(I386_ENABLE_MMX) || defined(I386_ENABLE_SSE)
#define SIMD_i386_c
#include "simd.inc"
#undef SIMD_i386_c
#endif

static bool pmcall(CPUI386 *cpu, bool opsz16, uword addr, int sel, bool isjmp);
static bool IRAM_ATTR pmret(CPUI386 *cpu, bool opsz16, int off, bool isiret);

static bool verbose;

#define ARGCOUNT_IMPL(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, ...) _17
#define ARGCOUNT(...) ARGCOUNT_IMPL(~, ## __VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define PASTE0(a, b) a ## b
#define PASTE(a, b) PASTE0(a, b)
#define C_1(_1)      CX(_1)
#define C_2(_1, ...) CX(_1) C_1(__VA_ARGS__)
#define C_3(_1, ...) CX(_1) C_2(__VA_ARGS__)
#define C_4(_1, ...) CX(_1) C_3(__VA_ARGS__)
#define C_5(_1, ...) CX(_1) C_4(__VA_ARGS__)
#define C_6(_1, ...) CX(_1) C_5(__VA_ARGS__)
#define C_7(_1, ...) CX(_1) C_6(__VA_ARGS__)
#define C_8(_1, ...) CX(_1) C_7(__VA_ARGS__)
#define C_9(_1, ...) CX(_1) C_8(__VA_ARGS__)
#define C_10(_1, ...) CX(_1) C_9(__VA_ARGS__)
#define C_11(_1, ...) CX(_1) C_10(__VA_ARGS__)
#define C_12(_1, ...) CX(_1) C_11(__VA_ARGS__)
#define C_13(_1, ...) CX(_1) C_12(__VA_ARGS__)
#define C_14(_1, ...) CX(_1) C_13(__VA_ARGS__)
#define C_15(_1, ...) CX(_1) C_14(__VA_ARGS__)
#define C_16(_1, ...) CX(_1) C_15(__VA_ARGS__)
#define C(...) PASTE(C_, ARGCOUNT(__VA_ARGS__))(__VA_ARGS__)

static bool IRAM_ATTR_CPU_EXEC1 cpu_exec1(CPUI386 *cpu, int stepcount)
{
#ifndef I386_OPT2
#define eswitch(b) switch(b)
#define ecase(a)   case a
#define ebreak     do { if (unlikely(insn_tf)) cpu->tf_trap_pending = true; } while(0); break
#define edefault   default
#define default_ud cpu_debug(cpu); THROW0(EX_UD)
#undef CX
#define CX(_1) case _1:
#else
#define eswitch(b)
#define ecase(a)   f ## a
#define ebreak     do { if (unlikely(insn_tf)) cpu->tf_trap_pending = true; } while(0); continue
#define edefault   f0xf1
#define default_ud THROW0(EX_UD)
#undef CX
#define CX(_1) f ## _1:
#endif

	u8 b1;
	u8 modrm;
	OptAddr meml;
	uword addr;
	/* seq_active persists across cpu_exec1 calls:
	 * ip check at iteration top handles cross-call correctness */
	for (; stepcount > 0; stepcount--) {
	/* TF single-step: #DB trap fires after the previous instruction completed.
	 * Must check BEFORE the next instruction starts so ip points to next insn. */
	if (unlikely(cpu->tf_trap_pending)) {
		cpu->tf_trap_pending = false;
		cpu->ip = cpu->next_ip;
		THROW0(EX_DB);
	}

	bool code16 = cpu->code16;
	uword sp_mask = cpu->sp_mask;

	if (code16) cpu->next_ip &= 0xffff;
	cpu->ip = cpu->next_ip;
	if (unlikely(wfw_monitor.ud63_fe95_0127.trace_active))
		wfw_ud63_trace_step(cpu);

	/* One-shot DPMI trace: capture INT 2Fh/1687h return and entry point call */
	if (unlikely(dpmi_trace.ret_active) && (cpu->flags & VM) &&
	    cpu->seg[SEG_CS].sel == dpmi_trace.ret_cs &&
	    (cpu->ip & 0xffff) == dpmi_trace.ret_ip) {
		dpmi_trace.ret_active = false;
		dpmi_trace.returned = true;
		dpmi_trace.ret_ax = lreg16(0);
		dpmi_trace.ret_bx = lreg16(3);
		dpmi_trace.ret_dx = lreg16(2);
		dpmi_trace.ret_si = lreg16(6);
		dpmi_trace.ret_es = cpu->seg[SEG_ES].sel;
		dpmi_trace.ret_di = lreg16(7);
		if (dpmi_trace.ret_ax == 0) {
			dpmi_trace.entry_cs = dpmi_trace.ret_es;
			dpmi_trace.entry_ip = dpmi_trace.ret_di;
			dpmi_trace.entry_active = true;
		}
	}
	if (unlikely(dpmi_trace.entry_active) && (cpu->flags & VM) &&
	    cpu->seg[SEG_CS].sel == dpmi_trace.entry_cs &&
	    (cpu->ip & 0xffff) == dpmi_trace.entry_ip) {
		dpmi_trace.entry_active = false;
		dpmi_trace.entry_called = true;
		dpmi_trace.entry_ax = lreg16(0);
		dpmi_trace.entry_es = cpu->seg[SEG_ES].sel;
	}

	/* Save TF before this instruction — POPF/IRET may change it, but
	 * the trap fires based on TF state when the instruction started. */
	bool insn_tf = !!(cpu->flags & TF);

	/* Initialize prefix state early (before seq dispatch which skips prefix parsing) */
	bool opsz16 = code16;
	bool adsz16 = code16;
	int rep = 0;
	/*bool lock = false;*/
	int curr_seg = -1;

#ifdef I386_SEQ_FASTPATH
	/* Sequential fast path: if the previous instruction was on the same page
	 * and execution is sequential (ip advanced by exactly pf_pos bytes),
	 * derive physical address directly without TLB lookup. */
	/* VM86 correctness first: disable sequential prefetch shortcut in VM mode.
	 * VM86 frequently transitions through fault/reflect paths, and we prefer
	 * conservative fetch behavior over speculative carry-over here. */
	if (likely(cpu->seq_active) && likely(!(cpu->flags & VM))) {
		uword prev_len = cpu->pf_pos;
		if (likely(cpu->ip == cpu->seq_prev_ip + prev_len)) {
			uword nphys = cpu->seq_phys + prev_len;
			/* Same-page check: new offset must be in [1, 0xFF7] to avoid
			 * page boundary crossing (0 = wrapped, >= 0xFF8 = near end) */
			unsigned page_off = nphys & 0xFFF;
			if (likely(page_off != 0 && page_off < 0xFF8)) {
				/* SEQ HIT — skip TLB lookup, set up prefetch directly */
				cpu->seq_hits++;
				cpu->pf_ptr = cpu->phys_mem + nphys;
				b1 = cpu->pf_ptr[0];
				cpu->pf_pos = 1;
				cpu->pf_avail = 8;
				cpu->next_ip++;
				cpu->cycle++;
				cpu->seq_phys = nphys;
				cpu->seq_prev_ip = cpu->ip;
				goto seq_dispatch;
			}
		}
		cpu->seq_active = false;
	}
#endif

	/* Fused prefetch + opcode fetch: set prefetch pointer and read first byte
	 * in one step, avoiding the redundant pf_pos check in fetch8. */
	{
		uword pf_laddr = likely(cpu->all_segs_flat)
			? cpu->next_ip
			: cpu->seg[SEG_CS].base + cpu->next_ip;
		if (likely((pf_laddr ^ cpu->ifetch.laddr) < (4096 - 7))) {
			cpu->pf_ptr = cpu->phys_mem + (cpu->ifetch.xaddr ^ pf_laddr);
			b1 = cpu->pf_ptr[0];
#ifdef I386_SEQ_FASTPATH
			/* Keep seq fastpath disabled in VM86 for correctness. */
			if (likely(!(cpu->flags & VM))) {
				/* Start sequential tracking from this TLB-validated address */
				cpu->seq_phys = cpu->ifetch.xaddr ^ pf_laddr;
				cpu->seq_prev_ip = cpu->ip;
				cpu->seq_active = true;
			} else {
				cpu->seq_active = false;
			}
#endif
			cpu->pf_pos = 1;
			cpu->pf_avail = 8;
			cpu->next_ip++;
		} else {
			cpu->pf_avail = 0;
			cpu->pf_pos = 0;
#ifdef I386_SEQ_FASTPATH
			cpu->seq_active = false;
#endif
			TRY(fetch8(cpu, &b1));
		}
	}
	cpu->cycle++;
#ifdef I386_SEQ_FASTPATH
	seq_dispatch: ;
#endif

#ifndef I386_OPT1
	if (verbose) {
		cpu_debug(cpu);
	}
#endif
	// prefix
#ifndef I386_OPT2
	for (;;) {
#define HANDLE_PREFIX(C, STMT) \
		if (b1 == C) { \
			STMT; \
			TRY(fetch8(cpu, &b1)); \
			continue; \
		}
		HANDLE_PREFIX(0x26, curr_seg = SEG_ES)
		HANDLE_PREFIX(0x2e, curr_seg = SEG_CS)
		HANDLE_PREFIX(0x36, curr_seg = SEG_SS)
		HANDLE_PREFIX(0x3e, curr_seg = SEG_DS)
		HANDLE_PREFIX(0x64, curr_seg = SEG_FS)
		HANDLE_PREFIX(0x65, curr_seg = SEG_GS)
		HANDLE_PREFIX(0x66, opsz16 = !code16)
		HANDLE_PREFIX(0x67, adsz16 = !code16)
		HANDLE_PREFIX(0xf3, rep = 1) // REP
		HANDLE_PREFIX(0xf2, rep = 2) // REPNE
		HANDLE_PREFIX(0xf0, /*lock = true*/)
#undef HANDLE_PREFIX
		break;
	}
#else

	/* CMP+Jcc / TEST+Jcc macro-op fusion: after CMP or TEST sets cc fields,
	 * peek at the next byte(s). If it's a short Jcc (0x70-0x7F) or near Jcc
	 * (0F 80-8F), evaluate the branch condition directly from cc.src1/src2/dst
	 * without a full dispatch cycle or get_*F() calls. PF conditions (0xA/0xB)
	 * bail out since parity computation isn't worth inlining. */

/* Evaluate CC_SUB condition: _ci is condition index (0x0-0xF), result in _cond */
#define EVAL_COND_SUB(_s1, _s2, _d, _ci, _cond) do { \
	int _cf = (u32)(_s1) < (u32)(_s2); \
	int _zf = ((_d) == 0); \
	int _sf = ((sword)(_d) < 0); \
	int _of = ((sword)(((_s1) ^ (_s2)) & ((_d) ^ (_s1))) < 0); \
	switch (_ci) { \
	case 0x0: _cond =  _of; break; \
	case 0x1: _cond = !_of; break; \
	case 0x2: _cond =  _cf; break; \
	case 0x3: _cond = !_cf; break; \
	case 0x4: _cond =  _zf; break; \
	case 0x5: _cond = !_zf; break; \
	case 0x6: _cond =  _zf || _cf; break; \
	case 0x7: _cond = !_zf && !_cf; break; \
	case 0x8: _cond =  _sf; break; \
	case 0x9: _cond = !_sf; break; \
	case 0xc: _cond =  _sf != _of; break; \
	case 0xd: _cond =  _sf == _of; break; \
	case 0xe: _cond =  _zf || (_sf != _of); break; \
	default:  _cond = !_zf && (_sf == _of); break; \
	} \
} while(0)

/* Evaluate CC_AND condition: CF=0, OF=0, only ZF and SF matter */
#define EVAL_COND_AND(_d, _ci, _cond) do { \
	int _zf = ((_d) == 0); \
	int _sf = ((sword)(_d) < 0); \
	switch (_ci) { \
	case 0x0: _cond = 0; break; \
	case 0x1: _cond = 1; break; \
	case 0x2: _cond = 0; break; \
	case 0x3: _cond = 1; break; \
	case 0x4: _cond =  _zf; break; \
	case 0x5: _cond = !_zf; break; \
	case 0x6: _cond =  _zf; break; \
	case 0x7: _cond = !_zf; break; \
	case 0x8: _cond =  _sf; break; \
	case 0x9: _cond = !_sf; break; \
	case 0xc: _cond =  _sf; break; \
	case 0xd: _cond = !_sf; break; \
	case 0xe: _cond = _zf || _sf; break; \
	default:  _cond = !_zf && !_sf; break; \
	} \
} while(0)

#define TRY_FUSE_JCC_SUB(cpu) do { \
	if (likely((cpu)->pf_pos + 1 < (cpu)->pf_avail)) { \
		u8 _nb = (cpu)->pf_ptr[(cpu)->pf_pos]; \
		if (likely((_nb & 0xF0) == 0x70) && (_nb & 0xf) != 0xa && (_nb & 0xf) != 0xb) { \
			s32 _disp = (s8)(cpu)->pf_ptr[(cpu)->pf_pos + 1]; \
			(cpu)->pf_pos += 2; (cpu)->next_ip += 2; (cpu)->cycle++; \
			int _cond; \
			EVAL_COND_SUB((cpu)->cc.src1, (cpu)->cc.src2, (cpu)->cc.dst, _nb & 0xf, _cond); \
			if (_cond) { (cpu)->next_ip += _disp; SEQ_INVALIDATE(cpu); } \
			(cpu)->fusion_count++; continue; \
		} \
		if (_nb == 0x0F && (cpu)->pf_pos + 5 < (cpu)->pf_avail) { \
			u8 _jop = (cpu)->pf_ptr[(cpu)->pf_pos + 1]; \
			if ((_jop & 0xF0) == 0x80 && (_jop & 0xf) != 0xa && (_jop & 0xf) != 0xb) { \
				const u8 *_p = &(cpu)->pf_ptr[(cpu)->pf_pos + 2]; \
				s32 _disp = (s32)(_p[0] | ((u32)_p[1] << 8) | ((u32)_p[2] << 16) | ((u32)_p[3] << 24)); \
				(cpu)->pf_pos += 6; (cpu)->next_ip += 6; (cpu)->cycle++; \
				int _cond; \
				EVAL_COND_SUB((cpu)->cc.src1, (cpu)->cc.src2, (cpu)->cc.dst, _jop & 0xf, _cond); \
				if (_cond) { (cpu)->next_ip += _disp; SEQ_INVALIDATE(cpu); } \
				(cpu)->fusion_count++; continue; \
			} \
		} \
	} \
} while(0)

#define TRY_FUSE_JCC_AND(cpu) do { \
	if (likely((cpu)->pf_pos + 1 < (cpu)->pf_avail)) { \
		u8 _nb = (cpu)->pf_ptr[(cpu)->pf_pos]; \
		if (likely((_nb & 0xF0) == 0x70) && (_nb & 0xf) != 0xa && (_nb & 0xf) != 0xb) { \
			s32 _disp = (s8)(cpu)->pf_ptr[(cpu)->pf_pos + 1]; \
			(cpu)->pf_pos += 2; (cpu)->next_ip += 2; (cpu)->cycle++; \
			int _cond; \
			EVAL_COND_AND((cpu)->cc.dst, _nb & 0xf, _cond); \
			if (_cond) { (cpu)->next_ip += _disp; SEQ_INVALIDATE(cpu); } \
			(cpu)->fusion_count++; continue; \
		} \
		if (_nb == 0x0F && (cpu)->pf_pos + 5 < (cpu)->pf_avail) { \
			u8 _jop = (cpu)->pf_ptr[(cpu)->pf_pos + 1]; \
			if ((_jop & 0xF0) == 0x80 && (_jop & 0xf) != 0xa && (_jop & 0xf) != 0xb) { \
				const u8 *_p = &(cpu)->pf_ptr[(cpu)->pf_pos + 2]; \
				s32 _disp = (s32)(_p[0] | ((u32)_p[1] << 8) | ((u32)_p[2] << 16) | ((u32)_p[3] << 24)); \
				(cpu)->pf_pos += 6; (cpu)->next_ip += 6; (cpu)->cycle++; \
				int _cond; \
				EVAL_COND_AND((cpu)->cc.dst, _jop & 0xf, _cond); \
				if (_cond) { (cpu)->next_ip += _disp; SEQ_INVALIDATE(cpu); } \
				(cpu)->fusion_count++; continue; \
			} \
		} \
	} \
} while(0)

	static const IRAM_ATTR void *pfxlabel[] = {
/* 0x00 */	&&f0x00, &&fast_0x01, &&f0x02, &&fast_0x03, &&f0x04, &&f0x05, &&f0x06, &&f0x07,
/* 0x08 */	&&f0x08, &&fast_0x09, &&f0x0a, &&fast_0x0b, &&f0x0c, &&f0x0d, &&f0x0e, &&fast_0x0f,
/* 0x10 */	&&f0x10, &&f0x11, &&f0x12, &&f0x13, &&f0x14, &&f0x15, &&f0x16, &&f0x17,
/* 0x18 */	&&f0x18, &&f0x19, &&f0x1a, &&f0x1b, &&f0x1c, &&f0x1d, &&f0x1e, &&f0x1f,
/* 0x20 */	&&f0x20, &&fast_0x21, &&f0x22, &&fast_0x23, &&f0x24, &&f0x25, &&pfx26, &&f0x27,
/* 0x28 */	&&f0x28, &&fast_0x29, &&f0x2a, &&fast_0x2b, &&f0x2c, &&f0x2d, &&pfx2e, &&f0x2f,
/* 0x30 */	&&f0x30, &&fast_0x31, &&f0x32, &&fast_0x33, &&f0x34, &&f0x35, &&pfx36, &&f0x37,
/* 0x38 */	&&f0x38, &&fast_0x39, &&f0x3a, &&fast_0x3b, &&f0x3c, &&f0x3d, &&pfx3e, &&f0x3f,
/* 0x40 */	&&fast_inc, &&fast_inc, &&fast_inc, &&fast_inc, &&fast_inc, &&fast_inc, &&fast_inc, &&fast_inc,
/* 0x48 */	&&fast_dec, &&fast_dec, &&fast_dec, &&fast_dec, &&fast_dec, &&fast_dec, &&fast_dec, &&fast_dec,
/* 0x50 */	&&fast_push, &&fast_push, &&fast_push, &&fast_push, &&fast_push, &&fast_push, &&fast_push, &&fast_push,
/* 0x58 */	&&fast_pop, &&fast_pop, &&fast_pop, &&fast_pop, &&fast_pop, &&fast_pop, &&fast_pop, &&fast_pop,
/* 0x60 */	&&f0x60, &&f0x61, &&f0x62, &&f0x63, &&pfx64, &&pfx65, &&pfx66, &&pfx67,
/* 0x68 */	&&f0x68, &&f0x69, &&f0x6a, &&f0x6b, &&f0x6c, &&f0x6d, &&f0x6e, &&f0x6f,
/* 0x70 */	&&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc,
/* 0x78 */	&&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc, &&fast_jcc,
/* 0x80 */	&&f0x80, &&f0x81, &&f0x82, &&fast_0x83, &&f0x84, &&fast_0x85, &&f0x86, &&f0x87,
/* 0x88 */	&&f0x88, &&fast_0x89, &&f0x8a, &&fast_0x8b, &&f0x8c, &&f0x8d, &&f0x8e, &&f0x8f,
/* 0x90 */	&&f0x90, &&f0x91, &&f0x92, &&f0x93, &&f0x94, &&f0x95, &&f0x96, &&f0x97,
/* 0x98 */	&&f0x98, &&f0x99, &&f0x9a, &&f0x9b, &&f0x9c, &&f0x9d, &&f0x9e, &&f0x9f,
/* 0xa0 */	&&f0xa0, &&f0xa1, &&f0xa2, &&f0xa3, &&f0xa4, &&f0xa5, &&f0xa6, &&f0xa7,
/* 0xa8 */	&&f0xa8, &&f0xa9, &&f0xaa, &&f0xab, &&f0xac, &&f0xad, &&f0xae, &&f0xaf,
/* 0xb0 */	&&f0xb0, &&f0xb1, &&f0xb2, &&f0xb3, &&f0xb4, &&f0xb5, &&f0xb6, &&f0xb7,
/* 0xb8 */	&&fast_movimm, &&fast_movimm, &&fast_movimm, &&fast_movimm, &&fast_movimm, &&fast_movimm, &&fast_movimm, &&fast_movimm,
/* 0xc0 */	&&f0xc0, &&f0xc1, &&f0xc2, &&fast_ret, &&f0xc4, &&f0xc5, &&f0xc6, &&f0xc7,
/* 0xc8 */	&&f0xc8, &&f0xc9, &&f0xca, &&f0xcb, &&f0xcc, &&f0xcd, &&f0xce, &&f0xcf,
/* 0xd0 */	&&f0xd0, &&f0xd1, &&f0xd2, &&f0xd3, &&f0xd4, &&f0xd5, &&f0xd6, &&f0xd7,
/* 0xd8 */	&&f0xd8, &&f0xd9, &&f0xda, &&f0xdb, &&f0xdc, &&f0xdd, &&f0xde, &&f0xdf,
/* 0xe0 */	&&f0xe0, &&f0xe1, &&f0xe2, &&f0xe3, &&f0xe4, &&f0xe5, &&f0xe6, &&f0xe7,
/* 0xe8 */	&&fast_call_rel32, &&fast_jmp_rel32, &&f0xea, &&fast_jmp_rel8, &&f0xec, &&f0xed, &&f0xee, &&f0xef,
/* 0xf0 */	&&pfxf0, &&f0xf1, &&pfxf2, &&pfxf3, &&f0xf4, &&f0xf5, &&f0xf6, &&f0xf7,
/* 0xf8 */	&&f0xf8, &&f0xf9, &&f0xfa, &&f0xfb, &&f0xfc, &&f0xfd, &&f0xfe, &&f0xff,
	};
	goto *pfxlabel[b1];
#define HANDLE_PREFIX(C, STMT) \
		pfx ## C: { \
			STMT; \
			TRY(fetch8(cpu, &b1)); \
			goto *pfxlabel[b1]; \
		}
		HANDLE_PREFIX(26, curr_seg = SEG_ES)
		HANDLE_PREFIX(2e, curr_seg = SEG_CS)
		HANDLE_PREFIX(36, curr_seg = SEG_SS)
		HANDLE_PREFIX(3e, curr_seg = SEG_DS)
		HANDLE_PREFIX(64, curr_seg = SEG_FS)
		HANDLE_PREFIX(65, curr_seg = SEG_GS)
			/* Prefix fusion for 0x66: inline common operations */
			pfx66: {
				opsz16 = !code16;
				TRY(fetch8(cpu, &b1));
				/* VM86 correctness first: route prefixed ops through the
				 * normal dispatcher instead of 32-bit fused shortcuts. */
				if (unlikely(cpu->flags & VM))
					goto *pfxlabel[b1];
				int reg = b1 & 7;
			/* INC r16/r32 (0x40-0x47) */
			if (b1 >= 0x40 && b1 <= 0x47) {
				int cf = get_CF(cpu);
				if (!code16) {
					cpu->cc.dst = sext16((u16)(lreg16(reg) + 1));
					cpu->cc.op = CC_INC16;
					sreg16(reg, cpu->cc.dst);
				} else {
					cpu->cc.dst = sext32((u32)(lreg32(reg) + 1));
					cpu->cc.op = CC_INC32;
					sreg32(reg, cpu->cc.dst);
				}
				SET_BIT(cpu->flags, cf, CF);
				cpu->cc.mask = PF | AF | ZF | SF | OF;
				continue;
			}
			/* DEC r16/r32 (0x48-0x4f) */
			if (b1 >= 0x48 && b1 <= 0x4f) {
				int cf = get_CF(cpu);
				if (!code16) {
					cpu->cc.dst = sext16((u16)(lreg16(reg) - 1));
					cpu->cc.op = CC_DEC16;
					sreg16(reg, cpu->cc.dst);
				} else {
					cpu->cc.dst = sext32((u32)(lreg32(reg) - 1));
					cpu->cc.op = CC_DEC32;
					sreg32(reg, cpu->cc.dst);
				}
				SET_BIT(cpu->flags, cf, CF);
				cpu->cc.mask = PF | AF | ZF | SF | OF;
				continue;
			}
			/* PUSH r16/r32 (0x50-0x57) */
			if (b1 >= 0x50 && b1 <= 0x57) {
				OptAddr meml1;
				uword sp = lreg32(4);
				if (!code16) {
					u16 val = lreg16(reg);
					TRY(translate16(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask));
					set_sp(sp - 2, sp_mask);
					saddr16(&meml1, val);
				} else {
					u32 val = lreg32(reg);
					TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask));
					set_sp(sp - 4, sp_mask);
					saddr32(&meml1, val);
				}
				continue;
			}
			/* POP r16/r32 (0x58-0x5f) */
			if (b1 >= 0x58 && b1 <= 0x5f) {
				OptAddr meml1;
				uword sp = lreg32(4);
				if (!code16) {
					TRY(translate16(cpu, &meml1, 1, SEG_SS, sp & sp_mask));
					u16 val = laddr16(&meml1);
					set_sp(sp + 2, sp_mask);
					sreg16(reg, val);
				} else {
					TRY(translate32(cpu, &meml1, 1, SEG_SS, sp & sp_mask));
					u32 val = laddr32(&meml1);
					set_sp(sp + 4, sp_mask);
					sreg32(reg, val);
				}
				continue;
			}
			/* MOV r/m, r or MOV r, r/m - fast path for reg-to-reg (mod=3) */
			if (b1 == 0x89 || b1 == 0x8b) {
				u8 modrm_peek;
				TRY(peek8(cpu, &modrm_peek));
				if ((modrm_peek >> 6) == 3) {
					/* mod=3: register to register, no memory */
					cpu->next_ip++;
					int rm = modrm_peek & 7;
					int r = (modrm_peek >> 3) & 7;
					if (!code16) {
						/* 32-bit mode + 0x66 = 16-bit MOV */
						if (b1 == 0x89) sreg16(rm, lreg16(r));
						else sreg16(r, lreg16(rm));
					} else {
						/* 16-bit mode + 0x66 = 32-bit MOV */
						if (b1 == 0x89) sreg32(rm, lreg32(r));
						else sreg32(r, lreg32(rm));
					}
					continue;
				}
				/* mod != 3: fall through to normal dispatch */
			}
			/* CMP r/m, r or CMP r, r/m - reg-to-reg fast path (0x39/0x3b, mod=3) */
			if (b1 == 0x39 || b1 == 0x3b) {
				u8 modrm_peek;
				TRY(peek8(cpu, &modrm_peek));
				if ((modrm_peek >> 6) == 3) {
					cpu->next_ip++;
					int rm = modrm_peek & 7;
					int r = (modrm_peek >> 3) & 7;
					if (!code16) {
						cpu->cc.src1 = sext16(lreg16(b1 == 0x39 ? rm : r));
						cpu->cc.src2 = sext16(lreg16(b1 == 0x39 ? r : rm));
					} else {
						cpu->cc.src1 = sext32(lreg32(b1 == 0x39 ? rm : r));
						cpu->cc.src2 = sext32(lreg32(b1 == 0x39 ? r : rm));
					}
					cpu->cc.dst = cpu->cc.src1 - cpu->cc.src2;
					cpu->cc.op = CC_SUB;
					cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
					TRY_FUSE_JCC_SUB(cpu);
					continue;
				}
			}
			/* ADD r/m, r or ADD r, r/m - reg-to-reg fast path (0x01/0x03, mod=3) */
			if (b1 == 0x01 || b1 == 0x03) {
				u8 modrm_peek;
				TRY(peek8(cpu, &modrm_peek));
				if ((modrm_peek >> 6) == 3) {
					cpu->next_ip++;
					int rm = modrm_peek & 7;
					int r = (modrm_peek >> 3) & 7;
					if (!code16) {
						cpu->cc.src1 = sext16(lreg16(b1 == 0x01 ? rm : r));
						cpu->cc.src2 = sext16(lreg16(b1 == 0x01 ? r : rm));
						cpu->cc.dst = sext16((u16)(cpu->cc.src1 + cpu->cc.src2));
						sreg16(b1 == 0x01 ? rm : r, cpu->cc.dst);
					} else {
						cpu->cc.src1 = sext32(lreg32(b1 == 0x01 ? rm : r));
						cpu->cc.src2 = sext32(lreg32(b1 == 0x01 ? r : rm));
						cpu->cc.dst = sext32((u32)(cpu->cc.src1 + cpu->cc.src2));
						sreg32(b1 == 0x01 ? rm : r, cpu->cc.dst);
					}
					cpu->cc.op = CC_ADD;
					cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
					continue;
				}
			}
			/* SUB r/m, r or SUB r, r/m - reg-to-reg fast path (0x29/0x2b, mod=3) */
			if (b1 == 0x29 || b1 == 0x2b) {
				u8 modrm_peek;
				TRY(peek8(cpu, &modrm_peek));
				if ((modrm_peek >> 6) == 3) {
					cpu->next_ip++;
					int rm = modrm_peek & 7;
					int r = (modrm_peek >> 3) & 7;
					if (!code16) {
						cpu->cc.src1 = sext16(lreg16(b1 == 0x29 ? rm : r));
						cpu->cc.src2 = sext16(lreg16(b1 == 0x29 ? r : rm));
						cpu->cc.dst = sext16((u16)(cpu->cc.src1 - cpu->cc.src2));
						sreg16(b1 == 0x29 ? rm : r, cpu->cc.dst);
					} else {
						cpu->cc.src1 = sext32(lreg32(b1 == 0x29 ? rm : r));
						cpu->cc.src2 = sext32(lreg32(b1 == 0x29 ? r : rm));
						cpu->cc.dst = sext32((u32)(cpu->cc.src1 - cpu->cc.src2));
						sreg32(b1 == 0x29 ? rm : r, cpu->cc.dst);
					}
					cpu->cc.op = CC_SUB;
					cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
					continue;
				}
			}
			/* TEST r/m, r - reg-to-reg fast path (0x85, mod=3) */
			if (b1 == 0x85) {
				u8 modrm_peek;
				TRY(peek8(cpu, &modrm_peek));
				if ((modrm_peek >> 6) == 3) {
					cpu->next_ip++;
					int rm = modrm_peek & 7;
					int r = (modrm_peek >> 3) & 7;
					if (!code16) {
						cpu->cc.dst = sext16((u16)(lreg16(rm) & lreg16(r)));
					} else {
						cpu->cc.dst = sext32((u32)(lreg32(rm) & lreg32(r)));
					}
					cpu->cc.op = CC_AND;
					cpu->cc.mask = CF | PF | ZF | SF | OF;
					TRY_FUSE_JCC_AND(cpu);
					continue;
				}
			}
			/* XCHG AX/EAX, r16/r32 (0x91-0x97) - 0x90 is NOP */
			if (b1 >= 0x91 && b1 <= 0x97) {
				if (!code16) {
					/* 32-bit mode + 0x66 = XCHG AX, r16 */
					u16 tmp = lreg16(0);
					sreg16(0, lreg16(reg));
					sreg16(reg, tmp);
				} else {
					/* 16-bit mode + 0x66 = XCHG EAX, r32 */
					u32 tmp = lreg32(0);
					sreg32(0, lreg32(reg));
					sreg32(reg, tmp);
				}
				continue;
			}
			/* MOV r16/r32, imm16/imm32 (0xb8-0xbf) */
			if (b1 >= 0xb8 && b1 <= 0xbf) {
				if (!code16) {
					u16 imm;
					TRY(fetch16(cpu, &imm));
					sreg16(reg, imm);
				} else {
					u32 imm;
					TRY(fetch32(cpu, &imm));
					sreg32(reg, imm);
				}
				continue;
			}
			/* Fallback to normal dispatch */
			goto *pfxlabel[b1];
		}
		HANDLE_PREFIX(67, adsz16 = !code16)
		HANDLE_PREFIX(f3, rep = 1) // REP
		HANDLE_PREFIX(f2, rep = 2) // REPNE
		HANDLE_PREFIX(f0, /*lock = true*/)
#undef HANDLE_PREFIX

	/* Non-prefixed 32-bit fast paths.
	 * In 32-bit PM, opsz16==false for non-prefixed instructions.
	 * Skip the opsz16 branch and macro chain for common opcodes. */

	fast_inc: {  /* INC r32 (0x40-0x47) */
		if (likely(!opsz16)) {
			int reg = b1 & 7;
			int cf = get_CF(cpu);
			cpu->cc.dst = sext32((u32)(lreg32(reg) + 1));
			cpu->cc.op = CC_INC32;
			SET_BIT(cpu->flags, cf, CF);
			cpu->cc.mask = PF | AF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x40;
	}
	fast_dec: {  /* DEC r32 (0x48-0x4f) */
		if (likely(!opsz16)) {
			int reg = b1 & 7;
			int cf = get_CF(cpu);
			cpu->cc.dst = sext32((u32)(lreg32(reg) - 1));
			cpu->cc.op = CC_DEC32;
			SET_BIT(cpu->flags, cf, CF);
			cpu->cc.mask = PF | AF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x48;
	}
	fast_push: {  /* PUSH r32 (0x50-0x57) */
		if (likely(!opsz16)) {
			int reg = b1 & 7;
			OptAddr meml1;
			uword sp = lreg32(4);
			u32 val = lreg32(reg);
			TRY(translate32(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask));
			set_sp(sp - 4, sp_mask);
			saddr32(&meml1, val);
			continue;
		}
		goto f0x50;
	}
	fast_pop: {  /* POP r32 (0x58-0x5f) */
		if (likely(!opsz16)) {
			int reg = b1 & 7;
			OptAddr meml1;
			uword sp = lreg32(4);
			TRY(translate32(cpu, &meml1, 1, SEG_SS, sp & sp_mask));
			u32 src = laddr32(&meml1);
			set_sp(sp + 4, sp_mask);
			sreg32(reg, src);
			continue;
		}
		goto f0x58;
	}
	fast_jcc: {  /* Jcc rel8 (0x70-0x7f) */
		if (likely(cpu->pf_pos < cpu->pf_avail)) {
			sword d = (s8)cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int cond;
			switch (b1 & 0xf) {
			case 0x0: cond =  get_OF(cpu); break;
			case 0x1: cond = !get_OF(cpu); break;
			case 0x2: cond =  get_CF(cpu); break;
			case 0x3: cond = !get_CF(cpu); break;
			case 0x4: cond =  get_ZF(cpu); break;
			case 0x5: cond = !get_ZF(cpu); break;
			case 0x6: cond =  get_ZF(cpu) ||  get_CF(cpu); break;
			case 0x7: cond = !get_ZF(cpu) && !get_CF(cpu); break;
			case 0x8: cond =  get_SF(cpu); break;
			case 0x9: cond = !get_SF(cpu); break;
			case 0xa: cond =  get_PF(cpu); break;
			case 0xb: cond = !get_PF(cpu); break;
			case 0xc: cond =  get_SF(cpu) != get_OF(cpu); break;
			case 0xd: cond =  get_SF(cpu) == get_OF(cpu); break;
			case 0xe: cond =  get_ZF(cpu) || get_SF(cpu) != get_OF(cpu); break;
			default:  cond = !get_ZF(cpu) && get_SF(cpu) == get_OF(cpu); break;
			}
			if (cond) cpu->next_ip += d;
			continue;
		}
		goto f0x70;
	}
	fast_movimm: {  /* MOV r32, imm32 (0xb8-0xbf) */
		if (likely(!opsz16 && cpu->pf_pos + 4 <= cpu->pf_avail)) {
			int p = cpu->pf_pos;
			u32 imm = cpu->pf_ptr[p] | ((u32)cpu->pf_ptr[p+1] << 8)
			         | ((u32)cpu->pf_ptr[p+2] << 16) | ((u32)cpu->pf_ptr[p+3] << 24);
			cpu->pf_pos = p + 4;
			cpu->next_ip += 4;
			sreg32(b1 & 7, imm);
			continue;
		}
		goto f0xb8;
	}

		fast_call_rel32: {  /* CALL rel32 (0xE8) */
			if (likely(!opsz16 && !(cpu->flags & VM) && cpu->pf_pos + 4 <= cpu->pf_avail)) {
			int p = cpu->pf_pos;
			sword d = (s32)(cpu->pf_ptr[p] | ((u32)cpu->pf_ptr[p+1] << 8)
			         | ((u32)cpu->pf_ptr[p+2] << 16) | ((u32)cpu->pf_ptr[p+3] << 24));
			cpu->pf_pos = p + 4;
			cpu->next_ip += 4;
			uword sp = lreg32(4);
			uword push_addr = (sp - 4) & sp_mask;
			/* Inline TLB write for flat mode (skip translate32 chain) */
			if (likely(cpu->all_segs_flat && (push_addr & 0xfff) <= 0xffc)) {
				uword lpgno = push_addr >> 12;
				struct tlb_entry *ent = &(cpu->tlb.tab[lpgno & tlb_mask]);
				if (likely(ent->lpgno == lpgno &&
				           ent->generation == cpu->tlb.generation &&
				           !ent->pte_lookup[cpu->cpl > 0][1])) {
					*(ent->ppte) |= (1 << 6); /* dirty */
					pstore32(cpu, ent->xaddr ^ push_addr, cpu->next_ip);
					set_sp(sp - 4, sp_mask);
					cpu->next_ip += d;
					continue;
				}
			}
			/* Fallback: full translate path */
			OptAddr meml1;
			TRY(translate32(cpu, &meml1, 2, SEG_SS, push_addr));
			set_sp(sp - 4, sp_mask);
			saddr32(&meml1, cpu->next_ip);
			cpu->next_ip += d;
			continue;
		}
		goto f0xe8;
	}
		fast_ret: {  /* RET near (0xC3) */
			if (likely(!opsz16 && !(cpu->flags & VM))) {
			uword sp = lreg32(4);
			uword pop_addr = sp & sp_mask;
			/* Inline TLB read for flat mode (skip translate32 chain) */
			if (likely(cpu->all_segs_flat && (pop_addr & 0xfff) <= 0xffc)) {
				uword lpgno = pop_addr >> 12;
				struct tlb_entry *ent = &(cpu->tlb.tab[lpgno & tlb_mask]);
				if (likely(ent->lpgno == lpgno &&
				           ent->generation == cpu->tlb.generation &&
				           !ent->pte_lookup[cpu->cpl > 0][0])) {
					set_sp(sp + 4, sp_mask);
					cpu->next_ip = pload32(cpu, ent->xaddr ^ pop_addr);
					continue;
				}
			}
			/* Fallback: full translate path */
			OptAddr meml1;
			TRY(translate32(cpu, &meml1, 1, SEG_SS, pop_addr));
			set_sp(sp + 4, sp_mask);
			cpu->next_ip = laddr32(&meml1);
			continue;
		}
		goto f0xc3;
	}
		fast_jmp_rel32: {  /* JMP rel32 (0xE9) */
			if (likely(!opsz16 && !(cpu->flags & VM) && cpu->pf_pos + 4 <= cpu->pf_avail)) {
			int p = cpu->pf_pos;
			sword d = (s32)(cpu->pf_ptr[p] | ((u32)cpu->pf_ptr[p+1] << 8)
			         | ((u32)cpu->pf_ptr[p+2] << 16) | ((u32)cpu->pf_ptr[p+3] << 24));
			cpu->pf_pos = p + 4;
			cpu->next_ip += 4 + d;
			continue;
		}
		goto f0xe9;
	}
	fast_jmp_rel8: {  /* JMP rel8 (0xEB) */
		if (likely(cpu->pf_pos < cpu->pf_avail)) {
			sword d = (s8)cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			cpu->next_ip += d;
			continue;
		}
		goto f0xeb;
	}
	fast_0x0f: {  /* Fast path for Jcc rel32 (0F 80-8F, no 66h override) */
		if (likely(!opsz16 && cpu->pf_pos + 5 <= cpu->pf_avail)) {
			u8 b2 = cpu->pf_ptr[cpu->pf_pos];
			if (likely((b2 & 0xf0) == 0x80)) {
				int p = cpu->pf_pos + 1;
				sword d = (s32)(cpu->pf_ptr[p] | ((u32)cpu->pf_ptr[p+1] << 8)
				         | ((u32)cpu->pf_ptr[p+2] << 16) | ((u32)cpu->pf_ptr[p+3] << 24));
				cpu->pf_pos = p + 4;
				cpu->next_ip += 5;
				int cond;
				switch (b2 & 0xf) {
				case 0x0: cond =  get_OF(cpu); break;
				case 0x1: cond = !get_OF(cpu); break;
				case 0x2: cond =  get_CF(cpu); break;
				case 0x3: cond = !get_CF(cpu); break;
				case 0x4: cond =  get_ZF(cpu); break;
				case 0x5: cond = !get_ZF(cpu); break;
				case 0x6: cond =  get_ZF(cpu) ||  get_CF(cpu); break;
				case 0x7: cond = !get_ZF(cpu) && !get_CF(cpu); break;
				case 0x8: cond =  get_SF(cpu); break;
				case 0x9: cond = !get_SF(cpu); break;
				case 0xa: cond =  get_PF(cpu); break;
				case 0xb: cond = !get_PF(cpu); break;
				case 0xc: cond =  get_SF(cpu) != get_OF(cpu); break;
				case 0xd: cond =  get_SF(cpu) == get_OF(cpu); break;
				case 0xe: cond =  get_ZF(cpu) || get_SF(cpu) != get_OF(cpu); break;
				default:  cond = !get_ZF(cpu) && get_SF(cpu) == get_OF(cpu); break;
				}
				if (cond) cpu->next_ip += d;
				continue;
			}
		}
		goto f0x0f;
	}

	/* Reg-to-reg ALU fast paths (mod=3 only) */
	fast_0x89: {  /* MOV Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			sreg32(mrm & 7, lreg32((mrm >> 3) & 7));
			continue;
		}
		goto f0x89;
	}
	fast_0x8b: {  /* MOV Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			sreg32((mrm >> 3) & 7, lreg32(mrm & 7));
			continue;
		}
		goto f0x8b;
	}
	fast_0x01: {  /* ADD Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.src1 = sext32(lreg32(rm));
			cpu->cc.src2 = sext32(lreg32(reg));
			cpu->cc.dst = sext32((u32)(cpu->cc.src1 + cpu->cc.src2));
			cpu->cc.op = CC_ADD;
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
			sreg32(rm, cpu->cc.dst);
			continue;
		}
		goto f0x01;
	}
	fast_0x03: {  /* ADD Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.src1 = sext32(lreg32(reg));
			cpu->cc.src2 = sext32(lreg32(rm));
			cpu->cc.dst = sext32((u32)(cpu->cc.src1 + cpu->cc.src2));
			cpu->cc.op = CC_ADD;
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x03;
	}
	fast_0x29: {  /* SUB Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.src1 = sext32(lreg32(rm));
			cpu->cc.src2 = sext32(lreg32(reg));
			cpu->cc.dst = sext32((u32)(cpu->cc.src1 - cpu->cc.src2));
			cpu->cc.op = CC_SUB;
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
			sreg32(rm, cpu->cc.dst);
			continue;
		}
		goto f0x29;
	}
	fast_0x2b: {  /* SUB Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.src1 = sext32(lreg32(reg));
			cpu->cc.src2 = sext32(lreg32(rm));
			cpu->cc.dst = sext32((u32)(cpu->cc.src1 - cpu->cc.src2));
			cpu->cc.op = CC_SUB;
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x2b;
	}
	fast_0x39: {  /* CMP Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.src1 = sext32(lreg32(rm));
			cpu->cc.src2 = sext32(lreg32(reg));
			cpu->cc.dst = sext32((u32)(cpu->cc.src1 - cpu->cc.src2));
			cpu->cc.op = CC_SUB;
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
			TRY_FUSE_JCC_SUB(cpu);
			continue;
		}
		goto f0x39;
	}
	fast_0x3b: {  /* CMP Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.src1 = sext32(lreg32(reg));
			cpu->cc.src2 = sext32(lreg32(rm));
			cpu->cc.dst = sext32((u32)(cpu->cc.src1 - cpu->cc.src2));
			cpu->cc.op = CC_SUB;
			cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
			TRY_FUSE_JCC_SUB(cpu);
			continue;
		}
		goto f0x3b;
	}
	fast_0x85: {  /* TEST Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(rm) & lreg32(reg));
			cpu->cc.op = CC_AND;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			TRY_FUSE_JCC_AND(cpu);
			continue;
		}
		goto f0x85;
	}
	fast_0x83: {  /* G1vIb: CMP r32, imm8 fast path (reg field=7, mod=3) */
		if (likely(!opsz16 && cpu->pf_pos + 1 < cpu->pf_avail)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos];
			if (likely(mrm >= 0xC0 && ((mrm >> 3) & 7) == 7)) {
				cpu->pf_pos++; cpu->next_ip++;
				int rm = mrm & 7;
				s8 imm = (s8)cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
				cpu->cc.src1 = sext32(lreg32(rm));
				cpu->cc.src2 = sext32((u32)(s32)imm);
				cpu->cc.dst = sext32((u32)(cpu->cc.src1 - cpu->cc.src2));
				cpu->cc.op = CC_SUB;
				cpu->cc.mask = CF | PF | AF | ZF | SF | OF;
				TRY_FUSE_JCC_SUB(cpu);
				continue;
			}
		}
		goto f0x83;
	}
	fast_0x31: {  /* XOR Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(rm) ^ lreg32(reg));
			cpu->cc.op = CC_XOR;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			sreg32(rm, cpu->cc.dst);
			continue;
		}
		goto f0x31;
	}
	fast_0x33: {  /* XOR Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(reg) ^ lreg32(rm));
			cpu->cc.op = CC_XOR;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x33;
	}
	fast_0x09: {  /* OR Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(rm) | lreg32(reg));
			cpu->cc.op = CC_OR;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			sreg32(rm, cpu->cc.dst);
			continue;
		}
		goto f0x09;
	}
	fast_0x0b: {  /* OR Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(reg) | lreg32(rm));
			cpu->cc.op = CC_OR;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x0b;
	}
	fast_0x21: {  /* AND Ev, Gv */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(rm) & lreg32(reg));
			cpu->cc.op = CC_AND;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			sreg32(rm, cpu->cc.dst);
			continue;
		}
		goto f0x21;
	}
	fast_0x23: {  /* AND Gv, Ev */
		if (likely(!opsz16 && cpu->pf_pos < cpu->pf_avail && cpu->pf_ptr[cpu->pf_pos] >= 0xC0)) {
			u8 mrm = cpu->pf_ptr[cpu->pf_pos++]; cpu->next_ip++;
			int rm = mrm & 7, reg = (mrm >> 3) & 7;
			cpu->cc.dst = sext32(lreg32(reg) & lreg32(rm));
			cpu->cc.op = CC_AND;
			cpu->cc.mask = CF | PF | ZF | SF | OF;
			sreg32(reg, cpu->cc.dst);
			continue;
		}
		goto f0x23;
	}

#endif
	eswitch(b1) {
#define I(_case, _rm, _rwm, _op) _case { _rm(_rwm, _op); ebreak; }
#include "i386ins.def"
#undef I

#undef CX
#define CX(_1) case _1:
#define GRPBEG TRY(peek8(cpu, &modrm)); switch((modrm >> 3) & 7) {
#define GRPCASE(_case, _rm, _rwm, _op) _case { _rm(_rwm, _op); ebreak; }
#define GRPEND default: default_ud; } ebreak;

	ecase(0x80): ecase(0x82): { // G1b
GRPBEG
#define IG1b GRPCASE
#include "i386ins.def"
#undef IG1b
GRPEND
	}

	ecase(0x81): { // G1v
GRPBEG
#define IG1v GRPCASE
#include "i386ins.def"
#undef IG1v
GRPEND
	}

	ecase(0x83): { // G1vIb
GRPBEG
#define IG1vIb GRPCASE
#include "i386ins.def"
#undef IG1vIb
GRPEND
	}

	ecase(0xc0): { // G2b
GRPBEG
#define IG2b GRPCASE
#include "i386ins.def"
#undef IG2b
GRPEND
	}

	ecase(0xc1): { // G2v
GRPBEG
#define IG2v GRPCASE
#include "i386ins.def"
#undef IG2v
GRPEND
	}

	ecase(0xd0): { // G2b1
GRPBEG
#define IG2b1 GRPCASE
#include "i386ins.def"
#undef IG2b1
GRPEND
	}

	ecase(0xd1): { // G2v1
GRPBEG
#define IG2v1 GRPCASE
#include "i386ins.def"
#undef IG2v1
GRPEND
	}

	ecase(0xd2): { // G2bC
GRPBEG
#define IG2bC GRPCASE
#include "i386ins.def"
#undef IG2bC
GRPEND
	}

	ecase(0xd3): { // G2v1
GRPBEG
#define IG2vC GRPCASE
#include "i386ins.def"
#undef IG2vC
GRPEND
	}

	ecase(0xf6): { // G3b
GRPBEG
#define IG3b GRPCASE
#include "i386ins.def"
#undef IG3b
GRPEND
	}

	ecase(0xf7): { // G3v
GRPBEG
#define IG3v GRPCASE
#include "i386ins.def"
#undef IG3v
GRPEND
	}

	ecase(0xfe): { // G4
GRPBEG
#define IG4 GRPCASE
#include "i386ins.def"
#undef IG4
GRPEND
	}

	ecase(0xff): { // G5
GRPBEG
#define IG5 GRPCASE
#include "i386ins.def"
#undef IG5
GRPEND
	}

	ecase(0x0f): { // two byte
		TRY(fetch8(cpu, &b1));
		switch(b1) {
#define I2(_case, _rm, _rwm, _op) _case { _rm(_rwm, _op); ebreak; }
#include "i386ins.def"
#undef I2

		case 0x00: { // G6
GRPBEG
#define IG6 GRPCASE
#include "i386ins.def"
#undef IG6
GRPEND
		}

		case 0x01: { // G7
GRPBEG
#define IG7 GRPCASE
#include "i386ins.def"
#undef IG7
GRPEND
		}

		case 0xba: { // G8
GRPBEG
#define IG8 GRPCASE
#include "i386ins.def"
#undef IG8
GRPEND
		}

		case 0xc7: { // G9
GRPBEG
#define IG9 GRPCASE
#include "i386ins.def"
#undef IG9
GRPEND
		}
		default: default_ud;
		}
		ebreak;
	}

	edefault: default_ud;
	}
	}
	return true;
}

// XXX: incomplete
enum { TS_JMP, TS_CALL, TS_IRET };
static bool task_switch(CPUI386 *cpu, int tss, int sw_type)
{
	OptAddr meml;
	int oldtss = cpu->seg[SEG_TR].sel;
	int tr_type = cpu->seg[SEG_TR].flags & 0xf;
	if (tr_type != 9 && tr_type != 11) THROW(EX_TS, cpu->seg[SEG_TR].sel & ~0x3);

	TRY1(translate(cpu, &meml, 2, SEG_TR, 0x20, 4, 0));
	store32(cpu, &meml, cpu->next_ip);

	refresh_flags(cpu);
	TRY1(translate(cpu, &meml, 2, SEG_TR, 0x24, 4, 0));
	if (sw_type == TS_IRET)
		store32(cpu, &meml, cpu->flags & ~NT);
	else
		store32(cpu, &meml, cpu->flags);

	for (int i = 0; i < 8; i++) {
		TRY1(translate(cpu, &meml, 2, SEG_TR, 0x28 + 4 * i, 4, 0));
		store32(cpu, &meml, REGi(i));
	}

	for (int i = 0; i < 6; i++) {
		TRY1(translate(cpu, &meml, 2, SEG_TR, 0x48 + 4 * i, 4, 0));
		store32(cpu, &meml, cpu->seg[i].sel);
	}

	// clear busy bit
	if (sw_type == TS_JMP || sw_type == TS_IRET) {
		uword addr = cpu->gdt.base + (cpu->seg[SEG_TR].sel & ~0x7);
		TRY1(translate_laddr(cpu, &meml, 3, addr + 4, 4, 0));
		store32(cpu, &meml, load32(cpu, &meml) & ~(1 << 9));
	}

	TRY1(set_seg(cpu, SEG_TR, tss));
	int new_tr_type = cpu->seg[SEG_TR].flags & 0xf;
	if (new_tr_type != 9 && new_tr_type != 11) THROW(EX_TS, tss & ~0x3);

	// set busy bit
	if (sw_type == TS_JMP || sw_type == TS_CALL) {
		uword addr = cpu->gdt.base + (tss & ~0x7);
		TRY1(translate_laddr(cpu, &meml, 3, addr + 4, 4, 0));
		store32(cpu, &meml, load32(cpu, &meml) | (1 << 9));
		cpu->seg[SEG_TR].flags |= 2;
	}

	cpu->cr0 |= 1 << 3; // set TS bit

	TRY1(translate(cpu, &meml, 1, SEG_TR, 0x60, 4, 0));
	TRY1(set_seg(cpu, SEG_LDT, load32(cpu, &meml)));

	for (int i = 0; i < 8; i++) {
		TRY1(translate(cpu, &meml, 1, SEG_TR, 0x28 + 4 * i, 4, 0));
		REGi(i) = load32(cpu, &meml);
	}

	for (int i = 0; i < 6; i++) {
		TRY1(translate(cpu, &meml, 1, SEG_TR, 0x48 + 4 * i, 4, 0));
		TRY1(set_seg(cpu, i, load32(cpu, &meml)));
	}

	TRY1(translate(cpu, &meml, 1, SEG_TR, 0x20, 4, 0));
	cpu->next_ip = load32(cpu, &meml);

	TRY1(translate(cpu, &meml, 1, SEG_TR, 0x24, 4, 0));
	cpu->flags = load32(cpu, &meml);
	cpu->flags &= EFLAGS_MASK;
	cpu->flags |= 0x2;
	if (sw_type == TS_CALL) {
		TRY1(translate(cpu, &meml, 2, SEG_TR, 0, 4, 0));
		store32(cpu, &meml, oldtss);
		cpu->flags |= NT;
	}

	TRY1(translate(cpu, &meml, 1, SEG_TR, 0x1c, 4, 0));
	cpu->cr3 = load32(cpu, &meml);
	tlb_flush_generation(cpu);
	SEQ_INVALIDATE(cpu);

	return true;
}

static bool pmcall(CPUI386 *cpu, bool opsz16, uword addr, int sel, bool isjmp)
{
	sel = sel & 0xffff;
	uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;

	if ((sel & ~0x3) == 0) THROW(EX_GP, 0);

	uword w1, w2;
	TRY(read_desc(cpu, sel, &w1, &w2));

	int s = (w2 >> 12) & 1;
	int dpl = (w2 >> 13) & 0x3;
	int p = (w2 >> 15) & 1;
	if (!p) {
//		dolog("pmcall: seg not present %04x\n", sel);
		THROW(EX_NP, sel & ~0x3);
	}

	if (s) {
		bool code = (w2 >> 8) & 0x8;
		bool conforming = (w2 >> 8) & 0x4;
		if (!code) THROW(EX_GP, sel & ~0x3);
		if (conforming) {
			// call conforming code segment
			if (dpl > cpu->cpl) THROW(EX_GP, sel & ~0x3);
			sel = (sel & 0xfffc) | cpu->cpl;
		} else {
			// call nonconforming code segment
			if ((sel & 0x3) > cpu->cpl || dpl != cpu->cpl)
				THROW(EX_GP, sel & ~0x3);
			sel = (sel & 0xfffc) | cpu->cpl;
		}

		if (!isjmp) {
			OptAddr meml1, meml2;
			uword sp = lreg32(4);
			if (opsz16) {
				TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 2) & sp_mask, 2, 0));
				TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 4) & sp_mask, 2, 0));
				set_sp(sp - 4, sp_mask);
				saddr16(&meml1, cpu->seg[SEG_CS].sel);
				saddr16(&meml2, cpu->next_ip);
			} else {
				TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 4) & sp_mask, 4, 0));
				TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 8) & sp_mask, 4, 0));
				set_sp(sp - 8, sp_mask);
				saddr32(&meml1, cpu->seg[SEG_CS].sel);
				saddr32(&meml2, cpu->next_ip);
			}
		}
//		if ((sel & 3) != cpu->cpl)
//			dolog("pmcall PVL %d => %d\n", cpu->cpl, sel & 3);
		TRY1(set_seg(cpu, SEG_CS, sel));
		cpu->next_ip = addr;
	} else {
		int newcs = w1 >> 16;
		int gt = (w2 >> 8) & 0xf;
		/* 16-bit call gate (type 4): offset is w1[15:0] only.
		 * 32-bit call gate (type 12): offset is w1[15:0] | w2[31:16]. */
		uword newip = (gt == 4) ? (w1 & 0xffff)
				        : ((w1 & 0xffff) | (w2 & 0xffff0000));
		int wc = w2 & 31;

		if (dpl < cpu->cpl || dpl < (sel & 3))
			THROW(EX_GP, sel & ~0x3);

		// only 32bit TSS is supported now
		int tr_type = cpu->seg[SEG_TR].flags & 0xf;
		if (tr_type == 9 || tr_type == 11) {
			if (gt == 9) {
				// 32 bit TSS avail segs
				return task_switch(cpu, sel,
						   isjmp ? TS_JMP : TS_CALL);
			}

			if (gt == 5) {
				// task gates
				return task_switch(cpu, newcs,
						   isjmp ? TS_JMP : TS_CALL);
			}
		}

		if (gt != 4 && gt != 12) {
			THROW(EX_GP, sel & ~0x3);
		}

		// call gates
		// examine code segment selector in call gate descriptor
		if ((newcs & ~0x3) == 0) THROW(EX_GP, 0);
		uword neww2;
		TRY(read_desc(cpu, newcs, NULL, &neww2));

		// if not code segment
		if (((neww2 >> 11) & 0x3) != 0x3)
			THROW(EX_GP, newcs & ~0x3);

		int newdpl = (neww2 >> 13) & 0x3;
		int newp = (neww2 >> 15) & 1;
		if (!newp) THROW(EX_NP, newcs & ~0x3);
		if (newdpl > cpu->cpl) THROW(EX_GP, newcs & ~0x3);

		bool conforming = (neww2 >> 8) & 0x4;
		bool gate16 = (gt == 4);
		if (!conforming && newdpl < cpu->cpl) {
			// more privilege
			OptAddr msp0, mss0;
			uword oldss = cpu->seg[SEG_SS].sel;
			uword oldsp = REGi(4);
			uword saved_cg_ss_sel = cpu->seg[SEG_SS].sel;
			uword saved_cg_ss_base = cpu->seg[SEG_SS].base;
			uword saved_cg_ss_limit = cpu->seg[SEG_SS].limit;
			uword saved_cg_ss_flags = cpu->seg[SEG_SS].flags;
			uword saved_cg_sp_mask = cpu->sp_mask;
			uword params[31];
			uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;

			if (!gate16) {
				for (int i = 0; i < wc; i++) {
					OptAddr meml;
					TRY(translate(cpu, &meml, 1, SEG_SS, (oldsp + 4 * i) & sp_mask, 4, 0));
					params[i] = laddr32(&meml);
				}
			} else {
				for (int i = 0; i < wc; i++) {
					OptAddr meml;
					TRY(translate(cpu, &meml, 1, SEG_SS, (oldsp + 2 * i) & sp_mask, 2, 0));
					params[i] = laddr16(&meml);
				}
			}

			if (!(cpu->seg[SEG_TR].flags & 0x8)) {
				TRY(translate(cpu, &msp0, 1, SEG_TR, 2 + 4 * newdpl, 2, 0));
				TRY(translate(cpu, &mss0, 1, SEG_TR, 4 + 4 * newdpl, 2, 0));
				// TODO: Check SS...
				REGi(4) = load16(cpu, &msp0);
				TRY(set_seg(cpu, SEG_SS, load16(cpu, &mss0)));
			} else {
				TRY(translate(cpu, &msp0, 1, SEG_TR, 4 + 8 * newdpl, 4, 0));
				TRY(translate(cpu, &mss0, 1, SEG_TR, 8 + 8 * newdpl, 4, 0));
				// TODO: Check SS...
				REGi(4) = load32(cpu, &msp0);
				TRY(set_seg(cpu, SEG_SS, load32(cpu, &mss0)));
			}
			sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;

			if (!isjmp) {
			if (!gate16) {
				OptAddr meml1, meml2, meml3, meml4;
				uword sp = lreg32(4);
				if (!translate(cpu, &meml1, 2, SEG_SS, (sp - 4 * 1) & sp_mask, 4, 0)) goto call_gate_pvl_fail;
				if (!translate(cpu, &meml2, 2, SEG_SS, (sp - 4 * 2) & sp_mask, 4, 0)) goto call_gate_pvl_fail;
				if (!translate(cpu, &meml3, 2, SEG_SS, (sp - 4 * (3 + wc)) & sp_mask, 4, 0)) goto call_gate_pvl_fail;
				if (!translate(cpu, &meml4, 2, SEG_SS, (sp - 4 * (4 + wc)) & sp_mask, 4, 0)) goto call_gate_pvl_fail;

				for (int i = 0; i < wc; i++) {
					OptAddr meml;
					if (!translate(cpu, &meml, 2, SEG_SS, (sp - 4 * (2 + wc - i)) & sp_mask, 4, 0)) goto call_gate_pvl_fail;
					saddr32(&meml, params[i]);
				}

				saddr32(&meml1, oldss);
				saddr32(&meml2, oldsp);
				saddr32(&meml3, cpu->seg[SEG_CS].sel);
				saddr32(&meml4, cpu->next_ip);
				set_sp(sp - 4 * (4 + wc), sp_mask);
			} else {
				OptAddr meml1, meml2, meml3, meml4;
				uword sp = lreg32(4);
				if (!translate(cpu, &meml1, 2, SEG_SS, (sp - 2 * 1) & sp_mask, 2, 0)) goto call_gate_pvl_fail;
				if (!translate(cpu, &meml2, 2, SEG_SS, (sp - 2 * 2) & sp_mask, 2, 0)) goto call_gate_pvl_fail;
				if (!translate(cpu, &meml3, 2, SEG_SS, (sp - 2 * (3 + wc)) & sp_mask, 2, 0)) goto call_gate_pvl_fail;
				if (!translate(cpu, &meml4, 2, SEG_SS, (sp - 2 * (4 + wc)) & sp_mask, 2, 0)) goto call_gate_pvl_fail;

				for (int i = 0; i < wc; i++) {
					OptAddr meml;
					if (!translate(cpu, &meml, 2, SEG_SS, (sp - 2 * (2 + wc - i)) & sp_mask, 2, 0)) goto call_gate_pvl_fail;
					saddr16(&meml, params[i]);
				}

				saddr16(&meml1, oldss);
				saddr16(&meml2, oldsp);
				saddr16(&meml3, cpu->seg[SEG_CS].sel);
				saddr16(&meml4, cpu->next_ip);
				set_sp(sp - 2 * (4 + wc), sp_mask);
			}
			}
			newcs = (newcs & 0xfffc) | newdpl;
			goto call_gate_pvl_ok;
		call_gate_pvl_fail:
			cpu->seg[SEG_SS].sel = saved_cg_ss_sel;
			cpu->seg[SEG_SS].base = saved_cg_ss_base;
			cpu->seg[SEG_SS].limit = saved_cg_ss_limit;
			cpu->seg[SEG_SS].flags = saved_cg_ss_flags;
			cpu->sp_mask = saved_cg_sp_mask;
			update_seg_flat(cpu, SEG_SS);
			REGi(4) = oldsp;
			return false;
		call_gate_pvl_ok:;
		} else {
			// same privilege
			if (!isjmp) {
			OptAddr meml1, meml2;
			uword sp = lreg32(4);
			uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
			if (gate16) {
				TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 2 * 1) & sp_mask, 2, 0));
				TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 2 * 2) & sp_mask, 2, 0));
				saddr16(&meml1, cpu->seg[SEG_CS].sel);
				saddr16(&meml2, cpu->next_ip);
				set_sp(sp - 2 * 2, sp_mask);
			} else {
				TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 4 * 1) & sp_mask, 4, 0));
				TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 4 * 2) & sp_mask, 4, 0));
				saddr32(&meml1, cpu->seg[SEG_CS].sel);
				saddr32(&meml2, cpu->next_ip);
				set_sp(sp - 4 * 2, sp_mask);
			}
			}
			newcs = (newcs & 0xfffc) | cpu->cpl;
		}

		TRY(set_seg(cpu, SEG_CS, newcs));

		cpu->next_ip = newip;
	}
	return true;
}

// 0: exception
// 1: intra PVL
// 2: inter PVL
// 3: from v8086
static int __call_isr_check_cs(CPUI386 *cpu, int sel, int ext, int *csdpl)
{
	sel = sel & 0xffff;
	OptAddr meml;
	uword off = sel & ~0x7;
	uword base;
	uword limit;
	if (sel & 0x4) {
		base = cpu->seg[SEG_LDT].base;
		limit = cpu->seg[SEG_LDT].limit;
	} else {
		base = cpu->gdt.base;
		limit = cpu->gdt.limit;
	}
	if ((sel & ~0x3) == 0 || off + 7 > limit)
		THROW(EX_GP, ext);

	TRY(translate_laddr(cpu, &meml, 1, base + off + 4, 4, 0));
	uword w2 = load32(cpu, &meml);
	int s = (w2 >> 12) & 1;
	bool code = (w2 >> 8) & 0x8;
	bool conforming = (w2 >> 8) & 0x4;
	int dpl = (w2 >> 13) & 0x3;
	int p = (w2 >> 15) & 1;
	*csdpl = dpl;
	if (!s || !code || dpl > cpu->cpl)
		THROW(EX_GP, (sel & ~0x3) | ext);

	if (!p) THROW(EX_NP, sel & ~0x3);

	if (!conforming && dpl < cpu->cpl) {
		if (!(cpu->flags & VM)) {
			return 2;
		} else {
			if (dpl != 0) {
				dolog("__call_isr_check_cs fail1: %d %d %d\n", conforming, dpl, cpu->cpl);
				THROW(EX_GP, (sel & ~0x3) | ext);
			} else {
				return 3;
			}
		}
	} else {
		if (cpu->flags & VM) {
			THROW(EX_GP, (sel & ~0x3) | ext);
		} else {
			if (conforming || dpl == cpu->cpl) {
				return 1;
			} else {
				dolog("__call_isr_check_cs fail2: %d %d %d\n", conforming, dpl, cpu->cpl);
				THROW(EX_GP, (sel & ~0x3) | ext);
			}
		}
	}
}

/* exc_ring buffer and EXC_RING_SIZE/MASK defined earlier (near cpu_monitor_dump) */
/* Delayed ring buffer dump: set to wall-clock time when a trigger fires;
 * cpui386_step checks and dumps 2 seconds later to capture post-trigger events. */
static uint32_t exc_ring_delayed_dump_time = 0;
static int exc_ring_trigger_pos = -1;  /* position where trigger fired (for marking in dump) */

static void dump_exc_ring(const char *label) {
	int n = exc_ring_pos > EXC_RING_SIZE ? EXC_RING_SIZE : exc_ring_pos;
	int start = exc_ring_pos - n;
	dolog("=== %s: Last %d exceptions (trigger@%d) ===\n", label, n, exc_ring_trigger_pos);
	for (int i = start; i < exc_ring_pos; i++) {
		int j = i & EXC_RING_MASK;
		const char *marker = (i == exc_ring_trigger_pos) ? ">>>" : "   ";
		if (exc_ring[j].no == 14)
			dolog("%s%c exc %2d @ %04x:%08x err=%x CR2=%08x\n",
				marker, exc_ring[j].mode, exc_ring[j].no,
				exc_ring[j].cs, exc_ring[j].ip, exc_ring[j].err, exc_ring[j].cr2);
		else
			dolog("%s%c exc %2d @ %04x:%08x err=%x\n",
				marker, exc_ring[j].mode, exc_ring[j].no,
				exc_ring[j].cs, exc_ring[j].ip, exc_ring[j].err);
	}
}

/* VM86 software INT n with IOPL=3 follows real-mode IVT semantics:
 * vector from linear 0:0 table, push FLAGS/CS/IP to VM stack, clear IF/TF. */
static bool IRAM_ATTR call_isr_vm86_softint(CPUI386 *cpu, int no)
{
	OptAddr memv, meml1, meml2, meml3;
	uword w1;
	int newcs;
	uword newip;
	uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
	uword sp = lreg32(4);

	TRY(translate_laddr(cpu, &memv, 1, (uword)no * 4, 4, 3));
	w1 = load32(cpu, &memv);
	newcs = w1 >> 16;
	newip = w1 & 0xffff;

	/* DPMI trace: save paged IVT target and first byte at handler */
	if (unlikely(no == 0x2f && cpu_diag_enabled && win386_diag.active &&
	    lreg16(0) == 0x1687)) {
		dpmi_trace.ivt_paged_cs = newcs;
		dpmi_trace.ivt_paged_ip = newip;
		dpmi_trace.ivt_mismatch = (w1 != *(uint32_t*)(cpu->phys_mem + 0x2f * 4));
		/* Read first byte through paging (VMM may remap handler page) */
		uint32_t target_lin = (uint32_t)newcs * 16 + newip;
		OptAddr meml_fb;
		if (translate_laddr(cpu, &meml_fb, 1, target_lin, 1, 3))
			dpmi_trace.target_first_byte = load8(cpu, &meml_fb);
		/* Also check physical byte for comparison */
		if (target_lin < (uint32_t)cpu->phys_mem_size)
			dpmi_trace.target_phys_byte = cpu->phys_mem[target_lin];
	}

	TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 2 * 1) & sp_mask, 2, 3));
	TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 2 * 2) & sp_mask, 2, 3));
	TRY(translate(cpu, &meml3, 2, SEG_SS, (sp - 2 * 3) & sp_mask, 2, 3));

	refresh_flags(cpu);
	cpu->cc.mask = 0;
	saddr16(&meml1, cpu->flags);
	saddr16(&meml2, cpu->seg[SEG_CS].sel);
	saddr16(&meml3, cpu->ip);
	set_sp(sp - 2 * 3, sp_mask);

	TRY(set_seg(cpu, SEG_CS, newcs));
	cpu->next_ip = newip;
	cpu->ip = newip;
	cpu->flags &= ~(IF | TF);
	return true;
}

static bool IRAM_ATTR call_isr(CPUI386 *cpu, int no, bool pusherr, int ext)
{
	/* Exception ring buffer & PTE watchpoint — only active when diagnostics enabled.
	 * Zero overhead when cpu_diag_enabled is false. */
	if (unlikely(cpu_diag_enabled)) {
		static int diag_generation = -1;
		if (diag_generation != cpu->diag_gen) {
			diag_generation = cpu->diag_gen;
			exc_ring_pos = 0;
			exc_ring_delayed_dump_time = 0;
			exc_ring_trigger_pos = -1;
			pf_diag.active = false;
		}
		/* Record ALL exceptions in ring buffer (faults/traps and external IRQs). */
		if (ext || no < 32) {
			int idx = exc_ring_pos & EXC_RING_MASK;
			exc_ring[idx].no = no;
			exc_ring[idx].mode = (cpu->flags & VM) ? 'V' : ((cpu->cr0 & 1) ? 'P' : 'R');
			exc_ring[idx].cs = cpu->seg[SEG_CS].sel;
			exc_ring[idx].ip = cpu->ip;
			exc_ring[idx].err = pusherr ? cpu->excerr : 0;
			exc_ring[idx].cr2 = (no == 14) ? cpu->cr2 : 0;
			exc_ring_pos++;
		}

		/* Exception loop detector: if the same exception fires at the
		 * same CS:EIP many times in a row, auto-dump and report.
		 * Catches infinite #GP/#TS loops where the handler can't advance. */
		{
			static uint8_t loop_last_no = 0xff;
			static uint16_t loop_last_cs = 0;
			static uint32_t loop_last_ip = 0;
			static uint16_t loop_repeat = 0;
			static bool loop_dumped = false;
			if (no < 32 && no == loop_last_no &&
			    cpu->seg[SEG_CS].sel == loop_last_cs && cpu->ip == loop_last_ip) {
				loop_repeat++;
				if (loop_repeat == 64 && !loop_dumped) {
					dolog("*** Exception loop detected: #%d x%u at %04x:%08x err=%04x ***\n",
						no, loop_repeat, loop_last_cs, loop_last_ip,
						pusherr ? cpu->excerr : 0);
					/* Dump faulting instruction bytes */
					{
						uint8_t ibuf[16];
						int ilen = 0;
						OptAddr imeml;
						for (int bi = 0; bi < 16; bi++) {
							if (translate(cpu, &imeml, 1, SEG_CS,
							    (cpu->ip + bi) & 0xffffffff, 1, 0)) {
								ibuf[bi] = load8(cpu, &imeml);
								ilen = bi + 1;
							} else break;
						}
						if (ilen > 0) {
							dolog("  fault insn @%04x:%08x:",
								cpu->seg[SEG_CS].sel, cpu->ip);
							for (int bi = 0; bi < ilen; bi++)
								dolog(" %02x", ibuf[bi]);
							dolog("\n");
						}
					}
					/* If error code looks like a selector, dump that descriptor */
					if (pusherr && (cpu->excerr & 0xfff8)) {
						uint16_t fsel = cpu->excerr & 0xfffc;
						uword fw1 = 0, fw2 = 0;
						if (read_desc_noexcept(cpu, fsel, &fw1, &fw2)) {
							uint32_t db = ((fw1 >> 16) | ((fw2 & 0xff) << 16) |
								       (fw2 & 0xff000000));
							uint32_t dl = (fw1 & 0xffff) | (fw2 & 0x0f0000);
							if (fw2 & 0x800000) dl = (dl << 12) | 0xfff;
							uint16_t df = (fw2 >> 8) & 0xffff;
							dolog("  fault sel %04x: w1=%08x w2=%08x base=%08x lim=%08x fl=%04x (s=%u type=%x dpl=%u p=%u)\n",
								fsel, fw1, fw2, db, dl, df,
								(df >> 4) & 1, df & 0xf,
								(df >> 5) & 3, (df >> 7) & 1);
						} else {
							dolog("  fault sel %04x: read_desc FAILED\n", fsel);
						}
					}
					/* Dump handler code bytes and stack frame for the looping vector */
					{
						uword iw1 = 0, iw2 = 0;
						if (read_idt_vector_raw_noexcept(cpu, no, &iw1, &iw2)) {
							int igt = (iw2 >> 8) & 0xf;
							bool ig16 = (igt == 6 || igt == 7);
							uint16_t hcs_sel = iw1 >> 16;
							uword hip = ig16 ? (iw1 & 0xffff)
								         : ((iw1 & 0xffff) | (iw2 & 0xffff0000));
							/* Read handler CS descriptor from GDT/LDT */
							uword hw1 = 0, hw2 = 0;
							if (read_desc_noexcept(cpu, hcs_sel, &hw1, &hw2)) {
								uint32_t hbase = (hw1 >> 16) | ((hw2 & 0xff) << 16) |
									         (hw2 & 0xff000000);
								uint32_t hlim = (hw1 & 0xffff) | (hw2 & 0x0f0000);
								if (hw2 & 0x800000) hlim = (hlim << 12) | 0xfff;
								int hdb = (hw2 >> 22) & 1; /* D/B bit */
								dolog("  handler CS=%04x: base=%08x lim=%08x D=%d type=%02x\n",
									hcs_sel, hbase, hlim, hdb, (hw2 >> 8) & 0xff);
								/* Dump first 48 bytes of handler code */
								uint32_t hlin = hbase + hip;
								dolog("  handler code @%04x:%08x (lin=%08x):",
									hcs_sel, hip, hlin);
								for (int bi = 0; bi < 48 && hlin + bi < (uint32_t)cpu->phys_mem_size; bi++)
									dolog(" %02x", cpu->phys_mem[hlin + bi]);
								dolog("\n");
								/* If first insn is CALL rel16 (e8), follow and dump target */
								if (hlin + 2 < (uint32_t)cpu->phys_mem_size &&
								    cpu->phys_mem[hlin] == 0xe8) {
									int16_t cdisp = (int16_t)(cpu->phys_mem[hlin+1] |
									                          (cpu->phys_mem[hlin+2] << 8));
									uint16_t ctgt = (uint16_t)(hip + 3 + cdisp);
									uint32_t ctgt_lin = hbase + ctgt;
									dolog("  common handler @%04x:%04x (lin=%08x):\n",
										hcs_sel, ctgt, ctgt_lin);
									for (int row = 0; row < 24; row++) {
										int off = row * 16;
										if (ctgt_lin + off >= (uint32_t)cpu->phys_mem_size) break;
										dolog("    %04x:", ctgt + off);
										for (int bi = 0; bi < 16 && ctgt_lin + off + bi < (uint32_t)cpu->phys_mem_size; bi++)
											dolog(" %02x", cpu->phys_mem[ctgt_lin + off + bi]);
										dolog("\n");
									}
								}
							}
							/* Dump ring-0 stack frame from TSS */
							int tr_type = cpu->seg[SEG_TR].flags & 0xf;
							uint32_t tr_base = cpu->seg[SEG_TR].base;
							uint16_t r0_sp, r0_ss;
							if (tr_type == 1 || tr_type == 3) {
								/* 16-bit TSS: SP0@2, SS0@4 */
								if (tr_base + 5 < (uint32_t)cpu->phys_mem_size) {
									r0_sp = cpu->phys_mem[tr_base + 2] |
									        (cpu->phys_mem[tr_base + 3] << 8);
									r0_ss = cpu->phys_mem[tr_base + 4] |
									        (cpu->phys_mem[tr_base + 5] << 8);
									/* Read SS0 descriptor for base */
									uword sw1 = 0, sw2 = 0;
									if (read_desc_noexcept(cpu, r0_ss, &sw1, &sw2)) {
										uint32_t ss_base = (sw1 >> 16) | ((sw2 & 0xff) << 16) |
											           (sw2 & 0xff000000);
										uint32_t ss_lim = (sw1 & 0xffff) | (sw2 & 0x0f0000);
										if (sw2 & 0x800000) ss_lim = (ss_lim << 12) | 0xfff;
										int ss_db = (sw2 >> 22) & 1;
										dolog("  r0 stack: SS0=%04x(base=%08x lim=%08x B=%d) SP0=%04x\n",
											r0_ss, ss_base, ss_lim, ss_db, r0_sp);
										/* Dump 16 bytes below SP0 (the pushed frame) */
										uint32_t slin = ss_base + r0_sp;
										if (ig16) {
											/* 16-bit gate pushes: SS(2) SP(2) FL(2) CS(2) IP(2) [err(2)] */
											int frame_sz = pusherr ? 12 : 10;
											uint32_t frame_top = slin - frame_sz;
											dolog("  r0 frame @%08x (16-bit, %d bytes):",
												frame_top, frame_sz);
											for (int bi = 0; bi < frame_sz && frame_top + bi < (uint32_t)cpu->phys_mem_size; bi++)
												dolog(" %02x", cpu->phys_mem[frame_top + bi]);
											dolog("\n");
											/* Decode frame fields */
											if (frame_top + frame_sz <= (uint32_t)cpu->phys_mem_size) {
												uint8_t *f = &cpu->phys_mem[frame_top];
												if (pusherr) {
													dolog("  frame: err=%04x IP=%04x CS=%04x FL=%04x SP=%04x SS=%04x\n",
														f[0]|(f[1]<<8), f[2]|(f[3]<<8),
														f[4]|(f[5]<<8), f[6]|(f[7]<<8),
														f[8]|(f[9]<<8), f[10]|(f[11]<<8));
												} else {
													dolog("  frame: IP=%04x CS=%04x FL=%04x SP=%04x SS=%04x\n",
														f[0]|(f[1]<<8), f[2]|(f[3]<<8),
														f[4]|(f[5]<<8), f[6]|(f[7]<<8),
														f[8]|(f[9]<<8));
												}
											}
										}
									}
								}
							} else if (tr_type == 9 || tr_type == 11) {
								/* 32-bit TSS: ESP0@4, SS0@8 */
								if (tr_base + 11 < (uint32_t)cpu->phys_mem_size) {
									uint32_t r0_esp = *(uint32_t*)&cpu->phys_mem[tr_base + 4];
									r0_ss = cpu->phys_mem[tr_base + 8] |
									        (cpu->phys_mem[tr_base + 9] << 8);
									dolog("  r0 stack (32-bit TSS): SS0=%04x ESP0=%08x\n",
										r0_ss, r0_esp);
								}
							}
						}
					}
					cpu_monitor_dump(cpu, "exc loop");
					loop_dumped = true;
				}
			} else {
				loop_last_no = no;
				loop_last_cs = cpu->seg[SEG_CS].sel;
				loop_last_ip = cpu->ip;
				loop_repeat = 1;
				loop_dumped = false;
			}
		}

		/* Track nested exceptions while PF handler is active */
		if (unlikely(pf_diag.active) && (ext || no < 32)) {
			pf_diag.nested_exc_count++;
			if (no == 14) {
				pf_diag.nested_pf_count++;
				pf_diag.nested_pf_last_cr2 = cpu->cr2;
			}
		}

		/* PTE watchpoint: set up on BFF-range #PF with not-present PTE.
		 * Silently arms the watchpoint — output only on failure. */
		if (no == 14 && cpu->cr2 >= 0xBFF00000 && cpu->cr2 < 0xC0000000 && !pf_diag.active) {
			uword a20 = cpu->a20_mask;
			uword cr3_base = (cpu->cr3 & ~0xfff) & a20;
			uword pdi = (cpu->cr2 >> 22) & 0x3ff;
			uword pti = (cpu->cr2 >> 12) & 0x3ff;
			uword pde_addr = cr3_base + pdi * 4;
			if ((uint64_t)pde_addr + 4 <= (uint64_t)cpu->phys_mem_size) {
				uword pde = pload32(cpu, pde_addr);
				if (pde & 1) {
					uword pt_base = (pde & ~0xfff) & a20;
					uword pte_addr = pt_base + pti * 4;
					if ((uint64_t)pte_addr + 4 <= (uint64_t)cpu->phys_mem_size) {
						uword pte = pload32(cpu, pte_addr);
						if (!(pte & 1) && pte != 0) {
							/* Arm watchpoint — demand-load or swapped PTE */
							pf_diag.active = true;
							pf_diag.watch_phys = pte_addr;
							pf_diag.initial_pte = pte;
							pf_diag.cr2 = cpu->cr2;
							pf_diag.write_count = 0;
							pf_diag.nested_exc_count = 0;
							pf_diag.nested_pf_count = 0;
							pf_diag.nested_pf_last_cr2 = 0;
							pf_diag.cr3_write_count = 0;
							pf_diag.invlpg_count = 0;
							pf_diag.oob_store_count = 0;
							pf_diag.oob_store_last_addr = 0;
							pf_diag.last_write_val = 0;
							pf_diag.last_write_eip = 0;
							pf_diag.last_write_cs = 0;
						}
					}
				}
			}
		}

		/* Trigger delayed dump on CS=0137 #PF (Win95 flat user-mode LDT selector).
		 * Mark trigger position but keep recording. */
		if (no == 14 && cpu->seg[SEG_CS].sel == 0x0137) {
			exc_ring_delayed_dump_time = get_uticks();
			exc_ring_trigger_pos = exc_ring_pos - 1; /* mark the entry we just wrote */
		}
	}

	/* WfW boot monitor counters — only active when win386 diag is running.
	 * Note: exc_counts[] is incremented at the exception dispatch site
	 * (not here) to avoid counting PIC IRQs that share vectors 0-31. */
	if (unlikely(cpu_diag_enabled) && win386_diag.active) {
		/* Dense causal timeline: PM ring-3 entries + VM86 low vectors. */
		if ((cpu->cr0 & 1) && !(cpu->flags & VM) && cpu->cpl == 3) {
			uint32_t ex = 0;
			if (no == EX_PF)
				ex = cpu->cr2;
			else if (pusherr)
				ex = cpu->excerr;
			/* Vector 30h is very chatty in this workload and hides decisive events. */
			if (no != 0x30)
				wfw_flow_push_here(cpu, (no < 32) ? 'E' : 'I', (uint8_t)no, ex);
		} else if ((cpu->flags & VM) && !ext && no < 0x20) {
			uint32_t ex = pusherr ? cpu->excerr : 0;
			wfw_flow_push_here(cpu, 'L', (uint8_t)no, ex);
		}
		if (!ext && no < 32) {
			wfw_monitor.lowvec_softint_total++;
			wfw_monitor.lowvec_softint_hist[no]++;
		}
		if (no < 32) {
			wfw_monitor.has_fatal = true;
			wfw_monitor.fatal_excno = no;
			wfw_monitor.fatal_cs = cpu->seg[SEG_CS].sel;
			wfw_monitor.fatal_ds = cpu->seg[SEG_DS].sel;
			wfw_monitor.fatal_es = cpu->seg[SEG_ES].sel;
			wfw_monitor.fatal_eip = cpu->ip;
			wfw_monitor.fatal_ss = cpu->seg[SEG_SS].sel;
			wfw_monitor.fatal_esp = REGi(4);
			refresh_flags(cpu);
			wfw_monitor.fatal_efl = cpu->flags;
			wfw_monitor.fatal_cpl = cpu->cpl;
			wfw_monitor.fatal_vm = (cpu->flags & VM) ? 1 : 0;
			wfw_monitor.fatal_ext = (uint8_t)(ext ? 1 : 0);
			wfw_monitor.fatal_pusherr = (uint8_t)(pusherr ? 1 : 0);
			wfw_capture_decode_and_bytes(cpu, wfw_monitor.fatal_decoded,
						     sizeof(wfw_monitor.fatal_decoded),
						     wfw_monitor.fatal_bytes,
						     &wfw_monitor.fatal_blen);
			if (wfw_monitor.iopl3_captured && !wfw_monitor.post_iopl3_fault_captured) {
				wfw_monitor.post_iopl3_fault_captured = true;
				wfw_monitor.post_iopl3_excno = no;
				wfw_monitor.post_iopl3_cs = cpu->seg[SEG_CS].sel;
				wfw_monitor.post_iopl3_eip = cpu->ip;
				wfw_monitor.post_iopl3_fl = cpu->flags;
				wfw_monitor.post_iopl3_err = pusherr ? cpu->excerr : 0;
				wfw_monitor.post_iopl3_last_evt = (wfw_monitor.vm86_int_evt_count > 0)
					? (int8_t)(wfw_monitor.vm86_int_evt_count - 1) : (int8_t)-1;
				wfw_capture_decode_and_bytes(cpu, wfw_monitor.post_iopl3_decoded,
							     sizeof(wfw_monitor.post_iopl3_decoded),
							     wfw_monitor.post_iopl3_bytes,
							     &wfw_monitor.post_iopl3_blen);
			}
		}
		if (cpu->flags & VM)
			wfw_monitor.vm86_ints++;
		else if (cpu->cr0 & 1) {
			wfw_monitor.pm_ints++;
			if (cpu->cpl == 3) {
				wfw_monitor.pm_ring3_ints++;
				/* Capture initial PM ring-3 state (first ring-3 event) */
				if (!wfw_monitor.r3_first_captured) {
					wfw_monitor.r3_first_captured = true;
					wfw_monitor.r3_first_cs = cpu->seg[SEG_CS].sel;
					wfw_monitor.r3_first_ds = cpu->seg[SEG_DS].sel;
					wfw_monitor.r3_first_es = cpu->seg[SEG_ES].sel;
					wfw_monitor.r3_first_ss = cpu->seg[SEG_SS].sel;
					wfw_monitor.r3_first_eip = cpu->ip;
					wfw_monitor.r3_first_esp = REGi(4);
					wfw_monitor.r3_first_eax = REGi(0);
					/* Dump PSP from ES (DPMI spec: ES=PSP selector after mode switch).
					 * PSP:80h = command tail length, PSP:81h = command tail.
					 * PSP:2Ch = environment (selector in PM, segment in RM). */
					uint32_t es_base = cpu->seg[SEG_ES].base;
					wfw_monitor.psp_ds_base = es_base; /* reuse field for ES base */
					if (es_base + 0x100 <= (uint32_t)cpu->phys_mem_size) {
						for (int j = 0; j < 16; j++)
							wfw_monitor.psp_header[j] = cpu->phys_mem[es_base + j];
						wfw_monitor.psp_cmdtail_len = cpu->phys_mem[es_base + 0x80];
						int clen = wfw_monitor.psp_cmdtail_len;
						if (clen > 127) clen = 127;
						for (int j = 0; j < clen; j++)
							wfw_monitor.psp_cmdtail[j] = cpu->phys_mem[es_base + 0x81 + j];
						wfw_monitor.psp_cmdtail[clen] = '\0';
						/* Get environment from PSP:2Ch.
						 * In PM after DPMI mode switch, VMM converts this from
						 * a real-mode segment to a PM selector. Detect which by
						 * checking if bit 2 (TI=LDT) is set. */
							wfw_monitor.psp_env_seg = cpu->phys_mem[es_base + 0x2c] |
								(cpu->phys_mem[es_base + 0x2d] << 8);
							uint32_t env_lin = 0;
							bool env_found = false;
							bool env_from_sel = false;
							if ((wfw_monitor.psp_env_seg & 4) && (cpu->cr0 & CR0_PG)) {
								/* PM selector — look up base in LDT via page walk */
								uint32_t ldt_lin = cpu->seg[SEG_LDT].base;
								uint32_t desc_off = (wfw_monitor.psp_env_seg >> 3) * 8;
								if (desc_off + 7 <= cpu->seg[SEG_LDT].limit) {
									/* Walk page table: LDT linear → physical */
									uint32_t pde_addr = (cpu->cr3 & 0xfffff000)
										+ (((ldt_lin + desc_off) >> 22) & 0x3ff) * 4;
									if (pde_addr + 4 <= (uint32_t)cpu->phys_mem_size) {
										uint32_t pde = *(uint32_t*)&cpu->phys_mem[pde_addr];
										if (pde & 1) {
											uint32_t pte_addr = (pde & 0xfffff000)
											+ (((ldt_lin + desc_off) >> 12) & 0x3ff) * 4;
										if (pte_addr + 4 <= (uint32_t)cpu->phys_mem_size) {
											uint32_t pte = *(uint32_t*)&cpu->phys_mem[pte_addr];
											if (pte & 1) {
												uint32_t phys = (pte & 0xfffff000)
													+ ((ldt_lin + desc_off) & 0xfff);
													if (phys + 8 <= (uint32_t)cpu->phys_mem_size) {
														uint8_t *d = &cpu->phys_mem[phys];
														env_lin = d[2] | (d[3]<<8) | (d[4]<<16) | (d[7]<<24);
														env_found = true;
														env_from_sel = true;
													}
												}
											}
										}
									}
							}
						}
						if (!env_found) {
							/* Fallback: treat as real-mode segment */
							env_lin = (uint32_t)wfw_monitor.psp_env_seg << 4;
							env_found = (env_lin > 0);
							}
							wfw_monitor.psp_env_lin = env_lin;
							wfw_monitor.psp_env_is_sel = env_from_sel;
						if (env_found && env_lin + 256 <= (uint32_t)cpu->phys_mem_size) {
							for (int j = 0; j < 255; j++)
								wfw_monitor.psp_env[j] = cpu->phys_mem[env_lin + j];
							wfw_monitor.psp_env[255] = '\0';
						}
					}
				}
				if (no < 32) {
					wfw_monitor.pm_ring3_faults++;
					wfw_monitor.pm_ring3_exc_hist[no]++;
					wfw_monitor.pm_r3_last_cs = cpu->seg[SEG_CS].sel;
					wfw_monitor.pm_r3_last_eip = cpu->ip;
					wfw_monitor.pm_r3_last_exc = no;
					/* Capture ring-3 #GP details */
					if (no == EX_GP && wfw_monitor.r3_gp_count < 8) {
						int gi = wfw_monitor.r3_gp_count++;
						uint8_t gp_bytes[8];
						uint8_t gp_blen = 0;
						wfw_monitor.r3_gp_log[gi].cs = cpu->seg[SEG_CS].sel;
						wfw_monitor.r3_gp_log[gi].eip = cpu->ip;
						wfw_monitor.r3_gp_log[gi].err = cpu->excerr;
						wfw_capture_decode_and_bytes(cpu,
							wfw_monitor.r3_gp_log[gi].dec,
							sizeof(wfw_monitor.r3_gp_log[gi].dec),
							gp_bytes, &gp_blen);
					}
					/* Capture ring-3 #PF details (CR2 + error code + decode). */
					if (no == EX_PF && wfw_monitor.r3_pf_count < 8) {
						int pi = wfw_monitor.r3_pf_count++;
						uint8_t pf_bytes[8];
						uint8_t pf_blen = 0;
						wfw_monitor.r3_pf_log[pi].cs = cpu->seg[SEG_CS].sel;
						wfw_monitor.r3_pf_log[pi].eip = cpu->ip;
						wfw_monitor.r3_pf_log[pi].err = cpu->excerr;
						wfw_monitor.r3_pf_log[pi].cr2 = cpu->cr2;
						wfw_capture_decode_and_bytes(cpu,
							wfw_monitor.r3_pf_log[pi].dec,
							sizeof(wfw_monitor.r3_pf_log[pi].dec),
							pf_bytes, &pf_blen);
					}
					/* High-address #PF page table snapshot (ring buffer, last 16).
					 * Captures PDE/PTE state BEFORE VMM's handler runs. */
					if (no == EX_PF && cpu->cr2 >= 0x80000000u) {
						int hi = wfw_monitor.r3_hipf_pos % 16;
						wfw_monitor.r3_hipf[hi].cr2 = cpu->cr2;
						wfw_monitor.r3_hipf[hi].err = cpu->excerr;
						wfw_monitor.r3_hipf[hi].cs = cpu->seg[SEG_CS].sel;
						wfw_monitor.r3_hipf[hi].eip = cpu->ip;
						wfw_monitor.r3_hipf[hi].cr3 = cpu->cr3;
						/* Walk page tables from CR3 */
						uint32_t _cr2 = cpu->cr2;
						uint32_t _cr3 = cpu->cr3;
						uint32_t pde_pa = (_cr3 & ~0xFFFu) | ((_cr2 >> 20) & 0xFFCu);
						uint32_t pde = 0, pte = 0;
						if (pde_pa + 3 < (uint32_t)cpu->phys_mem_size)
							pde = *(uint32_t *)(cpu->phys_mem + pde_pa);
						if (pde & 1) {
							uint32_t pte_pa = (pde & ~0xFFFu) | ((_cr2 >> 10) & 0xFFCu);
							if (pte_pa + 3 < (uint32_t)cpu->phys_mem_size)
								pte = *(uint32_t *)(cpu->phys_mem + pte_pa);
						}
						wfw_monitor.r3_hipf[hi].pde = pde;
						wfw_monitor.r3_hipf[hi].pte = pte;
						wfw_monitor.r3_hipf_pos++;
						if (wfw_monitor.r3_hipf_count < 16)
							wfw_monitor.r3_hipf_count++;
						/* Full register snapshot for last high-address PF
						 * (the unresolvable one is always last) */
						{
							wfw_monitor.fatal_pf.cr2 = cpu->cr2;
							wfw_monitor.fatal_pf.cs = cpu->seg[SEG_CS].sel;
							wfw_monitor.fatal_pf.eip = cpu->ip;
							wfw_monitor.fatal_pf.eax = REGi(0);
							wfw_monitor.fatal_pf.ebx = REGi(3);
							wfw_monitor.fatal_pf.ecx = REGi(1);
							wfw_monitor.fatal_pf.edx = REGi(2);
							wfw_monitor.fatal_pf.esi = REGi(6);
							wfw_monitor.fatal_pf.edi = REGi(7);
							wfw_monitor.fatal_pf.ebp = REGi(5);
							wfw_monitor.fatal_pf.esp = REGi(4);
							wfw_monitor.fatal_pf.ds = cpu->seg[SEG_DS].sel;
							wfw_monitor.fatal_pf.es = cpu->seg[SEG_ES].sel;
							wfw_monitor.fatal_pf.ss = cpu->seg[SEG_SS].sel;
							wfw_monitor.fatal_pf.fs = cpu->seg[SEG_FS].sel;
							wfw_monitor.fatal_pf.gs = cpu->seg[SEG_GS].sel;
							wfw_monitor.fatal_pf.ds_base = cpu->seg[SEG_DS].base;
							wfw_monitor.fatal_pf.es_base = cpu->seg[SEG_ES].base;
							wfw_monitor.fatal_pf.ss_base = cpu->seg[SEG_SS].base;
							wfw_monitor.fatal_pf.ds_limit = cpu->seg[SEG_DS].limit;
							wfw_monitor.fatal_pf.es_limit = cpu->seg[SEG_ES].limit;
							wfw_monitor.fatal_pf.cs_limit = cpu->seg[SEG_CS].limit;
							wfw_monitor.fatal_pf_captured = true;
						}
					}
				} else {
					wfw_monitor.pm_ring3_irqs++;
				}
				/* Track PM ring-3 software INTs */
				if (no == 0x2f) {
					uint16_t ax2f = REGi(0) & 0xffff;
					if (wfw_monitor.pm_r3_int2f_count < 16)
						wfw_monitor.pm_r3_int2f_ax[wfw_monitor.pm_r3_int2f_count] = ax2f;
					wfw_monitor.pm_r3_int2f_count++;
					/* For 168Ah, capture DS:SI string */
					if (ax2f == 0x168a && wfw_monitor.pm_r3_168a_count == 0) {
						uint32_t lin = cpu->seg[SEG_DS].base + (REGi(6) & 0xffff);
						int j;
						for (j = 0; j < 15; j++) {
							if (lin + j >= (uint32_t)cpu->phys_mem_size) break;
							uint8_t ch = cpu->phys_mem[lin + j];
							if (ch == 0) break;
							wfw_monitor.pm_r3_168a_str[j] = ch;
						}
						wfw_monitor.pm_r3_168a_str[j] = '\0';
					}
					if ((ax2f & 0xff00) == 0x1600)
						wfw_monitor.pm_r3_168a_count++;
					wfw_monitor.pending_int2f = true;
					wfw_monitor.pending_int2f_ax = ax2f;
				}
				if (no == 0x21) {
					int idx = wfw_monitor.pm_r3_int21_count % 16;
					uint8_t ah = (REGi(0) >> 8) & 0xff;
					wfw_monitor.pm_r3_int21_ah_ring[idx] = ah;
					if (wfw_monitor.pm_r3_int21_first_count < 16)
						wfw_monitor.pm_r3_int21_ah_first[wfw_monitor.pm_r3_int21_first_count++] = ah;
					wfw_monitor.pm_r3_int21_count++;
					wfw_monitor.pending_int21 = true;
					wfw_monitor.pending_int21_ah = ah;
					/* Capture filename for AH=3Dh (Open File).
					 * Use translated DS:DX reads (not raw phys_mem) so PM ring-3
					 * paging/segments still produce valid names in the monitor. */
					if (ah == 0x3d && wfw_monitor.r3_fopen_count < 8) {
						int fi = wfw_monitor.r3_fopen_count++;
						uword off = (uword)(REGi(2) & 0xffff);
						cpu_diag_copy_seg_string(cpu, SEG_DS, off, cpu->cpl, 0,
								 wfw_monitor.r3_fopen_log[fi].name,
								 sizeof(wfw_monitor.r3_fopen_log[fi].name));
					}
				} else if (no == 0x31) {
					uint16_t ax31 = REGi(0) & 0xffff;
					int idx = wfw_monitor.pm_r3_int31_count % 16;
					wfw_monitor.pm_r3_int31_ax_ring[idx] = ax31;
					if (wfw_monitor.pm_r3_int31_first_count < 16)
						wfw_monitor.pm_r3_int31_ax_first[wfw_monitor.pm_r3_int31_first_count++] = ax31;
					/* Histogram: count calls per function code */
					int hi;
					for (hi = 0; hi < wfw_monitor.int31_hist_len; hi++)
						if (wfw_monitor.int31_hist[hi].ax == ax31) break;
					if (hi < wfw_monitor.int31_hist_len)
						wfw_monitor.int31_hist[hi].count++;
					else if (wfw_monitor.int31_hist_len < 24) {
						wfw_monitor.int31_hist[wfw_monitor.int31_hist_len].ax = ax31;
						wfw_monitor.int31_hist[wfw_monitor.int31_hist_len].count = 1;
						wfw_monitor.int31_hist_len++;
					}
					/* Capture BX (selector) for Get Segment Base (0006) calls */
					if (ax31 == 0x0006) {
						uint16_t bx = REGi(3) & 0xffff;
						if (wfw_monitor.int31_0006_count < 16)
							wfw_monitor.int31_0006_bx[wfw_monitor.int31_0006_count++] = bx;
						/* Last-16 ring buffer for selectors */
						wfw_monitor.int31_0006_bx_ring[wfw_monitor.int31_0006_ret_count % 16] = bx;
						wfw_monitor.pending_dpmi_bx = bx;
					}
					/* Capture SetSegBase(0007) target bases for selector 0x100F */
					if (ax31 == 0x0007 && (REGi(3) & 0xffff) == 0x100f) {
						uint32_t base = ((uint32_t)(REGi(1) & 0xffff) << 16) | (REGi(2) & 0xffff);
						if (wfw_monitor.seg100f_base_count < 8) {
							/* Only store unique bases */
							int dup = 0;
							for (int k = 0; k < wfw_monitor.seg100f_base_count; k++)
								if (wfw_monitor.seg100f_bases[k] == base) { dup = 1; break; }
							if (!dup)
								wfw_monitor.seg100f_bases[wfw_monitor.seg100f_base_count++] = base;
						}
					}
					/* Log all descriptor-mutating DPMI calls */
					if (wfw_monitor.dpmi_seg_log_count < 32) {
						uint16_t f = ax31;
						if (f == 0x0007 || f == 0x0008 || f == 0x0009 ||
						    f == 0x000A || f == 0x000B || f == 0x000C ||
						    f == 0x0001) {
							int si = wfw_monitor.dpmi_seg_log_count++;
							wfw_monitor.dpmi_seg_log[si].func = f;
							wfw_monitor.dpmi_seg_log[si].sel = REGi(3) & 0xffff;
							wfw_monitor.dpmi_seg_log[si].val = 0;
							wfw_monitor.dpmi_seg_log[si].extra = 0;
							wfw_monitor.dpmi_seg_log[si].ret_cf = 0;
							wfw_monitor.dpmi_seg_log[si].ret_valid = 0;
							if (f == 0x0001) {
								/* AllocLDT: CX=count, return AX=first sel */
								wfw_monitor.dpmi_seg_log[si].sel = 0; /* filled on IRET */
								wfw_monitor.dpmi_seg_log[si].val = REGi(1) & 0xffff; /* CX=count */
							} else if (f == 0x000C) {
								/* SetDesc: BX=sel, ES:(E)DI→8-byte descriptor */
								/* Read descriptor bytes from ES:(E)DI with page walk */
								{
									uint32_t di = cpu->code16 ? (REGi(7) & 0xffff) : REGi(7);
									uint32_t desc_la = cpu->seg[SEG_ES].base + di;
									uint32_t d0 = 0, d1 = 0;
									for (int dw = 0; dw < 2; dw++) {
										uint32_t la = desc_la + dw * 4;
										uint32_t pa = la;
										if (cpu->cr0 & (1u << 31)) {
											uint32_t pde_a = (cpu->cr3 & 0xfffff000) | ((la >> 20) & 0xffc);
											if (pde_a + 4 > (uint32_t)cpu->phys_mem_size) break;
											uint32_t pde = *(uint32_t *)&cpu->phys_mem[pde_a];
											if (!(pde & 1)) break;
											uint32_t pte_a = (pde & 0xfffff000) | ((la >> 10) & 0xffc);
											if (pte_a + 4 > (uint32_t)cpu->phys_mem_size) break;
											uint32_t pte = *(uint32_t *)&cpu->phys_mem[pte_a];
											if (!(pte & 1)) break;
											pa = (pte & 0xfffff000) | (la & 0xfff);
										}
										if (pa + 4 <= (uint32_t)cpu->phys_mem_size) {
											if (dw == 0) d0 = *(uint32_t *)&cpu->phys_mem[pa];
											else         d1 = *(uint32_t *)&cpu->phys_mem[pa];
										}
									}
									wfw_monitor.dpmi_seg_log[si].val = d0;
									wfw_monitor.dpmi_seg_log[si].extra = d1;
								}
							} else if (f == 0x000B) {
								/* GetDesc: BX=selector, ES:(E)DI destination buffer */
								uint32_t di = cpu->code16 ? (REGi(7) & 0xffff) : REGi(7);
								wfw_monitor.dpmi_seg_log[si].val = cpu->seg[SEG_ES].base + di;
							} else {
								/* 0007/0008/0009: BX=sel, CX:DX=value
								 * 000A(CreateAlias): BX=src sel, AX=ret alias (on IRET) */
								wfw_monitor.dpmi_seg_log[si].val =
									((uint32_t)(REGi(1) & 0xffff) << 16) | (REGi(2) & 0xffff);
							}
						}
					}
					/* Capture input params for Allocate/Resize Memory Block */
					if (ax31 == 0x0501 || ax31 == 0x0503)
						wfw_monitor.pending_alloc_size =
							((uint32_t)(REGi(3) & 0xffff) << 16) | (REGi(1) & 0xffff);
					wfw_monitor.pm_r3_int31_count++;
					wfw_monitor.pending_dpmi = true;
					wfw_monitor.pending_dpmi_ax = ax31;
				}
			}
		}
	}

	if (!(cpu->cr0 & 1)) {
		/* REAL-ADDRESS-MODE */
		uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
		int newcs, newip;

		/* Fast path: use cached INT 8 (timer) vector.
		 * Timer interrupts are the hot path for music timing.
		 * We use a warmup period (10000 calls) before trusting the cache,
		 * since games hook INT 8 during early startup/sound detection. */
		#define INT8_WARMUP_THRESHOLD 10000
		if (no == 8 && cpu->int8_cache_valid && cpu->int8_warmup_counter >= INT8_WARMUP_THRESHOLD) {
			newcs = cpu->cached_int8_vector >> 16;
			newip = cpu->cached_int8_vector & 0xffff;
		} else {
			/* Normal IDT lookup */
			OptAddr meml;
			uword base = cpu->idt.base;
			int off = no * 4;
			TRY(translate_laddr(cpu, &meml, 1, base + off, 4, 0));
			uword w1 = load32(cpu, &meml);
			newcs = w1 >> 16;
			newip = w1 & 0xffff;

			/* Cache/update INT 8 vector and increment warmup counter */
			if (no == 8) {
				if (cpu->int8_cache_valid && cpu->cached_int8_vector != w1) {
					/* Vector changed - reset warmup counter */
					cpu->int8_warmup_counter = 0;
				}
				cpu->cached_int8_vector = w1;
				cpu->int8_cache_valid = true;
				if (cpu->int8_warmup_counter < INT8_WARMUP_THRESHOLD)
					cpu->int8_warmup_counter++;
			}
		}

		/* Fast path: batch stack operations with direct memory access.
		 * In real mode, we can write directly to physical memory if the
		 * 6 bytes of stack won't cross a page boundary. */
		uword sp = lreg32(4);
		uword ss_base = cpu->seg[SEG_SS].base;
		uword new_sp = (sp - 6) & sp_mask;

		if (likely((sp & 0xfff) >= 6 &&
			   !(cpu_diag_enabled && win386_diag.active))) {
			/* Fast path: direct memory write, no page crossing */
			uint8_t *stack = cpu->phys_mem + ss_base + new_sp;
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			*(uint16_t *)(stack + 0) = cpu->ip;           /* IP at lowest addr */
			*(uint16_t *)(stack + 2) = cpu->seg[SEG_CS].sel;  /* CS */
			*(uint16_t *)(stack + 4) = cpu->flags;        /* FLAGS at highest */
		} else {
			/* Slow path: stack crosses page boundary, use translate */
			OptAddr meml1, meml2, meml3;
			TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 2 * 1) & sp_mask, 2, 0));
			TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 2 * 2) & sp_mask, 2, 0));
			TRY(translate(cpu, &meml3, 2, SEG_SS, (sp - 2 * 3) & sp_mask, 2, 0));
			refresh_flags(cpu);
			cpu->cc.mask = 0;
			saddr16(&meml1, cpu->flags);
			saddr16(&meml2, cpu->seg[SEG_CS].sel);
			saddr16(&meml3, cpu->ip);
		}
		sreg32(4, new_sp);

		TRY(set_seg(cpu, SEG_CS, newcs));
		cpu->next_ip = newip;
		cpu->ip = newip;
		cpu->flags &= ~(IF|TF);
		return true;
	}

	/* PROTECTED-MODE */
	OptAddr meml;
	uword base = cpu->idt.base;
	int off = no << 3;
	if (off + 7 > cpu->idt.limit) {
		dolog("call_isr error0 %d %d\n", off, cpu->idt.limit);
		THROW(EX_GP, off | 2 | ext);
	}

	TRY(translate_laddr(cpu, &meml, 1, base + off, 4, 0));
	uword w1 = load32(cpu, &meml);
	TRY(translate_laddr(cpu, &meml, 1, base + off + 4, 4, 0));
	uword w2 = load32(cpu, &meml);

	int gt = (w2 >> 8) & 0xf;
	if (gt != 6 && gt != 7 && gt != 0xe && gt != 0xf && gt != 5) {
//		dolog("call_isr error1 gt=%d\n", gt);
		THROW(EX_GP, off | 2 | ext);
	}

	int dpl = (w2 >> 13) & 0x3;
	if (!ext && dpl < cpu->cpl) {
		/* Win3.x/WfW compat: some PM ring-3 probes issue INT 11h/12h and
		 * expect BIOS-equivalent values. If they hit a DPL gate and the
		 * VMM reflection path doesn't synthesize AX, startup can abort.
		 * Return BDA-backed values directly for these two vectors. */
		if (unlikely(win386_diag.active) && (no == 0x11 || no == 0x12)) {
			uint16_t val = 0;
			uint32_t off = (no == 0x11) ? 0x410u : 0x413u;
			if (off + 1 < (uint32_t)cpu->phys_mem_size)
				val = cpu->phys_mem[off] | ((uint16_t)cpu->phys_mem[off + 1] << 8);
			if (no == 0x11) {
				/* Some BIOS paths leave high equipment bits noisy (e.g. C065).
				 * Normalize to architecturally meaningful bits for Win3.x probes:
				 * keep low 12 bits and force a color text baseline if video bits are 00. */
				val &= 0x0fff;
				if ((val & 0x0030) == 0x0000)
					val = (uint16_t)((val & ~0x0030) | 0x0020);
			}
			sreg16(0, val);
			wfw_flow_push_here(cpu, 'H', (uint8_t)no, val);
			if (unlikely(cpu_diag_enabled) &&
			    cpu_diag_event_allow(cpu, &cpu_diag_events.swint_dpl)) {
				dolog("SW INT compat #%u vec=%02x AX<=%04x from BDA[%03x] at %04x:%08x\n",
					cpu_diag_events.swint_dpl, no, val, off,
					cpu->seg[SEG_CS].sel, cpu->ip);
				}
				return true;
		} else {
swint_dpl_reject:
			if (unlikely(cpu_diag_enabled) && win386_diag.active) {
				int si = wfw_swint_dpl_find_or_add((uint8_t)no);
				if (si >= 0 && si < wfw_monitor.swint_dpl_count) {
					wfw_monitor.swint_dpl[si].count++;
				if (wfw_monitor.swint_dpl[si].first_cs == 0 &&
				    wfw_monitor.swint_dpl[si].first_ip == 0) {
					wfw_monitor.swint_dpl[si].first_cs = cpu->seg[SEG_CS].sel;
					wfw_monitor.swint_dpl[si].first_ip = cpu->ip;
					wfw_monitor.swint_dpl[si].first_err = (off | 2);
				}
				wfw_monitor.swint_dpl[si].in_ax = REGi(0) & 0xffff;
				wfw_monitor.swint_dpl[si].in_bx = REGi(3) & 0xffff;
				wfw_monitor.pending_swint_dpl_idx = (int8_t)si;
				wfw_monitor.pending_swint_cs = cpu->seg[SEG_CS].sel;
				wfw_monitor.pending_swint_ip = cpu->ip;
				wfw_monitor.pending_swint_next_ip = cpu->next_ip;
			}
				/* INT 41h module name capture (LOADMODULE/LOADDLL) */
				if (no == 0x41 && wfw_monitor.int41_module_count < 8) {
					uint16_t ax41 = REGi(0) & 0xffff;
					if (ax41 == 0x005a || ax41 == 0x005c) {
						int mi = wfw_monitor.int41_module_count;
						uint16_t hmod;
						if (ax41 == 0x005a)
							hmod = REGi(1) & 0xffff; /* CX for LOADDLL */
						else
							hmod = REGi(3) & 0xffff; /* BX for LOADMODULE */
						wfw_monitor.int41_modules[mi].ax = ax41;
						wfw_monitor.int41_modules[mi].hmod = hmod;
						wfw_monitor.int41_modules[mi].call_cs = cpu->seg[SEG_CS].sel;
						read_ne_module_name(cpu, hmod,
							wfw_monitor.int41_modules[mi].name,
							sizeof(wfw_monitor.int41_modules[mi].name));
						/* If primary register didn't yield a name, try alternatives */
						if (!wfw_monitor.int41_modules[mi].name[0]) {
							uint16_t alts[] = {
								(uint16_t)(ax41 == 0x005a ? (REGi(3) & 0xffff) : (REGi(1) & 0xffff)),
								(uint16_t)(REGi(6) & 0xffff),
								(uint16_t)(REGi(7) & 0xffff),
								cpu->seg[SEG_ES].sel
							};
							for (int a = 0; a < 4; a++) {
								if (alts[a] == hmod || (alts[a] & ~7) == 0)
									continue;
								read_ne_module_name(cpu, alts[a],
									wfw_monitor.int41_modules[mi].name,
									sizeof(wfw_monitor.int41_modules[mi].name));
								if (wfw_monitor.int41_modules[mi].name[0]) {
									wfw_monitor.int41_modules[mi].hmod = alts[a];
									break;
								}
							}
						}
						wfw_monitor.int41_module_count++;
					}
				}
			}
			if (unlikely(cpu_diag_enabled) &&
			    cpu_diag_event_allow(cpu, &cpu_diag_events.swint_dpl)) {
				dolog("SW INT DPL reject #%u vec=%02x dpl=%d cpl=%d at %04x:%08x\n",
					cpu_diag_events.swint_dpl, no, dpl, cpu->cpl,
					cpu->seg[SEG_CS].sel, cpu->ip);
			}
			wfw_flow_push_here(cpu, 'D', (uint8_t)no, (uint32_t)(off | 2));
			THROW(EX_GP, off | 2);
		}
	}

	int p = (w2 >> 15) & 1;
	if (!p) {
		dolog("call_isr error3\n");
		THROW(EX_NP, off | 2 | ext);
	}

	/* task gate */
	if (gt == 5) {
		if (unlikely(cpu_diag_enabled) && cpu_diag_event_allow(cpu, &cpu_diag_events.task_gate)) {
			dolog("Task gate #%u vec=%02x sel=%04x from %04x:%08x\n",
				cpu_diag_events.task_gate, no, w1 >> 16,
				cpu->seg[SEG_CS].sel, cpu->ip);
		}
		return task_switch(cpu, w1 >> 16, TS_CALL);
	}

	/* TRAP-OR-INTERRUPT-GATE */
	int newcs = w1 >> 16;
	bool gate16 = gt == 6 || gt == 7;
	/* 16-bit gates (type 6/7): offset is w1[15:0] only.
	 * 32-bit gates (type E/F): offset is w1[15:0] | w2[31:16]. */
	uword newip = gate16 ? (w1 & 0xffff)
			     : ((w1 & 0xffff) | (w2 & 0xffff0000));

	int csdpl;
	int isr_result = __call_isr_check_cs(cpu, newcs, ext, &csdpl);
	switch(isr_result) {
	case 0: {
		return false;
	}
	case 1: /* intra PVL */ {
		OptAddr meml1, meml2, meml3, meml4;
		uword sp = lreg32(4);
		uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
		if (gate16) {
			TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 2 * 1) & sp_mask, 2, 0));
			TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 2 * 2) & sp_mask, 2, 0));
			TRY(translate(cpu, &meml3, 2, SEG_SS, (sp - 2 * 3) & sp_mask, 2, 0));
			if (pusherr) {
				TRY(translate(cpu, &meml4, 2, SEG_SS, (sp - 2 * 4) & sp_mask, 2, 0));
			}

			refresh_flags(cpu);
			cpu->cc.mask = 0;
			saddr16(&meml1, cpu->flags);

			saddr16(&meml2, cpu->seg[SEG_CS].sel);
			saddr16(&meml3, cpu->ip);
			if (pusherr) {
				saddr16(&meml4, cpu->excerr);
				set_sp(sp - 2 * 4, sp_mask);
			} else {
				set_sp(sp - 2 * 3, sp_mask);
			}
		} else {
			TRY(translate(cpu, &meml1, 2, SEG_SS, (sp - 4 * 1) & sp_mask, 4, 0));
			TRY(translate(cpu, &meml2, 2, SEG_SS, (sp - 4 * 2) & sp_mask, 4, 0));
			TRY(translate(cpu, &meml3, 2, SEG_SS, (sp - 4 * 3) & sp_mask, 4, 0));
			if (pusherr) {
				TRY(translate(cpu, &meml4, 2, SEG_SS, (sp - 4 * 4) & sp_mask, 4, 0));
			}

			refresh_flags(cpu);
			cpu->cc.mask = 0;
			saddr32(&meml1, cpu->flags);

			saddr32(&meml2, cpu->seg[SEG_CS].sel);
			saddr32(&meml3, cpu->ip);
			if (pusherr) {
				saddr32(&meml4, cpu->excerr);
				set_sp(sp - 4 * 4, sp_mask);
			} else {
				set_sp(sp - 4 * 3, sp_mask);
			}
		}
		newcs = (newcs & (~3)) | cpu->cpl;
		break;
	}
		case 2: /* inter PVL */ {
	//		dolog("call_isr %d %x PVL %d => %d\n", no, no, cpu->cpl, csdpl);
			OptAddr msp0, mss0;
			int newpl = csdpl;
			int oldcpl = cpu->cpl;

			/* Check for valid TR before stack switch.
			 * 32-bit TSS (type 9/B) minimum limit = 0x67 (104 bytes).
			 * 16-bit TSS (type 1/3) minimum limit = 0x2B (44 bytes). */
			{
			int __min_tss = (cpu->seg[SEG_TR].flags & 0x8) ? 0x67 : 0x2b;
			if ((cpu->seg[SEG_TR].sel & ~7) == 0 || cpu->seg[SEG_TR].limit < __min_tss) {
				dolog("call_isr: inter-PVL switch but TR is invalid (sel=%04x limit=%x)\n",
					cpu->seg[SEG_TR].sel, cpu->seg[SEG_TR].limit);
				THROW(EX_TS, 0);
			}
			}

			uword oldss = cpu->seg[SEG_SS].sel;
			uword oldsp = REGi(4);
			uword saved_ss_sel = cpu->seg[SEG_SS].sel;
			uword saved_ss_base = cpu->seg[SEG_SS].base;
			uword saved_ss_limit = cpu->seg[SEG_SS].limit;
		uword saved_ss_flags = cpu->seg[SEG_SS].flags;
		uword saved_sp_mask = cpu->sp_mask;
		uword newss, newsp;
		if (cpu->seg[SEG_TR].flags & 0x8) {
			TRY(translate(cpu, &msp0, 1, SEG_TR, 4 + 8 * newpl, 4, 0));
			TRY(translate(cpu, &mss0, 1, SEG_TR, 8 + 8 * newpl, 4, 0));
			newsp = load32(cpu, &msp0);
			newss = load32(cpu, &mss0) & 0xffff;
		} else {
			TRY(translate(cpu, &msp0, 1, SEG_TR, 2 + 4 * newpl, 2, 0));
			TRY(translate(cpu, &mss0, 1, SEG_TR, 4 + 4 * newpl, 2, 0));
				newsp = load16(cpu, &msp0);
				newss = load16(cpu, &mss0);
			}

			/* Hardware privilege transition updates CPL before loading SS:ESP
			 * from TSS. Match that ordering so SS permission/page checks run
			 * with the target privilege level. */
			cpu->cpl = newpl;
			REGi(4) = newsp;
			if (!set_seg(cpu, SEG_SS, newss)) {
				cpu->cpl = oldcpl;
				REGi(4) = oldsp;
				return false;
			}
		uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
		OptAddr meml1, meml2, meml3, meml4, meml5, meml6;
		uword sp = lreg32(4);
		if (gate16) {
			if (!translate(cpu, &meml1, 2, SEG_SS, (sp - 2 * 1) & sp_mask, 2, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml2, 2, SEG_SS, (sp - 2 * 2) & sp_mask, 2, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml3, 2, SEG_SS, (sp - 2 * 3) & sp_mask, 2, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml4, 2, SEG_SS, (sp - 2 * 4) & sp_mask, 2, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml5, 2, SEG_SS, (sp - 2 * 5) & sp_mask, 2, 0)) goto inter_pvl_fail;
			if (pusherr) {
				if (!translate(cpu, &meml6, 2, SEG_SS, (sp - 2 * 6) & sp_mask, 2, 0)) goto inter_pvl_fail;
			}
			saddr16(&meml1, oldss);
			saddr16(&meml2, oldsp);

			refresh_flags(cpu);
			cpu->cc.mask = 0;
			saddr16(&meml3, cpu->flags);

			saddr16(&meml4, cpu->seg[SEG_CS].sel);
			saddr16(&meml5, cpu->ip);
			if (pusherr) {
				saddr16(&meml6, cpu->excerr);
				set_sp(sp - 2 * 6, sp_mask);
			} else {
				set_sp(sp - 2 * 5, sp_mask);
			}
		} else {
			if (!translate(cpu, &meml1, 2, SEG_SS, (sp - 4 * 1) & sp_mask, 4, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml2, 2, SEG_SS, (sp - 4 * 2) & sp_mask, 4, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml3, 2, SEG_SS, (sp - 4 * 3) & sp_mask, 4, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml4, 2, SEG_SS, (sp - 4 * 4) & sp_mask, 4, 0)) goto inter_pvl_fail;
			if (!translate(cpu, &meml5, 2, SEG_SS, (sp - 4 * 5) & sp_mask, 4, 0)) goto inter_pvl_fail;
			if (pusherr) {
				if (!translate(cpu, &meml6, 2, SEG_SS, (sp - 4 * 6) & sp_mask, 4, 0)) goto inter_pvl_fail;
			}
			saddr32(&meml1, oldss);
			saddr32(&meml2, oldsp);

			refresh_flags(cpu);
			cpu->cc.mask = 0;
			saddr32(&meml3, cpu->flags);

			saddr32(&meml4, cpu->seg[SEG_CS].sel);
			saddr32(&meml5, cpu->ip);
			if (pusherr) {
				saddr32(&meml6, cpu->excerr);
				set_sp(sp - 4 * 6, sp_mask);
			} else {
				set_sp(sp - 4 * 5, sp_mask);
			}
		}
		newcs = (newcs & (~3)) | newpl;
		break;
		inter_pvl_fail:
			cpu->seg[SEG_SS].sel = saved_ss_sel;
			cpu->seg[SEG_SS].base = saved_ss_base;
			cpu->seg[SEG_SS].limit = saved_ss_limit;
			cpu->seg[SEG_SS].flags = saved_ss_flags;
			cpu->sp_mask = saved_sp_mask;
			cpu->cpl = oldcpl;
			update_seg_flat(cpu, SEG_SS);
			REGi(4) = oldsp;
			return false;
		}
		case 3: /* from v8086 */ {
//		dolog("int from v8086\n");
		if (unlikely(cpu_diag_enabled) && win386_diag.active)
			wfw_monitor.v86_to_ring0++;
		if (csdpl != 0) THROW(EX_GP, newcs & ~0x3);
		if (gate16) THROW(EX_GP, off | 2 | ext);
//		dolog("call_isr %d %x PVL %d => 0\n", no, no, cpu->cpl, csdpl);
		OptAddr msp0, mss0;
		int newpl = 0;
		uword oldss = cpu->seg[SEG_SS].sel;
		uword oldsp = REGi(4);
		uword newss, newsp;
		if (!(cpu->seg[SEG_TR].flags & 0x8)) THROW(EX_TS, cpu->seg[SEG_TR].sel & ~0x3);
		TRY(translate(cpu, &msp0, 1, SEG_TR, 4 + 8 * newpl, 4, 0));
		TRY(translate(cpu, &mss0, 1, SEG_TR, 8 + 8 * newpl, 4, 0));
		newsp = load32(cpu, &msp0);
		newss = load32(cpu, &mss0) & 0xffff;
				if (unlikely(cpu_diag_enabled) && !vm86_trip_silence_live_logs(cpu) &&
				    cpu_diag_event_allow(cpu, &cpu_diag_events.vm86_isr_entry)) {
					char decoded[80];
					bool is_io, needs_stack_dump;
					int io_port, io_bits;
					vm86_decode_fault_instruction(cpu, decoded, sizeof(decoded),
								     &is_io, &io_port, &io_bits, &needs_stack_dump);
					dolog("V86->r0 #%u vec=%02x %04x:%04x->%04x:%08x r0stk=%04x:%08x %s\n",
						cpu_diag_events.vm86_isr_entry, no,
						cpu->seg[SEG_CS].sel, cpu->ip & 0xffff, newcs, newip,
						newss, newsp, decoded);
				}
			vm86_trip_push(cpu, VM86_TRIP_V, (uint8_t)no,
				       cpu->seg[SEG_CS].sel, cpu->ip,
				       cpu->seg[SEG_SS].sel, REGi(4), cpu->flags,
				       newcs & 0xffff, newip, newss & 0xffff,
				       newsp, NULL, 0);
				uword oldflags = cpu->flags;
				int oldcpl = cpu->cpl;
			cpu->flags &= ~VM;
			cpu->cpl = newpl;
			REGi(4) = newsp;
			if (!set_seg(cpu, SEG_SS, newss)) {
				cpu->cpl = oldcpl;
				cpu->flags = oldflags;
				REGi(4) = oldsp;
				return false;
			}

		uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
		OptAddr memlg, memlf, memld, memle;
		OptAddr meml1, meml2, meml3, meml4, meml5, meml6;
		uword sp = lreg32(4);
		if (!translate(cpu, &memlg, 2, SEG_SS, (sp - 4 * 1) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &memlf, 2, SEG_SS, (sp - 4 * 2) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &memld, 2, SEG_SS, (sp - 4 * 3) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &memle, 2, SEG_SS, (sp - 4 * 4) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &meml1, 2, SEG_SS, (sp - 4 * 5) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &meml2, 2, SEG_SS, (sp - 4 * 6) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &meml3, 2, SEG_SS, (sp - 4 * 7) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &meml4, 2, SEG_SS, (sp - 4 * 8) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (!translate(cpu, &meml5, 2, SEG_SS, (sp - 4 * 9) & sp_mask, 4, 0)) goto v86_stack_fail;
		if (pusherr) {
			TRY(translate(cpu, &meml6, 2, SEG_SS, (sp - 4 * 10) & sp_mask, 4, 0));
		}
		saddr32(&memlg, cpu->seg[SEG_GS].sel);
		saddr32(&memlf, cpu->seg[SEG_FS].sel);
		saddr32(&memld, cpu->seg[SEG_DS].sel);
		saddr32(&memle, cpu->seg[SEG_ES].sel);
		saddr32(&meml1, oldss);
		saddr32(&meml2, oldsp);

		refresh_flags(cpu);
		cpu->cc.mask = 0;
		saddr32(&meml3, cpu->flags | VM);

		saddr32(&meml4, cpu->seg[SEG_CS].sel);
		saddr32(&meml5, cpu->ip);
		if (pusherr) {
			saddr32(&meml6, cpu->excerr);
			set_sp(sp - 4 * 10, sp_mask);
		} else {
			set_sp(sp - 4 * 9, sp_mask);
		}

		newcs = (newcs & (~3)) | newpl;
		if (!set_seg(cpu, SEG_DS, 0)) goto v86_stack_fail;
		if (!set_seg(cpu, SEG_ES, 0)) goto v86_stack_fail;
		if (!set_seg(cpu, SEG_FS, 0)) goto v86_stack_fail;
		if (!set_seg(cpu, SEG_GS, 0)) goto v86_stack_fail;
		cpu->flags &= ~(TF | RF | NT);
		if (!set_seg(cpu, SEG_CS, newcs)) goto v86_stack_fail;
		cpu->next_ip = newip;
		cpu->ip = newip;
		if (gt == 0x6 || gt == 0xe)
			cpu->flags &= ~IF;
		return true;
	v86_stack_fail:
		cpu->flags = oldflags;
		cpu->cpl = oldcpl;
		cpu->seg[SEG_SS].sel = oldss;
		cpu->seg[SEG_SS].base = (uword)oldss << 4;
		cpu->seg[SEG_SS].limit = 0xffff;
		cpu->seg[SEG_SS].flags = 0;
		cpu->sp_mask = 0xffff;
		update_seg_flat(cpu, SEG_SS);
		REGi(4) = oldsp;
		return false;
	}
	default: assert(false);
	}
	TRY(set_seg(cpu, SEG_CS, newcs));
	cpu->next_ip = newip;
	cpu->ip = newip;
	cpu->flags &= ~(TF | RF | NT);
	if (gt == 0x6 || gt == 0xe)
		cpu->flags &= ~IF;
	return true;
}

static bool __pmiret_check_cs_same(CPUI386 *cpu, int sel)
{
	sel = sel & 0xffff;
	if ((sel & ~0x3) == 0) {
		dolog("__pmiret_check_cs_same: sel %04x\n", sel);
		THROW(EX_GP, sel & ~0x3);
	}
	uword w2;
	TRY(read_desc(cpu, sel, NULL, &w2));

	int s = (w2 >> 12) & 1;
	bool code = (w2 >> 8) & 0x8;
	bool conforming = (w2 >> 8) & 0x4;
	int dpl = (w2 >> 13) & 0x3;
	int p = (w2 >> 15) & 1;

	if (!s || !code) THROW(EX_GP, sel & ~0x3);

	if (!conforming) {
		if (dpl != cpu->cpl) THROW(EX_GP, sel & ~0x3);
	} else {
		if (dpl > cpu->cpl) THROW(EX_GP, sel & ~0x3);
	}

	if (!p) {
//		dolog("__pmiret_check_cs_same: seg not present %04x\n", sel);
		THROW(EX_NP, sel & ~0x3);
	}
	return true;
}

static bool __pmiret_check_cs_outer(CPUI386 *cpu, int sel)
{
	sel = sel & 0xffff;
	if ((sel & ~0x3) == 0) {
		dolog("__pmiret_check_cs_outer: sel %04x\n", sel);
		THROW(EX_GP, sel & ~0x3);
	}
	uword w2;
	TRY(read_desc(cpu, sel, NULL, &w2));

	int s = (w2 >> 12) & 1;
	bool code = (w2 >> 8) & 0x8;
	bool conforming = (w2 >> 8) & 0x4;
	int dpl = (w2 >> 13) & 0x3;
	int p = (w2 >> 15) & 1;
	int rpl = sel & 3;

	if (!s || !code) THROW(EX_GP, sel & ~0x3);

	if (!conforming) {
		if (dpl != rpl) THROW(EX_GP, sel & ~0x3);
	} else {
		/* Conforming outer return: DPL must be <= target RPL. */
		if (dpl > rpl) THROW(EX_GP, sel & ~0x3);
	}

	if (!p) {
//		dolog("__pmiret_check_cs_outer: seg not present %04x\n", sel);
		THROW(EX_NP, sel & ~0x3);
	}
	return true;
}

static bool pmret(CPUI386 *cpu, bool opsz16, int off, bool isiret)
{
	if (isiret) {
		if ((cpu->flags & VM)) THROW(EX_GP, 0);
		if ((cpu->flags & NT)) {
			OptAddr meml;
			TRY(translate(cpu, &meml, 1, SEG_TR, 0, 2, 0));
			int tssback = laddr16(&meml);
			// win2000 needs it...
			if (tssback == 0) THROW(EX_TS, 0);
			return task_switch(cpu, tssback, TS_IRET);
		}
		if (opsz16)
			off += 2;
		else
			off += 4;
	}

	OptAddr meml1, meml2, meml3, meml4, meml5;
	uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
	uword sp = lreg32(4);
	uword oldflags = cpu->flags;
	uword newip;
	int newcs;
	uword newflags = 0; // make the compiler happy
	if (opsz16) {
		/* ip */ TRY(translate(cpu, &meml1, 1, SEG_SS, sp & sp_mask, 2, 0));
		/* cs */ TRY(translate(cpu, &meml2, 1, SEG_SS, (sp + 2) & sp_mask, 2, 0));
		if (isiret) {
			/* flags */ TRY(translate(cpu, &meml3, 1, SEG_SS, (sp + 4) & sp_mask, 2, 0));
			newflags = (oldflags & 0xffff0000) | laddr16(&meml3);
		}
		newip = laddr16(&meml1);
		newcs = laddr16(&meml2);
	} else {
		/* ip */ TRY(translate(cpu, &meml1, 1, SEG_SS, sp & sp_mask, 4, 0));
		/* cs */ TRY(translate(cpu, &meml2, 1, SEG_SS, (sp + 4) & sp_mask, 4, 0));
		if (isiret) {
			/* flags */ TRY(translate(cpu, &meml3, 1, SEG_SS, (sp + 8) & sp_mask, 4, 0));
			newflags = laddr32(&meml3);
		}
		newip = laddr32(&meml1);
		newcs = laddr32(&meml2);
	}

	if (isiret) {
		uword mask = 0;
		if (cpu->cpl > 0) mask |= IOPL;
		if (get_IOPL(cpu) < cpu->cpl) mask |= IF;
		newflags = (oldflags & mask) | (newflags & ~mask);
		newflags &= EFLAGS_MASK;
		newflags |= 0x2;
	}

	if (isiret && (newflags & VM)) {
		if (cpu->cpl != 0) THROW(EX_GP, 0);
		// return to v8086
//		dolog("pmiret PVL %d => %d (vm) %04x:%08x\n", cpu->cpl, 3, newcs, newip);
		OptAddr meml_vmes, meml_vmds, meml_vmfs, meml_vmgs;
		if (opsz16) THROW(EX_GP, 0);
		TRY(translate(cpu, &meml4, 1, SEG_SS, (sp + 12) & sp_mask, 4, 0));
		TRY(translate(cpu, &meml5, 1, SEG_SS, (sp + 16) & sp_mask, 4, 0));
		TRY(translate(cpu, &meml_vmes, 1, SEG_SS, (sp + 20) & sp_mask, 4, 0));
		TRY(translate(cpu, &meml_vmds, 1, SEG_SS, (sp + 24) & sp_mask, 4, 0));
		TRY(translate(cpu, &meml_vmfs, 1, SEG_SS, (sp + 28) & sp_mask, 4, 0));
		TRY(translate(cpu, &meml_vmgs, 1, SEG_SS, (sp + 32) & sp_mask, 4, 0));
		uword vm_esp = laddr32(&meml4);
		uword vm_ss = laddr32(&meml5);
		uword vm_es = laddr32(&meml_vmes);
		uword vm_ds = laddr32(&meml_vmds);
		uword vm_fs = laddr32(&meml_vmfs);
		uword vm_gs = laddr32(&meml_vmgs);
			if (unlikely(cpu_diag_enabled) && !vm86_trip_silence_live_logs(cpu) &&
			    cpu_diag_event_allow(cpu, &cpu_diag_events.iret_to_vm86)) {
				dolog("IRET->V86 #%u %04x:%08x FL=%08x SS:ESP=%04x:%08x\n",
					cpu_diag_events.iret_to_vm86, newcs, newip, newflags,
					vm_ss & 0xffff, vm_esp);
			}
			{
				uint8_t segbytes[8];
				segbytes[0] = vm_es & 0xff;
				segbytes[1] = (vm_es >> 8) & 0xff;
				segbytes[2] = vm_ds & 0xff;
				segbytes[3] = (vm_ds >> 8) & 0xff;
				segbytes[4] = vm_fs & 0xff;
				segbytes[5] = (vm_fs >> 8) & 0xff;
				segbytes[6] = vm_gs & 0xff;
				segbytes[7] = (vm_gs >> 8) & 0xff;
				vm86_trip_push(cpu, VM86_TRIP_R, 0,
					       cpu->seg[SEG_CS].sel, cpu->ip,
					       vm_ss & 0xffff, vm_esp, newflags,
					       newcs & 0xffff, newip, 0,
					       0, segbytes, 8);
			}
			/* Detect VMM crashing a VM: IRET to VM86 with CS=0000 */
			if (unlikely(cpu_diag_enabled) && (newcs & 0xFFFF) == 0) {
				dolog("VMM CRASH: IRET->V86 CS=0000 EIP=%08x from %04x:%08x FL=%08x BDA_mode=%02x\n",
				newip, cpu->seg[SEG_CS].sel, cpu->ip, newflags,
				(uint8_t)cpu->phys_mem[0x449]);
		}
		/* One-shot: capture first PM→V86 IRET that sets IOPL=3 */
		if (unlikely(cpu_diag_enabled) && win386_diag.active &&
		    !wfw_monitor.iopl3_captured) {
			wfw_monitor.iopl3_iret_count++;
			if ((newflags & IOPL) == IOPL) { /* IOPL=3 */
				wfw_monitor.iopl3_captured = true;
				wfw_monitor.iopl3_from_cs = cpu->seg[SEG_CS].sel;
				wfw_monitor.iopl3_from_eip = cpu->ip;
				wfw_monitor.iopl3_to_cs = newcs & 0xffff;
				wfw_monitor.iopl3_to_ip = newip;
				wfw_monitor.iopl3_eflags = newflags;
				wfw_monitor.iopl3_ivt_2f = *(uint32_t*)(cpu->phys_mem + 0x2f * 4);
			}
		}
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_swint_dpl_note_resume(cpu, (uint16_t)(newcs & 0xffff), newip, newflags);
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_ud63_note_resume(cpu, (uint16_t)(newcs & 0xffff), newip, newflags,
						     (uint16_t)(vm_ds & 0xffff));
			if (unlikely(cpu_diag_enabled) && win386_diag.active &&
			    wfw_monitor.pending_bios_fault_idx >= 0 &&
			    wfw_monitor.pending_bios_fault_idx < (int8_t)wfw_monitor.bios_hot_count) {
			int bi = wfw_monitor.pending_bios_fault_idx;
			if (wfw_monitor.bios_hot[bi].cs == (uint16_t)(newcs & 0xffff) &&
			    wfw_monitor.bios_hot[bi].ip == (uint16_t)(newip & 0xffff))
				wfw_monitor.bios_hot[bi].resume_same++;
			else
					wfw_monitor.bios_hot[bi].resume_other++;
			}
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_monitor_note_int13_return(cpu, (uint16_t)(newcs & 0xffff),
						      (uint16_t)(newip & 0xffff), newflags);
			if (unlikely(cpu_diag_enabled) && win386_diag.active)
				wfw_monitor.pending_bios_fault_idx = -1;
				cpu->flags = newflags;
		TRY(set_seg(cpu, SEG_CS, newcs));
		set_sp(sp + 12, sp_mask);
		cpu->next_ip = newip;
		TRY(set_seg(cpu, SEG_SS, vm_ss));
		TRY(set_seg(cpu, SEG_ES, vm_es));
		TRY(set_seg(cpu, SEG_DS, vm_ds));
		TRY(set_seg(cpu, SEG_FS, vm_fs));
		TRY(set_seg(cpu, SEG_GS, vm_gs));
		set_sp(vm_esp, 0xffffffff);
	} else {
		int rpl = newcs & 3;
		if (rpl < cpu->cpl) THROW(EX_GP, newcs & ~0x3);
		if (rpl == cpu->cpl) {
			// return to same level
			TRY(__pmiret_check_cs_same(cpu, newcs));
//			dolog("pmiret PVL %d => %d %04x:%08x\n", cpu->cpl, newcs & 3, newcs, newip);
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_swint_dpl_note_resume(cpu, (uint16_t)(newcs & 0xffff), newip, newflags);
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_ud63_note_resume(cpu, (uint16_t)(newcs & 0xffff), newip, newflags,
						     cpu->seg[SEG_DS].sel);
			if (isiret)
				cpu->flags = newflags;
			TRY(set_seg(cpu, SEG_CS, newcs));

			if (opsz16) {
				set_sp(sp + 4 + off, sp_mask);
			} else {
				set_sp(sp + 8 + off, sp_mask);
			}
			cpu->next_ip = newip;
		} else {
			// return to outer level
			TRY(__pmiret_check_cs_outer(cpu, newcs));
			uword newsp;
			uword newss;
//			dolog("pmiret PVL %d => %d %04x:%08x\n", cpu->cpl, newcs & 3, newcs, newip);
			if (opsz16) {
				/* sp */ TRY(translate(cpu, &meml4, 1, SEG_SS, (sp + 4 + off) & sp_mask, 2, 0));
				/* ss */ TRY(translate(cpu, &meml5, 1, SEG_SS, (sp + 6 + off) & sp_mask, 2, 0));
				newsp = laddr16(&meml4);
				newss = laddr16(&meml5);
			} else {
				/* sp */ TRY(translate(cpu, &meml4, 1, SEG_SS, (sp + 8 + off) & sp_mask, 4, 0));
				/* ss */ TRY(translate(cpu, &meml5, 1, SEG_SS, (sp + 12 + off) & sp_mask, 4, 0));
				newsp = laddr32(&meml4);
				newss = laddr32(&meml5);
			}

			if (isiret)
				cpu->flags = newflags;
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_swint_dpl_note_resume(cpu, (uint16_t)(newcs & 0xffff), newip, newflags);
			if (unlikely(cpu_diag_enabled) && isiret)
				wfw_ud63_note_resume(cpu, (uint16_t)(newcs & 0xffff), newip, newflags,
						     cpu->seg[SEG_DS].sel);
			TRY(set_seg(cpu, SEG_CS, newcs));
			TRY(set_seg(cpu, SEG_SS, newss));
			uword newsp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
			set_sp(newsp, newsp_mask);
			cpu->next_ip = newip;
			clear_segs(cpu);
			/* DPMI/INT21h return capture + failure detection on ring-0→ring-3 IRET */
			if (unlikely(cpu_diag_enabled && win386_diag.active) && isiret) {
				/* Unified PM ring-3 return tail (compact, bounded). */
				{
					uint8_t vec = 0;
					uint16_t in_ax = 0;
					if (wfw_monitor.pending_dpmi) {
						vec = 0x31;
						in_ax = wfw_monitor.pending_dpmi_ax;
					} else if (wfw_monitor.pending_int21) {
						vec = 0x21;
						in_ax = (uint16_t)wfw_monitor.pending_int21_ah << 8;
					} else if (wfw_monitor.pending_int2f) {
						vec = 0x2f;
						in_ax = wfw_monitor.pending_int2f_ax;
					}
					if (vec) {
						int ti = wfw_monitor.r3_ret_tail_pos % 16;
						wfw_monitor.r3_ret_tail[ti].vec = vec;
						wfw_monitor.r3_ret_tail[ti].in_ax = in_ax;
						wfw_monitor.r3_ret_tail[ti].out_ax = REGi(0) & 0xffff;
						wfw_monitor.r3_ret_tail[ti].out_bx = REGi(3) & 0xffff;
						wfw_monitor.r3_ret_tail[ti].out_cx = REGi(1) & 0xffff;
						wfw_monitor.r3_ret_tail[ti].out_dx = REGi(2) & 0xffff;
						wfw_monitor.r3_ret_tail[ti].out_ds = cpu->seg[SEG_DS].sel;
						wfw_monitor.r3_ret_tail[ti].out_cs = (uint16_t)(newcs & 0xffff);
						wfw_monitor.r3_ret_tail[ti].out_ip = newip;
						wfw_monitor.r3_ret_tail[ti].cf = !!(newflags & CF);
						wfw_monitor.r3_ret_tail_pos++;
						if (wfw_monitor.r3_ret_tail_count < 16)
							wfw_monitor.r3_ret_tail_count++;
						wfw_flow_push_raw(cpu, 'R', vec,
							       wfw_mode_from_state(newflags, cpu->cr0),
							       newcs & 3, (uint16_t)(newcs & 0xffff), newip,
							       ((uint32_t)in_ax << 16) |
							       ((newflags & CF) ? 1u : 0u));
					}
				}
				if (wfw_monitor.pending_dpmi) {
					/* Capture return values for first 8 DPMI calls */
					if (wfw_monitor.int31_ret_count < 8) {
						int ri = wfw_monitor.int31_ret_count++;
						wfw_monitor.int31_ret_log[ri].call_ax = wfw_monitor.pending_dpmi_ax;
						wfw_monitor.int31_ret_log[ri].ret_ax = REGi(0) & 0xffff;
						wfw_monitor.int31_ret_log[ri].ret_cx = REGi(1) & 0xffff;
						wfw_monitor.int31_ret_log[ri].ret_dx = REGi(2) & 0xffff;
						wfw_monitor.int31_ret_log[ri].cf = !!(newflags & CF);
					}
					/* Capture 0006 return values in ring buffer */
					if (wfw_monitor.pending_dpmi_ax == 0x0006) {
						int ri = wfw_monitor.int31_0006_ret_count % 16;
						wfw_monitor.int31_0006_ret_ring[ri].bx = wfw_monitor.pending_dpmi_bx;
						wfw_monitor.int31_0006_ret_ring[ri].base =
							((uint32_t)(REGi(1) & 0xffff) << 16) | (REGi(2) & 0xffff);
						wfw_monitor.int31_0006_ret_count++;
					}
					/* Capture 0501/0503 return: linear address + handle */
					if ((wfw_monitor.pending_dpmi_ax == 0x0501 ||
					     wfw_monitor.pending_dpmi_ax == 0x0503) &&
					    wfw_monitor.dpmi_alloc_count < 8) {
						int ai = wfw_monitor.dpmi_alloc_count++;
						wfw_monitor.dpmi_alloc_log[ai].func = wfw_monitor.pending_dpmi_ax;
						wfw_monitor.dpmi_alloc_log[ai].req_size = wfw_monitor.pending_alloc_size;
						wfw_monitor.dpmi_alloc_log[ai].ret_addr =
							((uint32_t)(REGi(3) & 0xffff) << 16) | (REGi(1) & 0xffff);
						wfw_monitor.dpmi_alloc_log[ai].ret_handle =
							((uint32_t)(REGi(6) & 0xffff) << 16) | (REGi(7) & 0xffff);
						wfw_monitor.dpmi_alloc_log[ai].cf = !!(newflags & CF);
					}
					/* Capture descriptor-op returns:
					 * 0001 AllocLDT (AX=first selector),
					 * 000A CreateAlias (AX=alias selector),
					 * 000B GetDesc (descriptor at ES:(E)DI buffer). */
					if (wfw_monitor.pending_dpmi_ax == 0x0001 ||
					    wfw_monitor.pending_dpmi_ax == 0x000A ||
					    wfw_monitor.pending_dpmi_ax == 0x000B ||
					    wfw_monitor.pending_dpmi_ax == 0x0007 ||
					    wfw_monitor.pending_dpmi_ax == 0x0008 ||
					    wfw_monitor.pending_dpmi_ax == 0x0009 ||
					    wfw_monitor.pending_dpmi_ax == 0x000C) {
						int di = -1;
						for (int k = wfw_monitor.dpmi_seg_log_count - 1; k >= 0; k--) {
							if (wfw_monitor.dpmi_seg_log[k].func == wfw_monitor.pending_dpmi_ax &&
							    !wfw_monitor.dpmi_seg_log[k].ret_valid) {
								di = k;
								break;
							}
						}
						if (di >= 0) {
							wfw_monitor.dpmi_seg_log[di].ret_valid = 1;
							wfw_monitor.dpmi_seg_log[di].ret_cf = !!(newflags & CF);
							if (!(newflags & CF)) {
								if (wfw_monitor.pending_dpmi_ax == 0x0001) {
									wfw_monitor.dpmi_seg_log[di].sel = REGi(0) & 0xffff;
								} else if (wfw_monitor.pending_dpmi_ax == 0x000A) {
									wfw_monitor.dpmi_seg_log[di].val = REGi(0) & 0xffff;
								} else if (wfw_monitor.pending_dpmi_ax == 0x000B) {
									/* Read returned 8-byte descriptor from saved linear buffer */
									uint32_t la = wfw_monitor.dpmi_seg_log[di].val;
									uint8_t b[8];
									bool ok = true;
									for (int j = 0; j < 8; j++) {
										if (!read_laddr_u8_noexcept(cpu, la + (uint32_t)j,
												     (newcs & 3), &b[j])) {
											ok = false;
											break;
										}
									}
									if (ok) {
										wfw_monitor.dpmi_seg_log[di].val =
											(uint32_t)b[0] |
											((uint32_t)b[1] << 8) |
											((uint32_t)b[2] << 16) |
											((uint32_t)b[3] << 24);
										wfw_monitor.dpmi_seg_log[di].extra =
											(uint32_t)b[4] |
											((uint32_t)b[5] << 8) |
											((uint32_t)b[6] << 16) |
											((uint32_t)b[7] << 24);
									}
								}
							}
						}
					}
					if ((newflags & CF) && wfw_monitor.int31_fail_count < 8) {
						int fi = wfw_monitor.int31_fail_count++;
						wfw_monitor.int31_fail_log[fi].ax = wfw_monitor.pending_dpmi_ax;
						wfw_monitor.int31_fail_log[fi].ret_eip = newip;
					}
					wfw_monitor.pending_dpmi = false;
				}
				if (wfw_monitor.pending_int21) {
					/* Capture return values for first 8 INT 21h calls */
					if (wfw_monitor.int21_ret_count < 8) {
						int ri = wfw_monitor.int21_ret_count++;
						wfw_monitor.int21_ret_log[ri].call_ah = wfw_monitor.pending_int21_ah;
						wfw_monitor.int21_ret_log[ri].ret_ax = REGi(0) & 0xffff;
						wfw_monitor.int21_ret_log[ri].cf = !!(newflags & CF);
					}
					if ((newflags & CF) && wfw_monitor.int21_fail_count < 8) {
						int fi = wfw_monitor.int21_fail_count++;
						wfw_monitor.int21_fail_log[fi].ah = wfw_monitor.pending_int21_ah;
						wfw_monitor.int21_fail_log[fi].ret_eip = newip;
					}
					wfw_monitor.pending_int21 = false;
				}
				if (wfw_monitor.pending_int2f) {
					/* Capture return values for first 4 INT 2Fh calls */
					if (wfw_monitor.int2f_ret_count < 4) {
						int ri = wfw_monitor.int2f_ret_count++;
						wfw_monitor.int2f_ret_log[ri].call_ax = wfw_monitor.pending_int2f_ax;
						wfw_monitor.int2f_ret_log[ri].ret_ax = REGi(0) & 0xffff;
						wfw_monitor.int2f_ret_log[ri].ret_ds = cpu->seg[SEG_DS].sel;
						wfw_monitor.int2f_ret_log[ri].ret_si = REGi(6) & 0xffff;
						wfw_monitor.int2f_ret_log[ri].ret_es = cpu->seg[SEG_ES].sel;
						wfw_monitor.int2f_ret_log[ri].ret_di = REGi(7) & 0xffff;
						wfw_monitor.int2f_ret_log[ri].cf = !!(newflags & CF);
					}
					wfw_monitor.pending_int2f = false;
				}
			}
		}
	}
	if (isiret)
		cpu->cc.mask = 0;

	/* PF handler completion check: when IRET returns to ring 3 (or V86),
	 * verify whether the watched PTE was resolved. Only produces output on FAILURE. */
	if (unlikely(cpu_diag_enabled && pf_diag.active) && isiret && cpu->cpl > 0) {
		uword final_pte = 0;
		if ((uint64_t)pf_diag.watch_phys + 4 <= (uint64_t)cpu->phys_mem_size)
			final_pte = pload32(cpu, pf_diag.watch_phys);
		bool resolved = (final_pte & 1) != 0; /* P bit set = resolved */
		if (!resolved) {
			/* FAILURE: PTE not resolved after handler ran */
			dolog("=== PF_DIAG: HANDLER FAILED ===\n");
			dolog("  CR2=%08x PTE@phys=%08x\n", pf_diag.cr2, pf_diag.watch_phys);
			dolog("  PTE before=%08x after=%08x (P=%d)\n",
				pf_diag.initial_pte, final_pte, !!(final_pte & 1));
			dolog("  writes_to_PTE=%d nested_exc=%d nested_PF=%d\n",
				pf_diag.write_count, pf_diag.nested_exc_count, pf_diag.nested_pf_count);
			if (pf_diag.write_count > 0)
				dolog("  last_write: val=%08x at %04x:%08x\n",
					pf_diag.last_write_val, pf_diag.last_write_cs, pf_diag.last_write_eip);
			if (pf_diag.nested_pf_count > 0)
				dolog("  last_nested_PF CR2=%08x\n", pf_diag.nested_pf_last_cr2);
			dolog("  CR3_writes=%d INVLPG=%d OOB_stores=%d\n",
				pf_diag.cr3_write_count, pf_diag.invlpg_count, pf_diag.oob_store_count);
			if (pf_diag.oob_store_count > 0)
				dolog("  last_OOB_store addr=%08x (phys_mem_size=%08lx)\n",
					pf_diag.oob_store_last_addr, (unsigned long)cpu->phys_mem_size);
			dolog("  IRET to %04x:%08x CPL=%d\n",
				cpu->seg[SEG_CS].sel, cpu->next_ip, cpu->cpl);
		}
		pf_diag.active = false;
	}

	return true;
}

void cpui386_step(CPUI386 *cpu, int stepcount)
{
	/* Halt/post-mortem detector state (per diagnostic generation). */
	static int halt_diag_gen = -1;
	static bool halt_window_active = false;
	static bool halt_dumped = false;
	static uint32_t halt_window_start_us = 0;
	static uint32_t last_irq_service_time_us = 0;
	static uint32_t alive_last_log_time_us = 0;
	if (halt_diag_gen != cpu->diag_gen) {
		halt_diag_gen = cpu->diag_gen;
		halt_window_active = false;
		halt_dumped = false;
		halt_window_start_us = 0;
		last_irq_service_time_us = get_uticks();
		alive_last_log_time_us = last_irq_service_time_us;
	}

	if ((cpu->flags & IF) && cpu->intr) {
		cpu->irq_count++;
		bool was_halted = cpu->halt;
		cpu->intr = false;
		cpu->halt = false;
		int no = cpu->cb.pic_read_irq(cpu->cb.pic);
		if (was_halted) {
			hlt_diag_sync(cpu);
			refresh_flags(cpu);
			hlt_diag.wake_count++;
			hlt_diag.last_wake_vec = no;
			hlt_diag.last_wake_eip = cpu->next_ip;
			hlt_diag.last_wake_cs = cpu->seg[SEG_CS].sel;
			hlt_diag.last_wake_ss = cpu->seg[SEG_SS].sel;
			hlt_diag.last_wake_sp = REGi(4);
			hlt_diag.last_wake_fl = cpu->flags;
			hlt_diag.last_wake_time_us = get_uticks();
		}
		if (unlikely(cpu_diag_enabled) && was_halted &&
		    cpu_diag_event_allow(cpu, &cpu_diag_events.halt_irq_wake)) {
			refresh_flags(cpu);
			dolog("=== HLT wake by IRQ #%u ===\n", cpu_diag_events.halt_irq_wake);
			dolog("  vec=%02x resume CS:EIP=%04x:%08x SS:ESP=%04x:%08x FL=%08x\n",
				no, cpu->seg[SEG_CS].sel, cpu->next_ip,
				cpu->seg[SEG_SS].sel, REGi(4), cpu->flags);
		}
		cpu->ip = cpu->next_ip;
		if (unlikely(!call_isr(cpu, no, false, 1))) {
			/* Hardware IRQ delivery failed — try #DF before triple fault */
			dolog("IRQ %d delivery failed (2nd exc %d)\n", no, cpu->excno);
			cpu->excerr = 0;
			if (!call_isr(cpu, EX_DF, true, 1))
				goto triple_fault;
		}
		last_irq_service_time_us = get_uticks();
	}

	if (cpu->halt) {
		/* Lost-latch safeguard: if PIC already has a deliverable IRQ while
		 * CPU is halted with IF=1, reassert the CPU interrupt latch so the
		 * next step will take it. */
		if ((cpu->flags & IF) && !cpu->intr && cpu->cb.pic) {
			int pending_irq = i8259_get_pending_irq((PicState2 *)cpu->cb.pic);
			if (pending_irq >= 0) {
				cpu->intr = true;
				return;
			}
		}

		/* Detect permanently-halted or error-looping system.
		 * We require both:
		 * 1) a continuous halt window longer than threshold
		 * 2) no successful hardware IRQ service in that window */
		uint32_t now = get_uticks();
		if (!halt_window_active) {
			halt_window_active = true;
			halt_window_start_us = now;
			halt_dumped = false;
		}
		uint32_t halt_cont_us = now - halt_window_start_us;
		uint32_t irq_quiet_us = (last_irq_service_time_us != 0)
			? (now - last_irq_service_time_us)
			: halt_cont_us;
		hlt_diag_sync(cpu);
		uint32_t wake_age_us = (hlt_diag.last_wake_time_us != 0)
			? (now - hlt_diag.last_wake_time_us)
			: halt_cont_us;
			if (cpu_diag_enabled && !halt_dumped &&
			    halt_cont_us > 5000000 && irq_quiet_us > 5000000) { /* 5s continuous halt + no IRQ service */
			halt_dumped = true;
			dolog("=== System halted %.1fs (IF=%d intr=%d, no-irq %.1fs) — post-mortem ===\n",
				halt_cont_us / 1000000.0f, !!(cpu->flags & IF), !!cpu->intr,
				irq_quiet_us / 1000000.0f);
			dolog("CS:EIP=%04x:%08x SS:ESP=%04x:%08x FL=%08x CR0=%08x %s\n",
				cpu->seg[SEG_CS].sel, cpu->ip,
				cpu->seg[SEG_SS].sel, REGi(4),
				cpu->flags, cpu->cr0,
				(cpu->flags & VM) ? "V86" : ((cpu->cr0 & 1) ? "PM" : "RM"));
			dolog("EAX=%08x EBX=%08x ECX=%08x EDX=%08x\n",
				REGi(0), REGi(3), REGi(1), REGi(2));
			if (hlt_diag.count > 0) {
				dolog("HLT origin: count=%u last=%04x:%08x SS:ESP=%04x:%08x FL=%08x CPL=%d VM=%d\n",
					hlt_diag.count, hlt_diag.cs, hlt_diag.ip,
					hlt_diag.ss, hlt_diag.sp, hlt_diag.fl,
					hlt_diag.cpl, hlt_diag.vm);
			} else {
				dolog("HLT origin: <none recorded>\n");
			}
			if (hlt_diag.wake_count > 0) {
				dolog("HLT wake: count=%u last_vec=%02x resume=%04x:%08x SS:ESP=%04x:%08x FL=%08x age_us=%u\n",
					hlt_diag.wake_count, hlt_diag.last_wake_vec,
					hlt_diag.last_wake_cs, hlt_diag.last_wake_eip,
					hlt_diag.last_wake_ss, hlt_diag.last_wake_sp,
					hlt_diag.last_wake_fl, wake_age_us);
			} else {
				dolog("HLT wake: count=0 age_us=%u\n", wake_age_us);
			}
			dolog("IRQ service quiet: age_us=%u\n", irq_quiet_us);
			dolog("Config: batch_setting=%d batch_effective=%d pit_burst=%d\n",
				pc_batch_size_setting, pc_last_batch_size, pc_pit_burst_setting);
			if (cpu->cb.pic) {
				PicState2 *pic = (PicState2 *)cpu->cb.pic;
				dolog("PIC irr=%04x isr=%04x imr=%04x pending_irq=%d\n",
					i8259_get_irr(pic), i8259_get_isr(pic),
					i8259_get_imr(pic), i8259_get_pending_irq(pic));
			}
			if (cpu->cb.pit) {
				PITState *pit = (PITState *)cpu->cb.pit;
				uint32_t pit_elapsed_us = pit_get_elapsed_us(pit, 0);
				uint32_t pit_last_irq_age = pit_get_last_irq_age_us(pit, 0);
				dolog("PIT ch0: pending=%d ticks=%llu mode=%d count=%d gate=%d out=%d\n",
					pit_get_pending_irqs(pit),
					(unsigned long long)pit_get_virtual_ticks(pit, 0),
					pit_get_mode(pit, 0),
					pit_get_initial_count(pit, 0),
					pit_get_gate(pit, 0),
					pit_get_out(pit, 0));
				dolog("PIT ch0 stats: loads=%llu pulses=%llu suppressed=%llu age_us=%u last_irq_age_us=%d\n",
					(unsigned long long)pit_get_load_count(pit, 0),
					(unsigned long long)pit_get_irq_pulse_count(pit, 0),
					(unsigned long long)pit_get_irq_suppressed_count(pit, 0),
					pit_elapsed_us,
					(pit_last_irq_age == 0xffffffffu) ? -1 : (int)pit_last_irq_age);
			}
				dump_exc_ring("halted post-mortem");
			}
			if (cpu_diag_enabled && (now - alive_last_log_time_us) > 3000000) {
				int pending_irq = -1;
				int pit_last_irq_age = -1;
				if (cpu->cb.pic) {
					pending_irq = i8259_get_pending_irq((PicState2 *)cpu->cb.pic);
				}
				if (cpu->cb.pit) {
					PITState *pit = (PITState *)cpu->cb.pit;
					uint32_t a = pit_get_last_irq_age_us(pit, 0);
					pit_last_irq_age = (a == 0xffffffffu) ? -1 : (int)a;
				}
				dolog("=== CPU alive (halt) %.1fs IF=%d intr=%d CS:EIP=%04x:%08x pending_irq=%d HLT(count=%u wake=%u) irq_quiet_us=%u wake_age_us=%u pit_last_irq_age_us=%d ===\n",
					halt_cont_us / 1000000.0f,
					!!(cpu->flags & IF), !!cpu->intr,
					cpu->seg[SEG_CS].sel, cpu->ip,
					pending_irq, hlt_diag.count, hlt_diag.wake_count,
					irq_quiet_us, wake_age_us, pit_last_irq_age);
				alive_last_log_time_us = now;
			}
			usleep(1);
			return;
		}
	halt_window_active = false;
	halt_window_start_us = 0;
	if (cpu_diag_enabled) {
		uint32_t now = get_uticks();
		if ((now - alive_last_log_time_us) > 3000000) {
			refresh_flags(cpu);
			dolog("=== CPU alive (run) IF=%d intr=%d CS:EIP=%04x:%08x next=%08x CPL=%d VM=%d ===\n",
				!!(cpu->flags & IF), !!cpu->intr,
				cpu->seg[SEG_CS].sel, cpu->ip, cpu->next_ip,
				cpu->cpl, !!(cpu->flags & VM));
			alive_last_log_time_us = now;
		}
	}

	/* Delayed ring buffer dump: fires 2s after last non-009f LDT #PF.
	 * This captures the crash aftermath without affecting timing. */
	if (cpu_diag_enabled && exc_ring_delayed_dump_time != 0 &&
	    (get_uticks() - exc_ring_delayed_dump_time) > 2000000) {
		exc_ring_delayed_dump_time = 0;
		exc_ring_trigger_pos = -1;  /* allow re-triggering */
		refresh_flags(cpu);
		dolog("=== Ring buffer dump (2s after last LDT #PF) ===\n");
		dolog("CS:EIP=%04x:%08x SS:ESP=%04x:%08x FL=%08x CPL=%d %s\n",
			cpu->seg[SEG_CS].sel, cpu->ip,
			cpu->seg[SEG_SS].sel, REGi(4),
			cpu->flags, cpu->cpl,
			(cpu->flags & VM) ? "V86" : ((cpu->cr0 & 1) ? "PM" : "RM"));
		dolog("EAX=%08x EBX=%08x ECX=%08x EDX=%08x\n",
			REGi(0), REGi(3), REGi(1), REGi(2));
		dolog("CR2=%08x CR3=%08x\n", cpu->cr2, cpu->cr3);
		dump_exc_ring("2s post-LDT-PF");

		/* Memory-state diagnostic: understand physical page usage and
		 * BFF-range PTE states at crash time. */
		{
			uword a20 = cpu->a20_mask;
			uword cr3_base = (cpu->cr3 & ~0xfff) & a20;
			dolog("=== Memory State at crash ===\n");
			dolog("  phys_mem_size=%lu (%lu KB, %lu MB)\n",
				(unsigned long)cpu->phys_mem_size,
				(unsigned long)cpu->phys_mem_size / 1024,
				(unsigned long)cpu->phys_mem_size / (1024 * 1024));

			/* Count present PDEs and total present PTEs */
			int pde_present = 0, pte_present = 0, pte_not_present_nonzero = 0;
			for (int pdi = 0; pdi < 1024; pdi++) {
				uword pde_addr = cr3_base + pdi * 4;
				if ((uint64_t)pde_addr + 4 > (uint64_t)cpu->phys_mem_size) continue;
				uword pde = pload32(cpu, pde_addr);
				if (!(pde & 1)) continue;
				pde_present++;
				if ((cpu->cr4 & CR4_PSE) && (pde & (1 << 7))) {
					pte_present += 1024; /* 4MB page = 1024 4KB pages */
					continue;
				}
				uword pt_base = (pde & ~0xfff) & a20;
				for (int pti = 0; pti < 1024; pti++) {
					uword pte_addr = pt_base + pti * 4;
					if ((uint64_t)pte_addr + 4 > (uint64_t)cpu->phys_mem_size) continue;
					uword pte = pload32(cpu, pte_addr);
					if (pte & 1) pte_present++;
					else if (pte) pte_not_present_nonzero++;
				}
			}
			dolog("  PDEs present: %d/1024\n", pde_present);
			dolog("  PTEs present: %d (~%d KB mapped)\n",
				pte_present, pte_present * 4);
			dolog("  PTEs not-present but non-zero: %d (demand-page/swapped)\n",
				pte_not_present_nonzero);
			dolog("  Phys pages total: %lu\n",
				(unsigned long)(cpu->phys_mem_size / 4096));

			/* Count distinct physical pages referenced by present PTEs.
			 * Uses a bitmap: 1 bit per physical page. */
			{
				uword total_phys_pages = cpu->phys_mem_size / 4096;
				uword bitmap_bytes = (total_phys_pages + 7) / 8;
				uint8_t *used_pages = calloc(bitmap_bytes, 1);
				if (used_pages) {
					int distinct_pages = 0;
					for (int pdi = 0; pdi < 1024; pdi++) {
						uword pde_addr = cr3_base + pdi * 4;
						if ((uint64_t)pde_addr + 4 > (uint64_t)cpu->phys_mem_size) continue;
						uword pde = pload32(cpu, pde_addr);
						if (!(pde & 1)) continue;
						/* Mark page table page itself as used */
						uword pt_phys = (pde >> 12) & (total_phys_pages - 1);
						if (pt_phys < total_phys_pages && !(used_pages[pt_phys / 8] & (1 << (pt_phys % 8)))) {
							used_pages[pt_phys / 8] |= (1 << (pt_phys % 8));
							distinct_pages++;
						}
						if ((cpu->cr4 & CR4_PSE) && (pde & (1 << 7))) continue;
						uword pt_base = (pde & ~0xfff) & a20;
						for (int pti = 0; pti < 1024; pti++) {
							uword pte_addr = pt_base + pti * 4;
							if ((uint64_t)pte_addr + 4 > (uint64_t)cpu->phys_mem_size) continue;
							uword pte = pload32(cpu, pte_addr);
							if (!(pte & 1)) continue;
							uword ppn = (pte >> 12) & (total_phys_pages - 1);
							if (ppn < total_phys_pages && !(used_pages[ppn / 8] & (1 << (ppn % 8)))) {
								used_pages[ppn / 8] |= (1 << (ppn % 8));
								distinct_pages++;
							}
						}
					}
					/* Also mark page directory itself */
					uword pd_phys = (cr3_base >> 12);
					if (pd_phys < total_phys_pages && !(used_pages[pd_phys / 8] & (1 << (pd_phys % 8)))) {
						distinct_pages++;
					}
					dolog("  Distinct physical pages in use: %d / %lu (free: ~%lu)\n",
						distinct_pages, (unsigned long)total_phys_pages,
						(unsigned long)(total_phys_pages - distinct_pages));
					free(used_pages);
				}
			}

			/* Dump BFF range PTEs (KERNEL32.DLL area) */
			uword bff_pdi = 0xBFF00000 >> 22; /* PDI for BFF range = 0x2FF */
			uword bff_pde_addr = cr3_base + bff_pdi * 4;
			if ((uint64_t)bff_pde_addr + 4 <= (uint64_t)cpu->phys_mem_size) {
				uword bff_pde = pload32(cpu, bff_pde_addr);
				dolog("  BFF PDE[%03x] = %08x P=%d\n", bff_pdi, bff_pde, !!(bff_pde & 1));
				if (bff_pde & 1) {
					uword bff_pt_base = (bff_pde & ~0xfff) & a20;
					int bff_present = 0, bff_swapped = 0;
					/* Scan PTI range 0x300-0x3FF (BFF00000-BFFFFFFF) */
					for (int pti = 0x300; pti < 0x400; pti++) {
						uword pte_addr = bff_pt_base + pti * 4;
						if ((uint64_t)pte_addr + 4 > (uint64_t)cpu->phys_mem_size) continue;
						uword pte = pload32(cpu, pte_addr);
						if (pte & 1) bff_present++;
						else if (pte) bff_swapped++;
					}
					dolog("  BFF PTEs (300-3FF): %d present, %d swapped/demand\n",
						bff_present, bff_swapped);
					/* Dump specific crash-area PTEs */
					int interesting[] = { 0x370, 0x371, 0x398, 0x399, 0x39A, 0x39B, 0x3BB, 0x3BC };
					for (int k = 0; k < 8; k++) {
						int pti = interesting[k];
						uword pte_addr = bff_pt_base + pti * 4;
						if ((uint64_t)pte_addr + 4 > (uint64_t)cpu->phys_mem_size) continue;
						uword pte = pload32(cpu, pte_addr);
						dolog("  BFF PTE[%03x] (LA=%08x) = %08x P=%d\n",
							pti, (bff_pdi << 22) | (pti << 12), pte, !!(pte & 1));
					}
				}
			}
		}

		/* Dump IDT[14] (#PF handler) and IDT[13] (#GP handler) to verify integrity */
		dump_idt_vector(cpu, 14, "IDT at crash");
		dump_idt_vector(cpu, 13, "IDT at crash");
	}

	/* Track live changes to IDT[0x41] (VxD call path) to catch descriptor clobbering. */
	if (cpu_diag_enabled && (cpu->cr0 & 1)) {
		static bool idt41_have_last = false;
		static uword idt41_last_w1 = 0, idt41_last_w2 = 0;
		static int idt41_diag_gen = -1;
		uword w1, w2;
		if (idt41_diag_gen != cpu->diag_gen) {
			idt41_diag_gen = cpu->diag_gen;
			idt41_have_last = false;
		}
		if (read_idt_vector_raw_noexcept(cpu, 0x41, &w1, &w2)) {
			if (!idt41_have_last || w1 != idt41_last_w1 || w2 != idt41_last_w2) {
				dolog("=== IDT[41] change ===\n");
				dolog("  from %08x %08x\n", idt41_last_w1, idt41_last_w2);
				dolog("  to   %08x %08x at CS:EIP=%04x:%08x CPL=%d\n",
					w1, w2, cpu->seg[SEG_CS].sel, cpu->ip, cpu->cpl);
				idt41_last_w1 = w1;
				idt41_last_w2 = w2;
				idt41_have_last = true;
			}
		}
	}

	if (!cpu_exec1(cpu, stepcount)) {
		bool pusherr = false;
		switch (cpu->excno) {
		case EX_DF: case EX_TS: case EX_NP: case EX_SS: case EX_GP:
		case EX_PF:
		case 17: /* #AC pushes error code 0 */
			pusherr = true;
		}
		int orig_exc = cpu->excno;
		cpu->next_ip = cpu->ip;

		/* Count real exceptions here (not in call_isr) to avoid
		 * conflating with PIC IRQs sharing vectors 0-31. */
		if (unlikely(cpu_diag_enabled) && win386_diag.active && orig_exc < 32)
			wfw_monitor.exc_counts[orig_exc]++;

		if (cpu_diag_enabled)
			cpu_diag_win386_sync(cpu);

		/* One-shot deep PF diagnostics for Win95 shared-user range faults. */
		if (cpu_diag_enabled && orig_exc == EX_PF) {
			static bool pf_bff_dumped = false;
			static bool pf_target_dumped = false;
			static int pf_diag_gen = -1;
			if (pf_diag_gen != cpu->diag_gen) {
				pf_diag_gen = cpu->diag_gen;
				pf_bff_dumped = false;
				pf_target_dumped = false;
			}
			if (!pf_bff_dumped && cpu->cr2 >= 0xbff00000 && cpu->cr2 < 0xc0000000) {
				pf_bff_dumped = true;
				refresh_flags(cpu);
				dolog("=== FIRST BFF #PF ===\n");
				dolog("  fault @ CS:EIP=%04x:%08x err=%x CPL=%d FL=%08x\n",
					cpu->seg[SEG_CS].sel, cpu->ip, cpu->excerr, cpu->cpl, cpu->flags);
				dump_pf_walk(cpu, cpu->cr2, "FIRST BFF #PF");
			}
			if (!pf_target_dumped && cpu->cr2 >= 0xbff98000 && cpu->cr2 < 0xbff99000) {
				pf_target_dumped = true;
				refresh_flags(cpu);
				dolog("=== TARGET #PF CR2 in BFF98000..BFF98FFF ===\n");
				dolog("  fault @ CS:EIP=%04x:%08x err=%x CPL=%d FL=%08x\n",
					cpu->seg[SEG_CS].sel, cpu->ip, cpu->excerr, cpu->cpl, cpu->flags);
				dump_pf_walk(cpu, cpu->cr2, "TARGET #PF");
			}
		}

		/* One-shot: diagnose IDT-referenced #GP (e.g., err=20a -> vector 0x41). */
		if (cpu_diag_enabled && orig_exc == EX_GP && (cpu->excerr & 0x2)) {
			static bool gp_idt_dumped = false;
			static int gp_diag_gen = -1;
			if (gp_diag_gen != cpu->diag_gen) {
				gp_diag_gen = cpu->diag_gen;
				gp_idt_dumped = false;
			}
			if (!gp_idt_dumped) {
				int vec = (cpu->excerr >> 3) & 0x1fff;
				gp_idt_dumped = true;
				refresh_flags(cpu);
				dolog("=== FIRST #GP with IDT err=%x (vec=%x) ===\n", cpu->excerr, vec);
				dolog("  at CS:EIP=%04x:%08x CPL=%d FL=%08x\n",
					cpu->seg[SEG_CS].sel, cpu->ip, cpu->cpl, cpu->flags);
				if (vec < 256) dump_idt_vector(cpu, vec, "FIRST GP(IDT)");
				dump_cs_bytes(cpu, "  opcodes @CS:EIP:", 8);
			}
		}

		/* One-shot: diagnose alignment-check exception context. */
		if (cpu_diag_enabled && orig_exc == 17) {
			static bool ac_dumped = false;
			static int ac_diag_gen = -1;
			if (ac_diag_gen != cpu->diag_gen) {
				ac_diag_gen = cpu->diag_gen;
				ac_dumped = false;
			}
			if (!ac_dumped) {
				ac_dumped = true;
				refresh_flags(cpu);
				dolog("=== FIRST #AC ===\n");
				dolog("  CS:EIP=%04x:%08x SS:ESP=%04x:%08x CPL=%d\n",
					cpu->seg[SEG_CS].sel, cpu->ip,
					cpu->seg[SEG_SS].sel, REGi(4), cpu->cpl);
				dolog("  FL=%08x CR0=%08x CR2=%08x CR3=%08x\n",
					cpu->flags, cpu->cr0, cpu->cr2, cpu->cr3);
			}
		}

			/* Capture ALL VM86 exceptions in fault ring (not just #GP/#UD)
			 * so we can see #PF, #SS, etc. that may cause VMM to crash the VM. */
			if (cpu_diag_enabled && (cpu->flags & VM)) {
				char last_decoded[80];
				uint8_t trip_bytes[8];
				uint8_t trip_blen = wfw_capture_cs_bytes(cpu, trip_bytes);
				if (orig_exc == EX_GP || orig_exc == EX_UD) {
					bool last_is_io, last_needs_stack_dump;
					int last_io_port, last_io_bits;
					vm86_decode_fault_instruction(cpu, last_decoded, sizeof(last_decoded),
								      &last_is_io, &last_io_port, &last_io_bits, &last_needs_stack_dump);
			} else if (orig_exc == EX_PF) {
				snprintf(last_decoded, sizeof(last_decoded), "#PF CR2=%08x err=%x", cpu->cr2, cpu->excerr);
			} else {
				static const char *exc_names[] = {
					"#DE","#DB","NMI","#BP","#OF","#BR","#UD","#NM",
					"#DF","","#TS","#NP","#SS","#GP","#PF","","","#AC"
				};
				const char *name = (orig_exc < 18) ? exc_names[orig_exc] : "?";
				snprintf(last_decoded, sizeof(last_decoded), "%s(%d) err=%x", name, orig_exc,
					 pusherr ? cpu->excerr : 0);
			}
				if (vm86_last_fault.gen != cpu->diag_gen) {
					memset(&vm86_last_fault, 0, sizeof(vm86_last_fault));
					vm86_last_fault.gen = cpu->diag_gen;
				}
				vm86_trip_push(cpu, VM86_TRIP_E, (uint8_t)orig_exc,
					       cpu->seg[SEG_CS].sel, cpu->ip,
					       cpu->seg[SEG_SS].sel, REGi(4), cpu->flags,
					       0, pusherr ? cpu->excerr : 0, 0,
					       0, trip_bytes, trip_blen);
				cpu_diag_vm86_fault_ring_push(cpu, orig_exc, pusherr, last_decoded);

			/* Detect VM86 crash: execution at IVT area (seg 0, low offset)
			 * means the VMM failed to handle an instruction and crashed the
			 * task.  Dump the fault ring to identify the failing instruction. */
			if (cpu->seg[SEG_CS].sel == 0 && cpu->ip < 0x100) {
				dolog("=== VM86 CRASH detected at %04x:%08x — fault ring tail: ===\n",
					cpu->seg[SEG_CS].sel, cpu->ip);
				cpu_diag_vm86_fault_ring_dump_tail(32);
				cpu_diag_vm86_iret_ring_dump_tail(16);
			}

			vm86_last_fault.valid = true;
			vm86_last_fault.excno = (uint8_t)orig_exc;
			vm86_last_fault.cs = cpu->seg[SEG_CS].sel;
			vm86_last_fault.ip = cpu->ip;
			vm86_last_fault.fl = cpu->flags;
			vm86_last_fault.err = pusherr ? cpu->excerr : 0;
			snprintf(vm86_last_fault.decoded, sizeof(vm86_last_fault.decoded), "%s", last_decoded);

				if (orig_exc == EX_GP || orig_exc == EX_UD) {
					uint16_t cs_sel = cpu->seg[SEG_CS].sel;
					uint16_t ip16 = cpu->ip & 0xffff;
						if (orig_exc == EX_UD && trip_blen > 0 && trip_bytes[0] == 0x63 &&
						    win386_diag.active) {
							int ui = wfw_ud63_find_or_add(cs_sel, ip16);
							if (ui >= 0 && ui < wfw_monitor.ud63_hot_count) {
							wfw_monitor.ud63_hot[ui].count++;
							wfw_monitor.ud63_hot[ui].ax = REGi(0) & 0xffff;
							wfw_monitor.ud63_hot[ui].bx = REGi(3) & 0xffff;
							wfw_monitor.ud63_hot[ui].cx = REGi(1) & 0xffff;
							wfw_monitor.ud63_hot[ui].dx = REGi(2) & 0xffff;
							wfw_monitor.ud63_hot[ui].si = REGi(6) & 0xffff;
							wfw_monitor.ud63_hot[ui].di = REGi(7) & 0xffff;
							wfw_monitor.ud63_hot[ui].ss = cpu->seg[SEG_SS].sel;
							wfw_monitor.ud63_hot[ui].sp = REGi(4);
							wfw_monitor.ud63_hot[ui].fl = cpu->flags;
							if (wfw_monitor.ud63_hot[ui].blen == 0)
								wfw_monitor.ud63_hot[ui].blen =
									wfw_capture_cs_bytes(cpu, wfw_monitor.ud63_hot[ui].bytes);
								wfw_monitor.pending_ud63_idx = (int8_t)ui;
								wfw_monitor.pending_ud63_cs = cs_sel;
								wfw_monitor.pending_ud63_ip = cpu->ip;
								/* Focused probe: ARPL resize callback at FE95:1638.
								 * Snapshot sel 0127 descriptor before handler runs,
								 * then compare with post-IRET snapshot. */
								if (cs_sel == 0xfe95 && ip16 == 0x1638) {
									uint32_t w1 = 0xffffffffu, w2 = 0xffffffffu;
									if (wfw_read_ldt_desc_noexcept(cpu, 0x0127, &w1, &w2)) {
										wfw_monitor.ud63_fe95_0127.pre_w1 = w1;
										wfw_monitor.ud63_fe95_0127.pre_w2 = w2;
									} else {
										wfw_monitor.ud63_fe95_0127.pre_w1 = 0xffffffffu;
										wfw_monitor.ud63_fe95_0127.pre_w2 = 0xffffffffu;
									}
									wfw_monitor.ud63_fe95_0127.seen++;
									wfw_monitor.ud63_fe95_0127.pre_cs = cs_sel;
									wfw_monitor.ud63_fe95_0127.pre_ip = ip16;
									wfw_monitor.ud63_fe95_0127.pre_ds = cpu->seg[SEG_DS].sel;
									wfw_monitor.ud63_fe95_0127.pre_ax = REGi(0) & 0xffff;
									wfw_monitor.ud63_fe95_0127.pre_fl = cpu->flags;
									wfw_ud63_snapshot_near_desc(cpu,
										wfw_monitor.ud63_fe95_0127.pre_near_w1,
										wfw_monitor.ud63_fe95_0127.pre_near_w2,
										&wfw_monitor.ud63_fe95_0127.pre_near_ok);
									if (!wfw_monitor.ud63_fe95_0127.trace_done) {
										wfw_monitor.ud63_fe95_0127.trace_active = true;
										wfw_monitor.ud63_fe95_0127.trace_count = 0;
										wfw_monitor.ud63_fe95_0127.trace_entry_ax = REGi(0) & 0xffff;
										wfw_monitor.ud63_fe95_0127.trace_prev_ax =
											wfw_monitor.ud63_fe95_0127.trace_entry_ax;
										wfw_monitor.ud63_fe95_0127.trace_ax5_valid = false;
										wfw_monitor.ud63_fe95_0127.trace_ax5_prev = 0;
										wfw_monitor.ud63_fe95_0127.trace_ax5_ax = 0;
										wfw_monitor.ud63_fe95_0127.trace_ax5_cs = 0;
										wfw_monitor.ud63_fe95_0127.trace_ax5_ip = 0;
									}
									wfw_monitor.ud63_fe95_0127.j6cab_pending = false;
									wfw_monitor.ud63_fe95_0127.step_prev_valid = false;
									wfw_monitor.pending_ud63_fe95_0127 = true;
								}
							}
						}
					if (cs_sel == 0xff57 ||
					    (cs_sel >= 0xf000 && ip16 >= 0xff00))
						vm86_trip_mark_first_bad(cpu, (uint8_t)orig_exc, cs_sel, cpu->ip, last_decoded);
					if (win386_diag.active && cs_sel >= 0xc000) {
						int bi = wfw_bios_hot_find_or_add(cs_sel, ip16, (uint8_t)orig_exc);
						if (bi >= 0 && bi < wfw_monitor.bios_hot_count) {
						wfw_monitor.bios_hot[bi].count++;
						if (wfw_monitor.bios_hot[bi].decoded[0] == '\0')
							snprintf(wfw_monitor.bios_hot[bi].decoded,
								 sizeof(wfw_monitor.bios_hot[bi].decoded),
								 "%s", last_decoded);
						if (wfw_monitor.bios_hot[bi].blen == 0)
							wfw_monitor.bios_hot[bi].blen =
								wfw_capture_cs_bytes(cpu, wfw_monitor.bios_hot[bi].bytes);
						wfw_monitor.pending_bios_fault_idx = (int8_t)bi;
					}
				}
				/* BIOS privileged instruction detection: always log
				 * #GP at BIOS addresses (f000/c000+) regardless of
				 * event counter — these reveal SeaBIOS encoding issues. */
				if (orig_exc == EX_GP && cs_sel >= 0xc000 && win386_diag.active) {
					char decoded[80];
					bool is_io, needs_stack_dump;
					int io_port, io_bits;
					vm86_decode_fault_instruction(cpu, decoded, sizeof(decoded),
								     &is_io, &io_port, &io_bits, &needs_stack_dump);
						/* Track LGDT/LIDT/LMSW specifically */
						if (decoded[0] == '0' && decoded[1] == 'F') {
							wfw_monitor.bios_lgdt_count++;
							if (!vm86_trip_silence_live_logs(cpu))
								dolog("*** V86 BIOS #GP %04x:%04x %s (SeaBIOS privileged!)\n",
									cs_sel, cpu->ip & 0xffff, decoded);
						} else if (!is_io) {
							wfw_monitor.bios_priv_count++;
							/* Only log first few — these can be frequent during
							 * VM86 trap/reflect phases (CLI/STI/PUSHF/POPF). */
							if (wfw_monitor.bios_priv_count <= 8 &&
							    !vm86_trip_silence_live_logs(cpu))
								dolog("V86 BIOS #GP %04x:%04x %s\n",
									cs_sel, cpu->ip & 0xffff, decoded);
						}
					}

					uint8_t *slot = (orig_exc == EX_GP) ? &cpu_diag_events.vm86_gp_decode :
									      &cpu_diag_events.vm86_ud_decode;
					if (!vm86_trip_silence_live_logs(cpu) && cpu_diag_event_allow(cpu, slot)) {
						char decoded[80];
						bool is_io, needs_stack_dump;
						int io_port, io_bits;
					vm86_decode_fault_instruction(cpu, decoded, sizeof(decoded),
								     &is_io, &io_port, &io_bits, &needs_stack_dump);
					dolog("V86 #%s #%u %04x:%04x %s\n",
						(orig_exc == EX_GP) ? "GP" : "UD", *slot,
						cpu->seg[SEG_CS].sel, cpu->ip & 0xffff, decoded);
				}
			}
		}

		if (cpu_diag_enabled && !(cpu->flags & VM) && win386_diag.active &&
		    orig_exc < 32) {
			win386_diag.pm_exc_count++;
		}

		if (unlikely(!call_isr(cpu, cpu->excno, pusherr, 1))) {
			/* Exception delivery failed — apply 386 double-fault
			 * escalation matrix instead of blindly escalating. */
			int second_exc = cpu->excno;
			refresh_flags(cpu);

			if (unlikely(cpu_diag_enabled) && win386_diag.active)
				wfw_monitor.isr_failures++;

			/* One-shot dump on first delivery failure */
			if (cpu_diag_enabled) {
				static bool isr_fail_dumped = false;
				static int isrf_diag_gen = -1;
				if (isrf_diag_gen != cpu->diag_gen) { isrf_diag_gen = cpu->diag_gen; isr_fail_dumped = false; }
				if (!isr_fail_dumped) {
					isr_fail_dumped = true;
					dolog("ISR fail: exc %d during %d (%d+%d) %04x:%08x %s CR2=%08x\n",
						second_exc, orig_exc,
						exc_class_386(orig_exc), exc_class_386(second_exc),
						cpu->seg[SEG_CS].sel, cpu->ip,
						(cpu->flags & VM) ? "V86" : "PM", cpu->cr2);
				}
			}

			if (orig_exc == EX_DF) {
				/* Double fault delivery failed — triple fault */
				goto triple_fault;
			}

			/* 386 escalation matrix:
			 *   contributory + contributory → #DF
			 *   page fault   + page fault   → #DF
			 *   everything else             → service second exception */
			int c1 = exc_class_386(orig_exc);
			int c2 = exc_class_386(second_exc);
			if (c1 != 0 && c1 == c2) {
				/* Escalate to #DF with error code 0 */
				if (unlikely(cpu_diag_enabled) && win386_diag.active)
					wfw_monitor.exc_counts[EX_DF]++;
				cpu->excerr = 0;
				if (!call_isr(cpu, EX_DF, true, 1))
					goto triple_fault;
			} else {
				/* Service second exception sequentially */
				if (unlikely(cpu_diag_enabled) && win386_diag.active && second_exc < 32)
					wfw_monitor.exc_counts[second_exc]++;
				bool pusherr2 = exc_pushes_error_code(second_exc);
				cpu->next_ip = cpu->ip;
				if (!call_isr(cpu, second_exc, pusherr2, 1)) {
					/* Second delivery also failed — escalate to #DF */
					if (unlikely(cpu_diag_enabled) && win386_diag.active)
						wfw_monitor.exc_counts[EX_DF]++;
					cpu->excerr = 0;
					if (!call_isr(cpu, EX_DF, true, 1))
						goto triple_fault;
				}
			}
		}
		/* (diagnostic dumps removed — using lightweight ring buffer only) */
	}
	return;

triple_fault:
	{
		static bool tf_dumped = false;
		static int tf_diag_gen = -1;
		if (tf_diag_gen != cpu->diag_gen) { tf_diag_gen = cpu->diag_gen; tf_dumped = false; }
		if (!tf_dumped) {
			tf_dumped = true;
			refresh_flags(cpu);
			dolog("=== TRIPLE FAULT — full state dump ===\n");
			dolog("CS:EIP=%04x:%08x SS:ESP=%04x:%08x FL=%08x CR0=%08x %s\n",
				cpu->seg[SEG_CS].sel, cpu->ip,
				cpu->seg[SEG_SS].sel, REGi(4),
				cpu->flags, cpu->cr0,
				(cpu->flags & VM) ? "V86" : "PM");
			dolog("EAX=%08x EBX=%08x ECX=%08x EDX=%08x\n",
				REGi(0), REGi(3), REGi(1), REGi(2));
			dolog("ESI=%08x EDI=%08x EBP=%08x\n",
				REGi(6), REGi(7), REGi(5));
			dolog("DS=%04x ES=%04x FS=%04x GS=%04x\n",
				cpu->seg[SEG_DS].sel, cpu->seg[SEG_ES].sel,
				cpu->seg[SEG_FS].sel, cpu->seg[SEG_GS].sel);
			dolog("LDT sel=%04x base=%08x limit=%x\n",
				cpu->seg[SEG_LDT].sel, cpu->seg[SEG_LDT].base,
				cpu->seg[SEG_LDT].limit);
			dolog("IDT base=%08x limit=%x TR sel=%04x base=%08x limit=%x\n",
				cpu->idt.base, cpu->idt.limit,
				cpu->seg[SEG_TR].sel, cpu->seg[SEG_TR].base,
				cpu->seg[SEG_TR].limit);
			dolog("CR2=%08x CR3=%08x CPL=%d\n",
				cpu->cr2, cpu->cr3, cpu->cpl);
		}
		if (win386_diag.active)
			cpu_monitor_dump(cpu, "triple fault");
		dolog("triple fault — resetting CPU\n");
		cpui386_reset(cpu);
	}
}

void cpu_setax(CPUI386 *cpu, u16 ax)
{
	sreg16(0, ax);
}

u16 cpu_getax(CPUI386 *cpu)
{
	return lreg16(0);
}

void cpu_setexc(CPUI386 *cpu, int excno, uword excerr)
{
	cpu->excno = excno;
	cpu->excerr = excerr;
}

void cpu_setflags(CPUI386 *cpu, uword set_mask, uword clear_mask)
{
	if (cpu->cc.mask & (set_mask | clear_mask)) {
		refresh_flags(cpu);
		cpu->cc.mask = 0;
	}
	cpu->flags |= set_mask;
	cpu->flags &= ~clear_mask;
	cpu->flags &= EFLAGS_MASK;
}

uword cpu_getflags(CPUI386 *cpu)
{
	if (cpu->cc.mask) {
		refresh_flags(cpu);
		cpu->cc.mask = 0;
	}
	return cpu->flags;
}

void cpui386_reset(CPUI386 *cpu)
{
	for (int i = 0; i < 8; i++) {
		REGi(i) = 0;
	}
	cpu->flags = 0x2;
	cpu->cpl = 0;
	cpu->code16 = true;
	cpu->sp_mask = 0xffff;
	cpu->halt = false;
	cpu->tf_trap_pending = false;

	for (int i = 0; i < 8; i++) {
		cpu->seg[i].sel = 0;
		cpu->seg[i].base = 0;
		cpu->seg[i].limit = 0;
		cpu->seg[i].flags = 0;
	}
	cpu->seg[2].flags = (1 << 22);
	cpu->seg[1].flags = (1 << 22);

	cpu->ip = 0xfff0;
	cpu->next_ip = cpu->ip;
	cpu->seg[SEG_CS].sel = 0xf000;
	cpu->seg[SEG_CS].base = 0xf0000;

	cpu->idt.base = 0;
	cpu->idt.limit = 0x3ff;
	cpu->gdt.base = 0;
	cpu->gdt.limit = 0;
	cpu->int8_cache_valid = false;
	cpu->int8_warmup_counter = 0;  /* Reset INT 8 cache */

	cpu->cr0 = cpu->fpu ? 0x10 : 0;
	cpu->cr2 = 0;
	cpu->cr3 = 0;
	cpu->cr4 = 0;
	cpu->a20_mask = 0xFFEFFFFF;  /* A20 disabled at reset (real hardware behavior) */
	for (int i = 0; i < 8; i++)
		cpu->dr[i] = 0;

	cpu->cc.mask = 0;
	tlb_clear(cpu);

	cpu->sysenter.cs = 0;
	cpu->sysenter.eip = 0;
	cpu->sysenter.esp = 0;

	/* No segments are flat at reset (all have limit=0, not 4GB) */
	cpu->seg_flat = 0;
	cpu->all_segs_flat = false;
	cpu->diag_gen++;

#ifdef I386_SEQ_FASTPATH
	cpu->seq_active = false;
#endif
}

void cpui386_reset_pm(CPUI386 *cpu, uint32_t start_addr)
{
	cpui386_reset(cpu);
	cpu->cr0 = 1;
	cpu->seg[SEG_CS].sel = 0x8;
	cpu->seg[SEG_CS].base = 0;
	cpu->seg[SEG_CS].limit = 0xffffffff;
	cpu->seg[SEG_CS].flags = SEG_D_BIT;
	cpu->next_ip = start_addr;
	cpu->cpl = 0;
	cpu->code16 = false;
	cpu->sp_mask = 0xffffffff;
	cpu->seg[SEG_SS].sel = 0x10;
	cpu->seg[SEG_SS].base = 0;
	cpu->seg[SEG_SS].limit = 0xffffffff;
	cpu->seg[SEG_SS].flags = SEG_B_BIT;

	cpu->seg[SEG_DS] = cpu->seg[SEG_SS];
	cpu->seg[SEG_ES] = cpu->seg[SEG_SS];
}

void IRAM_ATTR cpui386_raise_irq(CPUI386 *cpu)
{
	cpu->intr = true;
}

void cpui386_set_gpr(CPUI386 *cpu, int i, u32 val)
{
	sreg32(i, val);
}

void cpui386_set_a20(CPUI386 *cpu, int enabled)
{
	uint32_t new_mask = enabled ? 0xFFFFFFFF : 0xFFEFFFFF;
	if (cpu->a20_mask != new_mask) {
		if (unlikely(cpu_diag_enabled) && win386_diag.active) {
			wfw_monitor.a20_toggles++;
			wfw_monitor.a20_state = enabled;
		}
		cpu->a20_mask = new_mask;
		tlb_clear(cpu);
		cpu->ifetch.laddr = -1;
#ifdef I386_SEQ_FASTPATH
		cpu->seq_active = false;
#endif
	}
}

int cpui386_get_a20(CPUI386 *cpu)
{
	return (cpu->a20_mask >> 20) & 1;
}

void cpui386_set_diag(CPUI386 *cpu, bool enabled)
{
	(void)cpu;
	cpu_diag_enabled = enabled;
}

bool cpui386_is_halted(CPUI386 *cpu)
{
	return cpu->halt;
}

long IRAM_ATTR cpui386_get_cycle(CPUI386 *cpu)
{
	/* Periodic TSC sync to prevent 32-bit cycle delta overflow */
	cpu->tsc += (uint32_t)((uint32_t)cpu->cycle - (uint32_t)cpu->tsc_sync_cycle);
	cpu->tsc_sync_cycle = cpu->cycle;
	return cpu->cycle;
}

uint32_t cpui386_get_seq_hits(CPUI386 *cpu)
{
#ifdef I386_SEQ_FASTPATH
	return cpu->seq_hits;
#else
	return 0;
#endif
}

void cpui386_get_perf_counters(CPUI386 *cpu, uint32_t *tlb_miss, uint32_t *irq,
                               uint32_t *fusion, uint32_t *hle_hit, uint32_t *hle_call)
{
	*tlb_miss = cpu->tlb_miss_count;
	*irq = cpu->irq_count;
	*fusion = cpu->fusion_count;
	*hle_hit = cpu->hle_hit_count;
	*hle_call = cpu->hle_call_count;
}

CPUI386 *cpui386_new(int gen, char *phys_mem, long phys_mem_size, CPU_CB **cb)
{
	/* Initialize flags lookup tables (256KB in PSRAM for fast flag computation) */
	i386_init_flags_tables();

	/* Allocate CPU struct in TCM for fastest possible access (0-1 cycle).
	 * TCM is 8KB on ESP32-P4; CPUI386 is ~800B — fits easily. */
	static struct CPUI386 TCM_DRAM_ATTR cpu_tcm __attribute__((aligned(64)));
	CPUI386 *cpu = &cpu_tcm;
	memset(cpu, 0, sizeof(*cpu));
	switch (gen) {
	case 3: cpu->flags_mask = EFLAGS_MASK_386; break;
	case 4: cpu->flags_mask = EFLAGS_MASK_486; break;
	case 5: case 6: cpu->flags_mask = EFLAGS_MASK_586; break;
	default: assert(false);
	}
	cpu->gen = gen;

	cpu->tlb.size = tlb_size;
	cpu->tlb.generation = 1;  /* start at 1 so default-zero entries miss */
	// Allocate TLB in internal RAM for fastest access
	cpu->tlb.tab = heap_caps_malloc(sizeof(struct tlb_entry) * tlb_size,
	                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

	cpu->phys_mem = (u8 *) phys_mem;
	cpu->phys_mem_size = phys_mem_size;

	cpu->cycle = 0;
	cpu->tsc_sync_cycle = 0;

	cpu->intr = false;

	cpu->fpu = NULL;

	cpui386_reset(cpu);

	memset(&(cpu->cb), 0, sizeof(CPU_CB));
	if (cb)
		*cb = &(cpu->cb);
	return cpu;
}

void cpui386_enable_fpu(CPUI386 *cpu)
{
	if (!cpu->fpu)
		cpu->fpu = fpu_new();
}

void cpui386_delete(CPUI386 *cpu)
{
	if (cpu->fpu)
		fpu_delete(cpu->fpu);
	free(cpu->tlb.tab);
	free(cpu);
}

void cpui386_set_verbose() // for debugging
{
	verbose = true;
	freopen("/tmp/xlog", "w", stderr);
	setlinebuf(stderr);
}

static void cpu_debug(CPUI386 *cpu)
{
	static int nest;
	if (nest >= 1)
		return;
	nest++;
	bool code32 = cpu->seg[SEG_CS].flags & SEG_D_BIT;
	bool stack32 = cpu->seg[SEG_SS].flags & SEG_B_BIT;

	dolog("IP %08x|AX %08x|CX %08x|DX %08x|BX %08x|SP %08x|BP %08x|SI %08x|DI %08x|FL %08x|CS %04x|DS %04x|SS %04x|ES %04x|FS %04x|GS %04x|CR0 %08x|CR2 %08x|CR3 %08x|CR4 %08x|CPL %d|IOPL %d|CSBASE %08x/%08x|DSBASE %08x/%08x|SSBASE %08x/%08x|ESBASE %08x/%08x|GSBASE %08x/%08x %c%c\n",
		cpu->ip, REGi(0), REGi(1), REGi(2), REGi(3),
		REGi(4), REGi(5), REGi(6), REGi(7),
		cpu->flags, SEGi(SEG_CS), SEGi(SEG_DS), SEGi(SEG_SS),
		SEGi(SEG_ES), SEGi(SEG_FS), SEGi(SEG_GS),
		cpu->cr0, cpu->cr2, cpu->cr3, cpu->cr4, cpu->cpl, get_IOPL(cpu),
		cpu->seg[SEG_CS].base, cpu->seg[SEG_CS].limit,
		cpu->seg[SEG_DS].base, cpu->seg[SEG_DS].limit,
		cpu->seg[SEG_SS].base, cpu->seg[SEG_SS].limit,
		cpu->seg[SEG_ES].base, cpu->seg[SEG_ES].limit,
		cpu->seg[SEG_GS].base, cpu->seg[SEG_GS].limit,
		code32 ? 'D' : ' ', stack32 ? 'B' : ' ');
	uword cr2, excno, excerr;
	cr2 = cpu->cr2;
	excno = cpu->excno;
	excerr = cpu->excerr;
	dolog("code: ");
	for (int i = 0; i < 32; i++) {
		OptAddr res;
		if(translate8(cpu, &res, 1, SEG_CS, cpu->ip + i))
			dolog(" %02x", load8(cpu, &res));
		else
			dolog(" ??");
	}
	dolog("\n");
	dolog("stack: ");
	uword sp_mask = cpu->seg[SEG_SS].flags & SEG_B_BIT ? 0xffffffff : 0xffff;
	for (int i = 0; i < 32; i++) {
		OptAddr res;
		if(translate8(cpu, &res, 1, SEG_SS, (REGi(4) + i) & sp_mask))
			dolog(" %02x", load8(cpu, &res));
		else
			dolog(" ??");
	}
	dolog("\n");
	dolog("stkf : ");
	for (int i = 0; i < 32; i++) {
		OptAddr res;
		if(translate8(cpu, &res, 1, SEG_SS, (REGi(5) + i) & sp_mask))
			dolog(" %02x", load8(cpu, &res));
		else
			dolog(" ??");
	}
	dolog("\n");

	cpu->cr2 = cr2;
	cpu->excno = excno;
	cpu->excerr = excerr;
	nest--;
}
