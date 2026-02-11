# CPU Emulator Deficiency Assessment

## Context

Windows 95 boot fails with OE (Invalid Page Fault / Exception 0E) fatal
exceptions after VxD bluescreens on NetWare drivers. This assessment identifies
architectural deficiencies in `i386.c` that could cause these failures, ordered
by likely impact.

---

## CRITICAL — Most Likely Root Causes

### 1. Incorrect Double-Fault Escalation Logic (`i386.c:6640`)

**The single most probable cause of the OE fatal exceptions.**

Any `call_isr()` failure — regardless of exception class — immediately escalates
to double fault (#DF). The Intel 386 manual defines a precise contributory/benign
exception matrix:

| First Exception     | Second is Benign | Second is Contributory | Second is #PF  |
|---------------------|------------------|------------------------|----------------|
| Benign              | Sequential       | Sequential             | Sequential     |
| Contributory (#TS, #NP, #SS, #GP) | Sequential | **Double Fault** | Sequential |
| Page Fault (#PF)    | Sequential       | Sequential             | **Double Fault** |

The emulator ignores this classification entirely. The code at line 6640:

```c
if (unlikely(!call_isr(cpu, cpu->excno, pusherr, 1))) {
    // ...
    if (orig_exc == EX_DF) goto triple_fault;
    if (!call_isr(cpu, EX_DF, true, 1)) goto triple_fault;
}
```

**Impact:** When Windows 95's VMM demand-pages VxD code, a page fault during
delivery of a #GP (from a NetWare driver's IOPL-sensitive instruction in V86
mode) should be handled sequentially — service the #PF first, then retry #GP
delivery. Instead, the emulator escalates to #DF, which Windows 95's double
fault handler cannot recover from, producing the fatal OE exception.

Similarly, a benign exception (#DE, #UD) during delivery of a contributory
exception should be sequential, not #DF.

**Additionally:** Hardware interrupt delivery failure at line 6477 skips #DF
entirely and goes straight to triple fault — another deviation from the Intel
specification.

**Additionally:** The #DF error code is never explicitly zeroed (line 6690).
`cpu->excerr` retains the stale value from the failed second exception rather
than the architecturally-mandated 0.

### 2. Segment Limit Checks Completely Disabled (`i386.c:916-929`)

```c
static bool segcheck(CPUI386 *cpu, int rwm, int seg, uword addr, int size) {
    if (cpu->cr0 & 1) {
        if (cpu->seg[seg].limit == 0 && (cpu->seg[seg].sel & ~0x3) == 0) {
            THROW(seg == SEG_SS ? EX_SS : EX_GP, 0);
        }
        /* Limit checks remain disabled for compatibility... */
    }
    return true;
}
```

Only null-selector checks are performed. No offset-vs-limit validation, no
expand-down segment support, no read-only write protection.

**Impact:** Windows 95's VxD architecture relies on segment limits for memory
protection boundaries between kernel components. Without limit enforcement:
- Stack overflows silently corrupt adjacent memory instead of generating #SS
- Buffer overruns within segments go undetected instead of generating #GP
- Expand-down stack segments (used by Win95 kernel) function incorrectly
- The resulting silent memory corruption manifests later as inexplicable OE
  exceptions when corrupted pointers or data structures are used

### 3. Missing Segment Descriptor Permission Checks (`i386.c:1588`)

```c
// TODO: various permission checks
```

The `set_seg()` function performs almost no privilege validation:

**Missing checks for DS/ES/FS/GS loads:**
- No DPL >= MAX(CPL, RPL) check for data segments
- No readable-bit check for code segments loaded as data
- No code-vs-data type checking (only system-segment S-bit checked)

**Missing checks for SS loads:**
- No DPL == CPL check
- No RPL == CPL check
- No writable data segment type check
- NULL selector into SS accepted without #GP

**Missing checks for CS loads (far CALL/JMP/RET):**
- Conforming vs. non-conforming DPL checks are partially implemented in
  `pmcall`/`pmret` but incomplete

**Impact:** Ring-3 code can load ring-0 data segment selectors without faulting.
VxD drivers performing segment manipulation across privilege levels may silently
succeed where they should fault, leading to corrupted execution state.

---

## HIGH — Likely Contributing Factors

### 4. `cpu_abort()` Used Instead of Proper Exceptions

Multiple places in the emulator call `cpu_abort()` (a fatal, unrecoverable
emulator termination) for conditions that should generate architectural
exceptions:

| Location | Condition | Should Generate |
|----------|-----------|-----------------|
| `i386.c:6200` | 16-bit gate from V86 mode | #GP |
| `i386.c:6199` | Non-zero DPL target from V86 | #GP |
| `i386.c:6207` | 16-bit TSS in V86 interrupt | #GP or #TS |
| `i386.c:6399` | V86 return at CPL != 0 | #GP |
| `i386.c:5536` | 16-bit TSS in task_switch | #GP or #TS |
| `i386.c:5692-5694` | Various call gate errors | #GP or #TS |

**Impact:** If a NetWare VxD or Windows 95's VMM encounters any of these
conditions, the emulator crashes entirely instead of allowing the OS exception
handler to recover. Windows 95 installs exception handlers specifically to deal
with these situations during driver initialization.

### 5. No Write Protection for Read-Only Segments (`i386.c:926`)

```c
/* todo: readonly check */
```

The `rwm` (read/write/modify) parameter is passed through `translate()` to
`segcheck()` but is never used. Writing to a read-only data segment or a code
segment succeeds silently.

**Impact:** VxD drivers may use read-only segments for shared memory protection.
Silent writes to protected regions could corrupt shared state between VxDs.

### 6. TSS Validation Deficiencies (`i386.c:5531`)

The `task_switch()` function (marked `// XXX: incomplete`) has multiple issues:

- **No 16-bit TSS support** — encounters an `assert()` crash for type 1/3
- **No busy-bit validation** — does not check available-vs-busy state
- **No TSS limit checking** — does not verify limit >= 103 for 32-bit TSS
- **`TRY1` (fatal abort) used for segment loads** — segment faults during task
  switch should generate #TS, not crash the emulator
- **LTR does not set the busy bit** in the GDT descriptor
- **No T (debug trap) bit support** at TSS offset 100

**Impact:** Task gates in the IDT (used by some drivers for double-fault
handling) may fail catastrophically. The busy-bit mechanism for detecting
re-entrant task switches is non-functional.

### 7. SS Not Validated During Privilege Transitions

When switching privilege levels via INT, call gates, or IRET, the SS:ESP loaded
from the TSS (or popped from the stack) is never validated:

- Call gates: `i386.c:5739` — `// TODO: Check SS...`
- INT inter-PVL: `i386.c:6128` — `set_seg()` with no SS-specific checks
- IRET outer return: `i386.c:6452` — popped SS passed directly to `set_seg()`

Per Intel spec, the new SS must have RPL == new CPL, DPL == new CPL, and must be
a writable data segment. None of these are checked.

**Impact:** A corrupted TSS or stack frame with an invalid SS would be silently
accepted. The resulting invalid stack could cause cascading failures when
subsequent operations push/pop through the wrong memory.

---

## MEDIUM — Possible Contributing Factors

### 8. No CR4.VME Support (`i386.c:2812-2818`)

```c
u32 new_cr4 = lreg32(rm) & CR4_PSE;   // Only PSE preserved
```

CR4.VME (Virtual-8086 Mode Extensions) is not implemented. VME provides:
- Virtual Interrupt Flag (VIF/VIP) for efficient CLI/STI in V86 mode
- Software interrupt redirection bitmap in the TSS
- Reduced #GP overhead for IOPL-sensitive instructions

Windows 95's VMM should detect the absence of VME via CPUID and fall back to
IOPL-based V86 handling, so this is unlikely to be a direct cause. However, if
CPUID incorrectly reports VME support while CR4.VME writes are silently dropped,
the VMM would malfunction.

### 9. V86 32-bit IRET May Incorrectly Clear VM Flag (`i386.c:2902`)

```c
if (cpu->flags & VM)
    cpu->flags = (cpu->flags & IOPL) | (newflags_val & ~IOPL);
```

For V86 mode with IOPL==3, a 32-bit IRET preserves only IOPL bits (0x3000) from
the current flags. If the popped EFLAGS value has VM=0, the VM flag would be
cleared, improperly exiting V86 mode. The mask should include VM:

```c
cpu->flags = (cpu->flags & (IOPL | VM)) | (newflags_val & ~(IOPL | VM));
```

**Impact:** Could cause unexpected transitions from V86 to protected mode,
corrupting the execution environment. This scenario requires IOPL==3 and a
32-bit IRET opcode prefix in V86 code, which is uncommon but possible.

### 10. Conforming Code Segment Bug in IRET Outer Return (`i386.c:6331-6334`)

```c
if (!conforming) {
    if (dpl != rpl) THROW(EX_GP, sel & ~0x3);
} else {
    if (dpl <= cpu->cpl) THROW(EX_GP, sel & ~0x3);
}
```

For conforming code segments, the check uses `cpu->cpl` (current/inner privilege)
when it should use `rpl` (target/outer privilege). Per Intel: conforming segments
require DPL <= RPL. The emulator checks DPL <= CPL. Since RPL > CPL for outer
returns, this creates a stricter-than-correct check that could generate spurious
#GP faults.

### 11. IDT Memory Access Uses Fatal Abort on Page Fault (`i386.c:6020-6023`)

```c
TRY1(translate_laddr(cpu, &meml, 1, base + off, 4, 0));
```

`TRY1` calls `cpu_abort()` on failure. If the IDT-containing page is not
present, this should generate a page fault (serviceable by the page fault
handler), not crash the emulator. Under memory pressure, Windows 95 could
theoretically page out IDT-adjacent memory.

### 12. IOPB Boundary Check Off-By-One (`i386.c:4028`)

```c
if (iobase + port / 8 < cpu->seg[SEG_TR].limit) {
```

The check reads 16 bits (`load16`) from `iobase + port/8`, but the boundary
check doesn't account for the 2-byte read width. If the last IOPB byte is at
the TSS limit boundary, the second byte could be read from beyond the TSS.

### 13. Accessed Bit Never Set in Segment Descriptors

When loading a segment descriptor, the 386 should set the Accessed bit (bit 0 of
the type field) in the GDT/LDT entry. The emulator never writes back the
descriptor after setting this bit.

**Impact:** Software that checks the Accessed bit (uncommon but architecturally
defined) would see incorrect values.

### 14. No CR0 Invalid Combination Check

Setting CR0.PG=1 with CR0.PE=0 is illegal and should generate #GP. The emulator
silently accepts this combination. Similarly, LMSW does not flush the TLB when PE
transitions.

---

## Summary: Most Likely Failure Chain for the Windows 95 VxD Issue

1. A NetWare VxD driver executes during Windows 95 boot
2. The driver (or the VMM on its behalf) performs a privileged operation that
   generates #GP — for example, a V86 mode I/O or interrupt instruction
3. During delivery of the #GP, the CPU needs to access the IDT handler code,
   which resides in a demand-paged region
4. The page fault (#PF) during #GP delivery should be handled sequentially:
   service the #PF, then retry #GP delivery
5. **Instead, the emulator incorrectly escalates to #DF** (deficiency #1)
6. The #DF handler itself may also encounter a page fault (its code may also
   be paged out), leading to triple fault
7. This manifests as the OE fatal exception bluescreen

The missing segment limit checks (#2) and permission checks (#3) may additionally
allow corrupted state to accumulate before the fatal #DF escalation occurs,
making the symptom appear to be related to NetWare drivers when the root cause is
in the CPU emulation layer.
