# CPU Compliance Implementation Plan
## Prioritised for Windows 95 Boot and Runtime

Each item is an atomic, individually testable change. Priority reflects
direct impact on Win95 booting and running correctly.

---

### Phase 1 — Exception Handling (likely fixes boot failures)

**1. Double-fault escalation matrix**
- Replace the current "any exception during exception delivery → #DF" logic
  with the correct 386 contributory/benign/page-fault classification table.
- Track the *first* exception class across `call_isr()` failure so the second
  exception can be compared against it.
- Classification: benign (#DE,#DB,#BP,#OF,#BR,#MF), contributory (#UD,#NM,#TS,
  #NP,#SS,#GP), and #PF as its own class.
- Same-class contributory+contributory → #DF. #PF+#PF → #DF.
  #PF+contributory or contributory+#PF → #DF. Everything else → service the
  second exception normally.
- Location: `cpui386_step()` around line 6640.
- Risk: Low. Makes the emulator *less* likely to escalate; doesn't touch fast
  paths.

**2. #DF must push error code 0**
- `call_isr(EX_DF)` must push a 32-bit zero error code.
- Currently the error code from the second exception may leak through.
- Trivial one-line fix alongside item 1.

**3. Hardware IRQ delivery failure → escalate via #DF, not abort**
- When `call_isr()` fails for a hardware interrupt, the current code aborts the
  emulator. Instead: attempt #DF delivery, and only triple-fault (reset or
  abort) if #DF delivery itself fails.
- Location: the `!cpu_exec1` / hardware IRQ path in `cpui386_step()`.

---

### Phase 2 — V86 Mode Correctness (affects DOS boxes under Win95)

**4. PUSHF in V86 mode with IOPL < 3 must #GP**
- Add the missing check: `if ((cpu->flags & VM) && get_IOPL(cpu) < 3) THROW(EX_GP, 0);`
  at the top of the `PUSHF()` macro (matching the existing POPF check).
- Without this, DOS programs in Win95 DOS boxes see raw EFLAGS instead of
  trapping to the VMM for IF virtualisation.
- Risk: Trivial. One branch in an uncommon instruction handler.

---

### Phase 3 — Robustness (stops emulator crashes on guest edge cases)

**5. Convert `cpu_abort()` calls to proper exceptions**
- Audit each `cpu_abort()` call site and replace with the architecturally
  correct `THROW()`.
- Key sites:
  - `task_switch()` line 5536: assert on 16-bit TSS → `THROW(EX_TS, tss)`
  - Any abort in descriptor/gate loading → `THROW(EX_GP, selector)`
- Must ensure all validation checks occur *before* state mutations to avoid
  half-completed transitions.
- Risk: Medium — each site needs individual analysis of rollback safety.

**6. Single-step: TF → #DB after each instruction**
- After the instruction executes in `cpu_exec1()`, check
  `if (unlikely(cpu->flags & TF))` and generate `#DB` (vector 1).
- Must save TF state *before* the instruction (POPF/IRET can change TF;
  the #DB fires after the *next* instruction, not the one that set TF).
- Some Win95 VxDs use TF for instruction tracing during initialisation.
- Risk: Low — one `unlikely` branch per instruction, same cost class as the
  existing interrupt/stepcount checks.

---

### Phase 4 — Protected Mode Correctness (high risk, add diagnostic mode first)

**7. Segment limit checks (non-flat segments only)**
- In `segcheck()`, when the segment is NOT flat, verify
  `addr + size - 1 <= limit` (normal) or the inverted check for expand-down.
- Flat segments (base=0, limit=0xFFFFFFFF) must be an unconditional pass-through
  — no arithmetic, no branch.
- Add expand-down support: for data segments with the E bit set, valid range
  is `(limit+1)` to `0xFFFF` (16-bit) or `0xFFFFFFFF` (32-bit).
- **Implement diagnostic-log-only mode first**: log violations without faulting
  and run a full Win95 boot to catalogue what would break before enforcing.
- Risk: High — previous attempts broke emulation. Off-by-one in limit
  comparison or wrong expand-down logic will fault on every memory access.

**8. `set_seg()` DPL/RPL/CPL permission checks**
- When loading DS/ES/FS/GS: data segment or conforming code segment must have
  `DPL >= max(CPL, RPL)`.
- When loading SS: must have `DPL == CPL` and `RPL == CPL`.
- CS checks are different (conforming vs non-conforming, call gates, etc.) —
  defer to existing far-call/far-jmp/iret paths.
- **Implement diagnostic-log-only mode first** (same rationale as item 7).
- Critical ordering concern: CPL is updated when CS is loaded. Every call site
  that loads CS + other segments must load CS first, or the permission checks
  fire at the wrong privilege level.
- Risk: High — Win95 boot performs segment loads in unusual orders during mode
  transitions.

---

### Phase 5 — Completeness (lower Win95 impact, still guest-visible)

**9. 16-bit TSS support in `task_switch()`**
- Add a code path for TSS types 1 and 3 (16-bit TSS) using 16-bit offsets.
- Win95 uses 32-bit TSS exclusively, but this removes an `assert()`/abort
  that could be hit by DOS extenders or unusual guest code.
- Risk: Low — new code path, doesn't change existing 32-bit logic.

**10. CR0.NE — FPU error delivery mode**
- When NE=0, FPU errors should be routed via IRQ 13 (external error protocol)
  instead of exception #MF.
- Requires wiring the FPU error signal to the PIC.
- Affects DOS programs with x87 error handlers.
- Risk: Low-Medium — isolated to FPU error delivery path.

**11. Page-boundary CR2 value**
- When a write straddles two pages and the second page faults, pass the actual
  faulting linear address to CR2 instead of the page-aligned address.
- Risk: Trivial.

**12. LOCK prefix validation**
- When the LOCK prefix is present, verify the opcode is on the permitted list
  (ADD, ADC, AND, BTC, BTR, BTS, CMPXCHG, DEC, INC, NEG, NOT, OR, SBB, SUB,
  XADD, XCHG with memory destination). Generate #UD otherwise.
- Risk: Trivial.

**13. CR4.TSD — RDTSC privilege restriction**
- When CR4.TSD=1, RDTSC at CPL>0 should generate #GP(0).
- Risk: Trivial.

**14. Expand RDMSR/WRMSR**
- Add MSR 0x10 (TSC read/write).
- Risk: Trivial.

---

### Deferred — Explicit Decision

**Debug registers (DR0-DR7 hardware breakpoint matching)**
- Deliberately not implemented. Per-memory-access comparison cost is
  incompatible with ESP32-P4 performance budget. See comment at `dr[8]`
  declaration in `i386.c`. DR storage and MOV DR instructions work for
  guest context-switch compatibility.

**CR4 VME/PVI**
- Complex V86 interrupt virtualisation. Win95 falls back to the non-VME
  path gracefully. Pure performance optimisation, not a correctness issue.
  Deferred until profiling shows V86 interrupt overhead is a bottleneck.
