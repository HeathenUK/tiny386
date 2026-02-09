#ifndef I386_H
#define I386_H

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef u32 uword;
typedef s32 sword;

typedef struct CPUI386 CPUI386;

typedef struct {
	void *pic;
	int (*pic_read_irq)(void *);

	void *io;
	u8 (*io_read8)(void *, int);
	void (*io_write8)(void *, int, u8);
	u16 (*io_read16)(void *, int);
	void (*io_write16)(void *, int, u16);
	u32 (*io_read32)(void *, int);
	void (*io_write32)(void *, int, u32);
	int (*io_read_string)(void *, int, uint8_t *, int, int);
	int (*io_write_string)(void *, int, uint8_t *, int, int);

	void *iomem;
	u8 (*iomem_read8)(void *, uword);
	void (*iomem_write8)(void *, uword, u8);
	u16 (*iomem_read16)(void *, uword);
	void (*iomem_write16)(void *, uword, u16);
	u32 (*iomem_read32)(void *, uword);
	void (*iomem_write32)(void *, uword, u32);
	bool (*iomem_write_string)(void *, uword, uint8_t *, int);
	bool (*iomem_read_string)(void *, uword, uint8_t *, int);
	/* VGA-to-VGA copy for Mode X latch operations (REP MOVSB scrolling) */
	bool (*iomem_copy_string)(void *, uword dst, uword src, int len);

	/* Direct VGA memory access for chain-4 mode (mode 13h)
	 * When non-NULL, VGA reads/writes at 0xA0000-0xAFFFF bypass callbacks */
	u8 *vga_direct;
	void (*vga_direct_write_notify)(void *iomem, uword addr, int len);

	/* Mode X fast path state (updated by VGA on register changes)
	 * When non-NULL, CPU can inline Mode X read/write operations */
	u8 *vga_modex_ram;           /* VGA RAM pointer for Mode X access */
	u32 vga_modex_ram_size;      /* VGA RAM size in bytes */
	u8 vga_modex_write_mode;     /* 0=fast wm0, 1=fast wm1, 0xFF=use callback */
	u8 vga_modex_plane_mask;     /* Plane write mask (SR[2]) */
	u8 vga_modex_read_plane;     /* Read plane (GR[4]) */
	u32 *vga_modex_latch;        /* Pointer to latch register */
} CPU_CB;

CPUI386 *cpui386_new(int gen, char *phys_mem, long phys_mem_size, CPU_CB **cb);
void cpui386_delete(CPUI386 *cpu);
void cpui386_enable_fpu(CPUI386 *cpu);
void cpui386_reset(CPUI386 *cpu);
void cpui386_reset_pm(CPUI386 *cpu, uint32_t start_addr);
void cpui386_step(CPUI386 *cpu, int stepcount);
void cpui386_raise_irq(CPUI386 *cpu);
void cpui386_set_gpr(CPUI386 *cpu, int i, u32 val);
long cpui386_get_cycle(CPUI386 *cpu);
void cpui386_set_a20(CPUI386 *cpu, int enabled);
int cpui386_get_a20(CPUI386 *cpu);

bool cpu_load8(CPUI386 *cpu, int seg, uword addr, u8 *res);
bool cpu_store8(CPUI386 *cpu, int seg, uword addr, u8 val);
bool cpu_load16(CPUI386 *cpu, int seg, uword addr, u16 *res);
bool cpu_store16(CPUI386 *cpu, int seg, uword addr, u16 val);
bool cpu_load32(CPUI386 *cpu, int seg, uword addr, u32 *res);
bool cpu_store32(CPUI386 *cpu, int seg, uword addr, u32 val);
bool cpu_load64(CPUI386 *cpu, int seg, uword addr, uint64_t *res);
bool cpu_store64(CPUI386 *cpu, int seg, uword addr, uint64_t val);
void cpu_setax(CPUI386 *cpu, u16 ax);
u16 cpu_getax(CPUI386 *cpu);
void cpu_setexc(CPUI386 *cpu, int excno, uword excerr);
void cpu_setflags(CPUI386 *cpu, uword set_mask, uword clear_mask);
uword cpu_getflags(CPUI386 *cpu);
void cpu_abort(CPUI386 *cpu, int code);

#endif /* I386_H */
