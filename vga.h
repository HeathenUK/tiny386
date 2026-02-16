#ifndef VGA_H
#define VGA_H

#include <stdbool.h>

typedef struct FBDevice FBDevice;

typedef void SimpleFBDrawFunc(void *opaque,
                              int x, int y, int w, int h);

typedef struct VGAState VGAState;
VGAState *vga_init(char *vga_ram, int vga_ram_size,
                   uint8_t *fb, int width, int height);
void vga_set_force_8dm(VGAState *s, int v);

int vga_step(VGAState *vga);
void vga_refresh(VGAState *s,
                 SimpleFBDrawFunc *redraw_func, void *opaque, int full_update);

void vga_ioport_write(VGAState *s, uint32_t addr, uint32_t val);
uint32_t vga_ioport_read(VGAState *s, uint32_t addr);

/* Adaptive frame skipping (0=disabled, 1-10=max frames to skip) */
extern int vga_frame_skip_max;

/* Initialize BitScrambler for hardware-accelerated planar VGA rendering */
void vga_bitscrambler_init(void);


void vbe_write(VGAState *s, uint32_t offset, uint32_t val);
uint32_t vbe_read(VGAState *s, uint32_t offset);

void vga_mem_write(VGAState *s, uint32_t addr, uint8_t val);
uint8_t vga_mem_read(VGAState *s, uint32_t addr);
uint16_t vga_mem_read16(VGAState *s, uint32_t addr);
uint32_t vga_mem_read32(VGAState *s, uint32_t addr);
void vga_mem_write16(VGAState *s, uint32_t addr, uint16_t val);
void vga_mem_write32(VGAState *s, uint32_t addr, uint32_t val);
bool vga_mem_write_string(VGAState *s, uint32_t addr, uint8_t *buf, int len);
bool vga_mem_read_string(VGAState *s, uint32_t addr, uint8_t *buf, int len);
bool vga_mem_copy_string(VGAState *s, uint32_t dst, uint32_t src, int len);

/* Mode X inline access support - returns state for CPU fast path */
void vga_get_modex_state(VGAState *s, uint8_t **ram, uint32_t *ram_size,
                         uint8_t *write_mode, uint8_t *plane_mask,
                         uint8_t *read_plane, uint32_t **latch);

/* Direct VGA memory access for chain-4 mode (mode 13h) */
uint8_t *vga_get_direct_ptr(VGAState *s);
void vga_direct_mark_dirty(VGAState *s, uint32_t addr, int len);

/* Debug functions */
void vga_debug_direct_conditions(VGAState *s);
void vga_dump_io_trace(void);
void vga_snapshot_regs(VGAState *s, uint8_t *sr, uint8_t *gr, uint8_t *cr,
                       uint8_t *ar, uint8_t *msr);
void vga_validate_mode(VGAState *s, int mode);
void vga_validate_current(int mode);
void vga_dump_regs_summary(void);
void vga_dump_vram_to_file(VGAState *s, const char *path, int fb_height);
void vga_dump_screen_text(int max_lines);

typedef struct PCIDevice PCIDevice;
typedef struct PCIBus PCIBus;
PCIDevice *vga_pci_init(VGAState *s, PCIBus *bus,
                        void *o, void (*set_bar)(void *, int, uint32_t, bool));

#ifndef BPP
#define BPP 32
#endif

#endif /* VGA_H */
