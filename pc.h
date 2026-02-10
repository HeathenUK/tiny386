#ifndef PC_H
#define PC_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "i386.h"
#include "i8259.h"
#include "i8254.h"
#include "ide.h"
#include "vga.h"
#include "i8042.h"
#include "misc.h"
#include "adlib.h"
#include "ne2000.h"
#include "i8257.h"
#include "sb16.h"
#include "gus.h"
#include "pcspk.h"
#include "pci.h"
#include "ini.h"

/// Platform HAL
uint32_t get_uticks();
void *pcmalloc(long size);
void *bigmalloc(size_t size);
int load_rom(void *phys_mem, const char *file, uword addr, int backward);

/// PC
typedef CPUI386 CPU;

typedef struct {
	CPU *cpu;
	CPU_CB *cpu_cb;  /* CPU callbacks, used for VGA direct mode updates */
	PicState2 *pic;
	PITState *pit;
	U8250 *serial;
	CMOS *cmos;
	IDEIFState *ide, *ide2;
	VGAState *vga;
	char *phys_mem;
	long phys_mem_size;
	char *vga_mem;
	int vga_mem_size;
	int64_t boot_start_time;

	// CPU config (stored for OSD access)
	int cpu_gen;  // 3=386, 4=486, 5=586
	int fpu;      // 0=disabled, 1=enabled

	SimpleFBDrawFunc *redraw;
	void *redraw_data;
	void (*poll)(void *);

	KBDState *i8042;
	PS2KbdState *kbd;
	PS2MouseState *mouse;
	AdlibState *adlib;
	NE2000State *ne2000;
	I8257State *isa_dma, *isa_hdma;
	SB16State *sb16;
	GUSState *gus;
	PCSpkState *pcspk;

	I440FXState *i440fx;
	PCIBus *pcibus;
	PCIDevice *pci_ide;
	PCIDevice *pci_vga;
	uword pci_vga_ram_addr;

	EMULINK *emulink;

	const char *bios;
	const char *vga_bios;
	const char *hda_path;  // HDA path for OSD (stored for runtime access)

	u8 port92;
	int fill_cmos;
	int shutdown_state;
	int reset_request;

	const char *linuxstart;
	const char *kernel;
	const char *initrd;
	const char *cmdline;
	int enable_serial;
	int full_update;

	const char *ini_path;  // Path to ini file for saving settings
} PC;

typedef struct {
	const char *linuxstart;
	const char *kernel;
	const char *initrd;
	const char *cmdline;
	const char *bios;
	const char *vga_bios;
	long mem_size;
	long vga_mem_size;
	const char *disks[4];
	int iscd[4];
	const char *fdd[2];
	int fill_cmos;
	int width;
	int height;
	int cpu_gen;
	int fpu;
	int enable_serial;
	int vga_force_8dm;
	int boot_order;  // 0=HDD, 1=Floppy, 2=CD (see misc.h BOOT_ORDER_*)
	int frame_skip;  // 0=disabled, 1-10=max frames to skip (adaptive)
	int batch_size;  // 0=auto, or fixed value 512-4096 (ESP32 only)
	int brightness;  // 0-100 display brightness (ESP32 only)
	int volume;      // 0-100 audio volume (ESP32 only)
	int mouse_speed; // 1-10 mouse emulation speed (ESP32 only)
	int usb_passthru; // 1=enabled, 0=disabled - pass USB storage to emulator (ESP32 only)
	int sound_device; // 0=SB16+AdLib (default), 1=GUS
	int hda_heads;    // 0=auto-detect, or override CHS heads for hda
	int hda_spt;      // 0=auto-detect, or override CHS sectors-per-track for hda
	const char *ini_path;  // Path to ini file for saving settings
} PCConfig;

PC *pc_new(SimpleFBDrawFunc *redraw, void (*poll)(void *), void *redraw_data,
	   u8 *fb, PCConfig *conf);
void pc_reset(PC *pc);

// XXX: still contains ESP32-specific logic
void pc_vga_step(void *o);
void pc_step(PC *pc);

void mixer_callback(void *opaque, uint8_t *stream, int free);

// Batch size setting (ESP32 only): 0=auto, or fixed value 512-4096
extern int pc_batch_size_setting;

int parse_conf_ini(void* user, const char* section,
		   const char* name, const char* value);
void load_bios_and_reset(PC *pc);

#endif /* PC_H */
