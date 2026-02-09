#include "pc.h"
#include "i8042.h"
#include "sb16.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include "common.h"

// Batch size setting: 0=auto (dynamic), or fixed value 512-4096
int pc_batch_size_setting = 0;

#define cpu_raise_irq cpui386_raise_irq
#define cpu_get_cycle cpui386_get_cycle

static u8 pc_io_read(void *o, int addr)
{
	PC *pc = o;
	u8 val;

	switch(addr) {
	case 0x20: case 0x21: case 0xa0: case 0xa1:
		val = i8259_ioport_read(pc->pic, addr);
		return val;
	case 0x3f8: case 0x3f9: case 0x3fa: case 0x3fb:
	case 0x3fc: case 0x3fd: case 0x3fe: case 0x3ff:
		val = 0xff;
		if (pc->enable_serial)
			val = u8250_reg_read(pc->serial, addr - 0x3f8);
		return val;
	case 0x2f8: case 0x2f9: case 0x2fa: case 0x2fb:
	case 0x2fc: case 0x2fd: case 0x2fe: case 0x2ff:
	case 0x2e8: case 0x2e9: case 0x2ea: case 0x2eb:
	case 0x2ec: case 0x2ed: case 0x2ee: case 0x2ef:
	case 0x3e8: case 0x3e9: case 0x3ea: case 0x3eb:
	case 0x3ec: case 0x3ed: case 0x3ee: case 0x3ef:
		return 0;
	case 0x42:
		/* read delay for PIT channel 2 */
		/* certain guest code needs it to drive pc speaker properly */
		usleep(0);
		/* fall through */
	case 0x40: case 0x41: case 0x43:
		val = i8254_ioport_read(pc->pit, addr);
		return val;
	case 0x70: case 0x71:
		val = cmos_ioport_read(pc->cmos, addr);
		return val;
	case 0x1f0: case 0x1f1: case 0x1f2: case 0x1f3:
	case 0x1f4: case 0x1f5: case 0x1f6: case 0x1f7:
		val = ide_ioport_read(pc->ide, addr - 0x1f0);
		return val;
	case 0x170: case 0x171: case 0x172: case 0x173:
	case 0x174: case 0x175: case 0x176: case 0x177:
		val = ide_ioport_read(pc->ide2, addr - 0x170);
		return val;
	case 0x3f6:
		val = ide_status_read(pc->ide);
		return val;
	case 0x376:
		val = ide_status_read(pc->ide2);
		return val;
	case 0x3c0: case 0x3c1: case 0x3c2: case 0x3c3:
	case 0x3c4: case 0x3c5: case 0x3c6: case 0x3c7:
	case 0x3c8: case 0x3c9: case 0x3ca: case 0x3cb:
	case 0x3cc: case 0x3cd: case 0x3ce: case 0x3cf:
	case 0x3d0: case 0x3d1: case 0x3d2: case 0x3d3:
	case 0x3d4: case 0x3d5: case 0x3d6: case 0x3d7:
	case 0x3d8: case 0x3d9: case 0x3da: case 0x3db:
	case 0x3dc: case 0x3dd: case 0x3de: case 0x3df:
		val = vga_ioport_read(pc->vga, addr);
		return val;
	case 0x92:
		return pc->port92;
	case 0x60:
		val = kbd_read_data(pc->i8042, addr);
		return val;
	case 0x64:
		val = kbd_read_status(pc->i8042, addr);
		return val;
	case 0x61:
		val = pcspk_ioport_read(pc->pcspk);
		return val;
	case 0x220: case 0x221: case 0x222: case 0x223:
	case 0x228: case 0x229:
	case 0x388: case 0x389: case 0x38a: case 0x38b:
		return adlib_read(pc->adlib, addr);
	case 0xcfc: case 0xcfd: case 0xcfe: case 0xcff:
		val = i440fx_read_data(pc->i440fx, addr - 0xcfc, 0);
		return val;
	case 0x300: case 0x301: case 0x302: case 0x303:
	case 0x304: case 0x305: case 0x306: case 0x307:
	case 0x308: case 0x309: case 0x30a: case 0x30b:
	case 0x30c: case 0x30d: case 0x30e: case 0x30f:
		val = ne2000_ioport_read(pc->ne2000, addr);
		return val;
	case 0x310:
		val = ne2000_asic_ioport_read(pc->ne2000, addr);
		return val;
	case 0x31f:
		val = ne2000_reset_ioport_read(pc->ne2000, addr);
		return val;
	case 0x00: case 0x01: case 0x02: case 0x03:
	case 0x04: case 0x05: case 0x06: case 0x07:
		val = i8257_read_chan(pc->isa_dma, addr - 0x00, 1);
		return val;
	case 0x08: case 0x09: case 0x0a: case 0x0b:
	case 0x0c: case 0x0d: case 0x0e: case 0x0f:
		val = i8257_read_cont(pc->isa_dma, addr - 0x08, 1);
		return val;
	case 0x81: case 0x82: case 0x83: case 0x87:
		val = i8257_read_page(pc->isa_dma, addr - 0x80);
		return val;
	case 0x481: case 0x482: case 0x483: case 0x487:
		val = i8257_read_pageh(pc->isa_dma, addr - 0x480);
		return val;
	case 0xc0: case 0xc2: case 0xc4: case 0xc6:
	case 0xc8: case 0xca: case 0xcc: case 0xce:
		val = i8257_read_chan(pc->isa_hdma, addr - 0xc0, 1);
		return val;
	case 0xd0: case 0xd2: case 0xd4: case 0xd6:
	case 0xd8: case 0xda: case 0xdc: case 0xde:
		val = i8257_read_cont(pc->isa_hdma, addr - 0xd0, 1);
		return val;
	case 0x89: case 0x8a: case 0x8b: case 0x8f:
		val = i8257_read_page(pc->isa_hdma, addr - 0x88);
		return val;
	case 0x489: case 0x48a: case 0x48b: case 0x48f:
		val = i8257_read_pageh(pc->isa_hdma, addr - 0x488);
		return val;
	case 0x225:
		val = sb16_mixer_read(pc->sb16, addr);
		return val;
	case 0x226: case 0x22a: case 0x22c: case 0x22d: case 0x22e: case 0x22f:
		val = sb16_dsp_read(pc->sb16, addr);
		return val;
	case 0xf1f4:
		val = 0;
		emulink_data_read_string(pc->emulink, &val, 1, 1);
		return val;
	default:
		//fprintf(stderr, "in 0x%x <= 0x%x\n", addr, 0xff);
		return 0xff;
	}
}

static u16 pc_io_read16(void *o, int addr)
{
	PC *pc = o;
	u16 val;

	switch(addr) {
	case 0x1ce: case 0x1cf:
		val = vbe_read(pc->vga, addr - 0x1ce);
		return val;
	case 0x1f0:
		val = ide_data_readw(pc->ide);
		return val;
	case 0x170:
		val = ide_data_readw(pc->ide2);
		return val;
	case 0xcf8:
		val = i440fx_read_addr(pc->i440fx, 0, 1);
		return val;
	case 0xcfc: case 0xcfe:
		val = i440fx_read_data(pc->i440fx, addr - 0xcfc, 1);
		return val;
	case 0x310:
		val = ne2000_asic_ioport_read(pc->ne2000, addr);
		return val;
	case 0x220:
		return adlib_read(pc->adlib, addr);
	default:
#ifdef DEBUG_IO
		fprintf(stderr, "inw 0x%x <= 0x%x\n", addr, 0xffff);
#endif
		return 0xffff;
	}
}

static u32 pc_io_read32(void *o, int addr)
{
	PC *pc = o;
	u32 val;
	switch(addr) {
	case 0x1f0:
		val = ide_data_readl(pc->ide);
		return val;
	case 0x170:
		val = ide_data_readl(pc->ide2);
		return val;
	case 0x3cc:
		return (get_uticks() - pc->boot_start_time) / 1000;
	case 0xcf8:
		val = i440fx_read_addr(pc->i440fx, 0, 2);
		return val;
	case 0xcfc:
		val = i440fx_read_data(pc->i440fx, 0, 2);
		return val;
	case 0xf1f0:
		val = emulink_status_read(pc->emulink);
		return val;
	default:
		fprintf(stderr, "ind 0x%x <= 0x%x\n", addr, 0xffffffff);
	}
	return 0xffffffff;
}

static int pc_io_read_string(void *o, int addr, uint8_t *buf, int size, int count)
{
	PC *pc = o;
	switch(addr) {
	case 0x1f0:
		return ide_data_read_string(pc->ide, buf, size, count);
	case 0x170:
		return ide_data_read_string(pc->ide2, buf, size, count);
	case 0xf1f4:
		return emulink_data_read_string(pc->emulink, buf, size, count);
	}
	return 0;
}

static void pc_io_write(void *o, int addr, u8 val)
{
	PC *pc = o;
	switch(addr) {
	case 0x80: case 0xed:
		/* used by linux, for io delay */
		return;
	case 0x20: case 0x21: case 0xa0: case 0xa1:
		i8259_ioport_write(pc->pic, addr, val);
		return;
	case 0x3f8: case 0x3f9: case 0x3fa: case 0x3fb:
	case 0x3fc: case 0x3fd: case 0x3fe: case 0x3ff:
		u8250_reg_write(pc->serial, addr - 0x3f8, val);
		return;
	case 0x2f8: case 0x2f9: case 0x2fa: case 0x2fb:
	case 0x2fc: case 0x2fd: case 0x2fe: case 0x2ff:
	case 0x2e8: case 0x2e9: case 0x2ea: case 0x2eb:
	case 0x2ec: case 0x2ed: case 0x2ee: case 0x2ef:
	case 0x3e8: case 0x3e9: case 0x3ea: case 0x3eb:
	case 0x3ec: case 0x3ed: case 0x3ee: case 0x3ef:
		return;
	case 0x40: case 0x41: case 0x42: case 0x43:
		i8254_ioport_write(pc->pit, addr, val);
		return;
	case 0x70: case 0x71:
		cmos_ioport_write(pc->cmos, addr, val);
		return;
	case 0x1f0: case 0x1f1: case 0x1f2: case 0x1f3:
	case 0x1f4: case 0x1f5: case 0x1f6: case 0x1f7:
		ide_ioport_write(pc->ide, addr - 0x1f0, val);
		return;
	case 0x170: case 0x171: case 0x172: case 0x173:
	case 0x174: case 0x175: case 0x176: case 0x177:
		ide_ioport_write(pc->ide2, addr - 0x170, val);
		return;
	case 0x3f6:
		ide_cmd_write(pc->ide, val);
		return;
	case 0x376:
		ide_cmd_write(pc->ide2, val);
		return;
	case 0x3c0: case 0x3c1: case 0x3c2: case 0x3c3:
	case 0x3c4: case 0x3c5: case 0x3c6: case 0x3c7:
	case 0x3c8: case 0x3c9: case 0x3ca: case 0x3cb:
	case 0x3cc: case 0x3cd: case 0x3ce: case 0x3cf:
	case 0x3d0: case 0x3d1: case 0x3d2: case 0x3d3:
	case 0x3d4: case 0x3d5: case 0x3d6: case 0x3d7:
	case 0x3d8: case 0x3d9: case 0x3da: case 0x3db:
	case 0x3dc: case 0x3dd: case 0x3de: case 0x3df:
		vga_ioport_write(pc->vga, addr, val);
		return;
	case 0x402:
		putchar(val);
		fflush(stdout);
		return;
	case 0x92:
		pc->port92 = val;
		cpui386_set_a20(pc->cpu, (val >> 1) & 1);
		return;
	case 0x60:
		kbd_write_data(pc->i8042, addr, val);
		return;
	case 0x64:
		kbd_write_command(pc->i8042, addr, val);
		return;
	case 0x61:
		pcspk_ioport_write(pc->pcspk, val);
		return;
	case 0x220: case 0x221: case 0x222: case 0x223:
	case 0x228: case 0x229:
	case 0x388: case 0x389: case 0x38a: case 0x38b:
		adlib_write(pc->adlib, addr, val);
		return;
	case 0x8900:
		switch (val) {
		case 'S': if (pc->shutdown_state == 0) pc->shutdown_state = 1; break;
		case 'h': if (pc->shutdown_state == 1) pc->shutdown_state = 2; break;
		case 'u': if (pc->shutdown_state == 2) pc->shutdown_state = 3; break;
		case 't': if (pc->shutdown_state == 3) pc->shutdown_state = 4; break;
		case 'd': if (pc->shutdown_state == 4) pc->shutdown_state = 5; break;
		case 'o': if (pc->shutdown_state == 5) pc->shutdown_state = 6; break;
		case 'w': if (pc->shutdown_state == 6) pc->shutdown_state = 7; break;
		case 'n': if (pc->shutdown_state == 7) pc->shutdown_state = 8; break;
		default : pc->shutdown_state = 0; break;
		}
		return;
	case 0xcfc: case 0xcfd: case 0xcfe: case 0xcff:
		i440fx_write_data(pc->i440fx, addr - 0xcfc, val, 0);
		return;
	case 0x300: case 0x301: case 0x302: case 0x303:
	case 0x304: case 0x305: case 0x306: case 0x307:
	case 0x308: case 0x309: case 0x30a: case 0x30b:
	case 0x30c: case 0x30d: case 0x30e: case 0x30f:
		ne2000_ioport_write(pc->ne2000, addr, val);
		return;
	case 0x310:
		ne2000_asic_ioport_write(pc->ne2000, addr, val);
		return;
	case 0x31f:
		ne2000_reset_ioport_write(pc->ne2000, addr, val);
		return;
	case 0x00: case 0x01: case 0x02: case 0x03:
	case 0x04: case 0x05: case 0x06: case 0x07:
		i8257_write_chan(pc->isa_dma, addr - 0x00, val, 1);
		return;
	case 0x08: case 0x09: case 0x0a: case 0x0b:
	case 0x0c: case 0x0d: case 0x0e: case 0x0f:
		i8257_write_cont(pc->isa_dma, addr - 0x08, val, 1);
		return;
	case 0x81: case 0x82: case 0x83: case 0x87:
		i8257_write_page(pc->isa_dma, addr - 0x80, val);
		return;
	case 0x481: case 0x482: case 0x483: case 0x487:
		i8257_write_pageh(pc->isa_dma, addr - 0x480, val);
		return;
	case 0xc0: case 0xc2: case 0xc4: case 0xc6:
	case 0xc8: case 0xca: case 0xcc: case 0xce:
		i8257_write_chan(pc->isa_hdma, addr - 0xc0, val, 1);
		return;
	case 0xd0: case 0xd2: case 0xd4: case 0xd6:
	case 0xd8: case 0xda: case 0xdc: case 0xde:
		i8257_write_cont(pc->isa_hdma, addr - 0xd0, val, 1);
		return;
	case 0x89: case 0x8a: case 0x8b: case 0x8f:
		i8257_write_page(pc->isa_hdma, addr - 0x88, val);
		return;
	case 0x489: case 0x48a: case 0x48b: case 0x48f:
		i8257_write_pageh(pc->isa_hdma, addr - 0x488, val);
		return;
	case 0x224:
		sb16_mixer_write_indexb(pc->sb16, addr, val);
		return;
	case 0x225:
		sb16_mixer_write_datab(pc->sb16, addr, val);
		return;
	case 0x226: case 0x22c:
		sb16_dsp_write(pc->sb16, addr, val);
		return;
	case 0xf1f4:
		emulink_data_write_string(pc->emulink, &val, 1, 1);
		return;
	default:
		// fprintf(stderr, "out 0x%x => 0x%x\n", val, addr);
		return;
	}
}

static void pc_io_write16(void *o, int addr, u16 val)
{
	PC *pc = o;
	switch(addr) {
	case 0x1f0:
		ide_data_writew(pc->ide, val);
		return;
	case 0x170:
		ide_data_writew(pc->ide2, val);
		return;
	case 0x3c0: case 0x3c1: case 0x3c2: case 0x3c3:
	case 0x3c4: case 0x3c5: case 0x3c6: case 0x3c7:
	case 0x3c8: case 0x3c9: case 0x3ca: case 0x3cb:
	case 0x3cc: case 0x3cd: case 0x3ce: case 0x3cf:
	case 0x3d0: case 0x3d1: case 0x3d2: case 0x3d3:
	case 0x3d4: case 0x3d5: case 0x3d6: case 0x3d7:
	case 0x3d8: case 0x3d9: case 0x3da: case 0x3db:
	case 0x3dc: case 0x3dd: case 0x3de:
		vga_ioport_write(pc->vga, addr, val & 0xff);
		vga_ioport_write(pc->vga, addr + 1, (val >> 8) & 0xff);
		return;
	case 0x1ce: case 0x1cf:
		vbe_write(pc->vga, addr - 0x1ce, val);
		return;
	case 0xcfc: case 0xcfe:
		i440fx_write_data(pc->i440fx, addr - 0xcfc, val, 1);
		return;
	case 0x310:
		ne2000_asic_ioport_write(pc->ne2000, addr, val);
		return;
	default:
#ifdef DEBUG_IO
		fprintf(stderr, "outw 0x%x => 0x%x\n", val, addr);
#endif
		return;
	}
}

static void pc_io_write32(void *o, int addr, u32 val)
{
	PC *pc = o;
	switch(addr) {
	case 0x1f0:
		ide_data_writel(pc->ide, val);
		return;
	case 0x170:
		ide_data_writel(pc->ide2, val);
		return;
	case 0xcf8:
		i440fx_write_addr(pc->i440fx, 0, val, 2);
		return;
	case 0xcfc:
		i440fx_write_data(pc->i440fx, 0, val, 2);
		return;
	case 0xf1f0:
		emulink_cmd_write(pc->emulink, val);
		return;
	case 0xf1f4:
		emulink_data_write(pc->emulink, val);
		return;
	default:
		// fprintf(stderr, "outd 0x%x => 0x%x\n", val, addr);
		return;
	}
}

static int pc_io_write_string(void *o, int addr, uint8_t *buf, int size, int count)
{
	PC *pc = o;
	switch(addr) {
	case 0x1f0:
		return ide_data_write_string(pc->ide, buf, size, count);
	case 0x170:
		return ide_data_write_string(pc->ide2, buf, size, count);
	case 0xf1f4:
		return emulink_data_write_string(pc->emulink, buf, size, count);
	}
	return 0;
}

void pc_vga_step(void *o)
{
	PC *pc = o;
	int refresh = vga_step(pc->vga);
	int full_update = 0;
	/* Check if a full redraw was requested (e.g., OSD closed) */
	if (globals.vga_force_redraw) {
		globals.vga_force_redraw = false;
		full_update = 1;
		refresh = 1;  /* Force refresh even if VGA unchanged */
	}
	if (refresh) {
		vga_refresh(pc->vga, pc->redraw, pc->redraw_data, full_update);
	}
}

void pc_step(PC *pc)
{
	/* Batch sizing: fixed if pc_batch_size_setting > 0, else dynamic.
	 * Dynamic mode adjusts instruction count based on step time,
	 * scaling up when fast and down when slow, with hysteresis. */
	static int batch_size = 0;             /* Current batch: 512-4096 */
	static uint32_t step_time_accum = 0;   /* Accumulated step time (dynamic mode) */
	static uint32_t step_count = 0;        /* Steps since last evaluation */

	/* Initialize batch_size on first call or when setting changes */
	if (batch_size == 0 || (pc_batch_size_setting > 0 && batch_size != pc_batch_size_setting)) {
		batch_size = (pc_batch_size_setting > 0) ? pc_batch_size_setting : 512;
		step_time_accum = 0;
		step_count = 0;
	}

	if (pc->reset_request) {
		pc->reset_request = 0;
		/* Reset batch size on reboot - hardware detection is timing-sensitive */
		if (pc_batch_size_setting == 0) {
			batch_size = 512;  /* Only reset if in dynamic mode */
		}
		step_time_accum = 0;
		step_count = 0;
		load_bios_and_reset(pc);
	}

	/* Update VGA direct access pointer (detects mode changes) */
	{
		static u8 *last_direct = (u8*)1;  /* Invalid initial value to force first report */
		u8 *new_direct = vga_get_direct_ptr(pc->vga);
		if (new_direct != last_direct) {
			if (new_direct) {
				fprintf(stderr, "VGA direct mode ENABLED at %p\n", (void*)new_direct);
			} else {
				/* Debug: report why disabled */
				vga_debug_direct_conditions(pc->vga);
			}
			last_direct = new_direct;
		}
		pc->cpu_cb->vga_direct = new_direct;

		/* Update Mode X inline fast path state */
		vga_get_modex_state(pc->vga, &pc->cpu_cb->vga_modex_ram,
		                    &pc->cpu_cb->vga_modex_ram_size,
		                    &pc->cpu_cb->vga_modex_write_mode,
		                    &pc->cpu_cb->vga_modex_plane_mask,
		                    &pc->cpu_cb->vga_modex_read_plane,
		                    &pc->cpu_cb->vga_modex_latch);
	}

	/* Check if VGA mode changed - reset batch for new program (dynamic mode only) */
	if (globals.batch_reset_pending) {
		globals.batch_reset_pending = false;
		if (pc_batch_size_setting == 0) {
			batch_size = 512;
		}
		step_time_accum = 0;
		step_count = 0;
	}

	/* Instrumentation for OSD stats display */
	static uint32_t stat_cpu_us = 0;
	static uint32_t stat_periph_us = 0;
	static uint32_t stat_calls = 0;
	static uint32_t stat_last_report = 0;
	static long stat_last_cycles = 0;

	uint32_t t0 = get_uticks();

	/* Burst catch-up for high-frequency PIT interrupts.
	 * When games program PIT for music timing (e.g., 6000+ Hz), we can't
	 * deliver that many interrupts per second. Instead of dropping them,
	 * we process multiple ISRs per step to catch up.
	 *
	 * Optimized: use pit_burst_start/fire to avoid repeated get_uticks()
	 * calls and mode validation on each iteration. */
	uint32_t pit_d;
	int pit_irq;
	int pending = pit_burst_start(pc->pit, &pit_d, &pit_irq);
	if (pending > 1) {
		/* Limit burst size to avoid stalling other emulation too long. */
		int burst = pending > 48 ? 48 : pending;
		for (int i = 0; i < burst; i++) {
			if (!pit_burst_fire(pc->pit, pit_d, pit_irq))
				break;
			/* Run enough instructions for a typical music ISR (~60-100) */
			cpui386_step(pc->cpu, 100);
		}
	} else if (pending == 1) {
		/* Single IRQ pending - use normal path */
		i8254_update_irq(pc->pit);
	}

	cmos_update_irq(pc->cmos);
	if (pc->enable_serial)
		u8250_update(pc->serial);
	kbd_step(pc->i8042);
	ne2000_step(pc->ne2000);
	i8257_dma_run(pc->isa_dma);
	i8257_dma_run(pc->isa_hdma);
	ide_poll_async();
	uint32_t t1 = 0, t2 = 0;
	bool collecting = globals.stats_collecting;
	if (collecting) t1 = get_uticks();  /* After peripherals */
	cpui386_step(pc->cpu, batch_size);
	if (collecting) t2 = get_uticks();  /* After CPU */

	/* Dynamic batch adjustment (only when pc_batch_size_setting == 0) */
	if (pc_batch_size_setting == 0) {
		uint32_t step_time = get_uticks() - t0;
		step_time_accum += step_time;
		step_count++;

		/* Evaluate and adjust batch size every 100 steps */
		if (step_count >= 100) {
			uint32_t avg_step_time = step_time_accum / step_count;

			/* Hysteresis thresholds to prevent oscillation:
			 * - Scale UP by 256 if avg < 800us (fast steps, room to grow)
			 * - Scale DOWN by 256 if avg > 2000us (slow steps, reduce load)
			 * - Dead zone 800-2000us: hold current batch size
			 * Range: 512-4096, step size 256 (15 levels) */
			if (avg_step_time < 800 && batch_size < 4096) {
				batch_size += 256;
			} else if (avg_step_time > 2000 && batch_size > 512) {
				batch_size -= 256;
			}

			step_time_accum = 0;
			step_count = 0;
		}
	}

	/* Lazy instrumentation - only when Status panel is open */
	if (collecting) {
		stat_periph_us += (t1 - t0);
		stat_cpu_us += (t2 - t1);
		stat_calls++;

		/* Update globals every ~2 seconds */
		if (t2 - stat_last_report >= 2000000) {
			uint32_t total = stat_cpu_us + stat_periph_us;
			if (total > 0) {
				globals.emu_cpu_percent = (stat_cpu_us * 100) / total;
				globals.emu_periph_percent = (stat_periph_us * 100) / total;
				globals.emu_calls_per_sec = (stat_calls * 1000000ULL) / (t2 - stat_last_report);
			}
			long cur_cycles = cpu_get_cycle(pc->cpu);
			uint32_t elapsed_us = t2 - stat_last_report;
			if (elapsed_us > 0) {
				globals.emu_cycles_per_sec = (uint32_t)(((uint64_t)(cur_cycles - stat_last_cycles) * 1000000ULL) / elapsed_us);
			}
			stat_last_cycles = cur_cycles;
			stat_cpu_us = 0;
			stat_periph_us = 0;
			stat_calls = 0;
			stat_last_report = t2;
		}
	}
	/* Batch size is always available (tracked for dynamic batching) */
	globals.emu_batch_size = batch_size;
}

static void raise_irq(void *o, PicState2 *s)
{
	cpu_raise_irq(o);
}

static int read_irq(void *o)
{
	PicState2 *s = o;
	return i8259_read_irq(s);
}

static void set_irq(void *o, int irq, int level)
{
	PicState2 *s = o;
	return i8259_set_irq(s, irq, level);
}

static void set_pci_vga_bar(void *opaque, int bar_num, uint32_t addr, bool enabled)
{
	PC *pc = opaque;
	if (enabled)
		pc->pci_vga_ram_addr = addr;
	else
		pc->pci_vga_ram_addr = -1;
}

static u8 IRAM_ATTR iomem_read8(void *iomem, uword addr)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X uses 0xA0000-0xAFFFF) */
	if (addr < 0xc0000)
		return vga_mem_read(pc->vga, addr - 0xa0000);
	/* PCI VGA linear framebuffer (VBE modes) */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr < pc->vga_mem_size)
			return pc->vga_mem[addr];
	}
	return 0;
}

static void IRAM_ATTR iomem_write8(void *iomem, uword addr, u8 val)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X uses 0xA0000-0xAFFFF) */
	if (addr < 0xc0000) {
		vga_mem_write(pc->vga, addr - 0xa0000, val);
		return;
	}
	/* PCI VGA linear framebuffer (VBE modes) */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr < pc->vga_mem_size)
			pc->vga_mem[addr] = val;
	}
}

static u16 IRAM_ATTR iomem_read16(void *iomem, uword addr)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X) */
	if (addr < 0xc0000)
		return vga_mem_read16(pc->vga, addr - 0xa0000);
	/* PCI VGA linear framebuffer */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr + 1 < pc->vga_mem_size)
			return *(uint16_t *)&(pc->vga_mem[addr]);
	}
	return 0;
}

static void IRAM_ATTR iomem_write16(void *iomem, uword addr, u16 val)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X) */
	if (addr < 0xc0000) {
		vga_mem_write16(pc->vga, addr - 0xa0000, val);
		return;
	}
	/* PCI VGA linear framebuffer */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr + 1 < pc->vga_mem_size)
			*(uint16_t *)&(pc->vga_mem[addr]) = val;
	}
}

static u32 IRAM_ATTR iomem_read32(void *iomem, uword addr)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X) */
	if (addr < 0xc0000)
		return vga_mem_read32(pc->vga, addr - 0xa0000);
	/* PCI VGA linear framebuffer */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr + 3 < pc->vga_mem_size)
			return *(uint32_t *)&(pc->vga_mem[addr]);
	}
	return 0;
}

static void IRAM_ATTR iomem_write32(void *iomem, uword addr, u32 val)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X) */
	if (addr < 0xc0000) {
		vga_mem_write32(pc->vga, addr - 0xa0000, val);
		return;
	}
	/* PCI VGA linear framebuffer */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr + 3 < pc->vga_mem_size)
			*(uint32_t *)&(pc->vga_mem[addr]) = val;
	}
	vga_mem_write32(pc->vga, addr - 0xa0000, val);
}

static bool IRAM_ATTR iomem_write_string(void *iomem, uword addr, uint8_t *buf, int len)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X) */
	if (addr < 0xc0000)
		return vga_mem_write_string(pc->vga, addr - 0xa0000, buf, len);
	/* PCI VGA linear framebuffer */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr + len < pc->vga_mem_size) {
			memcpy(pc->vga_mem + addr, buf, len);
			return true;
		}
	}
	return false;
}

static bool IRAM_ATTR iomem_read_string(void *iomem, uword addr, uint8_t *buf, int len)
{
	PC *pc = iomem;
	/* Fast path for legacy VGA (Mode X) */
	if (addr < 0xc0000)
		return vga_mem_read_string(pc->vga, addr - 0xa0000, buf, len);
	/* PCI VGA linear framebuffer */
	uword vga_addr2 = pc->pci_vga_ram_addr;
	if (addr >= vga_addr2) {
		addr -= vga_addr2;
		if (addr + len <= pc->vga_mem_size) {
			memcpy(buf, pc->vga_mem + addr, len);
			return true;
		}
	}
	return false;
}

static bool IRAM_ATTR iomem_copy_string(void *iomem, uword dst, uword src, int len)
{
	PC *pc = iomem;
	/* Only handle VGA memory copies (Mode X latch operations) */
	if (dst < 0xc0000 && src < 0xc0000)
		return vga_mem_copy_string(pc->vga, dst - 0xa0000, src - 0xa0000, len);
	return false;
}

static void pc_reset_request(void *p)
{
	PC *pc = p;
	pc->reset_request = 1;
}

static void pc_set_a20(void *p, int val)
{
	PC *pc = p;
	cpui386_set_a20(pc->cpu, val);
}

static int pc_get_a20(void *p)
{
	PC *pc = p;
	return cpui386_get_a20(pc->cpu);
}

/* Callback for direct VGA writes - marks lines dirty for rendering */
static void vga_direct_write_notify(void *iomem, uword addr, int len)
{
	PC *pc = iomem;
	/* Convert from physical address (0xA0000+) to VGA-relative address */
	vga_direct_mark_dirty(pc->vga, addr - 0xa0000, len);
}

PC *pc_new(SimpleFBDrawFunc *redraw, void (*poll)(void *), void *redraw_data,
	   u8 *fb, PCConfig *conf)
{
	PC *pc = malloc(sizeof(PC));
	/* Cap guest RAM to fit in PSRAM pool, leaving room for VGA + peripherals */
	extern long psram_remaining(void);
	long reserve = conf->vga_mem_size + 1024 * 1024; /* VGA + 1MB safety */
	long avail = psram_remaining() - reserve;
	if (avail < 1024 * 1024) avail = 1024 * 1024;
	avail = (avail / (1024 * 1024)) * (1024 * 1024); /* round down to 1MB */
	if (conf->mem_size > avail) {
		fprintf(stderr, "Reducing guest RAM from %ldMB to %ldMB (PSRAM limit)\n",
			conf->mem_size / (1024 * 1024), avail / (1024 * 1024));
		conf->mem_size = avail;
	}
	char *mem = bigmalloc(conf->mem_size);
	CPU_CB *cb = NULL;
	memset(mem, 0, conf->mem_size);
	extern char *pcram;
	extern long pcram_len;
	pcram = mem + 0xa0000;
	pcram_len = 0xc0000 - 0xa0000;  // 128KB
	pc->cpu = cpui386_new(conf->cpu_gen, mem, conf->mem_size, &cb);
	if (conf->fpu)
		cpui386_enable_fpu(pc->cpu);
	pc->cpu_cb = cb;
	pc->bios = conf->bios;
	pc->vga_bios = conf->vga_bios;
	pc->linuxstart = conf->linuxstart;
	pc->kernel = conf->kernel;
	pc->initrd = conf->initrd;
	pc->cmdline = conf->cmdline;
	pc->ini_path = conf->ini_path;
	pc->enable_serial = conf->enable_serial;
	pc->cpu_gen = conf->cpu_gen;
	pc->fpu = conf->fpu;
	if (pc->enable_serial)
		CaptureKeyboardInput();
	pc->full_update = 0;

	pc->pic = i8259_init(raise_irq, pc->cpu);
	cb->pic = pc->pic;
	cb->pic_read_irq = read_irq;

	pc->pit = i8254_init(0, pc->pic, set_irq);
	pc->serial = u8250_init(4, pc->pic, set_irq);
	pc->cmos = cmos_init(conf->mem_size, 8, pc->pic, set_irq);
	pc->ide = ide_allocate(14, pc->pic, set_irq);
	pc->ide2 = ide_allocate(15, pc->pic, set_irq);
	const char **disks = conf->disks;
	// Tanmatsu fixed layout:
	// Slot 0 (primary master): HDA - main hard drive
	// Slot 1 (primary slave): CD-ROM - always created for hot-mounting
	// Slot 2 (secondary master): USB storage
	// Slot 3 (secondary slave): unused
	if (disks[0] && disks[0][0]) {
		int ret = ide_attach(pc->ide, 0, disks[0]);
		assert(ret == 0);
		if (conf->hda_heads > 0 && conf->hda_spt > 0)
			ide_set_geometry(pc->ide, 0, conf->hda_heads, conf->hda_spt);
		pc->hda_path = disks[0];  // Store for OSD access
	} else {
		pc->hda_path = NULL;
	}
	// Always create CD-ROM on slot 1 for hot-mounting
	{
		const char *cd_path = (disks[1] && disks[1][0]) ? disks[1] : NULL;
		int ret = ide_attach_cd(pc->ide, 1, cd_path);
		assert(ret == 0);
	}
	// USB on slot 2 (if passthrough enabled)
	if (conf->usb_passthru) {
		ide_attach_usb(pc->ide2, 0);
	}

	pc->fill_cmos = conf->fill_cmos;
	if (conf->fill_cmos)
		ide_fill_cmos(pc->ide, pc->cmos, cmos_set);

	// Set boot order from config (default: HDD first) - can be changed via OSD
	cmos_set_boot_order(pc->cmos, conf->boot_order);

	int piix3_devfn;
	pc->i440fx = i440fx_init(&pc->pcibus, &piix3_devfn);
	pc->pci_ide = piix3_ide_init(pc->pcibus, piix3_devfn + 1);

	pc->phys_mem = mem;
	pc->phys_mem_size = conf->mem_size;

	cb->io = pc;
	cb->io_read8 = pc_io_read;
	cb->io_write8 = pc_io_write;
	cb->io_read16 = pc_io_read16;
	cb->io_write16 = pc_io_write16;
	cb->io_read32 = pc_io_read32;
	cb->io_write32 = pc_io_write32;
	cb->io_read_string = pc_io_read_string;
	cb->io_write_string = pc_io_write_string;

	pc->boot_start_time = 0;

	pc->vga_mem_size = conf->vga_mem_size;
	pc->vga_mem = bigmalloc(pc->vga_mem_size);
	memset(pc->vga_mem, 0, pc->vga_mem_size);
	pc->vga = vga_init(pc->vga_mem, pc->vga_mem_size,
			   fb, conf->width, conf->height);
	vga_set_force_8dm(pc->vga, conf->vga_force_8dm);
	pc->pci_vga = vga_pci_init(pc->vga, pc->pcibus, pc, set_pci_vga_bar);
	pc->pci_vga_ram_addr = -1;

	pc->emulink = emulink_init();
	const char **fdd = conf->fdd;
	for (int i = 0; i < 2; i++) {
		if (!fdd[i] || fdd[i][0] == 0)
			continue;
		int ret;
		ret = emulink_attach_floppy(pc->emulink, i, fdd[i]);
		if (ret != 0) {
			fprintf(stderr, "Warning: failed to attach floppy %d: %s\n", i, fdd[i]);
		}
	}

	cb->iomem = pc;
	cb->iomem_read8 = iomem_read8;
	cb->iomem_write8 = iomem_write8;
	cb->iomem_read16 = iomem_read16;
	cb->iomem_write16 = iomem_write16;
	cb->iomem_read32 = iomem_read32;
	cb->iomem_write32 = iomem_write32;
	cb->iomem_write_string = iomem_write_string;
	cb->iomem_read_string = iomem_read_string;
	cb->iomem_copy_string = iomem_copy_string;

	/* Set up direct VGA access for chain-4 mode (mode 13h) */
	cb->vga_direct = vga_get_direct_ptr(pc->vga);
	cb->vga_direct_write_notify = vga_direct_write_notify;

	/* Set up Mode X inline fast path (initially may be NULL) */
	vga_get_modex_state(pc->vga, &cb->vga_modex_ram, &cb->vga_modex_ram_size,
	                    &cb->vga_modex_write_mode, &cb->vga_modex_plane_mask,
	                    &cb->vga_modex_read_plane, &cb->vga_modex_latch);

	pc->redraw = redraw;
	pc->redraw_data = redraw_data;
	pc->poll = poll;

	pc->i8042 = i8042_init(&(pc->kbd), &(pc->mouse),
			       1, 12, pc->pic, set_irq,
			       pc, pc_reset_request);
	i8042_set_a20_cb(pc->i8042, pc_set_a20, pc_get_a20);
	pc->adlib = adlib_new();
	pc->ne2000 = isa_ne2000_init(0x300, 9, pc->pic, set_irq);
	pc->isa_dma = i8257_new(pc->phys_mem, pc->phys_mem_size,
				0x00, 0x80, 0x480, 0);
	pc->isa_hdma = i8257_new(pc->phys_mem, pc->phys_mem_size,
				 0xc0, 0x88, 0x488, 1);
	pc->sb16 = sb16_new(0x220, 5,
			    pc->isa_dma, pc->isa_hdma,
			    pc->pic, set_irq);
	pc->pcspk = pcspk_init(pc->pit);
	pc->port92 = 0;  /* A20 disabled at reset (bit 1 = 0) */
	pc->shutdown_state = 0;
	pc->reset_request = 0;
	return pc;
}

#define MIXER_BUF_LEN 128
void mixer_callback (void *opaque, uint8_t *stream, int free)
{
	uint8_t tmpbuf[MIXER_BUF_LEN];
	PC *pc = opaque;
	if (!pc) {
		memset(stream, 0, free);
		return;
	}
	assert(free / 2 <= MIXER_BUF_LEN);
	memset(tmpbuf, 0, MIXER_BUF_LEN);
	if (!pc->adlib || !pc->sb16 || !pc->pcspk) {
		memset(stream, 0, free);
		return;
	}
	adlib_callback(pc->adlib, tmpbuf, free / 2); // s16, mono
	sb16_audio_callback(pc->sb16, stream, free); // s16, stereo

	/* Get volume levels from SB16 mixer (0-7 scale per channel) */
	int fm_vol_l, fm_vol_r;
	int voice_vol_l, voice_vol_r;
	int master_vol_l, master_vol_r;
	sb16_get_fm_volume(pc->sb16, &fm_vol_l, &fm_vol_r);
	sb16_get_voice_volume(pc->sb16, &voice_vol_l, &voice_vol_r);
	sb16_get_master_volume(pc->sb16, &master_vol_l, &master_vol_r);

	int16_t *d2 = (int16_t *) stream;
	int16_t *d1 = (int16_t *) tmpbuf;
	for (int i = 0; i < free / 2; i += 2) {
		/* Apply FM volume to OPL samples (0-7 range, 7 = full volume) */
		int fm_l = (d1[i / 2] * fm_vol_l) / 7;
		int fm_r = (d1[i / 2] * fm_vol_r) / 7;
		/* Apply voice volume to SB16 digital audio */
		int voice_l = (d2[i] * voice_vol_l) / 7;
		int voice_r = (d2[i + 1] * voice_vol_r) / 7;
		/* Mix FM with voice */
		int mix_l = voice_l + fm_l;
		int mix_r = voice_r + fm_r;
		/* Apply master volume */
		mix_l = (mix_l * master_vol_l) / 7;
		mix_r = (mix_r * master_vol_r) / 7;
		/* Clamp to 16-bit range */
		if (mix_l > 32767) mix_l = 32767;
		if (mix_l < -32768) mix_l = -32768;
		if (mix_r > 32767) mix_r = 32767;
		if (mix_r < -32768) mix_r = -32768;
		d2[i] = mix_l;
		d2[i + 1] = mix_r;
	}

	/* PC Speaker bypasses SB16 mixer (separate circuit on real hardware) */
	if (pcspk_get_active_out(pc->pcspk)) {
		memset(tmpbuf, 0x80, MIXER_BUF_LEN / 2);
		pcspk_callback(pc->pcspk, tmpbuf, free / 4); // u8, mono
		for (int i = 0; i < free / 2; i++) {
			int res = d2[i];
			res += ((int) tmpbuf[i / 2] - 0x80) << 5;
			if (res > 32767) res = 32767;
			if (res < -32768) res = -32768;
			d2[i] = res;
		}
	}
}

void load_bios_and_reset(PC *pc)
{
	if (pc->bios && pc->bios[0])
		load_rom(pc->phys_mem, pc->bios, 0x100000, 1);
	if (pc->vga_bios && pc->vga_bios[0])
		load_rom(pc->phys_mem, pc->vga_bios, 0xc0000, 0);
	if (pc->kernel && pc->kernel[0]) {
		int start_addr = 0x10000;
		int cmdline_addr = 0xf800;
		int kernel_size = load_rom(pc->phys_mem, pc->kernel, 0x00100000, 0);
		int initrd_size = 0;
		if (pc->initrd && pc->initrd[0])
			initrd_size = load_rom(pc->phys_mem, pc->initrd, 0x00400000, 0);
		if (pc->cmdline && pc->cmdline[0])
			strcpy(pc->phys_mem + cmdline_addr, pc->cmdline);
		else
			strcpy(pc->phys_mem + cmdline_addr, "");

		load_rom(pc->phys_mem, pc->linuxstart, start_addr, 0);
		cpui386_reset_pm(pc->cpu, 0x10000);
		cpui386_set_gpr(pc->cpu, 0, pc->phys_mem_size);
		cpui386_set_gpr(pc->cpu, 3, initrd_size);
		cpui386_set_gpr(pc->cpu, 1, cmdline_addr);
		cpui386_set_gpr(pc->cpu, 2, kernel_size);
	} else {
		cpui386_reset(pc->cpu);
	}
}

void pc_reset(PC *pc)
{
	// Clear conventional memory (0-640KB) but preserve video/BIOS areas
	memset(pc->phys_mem, 0, 0xa0000);
	// Reset keyboard controller and clear any pending input
	i8042_reset(pc->i8042);
	// Reset port 92 (A20 gate)
	pc->port92 = 0;  /* A20 disabled at reset (bit 1 = 0) */
	// Update CMOS disk geometry (may have changed via OSD disk swap)
	if (pc->fill_cmos)
		ide_fill_cmos(pc->ide, pc->cmos, cmos_set);
	// Reload BIOS and reset CPU
	load_bios_and_reset(pc);
}

static long parse_mem_size(const char *value)
{
	if (!value || !value[0]) return 0;
	int len = strlen(value);
	long a = atol(value);
	/* atol returns 0 for non-numeric strings - check if input was actually "0" */
	if (a == 0 && value[0] != '0') {
		fprintf(stderr, "Warning: Invalid memory size '%s', using 0\n", value);
		return 0;
	}
	if (len) {
		switch (value[len - 1]) {
		case 'G': case 'g': a *= 1024 * 1024 * 1024; break;
		case 'M': case 'm': a *= 1024 * 1024; break;
		case 'K': case 'k': a *= 1024; break;
		}
	}
	/* Sanity check for negative or zero values */
	if (a <= 0) {
		fprintf(stderr, "Warning: Memory size '%s' resulted in %ld, using 0\n", value, a);
		return 0;
	}
	return a;
}

int parse_conf_ini(void* user, const char* section,
		   const char* name, const char* value)
{
	PCConfig *conf = user;
#define SEC(a) (strcmp(section, a) == 0)
#define NAME(a) (strcmp(name, a) == 0)
	if (SEC("pc")) {
		if (NAME("bios")) {
			conf->bios = strdup(value);
		} else if (NAME("vga_bios")) {
			conf->vga_bios = strdup(value);
		} else if (NAME("mem_size")) {
			conf->mem_size = parse_mem_size(value);
		} else if (NAME("vga_mem_size")) {
			conf->vga_mem_size = parse_mem_size(value);
		} else if (NAME("hda")) {
			conf->disks[0] = strdup(value);
			conf->iscd[0] = 0;
		} else if (NAME("hda_heads")) {
			conf->hda_heads = atoi(value);
		} else if (NAME("hda_spt")) {
			conf->hda_spt = atoi(value);
		} else if (NAME("hdb")) {
			conf->disks[1] = strdup(value);
			conf->iscd[1] = 0;
		} else if (NAME("hdc")) {
			conf->disks[2] = strdup(value);
			conf->iscd[2] = 0;
		} else if (NAME("hdd")) {
			conf->disks[3] = strdup(value);
			conf->iscd[3] = 0;
		} else if (NAME("cda")) {
			conf->disks[0] = strdup(value);
			conf->iscd[0] = 1;
		} else if (NAME("cdb")) {
			conf->disks[1] = strdup(value);
			conf->iscd[1] = 1;
		} else if (NAME("cdc")) {
			conf->disks[2] = strdup(value);
			conf->iscd[2] = 1;
		} else if (NAME("cdd")) {
			conf->disks[3] = strdup(value);
			conf->iscd[3] = 1;
		} else if (NAME("fda")) {
			conf->fdd[0] = strdup(value);
		} else if (NAME("fdb")) {
			conf->fdd[1] = strdup(value);
		} else if (NAME("fill_cmos")) {
			conf->fill_cmos = atoi(value);
		} else if (NAME("linuxstart")) {
			conf->linuxstart = strdup(value);
		} else if (NAME("kernel")) {
			conf->kernel = strdup(value);
		} else if (NAME("initrd")) {
			conf->initrd = strdup(value);
		} else if (NAME("cmdline")) {
			conf->cmdline = strdup(value);
		} else if (NAME("enable_serial")) {
			conf->enable_serial = atoi(value);
		} else if (NAME("vga_force_8dm")) {
			conf->vga_force_8dm = atoi(value);
		} else if (NAME("boot_order")) {
			// Index 0-5 or name like "hdd,floppy,cd"
			printf("INI: boot_order value='%s'\n", value);
			int idx = atoi(value);
			if (idx >= 0 && idx < BOOT_ORDER_COUNT && value[0] >= '0' && value[0] <= '9') {
				conf->boot_order = idx;
				printf("INI: boot_order parsed as index %d\n", idx);
			} else {
				// Try to match by name
				for (int i = 0; i < BOOT_ORDER_COUNT; i++) {
					if (strcasecmp(value, boot_order_names[i]) == 0) {
						conf->boot_order = i;
						printf("INI: boot_order matched name '%s' -> %d\n", boot_order_names[i], i);
						break;
					}
				}
			}
		} else if (NAME("brightness")) {
			conf->brightness = atoi(value);
			if (conf->brightness < 0) conf->brightness = 0;
			if (conf->brightness > 100) conf->brightness = 100;
		} else if (NAME("volume")) {
			conf->volume = atoi(value);
			if (conf->volume < 0) conf->volume = 0;
			if (conf->volume > 100) conf->volume = 100;
		} else if (NAME("frame_skip")) {
			// Also accept frame_skip in [pc] section (save_settings_to_ini writes here)
			conf->frame_skip = atoi(value);
			if (conf->frame_skip < 0) conf->frame_skip = 0;
			if (conf->frame_skip > 10) conf->frame_skip = 10;
		} else if (NAME("mouse_speed")) {
			conf->mouse_speed = atoi(value);
			if (conf->mouse_speed < 1) conf->mouse_speed = 1;
			if (conf->mouse_speed > 10) conf->mouse_speed = 10;
		} else if (NAME("usb_passthru")) {
			conf->usb_passthru = atoi(value) ? 1 : 0;
		}
	} else if (SEC("display")) {
		if (NAME("width")) {
			conf->width = atoi(value);
		} else if (NAME("height")) {
			conf->height = atoi(value);
		} else if (NAME("frame_skip")) {
			conf->frame_skip = atoi(value);
			if (conf->frame_skip < 0) conf->frame_skip = 0;
			if (conf->frame_skip > 10) conf->frame_skip = 10;
		} else if (NAME("double_buffer")) {
			// Ignored - TE vsync replaces software double buffering
		}
	} else if (SEC("cpu")) {
		if (NAME("gen")) {
			int gen = atoi(value);
			if (gen < 3 || gen > 6) {
				fprintf(stderr, "Warning: Invalid cpu gen '%s' (valid: 3-6), using 4\n", value);
				gen = 4;
			}
			conf->cpu_gen = gen;
		} else if (NAME("fpu")) {
			conf->fpu = atoi(value);
		} else if (NAME("batch_size")) {
			int bs = atoi(value);
			// Validate: 0 (auto) or 512-4096
			if (bs != 0 && (bs < 512 || bs > 4096)) {
				fprintf(stderr, "Warning: Invalid batch_size '%s' (valid: 0 or 512-4096), using auto\n", value);
				bs = 0;
			}
			conf->batch_size = bs;
		}
	}
#undef SEC
#undef NAME
	return 1;
}
