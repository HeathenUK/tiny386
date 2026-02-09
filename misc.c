#include "misc.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include <stdio.h>

#include "led_activity.h"
#define FLOPPY_ACTIVITY() led_activity_floppy()
#include <sys/ioctl.h>
#include <termios.h>
#include <signal.h>
#include "driver/uart.h"

static void CtrlC(int _)
{
	exit( 0 );
}

static void ResetKeyboardInput()
{
	// Re-enable echo, etc. on keyboard.
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag |= ICANON | ECHO;
	tcsetattr(0, TCSANOW, &term);
}

// Override keyboard, so we can capture all keyboard input for the VM.
void CaptureKeyboardInput()
{
	// Hook exit, because we want to re-enable keyboard.

	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag &= ~(ICANON | ECHO | ISIG); // Disable echo as well
	tcsetattr(0, TCSANOW, &term);
}

static int ReadKBByte()
{
	char data;
	if (uart_read_bytes(0, &data, 1, 20 / portTICK_PERIOD_MS) > 0) {
		return data;
	}
	return -1;
}

static int IsKBHit()
{
	size_t len;
	if (uart_get_buffered_data_len(0, &len) == ESP_OK) {
		if (len)
			return 1;
	}
	return 0;
}

/* sysprog21/semu */
struct U8250 {
	uint8_t dll, dlh;
	uint8_t lcr;
	uint8_t ier;
	uint8_t mcr;
	uint8_t ioready;
	int out_fd;
	uint8_t in;

	int irq;
	void *pic;
	void (*set_irq)(void *pic, int irq, int level);
};

U8250 *u8250_init(int irq, void *pic, void (*set_irq)(void *pic, int irq, int level))
{
	U8250 *s = malloc(sizeof(U8250));
	memset(s, 0, sizeof(U8250));
	s->out_fd = 1;

	s->irq = irq;
	s->pic = pic;
	s->set_irq = set_irq;
	return s;
}

struct CMOS {
	uint8_t data[128];
	int index;
	int irq;
	uint32_t irq_timeout;
	uint32_t irq_period;
	void *pic;
	void (*set_irq)(void *pic, int irq, int level);
};

static int bin2bcd(int a)
{
	return ((a / 10) << 4) | (a % 10);
}

static void cmos_update_time(CMOS *s)
{
	struct tm tm;
	time_t ti;

	ti = time(NULL);
	gmtime_r(&ti, &tm);
	s->data[0] = bin2bcd(tm.tm_sec);
	s->data[2] = bin2bcd(tm.tm_min);
	s->data[4] = bin2bcd(tm.tm_hour);
	s->data[6] = bin2bcd(tm.tm_wday);
	s->data[7] = bin2bcd(tm.tm_mday);
	s->data[8] = bin2bcd(tm.tm_mon + 1);
	s->data[9] = bin2bcd(tm.tm_year % 100);
	s->data[0x32] = bin2bcd((tm.tm_year / 100) + 19);
}

CMOS *cmos_init(long mem_size, int irq, void *pic, void (*set_irq)(void *pic, int irq, int level))
{
	CMOS *c = malloc(sizeof(CMOS));
	memset(c, 0, sizeof(CMOS));
	c->irq = irq;
	c->pic = pic;
	c->set_irq = set_irq;

	cmos_update_time(c);
	c->data[10] = 0x26;
	c->data[11] = 0x02;
	c->data[12] = 0x00;
	c->data[13] = 0x80;
	/* Base memory: 640KB (CMOS 0x15/0x16, little-endian KB) */
	c->data[0x15] = 0x80;  /* 640 & 0xFF */
	c->data[0x16] = 0x02;  /* 640 >> 8 */

	if (mem_size >= 1024 * 1024) {
		/* Extended memory in KB above 1MB (CMOS 0x30/0x31).
		 * Capped at 63MB (64512 KB = 0xFC00) per CMOS convention. */
		long ext_kb = (mem_size - 1024 * 1024) / 1024;
		if (ext_kb > 0xFC00) ext_kb = 0xFC00;
		c->data[0x30] = ext_kb & 0xFF;
		c->data[0x31] = (ext_kb >> 8) & 0xFF;

		/* Memory above 16MB in 64KB blocks (CMOS 0x34/0x35).
		 * Only set if we actually have > 16MB. */
		if (mem_size > 16 * 1024 * 1024) {
			long above16m = (mem_size - 16 * 1024 * 1024) / (64 * 1024);
			if (above16m > 0xFFFF) above16m = 0xFFFF;
			c->data[0x34] = above16m & 0xFF;
			c->data[0x35] = (above16m >> 8) & 0xFF;
		}
	}
	return c;
}

static void u8250_update_interrupts(U8250 *uart)
{
	if (uart->ier & uart->ioready) {
		uart->set_irq(uart->pic, uart->irq, 0);
		uart->set_irq(uart->pic, uart->irq, 1);
	} else {
		uart->set_irq(uart->pic, uart->irq, 0);
	}
}

uint8_t u8250_reg_read(U8250 *uart, int off)
{
	uint8_t val;
	switch (off) {
	case 0:
		if (uart->lcr & (1 << 7)) { /* DLAB */
			val = uart->dll;
			break;
		}
		val = uart->in;
		uart->ioready &= ~1;
		u8250_update_interrupts(uart);
		break;
	case 1:
		if (uart->lcr & (1 << 7)) { /* DLAB */
			val = uart->dlh;
			break;
		}
		val = uart->ier;
		break;
	case 2:
		val = (uart->ier & uart->ioready) ? 0 : 1;
		break;
	case 3:
		val = uart->lcr;
		break;
	case 4:
		val = uart->mcr;
		break;
	case 5:
		/* LSR = no error, TX done & ready */
		val = 0x60 | (uart->ioready & 1);
		break;
	case 6:
		/* MSR = carrier detect, no ring, data ready, clear to send. */
		val = 0xb0;
		break;
		/* no scratch register, so we should be detected as a plain 8250. */
	default:
		val = 0;
	}
	return val;
}

void u8250_reg_write(U8250 *uart, int off, uint8_t val)
{
	switch (off) {
	case 0:
		if (uart->lcr & (1 << 7)) {
			uart->dll = val;
			break;
		} else {
			ssize_t r;
			do {
				r = write(uart->out_fd, &val, 1);
			} while (r == -1 && errno == EINTR);
		}
		break;
	case 1:
		if (uart->lcr & (1 << 7)) {
			uart->dlh = val;
			break;
		} else {
			uart->ier = val;
			if (uart->ier & 2)
				uart->ioready |= 2;
			else
				uart->ioready &= ~2;
			u8250_update_interrupts(uart);
		}
		break;
	case 3:
		uart->lcr = val;
		break;
	case 4:
		uart->mcr = val;
		break;
	}
}

void u8250_update(U8250 *uart)
{
	if (IsKBHit()) {
		if (!(uart->ioready & 1)) {
			uart->in = ReadKBByte();
			uart->ioready |= 1;
			u8250_update_interrupts(uart);
		}
	}
}

#define CMOS_FREQ 32768
#define RTC_REG_A               10
#define RTC_REG_B               11
#define RTC_REG_C               12
#define RTC_REG_D               13
#define REG_A_UIP 0x80
#define REG_B_SET 0x80
#define REG_B_PIE 0x40
#define REG_B_AIE 0x20
#define REG_B_UIE 0x10

static uint32_t cmos_get_timer(CMOS *s)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint32_t)ts.tv_sec * CMOS_FREQ +
		((uint64_t)ts.tv_nsec * CMOS_FREQ / 1000000000);
}

static void cmos_update_timer(CMOS *s)
{
	int period_code;

	period_code = s->data[RTC_REG_A] & 0x0f;
	if ((s->data[RTC_REG_B] & REG_B_PIE) &&
	    period_code != 0) {
		if (period_code <= 2)
			period_code += 7;
		s->irq_period = 1 << (period_code - 1);
		s->irq_timeout = (cmos_get_timer(s) + s->irq_period) &
			~(s->irq_period - 1);
	}
}

void cmos_update_irq(CMOS *s)
{
	uint32_t d;
	if (s->data[RTC_REG_B] & REG_B_PIE) {
		d = cmos_get_timer(s) - s->irq_timeout;
		if ((int32_t)d >= 0) {
			/* this is not what the real RTC does. Here we sent the IRQ
			   immediately */
			s->data[RTC_REG_C] |= 0xc0;
			s->set_irq(s->pic, s->irq, 1);
			s->set_irq(s->pic, s->irq, 0);
			/* update for the next irq */
			s->irq_timeout += s->irq_period;
		}
	}
}

uint8_t cmos_ioport_read(CMOS *cmos, int addr)
{
	if (addr == 0x70)
		return 0xff;
	cmos_update_time(cmos);
	uint8_t val = cmos->data[cmos->index];
	/* Reading Register C clears interrupt flags and deasserts IRQ 8 */
	if (cmos->index == RTC_REG_C)
		cmos->data[RTC_REG_C] = 0;
	return val;
}

void cmos_ioport_write(CMOS *cmos, int addr, uint8_t val)
{
	if (addr == 0x70)
		cmos->index = val & 0x7f;
	else {
		CMOS *s = cmos;
		switch(s->index) {
		case RTC_REG_A:
			s->data[RTC_REG_A] = (val & ~REG_A_UIP) |
				(s->data[RTC_REG_A] & REG_A_UIP);
			cmos_update_timer(s);
			break;
		case RTC_REG_B:
			s->data[s->index] = val;
			cmos_update_timer(s);
			break;
		default:
			s->data[s->index] = val;
			break;
		}
	}
}

uint8_t cmos_set(void *cmos, int addr, uint8_t val)
{
	CMOS *s = cmos;
	if (addr < 128)
		s->data[addr] = val;
	return val;
}

// SeaBIOS boot device codes: 1=floppy, 2=HDD, 3=CD-ROM
// CMOS 0x38: bit 0 = enable extended boot order, bits 4-7 = 3rd device
// CMOS 0x3d: bits 0-3 = 1st device, bits 4-7 = 2nd device

// All permutations of boot order
const int boot_orders[BOOT_ORDER_COUNT][3] = {
	{2, 1, 3},  // HDD, Floppy, CD
	{2, 3, 1},  // HDD, CD, Floppy
	{1, 2, 3},  // Floppy, HDD, CD
	{1, 3, 2},  // Floppy, CD, HDD
	{3, 2, 1},  // CD, HDD, Floppy
	{3, 1, 2},  // CD, Floppy, HDD
};

const char *boot_order_names[BOOT_ORDER_COUNT] = {
	"HDD, Floppy, CD",
	"HDD, CD, Floppy",
	"Floppy, HDD, CD",
	"Floppy, CD, HDD",
	"CD, HDD, Floppy",
	"CD, Floppy, HDD",
};

void cmos_set_boot_order(CMOS *cmos, int order_index)
{
	if (order_index < 0 || order_index >= BOOT_ORDER_COUNT)
		order_index = 0;

	const int *order = boot_orders[order_index];
	// CMOS 0x3d: low nibble = 1st, high nibble = 2nd
	uint8_t reg_3d = (order[0] & 0xf) | ((order[1] & 0xf) << 4);
	// CMOS 0x38: bit 0 = enable, high nibble = 3rd device
	uint8_t reg_38 = 0x01 | ((order[2] & 0xf) << 4);

	cmos_set(cmos, 0x38, reg_38);
	cmos_set(cmos, 0x3d, reg_3d);
}

int cmos_get_boot_order(CMOS *cmos)
{
	uint8_t reg_3d = cmos->data[0x3d];
	uint8_t reg_38 = cmos->data[0x38];

	int dev1 = reg_3d & 0xf;
	int dev2 = (reg_3d >> 4) & 0xf;
	int dev3 = (reg_38 >> 4) & 0xf;

	// Find matching preset
	for (int i = 0; i < BOOT_ORDER_COUNT; i++) {
		if (boot_orders[i][0] == dev1 &&
		    boot_orders[i][1] == dev2 &&
		    boot_orders[i][2] == dev3)
			return i;
	}
	return 0;  // Default to first preset
}

// Helper to check if a line starts with a key
static int line_starts_with(const char *line, const char *key)
{
	size_t len = strlen(key);
	return strncmp(line, key, len) == 0 && (line[len] == ' ' || line[len] == '=');
}

// Save all settings to ini file
int save_settings_to_ini(const char *ini_path, int boot_order,
                         const char *hda,
                         const char *fda, const char *fdb,
                         const char *cda, const char *cdb,
                         const char *cdc, const char *cdd,
                         int cpu_gen, int fpu, long mem_size,
                         int brightness, int volume, int frame_skip,
                         int batch_size, int mouse_speed,
                         int usb_passthru)
{
	if (!ini_path) return -1;
	if (boot_order < 0 || boot_order >= BOOT_ORDER_COUNT)
		boot_order = 0;

	// Read existing file
	FILE *f = fopen(ini_path, "r");
	if (!f) return -1;

	// Read all lines into memory (heap allocated to avoid stack overflow)
	char (*lines)[256] = malloc(64 * 256);
	if (!lines) {
		fclose(f);
		return -1;
	}
	int line_count = 0;
	int in_pc_section = 0;
	int in_cpu_section = 0;
	int in_display_section = 0;
	int pc_section_end = -1;
	int cpu_section_end = -1;
	int display_section_end = -1;

	// Track which settings we found
	int found_boot_order = -1, found_hda = -1, found_fda = -1, found_fdb = -1;
	int found_cda = -1, found_cdb = -1, found_cdc = -1, found_cdd = -1;
	int found_mem_size = -1;
	int found_gen = -1, found_fpu = -1, found_batch_size = -1;
	int found_brightness = -1, found_volume = -1, found_frame_skip = -1;
	int found_mouse_speed = -1, found_usb_passthru = -1;

	while (line_count < 64 && fgets(lines[line_count], 256, f)) {
		// Check for section headers
		if (strncmp(lines[line_count], "[pc]", 4) == 0) {
			if (in_cpu_section && cpu_section_end < 0) cpu_section_end = line_count;
			if (in_display_section && display_section_end < 0) display_section_end = line_count;
			in_pc_section = 1;
			in_cpu_section = 0;
			in_display_section = 0;
		} else if (strncmp(lines[line_count], "[cpu]", 5) == 0) {
			if (in_pc_section && pc_section_end < 0) pc_section_end = line_count;
			if (in_display_section && display_section_end < 0) display_section_end = line_count;
			in_pc_section = 0;
			in_cpu_section = 1;
			in_display_section = 0;
		} else if (strncmp(lines[line_count], "[display]", 9) == 0) {
			if (in_pc_section && pc_section_end < 0) pc_section_end = line_count;
			if (in_cpu_section && cpu_section_end < 0) cpu_section_end = line_count;
			in_pc_section = 0;
			in_cpu_section = 0;
			in_display_section = 1;
		} else if (lines[line_count][0] == '[') {
			if (in_pc_section && pc_section_end < 0) pc_section_end = line_count;
			if (in_cpu_section && cpu_section_end < 0) cpu_section_end = line_count;
			if (in_display_section && display_section_end < 0) display_section_end = line_count;
			in_pc_section = 0;
			in_cpu_section = 0;
			in_display_section = 0;
		}

		// Check for existing settings in [pc] section
		if (in_pc_section) {
			if (line_starts_with(lines[line_count], "boot_order")) found_boot_order = line_count;
			else if (line_starts_with(lines[line_count], "hda")) found_hda = line_count;
			else if (line_starts_with(lines[line_count], "fda")) found_fda = line_count;
			else if (line_starts_with(lines[line_count], "fdb")) found_fdb = line_count;
			else if (line_starts_with(lines[line_count], "cda")) found_cda = line_count;
			else if (line_starts_with(lines[line_count], "cdb")) found_cdb = line_count;
			else if (line_starts_with(lines[line_count], "cdc")) found_cdc = line_count;
			else if (line_starts_with(lines[line_count], "cdd")) found_cdd = line_count;
			else if (line_starts_with(lines[line_count], "mem_size")) found_mem_size = line_count;
			else if (line_starts_with(lines[line_count], "brightness")) found_brightness = line_count;
			else if (line_starts_with(lines[line_count], "volume")) found_volume = line_count;
			else if (line_starts_with(lines[line_count], "frame_skip")) found_frame_skip = line_count;
			else if (line_starts_with(lines[line_count], "mouse_speed")) found_mouse_speed = line_count;
			else if (line_starts_with(lines[line_count], "usb_passthru")) found_usb_passthru = line_count;
		}

		// Check for existing settings in [cpu] section
		if (in_cpu_section) {
			if (line_starts_with(lines[line_count], "gen")) found_gen = line_count;
			else if (line_starts_with(lines[line_count], "fpu")) found_fpu = line_count;
			else if (line_starts_with(lines[line_count], "batch_size")) found_batch_size = line_count;
		}

		line_count++;
	}
	fclose(f);

	// If no section end found, it goes to end of file
	if (in_pc_section && pc_section_end < 0) {
		pc_section_end = line_count;
	}
	if (in_cpu_section && cpu_section_end < 0) {
		cpu_section_end = line_count;
	}
	if (in_display_section && display_section_end < 0) {
		display_section_end = line_count;
	}

	// Write back with updated settings
	f = fopen(ini_path, "w");
	if (!f) {
		free(lines);
		return -1;
	}

	for (int i = 0; i < line_count; i++) {
		// Replace existing lines
		if (i == found_boot_order) {
			fprintf(f, "boot_order = %s\n", boot_order_names[boot_order]);
		} else if (i == found_hda) {
			if (hda && hda[0]) fprintf(f, "hda = %s\n", hda);
			// else skip line (remove setting)
		} else if (i == found_fda) {
			if (fda && fda[0]) fprintf(f, "fda = %s\n", fda);
			// else skip line (remove setting)
		} else if (i == found_fdb) {
			if (fdb && fdb[0]) fprintf(f, "fdb = %s\n", fdb);
		} else if (i == found_cda) {
			if (cda && cda[0]) fprintf(f, "cda = %s\n", cda);
		} else if (i == found_cdb) {
			if (cdb && cdb[0]) fprintf(f, "cdb = %s\n", cdb);
		} else if (i == found_cdc) {
			if (cdc && cdc[0]) fprintf(f, "cdc = %s\n", cdc);
		} else if (i == found_cdd) {
			if (cdd && cdd[0]) fprintf(f, "cdd = %s\n", cdd);
		} else if (i == found_mem_size) {
			fprintf(f, "mem_size = %ldM\n", mem_size / (1024 * 1024));
		} else if (i == found_brightness) {
			fprintf(f, "brightness = %d\n", brightness);
		} else if (i == found_volume) {
			fprintf(f, "volume = %d\n", volume);
		} else if (i == found_frame_skip) {
			fprintf(f, "frame_skip = %d\n", frame_skip);
		} else if (i == found_mouse_speed) {
			fprintf(f, "mouse_speed = %d\n", mouse_speed);
		} else if (i == found_usb_passthru) {
			fprintf(f, "usb_passthru = %d\n", usb_passthru);
		} else if (i == found_gen) {
			fprintf(f, "gen = %d\n", cpu_gen);
		} else if (i == found_fpu) {
			fprintf(f, "fpu = %d\n", fpu);
		} else if (i == found_batch_size) {
			fprintf(f, "batch_size = %d\n", batch_size);
		} else {
			fputs(lines[i], f);
		}

		// Add new settings at end of [pc] section
		if (i == pc_section_end - 1) {
			if (found_boot_order < 0)
				fprintf(f, "boot_order = %s\n", boot_order_names[boot_order]);
			if (found_hda < 0 && hda && hda[0])
				fprintf(f, "hda = %s\n", hda);
			if (found_fda < 0 && fda && fda[0])
				fprintf(f, "fda = %s\n", fda);
			if (found_fdb < 0 && fdb && fdb[0])
				fprintf(f, "fdb = %s\n", fdb);
			if (found_cda < 0 && cda && cda[0])
				fprintf(f, "cda = %s\n", cda);
			if (found_cdb < 0 && cdb && cdb[0])
				fprintf(f, "cdb = %s\n", cdb);
			if (found_cdc < 0 && cdc && cdc[0])
				fprintf(f, "cdc = %s\n", cdc);
			if (found_cdd < 0 && cdd && cdd[0])
				fprintf(f, "cdd = %s\n", cdd);
			if (found_mem_size < 0) {
				fprintf(f, "mem_size = %ldM\n", mem_size / (1024 * 1024));
			}
			if (found_brightness < 0)
				fprintf(f, "brightness = %d\n", brightness);
			if (found_volume < 0)
				fprintf(f, "volume = %d\n", volume);
			if (found_frame_skip < 0)
				fprintf(f, "frame_skip = %d\n", frame_skip);
			if (found_mouse_speed < 0)
				fprintf(f, "mouse_speed = %d\n", mouse_speed);
			if (found_usb_passthru < 0)
				fprintf(f, "usb_passthru = %d\n", usb_passthru);
		}

		// Add new settings at end of [cpu] section
		if (i == cpu_section_end - 1) {
			if (found_gen < 0)
				fprintf(f, "gen = %d\n", cpu_gen);
			if (found_fpu < 0)
				fprintf(f, "fpu = %d\n", fpu);
			if (found_batch_size < 0)
				fprintf(f, "batch_size = %d\n", batch_size);
		}

	}

	/* Ensure INI is written to physical media before closing */
	fflush(f);
	fsync(fileno(f));

	fclose(f);
	free(lines);
	return 0;
}

struct fdfmt {
	uint8_t sectors;
	uint8_t tracks;
	uint8_t heads;
};

static const struct fdfmt fmt[] = {
	/* 1.44 MB 3.5 */
	{ 18, 80, 2 }, /* 3.5 2880 */
	{ 20, 80, 2 }, /* 3.5 3200 */
	{ 21, 80, 2 },
	{ 21, 82, 2 },
	{ 21, 83, 2 },
	{ 22, 80, 2 },
	{ 23, 80, 2 },
	{ 24, 80, 2 },
	/* 2.88 MB 3.5 */
	{ 36, 80, 2 },
	{ 39, 80, 2 },
	{ 40, 80, 2 },
	{ 44, 80, 2 },
	{ 48, 80, 2 },
	/* 720 KB 3.5 */
	{  9, 80, 2 }, /* 3.5 1440 */
	{ 10, 80, 2 },
	{ 10, 82, 2 },
	{ 10, 83, 2 },
	{ 13, 80, 2 },
	{ 14, 80, 2 },
	/* 1.2 MB 5.25 */
	{ 15, 80, 2 },
	{ 18, 80, 2 }, /* 5.25 2880 */
	{ 18, 82, 2 },
	{ 18, 83, 2 },
	{ 20, 80, 2 }, /* 5.25 3200 */
	/* 720 KB 5.25 */
	{  9, 80, 2 }, /* 5.25 1440 */
	{ 11, 80, 2 },
	/* 360 KB 5.25 */
	{  9, 40, 2 },
	{  9, 40, 1 },
	{ 10, 41, 2 },
	{ 10, 42, 2 },
	/* 320 KB 5.25 */
	{  8, 40, 2 },
	{  8, 40, 1 },
	{  0,  0, 0 },
};

struct EMULINK {
	uint32_t status;
	uint32_t cmd;
	uint32_t args[4];
	int argi;
	int dataleft;

	// fake floppy drive
	FILE *fdd[2];
	int fdfmti[2];
	char fdd_path[2][256];  // Store filenames for OSD display
};

EMULINK *emulink_init()
{
	EMULINK *e = malloc(sizeof(EMULINK));
	memset(e, 0, sizeof(EMULINK));
	e->cmd = -1;
	return e;
}

void emulink_close(EMULINK *e)
{
	if (!e) return;
	for (int i = 0; i < 2; i++) {
		if (e->fdd[i]) {
			fclose(e->fdd[i]);
			e->fdd[i] = NULL;
		}
		e->fdd_path[i][0] = '\0';
	}
}

int emulink_attach_floppy(EMULINK *e, int i, const char *filename)
{
	if (i >= 0 && i < 2) {
		if (e->fdd[i]) {
			fclose(e->fdd[i]);
			e->fdd[i] = NULL;
		}
		e->fdd_path[i][0] = '\0';
		if (filename) {
			e->fdd[i] = fopen(filename, "r+b");
			if (e->fdd[i]) {
				strncpy(e->fdd_path[i], filename, sizeof(e->fdd_path[i]) - 1);
				e->fdd_path[i][sizeof(e->fdd_path[i]) - 1] = '\0';
			}
		}
		if (!e->fdd[i])
			return -1;
		fseek(e->fdd[i], 0, SEEK_END);
		int size = ftell(e->fdd[i]);
		for (int j = 0; fmt[j].sectors; j++) {
			if (size ==
			    fmt[j].sectors * fmt[j].tracks * fmt[j].heads * 512) {
				e->fdfmti[i] = j;
				return 0;
			}
		}
		return -1;
	}
	return 0;
}

const char *emulink_get_floppy_path(EMULINK *e, int i)
{
	if (i >= 0 && i < 2)
		return e->fdd_path[i];
	return NULL;
}

uint32_t emulink_status_read(void *s)
{
	EMULINK *e = s;
	return e->status;
}

static void exec_cmd(EMULINK *e)
{
	switch (e->cmd) {
	case 0:
		e->status = 0xaa55ff00;
		e->cmd = -1;
		break;
	case 0x100:
		// Always report floppy drives as present (even if no disk mounted)
		// This allows hot-mounting floppies at runtime via OSD
		// Bit 0x40 = drive 0 present, Bit 0x04 = drive 1 present
		e->status = 0x40 | 0x04;
		e->cmd = -1;
		break;
	case 0x101:
	case 0x102:
		if (e->argi == 3) {
			int ret;
			if (!e->fdd[e->args[0]]) {
				e->status = -EIO;
				e->cmd = -1;
				break;
			}
			if (e->args[0] < 2) {
				int c = e->args[1] >> 16;
				int h = (e->args[1] >> 8) & 0xff;
				int s = e->args[1] & 0xff;
				const struct fdfmt *f = &(fmt[e->fdfmti[e->args[0]]]);
				int lba = (c * f->heads + h) * f->sectors + (s - 1);
				ret = fseek(e->fdd[e->args[0]], 512 * lba, SEEK_SET);
				if (ret < 0) {
					e->status = -errno;
					e->cmd = -1;
				} else {
					e->status = 0;
					e->dataleft = 512 * e->args[2];
				}
			}
		}
		break;
	case -1:
		break;
	}
}

void emulink_cmd_write(void *s, uint32_t val)
{
	EMULINK *e = s;
	e->cmd = val;
	e->argi = 0;
	exec_cmd(e);
}

void emulink_data_write(void *s, uint32_t val)
{
	EMULINK *e = s;
	if (e->argi < 4) {
		e->args[e->argi++] = val;
	}
	exec_cmd(e);
}

int emulink_data_write_string(void *s, uint8_t *buf, int size, int count)
{
	EMULINK *e = s;
	switch (e->cmd) {
	case 0x102: // floppy write
		if (!e->fdd[e->args[0]])
			break;
		if (e->argi == 3) {
			int len = size * count;
			if (len > e->dataleft)
				break;
			FLOPPY_ACTIVITY();
			int ret = fwrite(buf, 1, len, e->fdd[e->args[0]]);
			if (ret != len)
				break;
			e->dataleft -= len;
			if (e->dataleft == 0) {
				e->cmd = -1;
				e->status = 0;
				return count;
			}
			return count;
		}
		break;
	}
	e->cmd = -1;
	e->status = -1;
	return count;
}

int emulink_data_read_string(void *s, uint8_t *buf, int size, int count)
{
	EMULINK *e = s;
	switch (e->cmd) {
	case 0x101: // floppy read
		if (!e->fdd[e->args[0]])
			break;
		if (e->argi == 3) {
			int len = size * count;
			if (len > e->dataleft)
				break;
			FLOPPY_ACTIVITY();
			int ret = fread(buf, 1, len, e->fdd[e->args[0]]);
			if (ret != len)
				break;
			e->dataleft -= len;
			if (e->dataleft == 0) {
				e->cmd = -1;
				e->status = 0;
				return count;
			}
			return count;
		}
		break;
	}
	e->cmd = -1;
	e->status = -1;
	return count;
}
