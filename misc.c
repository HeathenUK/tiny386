#include "misc.h"
#include "ide.h"
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
		/* Extended memory in KB above 1MB, capped at 63MB (0xFC00 KB).
		 * CMOS 0x17/0x18 = POST-detected value (read by DOS extenders).
		 * CMOS 0x30/0x31 = user-configured value (read by BIOS setup).
		 * Both must be set — DOSX reads 0x17/0x18 for memory detection. */
		long ext_kb = (mem_size - 1024 * 1024) / 1024;
		if (ext_kb > 0xFC00) ext_kb = 0xFC00;
		c->data[0x17] = ext_kb & 0xFF;
		c->data[0x18] = (ext_kb >> 8) & 0xFF;
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

uint32_t cmos_next_irq_us(CMOS *s)
{
	/* If periodic interrupt is not enabled, no RTC wake source. */
	if (!(s->data[RTC_REG_B] & REG_B_PIE) || s->irq_period == 0)
		return UINT32_MAX;

	uint32_t now = cmos_get_timer(s);
	int32_t remaining = (int32_t)(s->irq_timeout - now);
	if (remaining <= 0)
		return 0;  /* IRQ already due */

	/* Convert 32768 Hz ticks to microseconds */
	return (uint64_t)(uint32_t)remaining * 1000000 / CMOS_FREQ;
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
	while (*line == ' ' || *line == '\t')
		line++;
	size_t len = strlen(key);
	return strncmp(line, key, len) == 0 &&
	       (line[len] == ' ' || line[len] == '=' || line[len] == '\t');
}

// Save all settings to ini file
int save_settings_to_ini(const char *ini_path, int boot_order,
                         const char *hda,
                         const char *fda, const char *fdb,
                         const char *cda, const char *cdb,
                         const char *cdc, const char *cdd,
                         int cpu_gen, int fpu, long mem_size,
                         int brightness, int volume, int frame_skip,
                         int batch_size, int pit_burst, int mouse_speed,
                         int usb_passthru, int accuracy)
{
	if (!ini_path) return -1;
	if (boot_order < 0 || boot_order >= BOOT_ORDER_COUNT)
		boot_order = 0;
	if (brightness < 0) brightness = 0;
	if (brightness > 110) brightness = 110;

	// Read existing file
	FILE *f = fopen(ini_path, "r");
	if (!f) return -1;

	// Read all lines into memory (dynamic; do not truncate large INI files)
	char **lines = NULL;
	int line_cap = 0;
	int line_count = 0;
	char readbuf[1024];
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
	int found_gen = -1, found_fpu = -1, found_batch_size = -1, found_pit_burst = -1, found_accuracy = -1;
	int found_brightness = -1, found_volume = -1, found_frame_skip = -1;
	int found_mouse_speed = -1, found_usb_passthru = -1;

	while (fgets(readbuf, sizeof(readbuf), f)) {
		if (line_count >= line_cap) {
			int new_cap = line_cap ? line_cap * 2 : 128;
			char **new_lines = realloc(lines, (size_t)new_cap * sizeof(*new_lines));
			if (!new_lines) {
				for (int i = 0; i < line_count; i++) free(lines[i]);
				free(lines);
				fclose(f);
				return -1;
			}
			lines = new_lines;
			line_cap = new_cap;
		}

		lines[line_count] = strdup(readbuf);
		if (!lines[line_count]) {
			for (int i = 0; i < line_count; i++) free(lines[i]);
			free(lines);
			fclose(f);
			return -1;
		}

		const char *line = lines[line_count];
		while (*line == ' ' || *line == '\t') line++;

		// Check for section headers
		if (strncmp(line, "[pc]", 4) == 0) {
			if (in_cpu_section && cpu_section_end < 0) cpu_section_end = line_count;
			if (in_display_section && display_section_end < 0) display_section_end = line_count;
			in_pc_section = 1;
			in_cpu_section = 0;
			in_display_section = 0;
		} else if (strncmp(line, "[cpu]", 5) == 0) {
			if (in_pc_section && pc_section_end < 0) pc_section_end = line_count;
			if (in_display_section && display_section_end < 0) display_section_end = line_count;
			in_pc_section = 0;
			in_cpu_section = 1;
			in_display_section = 0;
		} else if (strncmp(line, "[display]", 9) == 0) {
			if (in_pc_section && pc_section_end < 0) pc_section_end = line_count;
			if (in_cpu_section && cpu_section_end < 0) cpu_section_end = line_count;
			in_pc_section = 0;
			in_cpu_section = 0;
			in_display_section = 1;
		} else if (line[0] == '[') {
			if (in_pc_section && pc_section_end < 0) pc_section_end = line_count;
			if (in_cpu_section && cpu_section_end < 0) cpu_section_end = line_count;
			if (in_display_section && display_section_end < 0) display_section_end = line_count;
			in_pc_section = 0;
			in_cpu_section = 0;
			in_display_section = 0;
		}

		// Check for existing settings in [pc] section
		if (in_pc_section) {
			if (line_starts_with(line, "boot_order")) found_boot_order = line_count;
			else if (line_starts_with(line, "hda")) found_hda = line_count;
			else if (line_starts_with(line, "fda")) found_fda = line_count;
			else if (line_starts_with(line, "fdb")) found_fdb = line_count;
			else if (line_starts_with(line, "cda")) found_cda = line_count;
			else if (line_starts_with(line, "cdb")) found_cdb = line_count;
			else if (line_starts_with(line, "cdc")) found_cdc = line_count;
			else if (line_starts_with(line, "cdd")) found_cdd = line_count;
			else if (line_starts_with(line, "mem_size")) found_mem_size = line_count;
			else if (line_starts_with(line, "brightness")) found_brightness = line_count;
			else if (line_starts_with(line, "volume")) found_volume = line_count;
			else if (line_starts_with(line, "frame_skip")) found_frame_skip = line_count;
			else if (line_starts_with(line, "mouse_speed")) found_mouse_speed = line_count;
			else if (line_starts_with(line, "usb_passthru")) found_usb_passthru = line_count;
		}

		// Check for existing settings in [cpu] section
		if (in_cpu_section) {
			if (line_starts_with(line, "gen")) found_gen = line_count;
			else if (line_starts_with(line, "fpu")) found_fpu = line_count;
			else if (line_starts_with(line, "batch_size")) found_batch_size = line_count;
			else if (line_starts_with(line, "pit_burst")) found_pit_burst = line_count;
			else if (line_starts_with(line, "accuracy")) found_accuracy = line_count;
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
		for (int i = 0; i < line_count; i++) free(lines[i]);
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
			} else if (i == found_pit_burst) {
				fprintf(f, "pit_burst = %d\n", pit_burst ? 1 : 0);
		} else if (i == found_accuracy) {
				fprintf(f, "accuracy = %s\n", accuracy ? "fast" : "full");
			} else {
				fputs(lines[i] ? lines[i] : "", f);
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
				if (found_pit_burst < 0)
					fprintf(f, "pit_burst = %d\n", pit_burst ? 1 : 0);
				if (found_accuracy < 0)
					fprintf(f, "accuracy = %s\n", accuracy ? "fast" : "full");
			}

	}

	// If section missing, append it so settings persist.
	if (pc_section_end < 0) {
		fprintf(f, "\n[pc]\n");
		fprintf(f, "boot_order = %s\n", boot_order_names[boot_order]);
		if (hda && hda[0]) fprintf(f, "hda = %s\n", hda);
		if (fda && fda[0]) fprintf(f, "fda = %s\n", fda);
		if (fdb && fdb[0]) fprintf(f, "fdb = %s\n", fdb);
		if (cda && cda[0]) fprintf(f, "cda = %s\n", cda);
		if (cdb && cdb[0]) fprintf(f, "cdb = %s\n", cdb);
		if (cdc && cdc[0]) fprintf(f, "cdc = %s\n", cdc);
		if (cdd && cdd[0]) fprintf(f, "cdd = %s\n", cdd);
		fprintf(f, "mem_size = %ldM\n", mem_size / (1024 * 1024));
		fprintf(f, "brightness = %d\n", brightness);
		fprintf(f, "volume = %d\n", volume);
		fprintf(f, "frame_skip = %d\n", frame_skip);
		fprintf(f, "mouse_speed = %d\n", mouse_speed);
		fprintf(f, "usb_passthru = %d\n", usb_passthru);
	}

	if (cpu_section_end < 0) {
		fprintf(f, "\n[cpu]\n");
		fprintf(f, "gen = %d\n", cpu_gen);
		fprintf(f, "fpu = %d\n", fpu);
		fprintf(f, "batch_size = %d\n", batch_size);
		fprintf(f, "pit_burst = %d\n", pit_burst ? 1 : 0);
		fprintf(f, "accuracy = %s\n", accuracy ? "fast" : "full");
	}

	/* Ensure INI is written to physical media before closing */
	fflush(f);
	fsync(fileno(f));

	fclose(f);
	for (int i = 0; i < line_count; i++) free(lines[i]);
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
	BlockDevice *fdd_bs[2];
	int fdfmti[2];
	uint8_t *fdd_buf[2];     // Optional in-memory floppy image cache
	int fdd_size[2];
	int fdd_dirty[2];
	char fdd_path[2][256];  // Store filenames for OSD display
	int dataoff;
};

static int emulink_reopen_floppy(EMULINK *e, int i)
{
	if (!e || i < 0 || i >= 2)
		return -1;
	if (e->fdd_path[i][0] == '\0')
		return -1;
	if (e->fdd[i]) {
		fclose(e->fdd[i]);
		e->fdd[i] = NULL;
	}
	e->fdd[i] = fopen(e->fdd_path[i], "r+b");
	return e->fdd[i] ? 0 : -1;
}

static int emulink_reopen_floppy_block(EMULINK *e, int i)
{
	if (!e || i < 0 || i >= 2)
		return -1;
	if (e->fdd_path[i][0] == '\0')
		return -1;
	if (e->fdd_bs[i]) {
		ide_block_close(e->fdd_bs[i]);
		e->fdd_bs[i] = NULL;
	}
	e->fdd_bs[i] = ide_block_open_rw(e->fdd_path[i]);
	return e->fdd_bs[i] ? 0 : -1;
}

static int emulink_block_read_bytes(EMULINK *e, int drive, int offset, uint8_t *buf, int len)
{
	if (!e || drive < 0 || drive >= 2 || !buf || len < 0 || !e->fdd_bs[drive])
		return -1;

	while (len > 0) {
		uint64_t sector = (uint64_t)offset >> 9;
		int sector_off = offset & 511;

		if (sector_off == 0 && len >= 512) {
			int nsec = len >> 9;
			if (nsec > 64)
				nsec = 64;
			if (ide_block_read(e->fdd_bs[drive], sector, buf, nsec) < 0)
				return -1;
			int adv = nsec << 9;
			offset += adv;
			buf += adv;
			len -= adv;
			continue;
		}

		uint8_t tmp[512];
		if (ide_block_read(e->fdd_bs[drive], sector, tmp, 1) < 0)
			return -1;
		int chunk = 512 - sector_off;
		if (chunk > len)
			chunk = len;
		memcpy(buf, tmp + sector_off, chunk);
		offset += chunk;
		buf += chunk;
		len -= chunk;
	}

	return 0;
}

static int emulink_block_write_bytes(EMULINK *e, int drive, int offset, const uint8_t *buf, int len)
{
	if (!e || drive < 0 || drive >= 2 || !buf || len < 0 || !e->fdd_bs[drive])
		return -1;

	while (len > 0) {
		uint64_t sector = (uint64_t)offset >> 9;
		int sector_off = offset & 511;

		if (sector_off == 0 && len >= 512) {
			int nsec = len >> 9;
			if (nsec > 64)
				nsec = 64;
			if (ide_block_write(e->fdd_bs[drive], sector, buf, nsec) < 0)
				return -1;
			int adv = nsec << 9;
			offset += adv;
			buf += adv;
			len -= adv;
			continue;
		}

		uint8_t tmp[512];
		if (ide_block_read(e->fdd_bs[drive], sector, tmp, 1) < 0)
			return -1;
		int chunk = 512 - sector_off;
		if (chunk > len)
			chunk = len;
		memcpy(tmp + sector_off, buf, chunk);
		if (ide_block_write(e->fdd_bs[drive], sector, tmp, 1) < 0)
			return -1;
		offset += chunk;
		buf += chunk;
		len -= chunk;
	}

	return 0;
}

static void emulink_free_floppy_buffer(EMULINK *e, int i)
{
	if (!e || i < 0 || i >= 2)
		return;
	if (e->fdd_buf[i]) {
		free(e->fdd_buf[i]);
		e->fdd_buf[i] = NULL;
	}
	e->fdd_size[i] = 0;
	e->fdd_dirty[i] = 0;
}

static int emulink_cache_load_from_block(EMULINK *e, int i, int size)
{
	if (!e || i < 0 || i >= 2 || !e->fdd_buf[i] || !e->fdd_bs[i] || size <= 0)
		return -1;
	int total = size / 512;
	int done = 0;
	while (done < total) {
		int chunk = total - done;
		if (chunk > 64)
			chunk = 64;
		if (ide_block_read(e->fdd_bs[i], done, e->fdd_buf[i] + done * 512, chunk) < 0)
			return -1;
		done += chunk;
	}
	return 0;
}

static void emulink_flush_floppy_buffer(EMULINK *e, int i)
{
	if (!e || i < 0 || i >= 2)
		return;
	if (!e->fdd_buf[i] || !e->fdd_dirty[i] || e->fdd_size[i] <= 0)
		return;
	if (e->fdd_bs[i]) {
		int total = e->fdd_size[i] / 512;
		int done = 0;
		while (done < total) {
			int chunk = total - done;
			if (chunk > 64)
				chunk = 64;
			if (ide_block_write(e->fdd_bs[i], done,
					    e->fdd_buf[i] + done * 512, chunk) < 0)
				return;
			done += chunk;
		}
		e->fdd_dirty[i] = 0;
		return;
	}
	if (!e->fdd[i])
		return;
	clearerr(e->fdd[i]);
	if (fseek(e->fdd[i], 0, SEEK_SET) != 0)
		return;
	if ((int)fwrite(e->fdd_buf[i], 1, e->fdd_size[i], e->fdd[i]) != e->fdd_size[i])
		return;
	fflush(e->fdd[i]);
	fsync(fileno(e->fdd[i]));
	e->fdd_dirty[i] = 0;
}

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
		emulink_flush_floppy_buffer(e, i);
		if (e->fdd[i]) {
			fclose(e->fdd[i]);
			e->fdd[i] = NULL;
		}
		if (e->fdd_bs[i]) {
			ide_block_close(e->fdd_bs[i]);
			e->fdd_bs[i] = NULL;
		}
		emulink_free_floppy_buffer(e, i);
		e->fdd_path[i][0] = '\0';
	}
}

int emulink_attach_floppy(EMULINK *e, int i, const char *filename)
{
	if (i >= 0 && i < 2) {
		/* Drop any in-flight floppy command state on media change. */
		e->cmd = -1;
		e->argi = 0;
		e->dataleft = 0;
		e->status = 0;
		e->dataoff = 0;

		emulink_flush_floppy_buffer(e, i);
		if (e->fdd[i]) {
			fclose(e->fdd[i]);
			e->fdd[i] = NULL;
		}
		if (e->fdd_bs[i]) {
			ide_block_close(e->fdd_bs[i]);
			e->fdd_bs[i] = NULL;
		}
		emulink_free_floppy_buffer(e, i);
		if (filename && filename[0]) {
			/* Uses the same raw-SD accelerated backend as IDE/CD when available. */
			e->fdd_bs[i] = ide_block_open_rw(filename);
		} else {
			e->fdd_bs[i] = NULL;
		}
		e->fdd_path[i][0] = '\0';
		if (filename) {
			e->fdd[i] = fopen(filename, "r+b");
			if (e->fdd[i]) {
				strncpy(e->fdd_path[i], filename, sizeof(e->fdd_path[i]) - 1);
				e->fdd_path[i][sizeof(e->fdd_path[i]) - 1] = '\0';
			}
		}
		/* Eject is valid and intentional. */
			if (!filename)
				return 0;
			if (!e->fdd[i] && !e->fdd_bs[i])
				return -1;
			int size = -1;
			if (e->fdd_bs[i]) {
				int64_t sc = ide_block_sector_count(e->fdd_bs[i]);
				if (sc > 0)
					size = (int)(sc * 512);
			}
			if (size <= 0 && e->fdd[i]) {
				fseek(e->fdd[i], 0, SEEK_END);
				size = ftell(e->fdd[i]);
			}
			for (int j = 0; fmt[j].sectors; j++) {
				if (size ==
				    fmt[j].sectors * fmt[j].tracks * fmt[j].heads * 512) {
				e->fdfmti[i] = j;
					e->fdd_buf[i] = malloc(size);
					if (e->fdd_buf[i]) {
						int load_ok = -1;
						if (e->fdd_bs[i]) {
							load_ok = emulink_cache_load_from_block(e, i, size);
						}
						if (load_ok < 0 && e->fdd[i]) {
							clearerr(e->fdd[i]);
							if (fseek(e->fdd[i], 0, SEEK_SET) == 0 &&
							    (int)fread(e->fdd_buf[i], 1, size, e->fdd[i]) == size)
								load_ok = 0;
						}
						if (load_ok == 0) {
							e->fdd_size[i] = size;
							e->fdd_dirty[i] = 0;
							fprintf(stderr, "floppy%d: cache ready (%d KB, source=%s)\n",
							        i, size / 1024, e->fdd_bs[i] ? "block" : "stdio");
						} else {
							fprintf(stderr, "floppy%d: cache load failed (block=%d stdio=%d)\n",
							        i, e->fdd_bs[i] ? 1 : 0, e->fdd[i] ? 1 : 0);
							emulink_free_floppy_buffer(e, i);
						}
					}
					return 0;
				}
			}
			if (e->fdd[i]) {
				fclose(e->fdd[i]);
				e->fdd[i] = NULL;
			}
			e->fdd_path[i][0] = '\0';
			if (e->fdd_bs[i]) {
				ide_block_close(e->fdd_bs[i]);
				e->fdd_bs[i] = NULL;
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
				int drive = e->args[0];
				int ret;
				if (drive < 0 || drive >= 2 || (!e->fdd[drive] && !e->fdd_bs[drive])) {
					e->status = -EIO;
					e->cmd = -1;
					break;
				}
			int c = e->args[1] >> 16;
			int h = (e->args[1] >> 8) & 0xff;
			int s = e->args[1] & 0xff;
			const struct fdfmt *f = &(fmt[e->fdfmti[drive]]);
			if (s < 1 || s > f->sectors || h >= f->heads || c >= f->tracks) {
				e->status = -EIO;
				e->cmd = -1;
				break;
			}
				int lba = (c * f->heads + h) * f->sectors + (s - 1);
				e->dataoff = 512 * lba;
					if (e->fdd_buf[drive] || e->fdd_bs[drive]) {
						e->status = 0;
						e->dataleft = 512 * e->args[2];
						break;
					}
				if (!e->fdd[drive]) {
					e->status = -EIO;
					e->cmd = -1;
					break;
				}
				clearerr(e->fdd[drive]);
				ret = fseek(e->fdd[drive], e->dataoff, SEEK_SET);
			if (ret != 0) {
				/* Some hosts recover after reopening the image (same path). */
				if (emulink_reopen_floppy(e, drive) == 0) {
					clearerr(e->fdd[drive]);
					ret = fseek(e->fdd[drive], e->dataoff, SEEK_SET);
				}
			}
			if (ret != 0) {
				e->status = -errno;
				e->cmd = -1;
			} else {
				e->status = 0;
				e->dataleft = 512 * e->args[2];
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
		if (e->args[0] >= 2 || (!e->fdd[e->args[0]] && !e->fdd_bs[e->args[0]]))
			break;
		if (e->argi == 3) {
			int drive = e->args[0];
			int len = size * count;
			if (len > e->dataleft)
				break;
			FLOPPY_ACTIVITY();
			if (e->fdd_buf[drive] &&
			    e->dataoff >= 0 &&
			    e->dataoff + len <= e->fdd_size[drive]) {
				memcpy(e->fdd_buf[drive] + e->dataoff, buf, len);
				e->dataoff += len;
				e->fdd_dirty[drive] = 1;
				e->dataleft -= len;
				if (e->dataleft == 0) {
					emulink_flush_floppy_buffer(e, drive);
					e->cmd = -1;
					e->status = 0;
					return count;
				}
				return count;
			}
			if (e->fdd_bs[drive]) {
				int ret = emulink_block_write_bytes(e, drive, e->dataoff, buf, len);
				if (ret < 0 && emulink_reopen_floppy_block(e, drive) == 0)
					ret = emulink_block_write_bytes(e, drive, e->dataoff, buf, len);
				if (ret < 0)
					break;
				e->dataoff += len;
				e->dataleft -= len;
				if (e->dataleft == 0) {
					e->cmd = -1;
					e->status = 0;
					return count;
				}
				return count;
			}
			if (!e->fdd[drive])
				break;
			long pos = ftell(e->fdd[drive]);
			clearerr(e->fdd[drive]);
			int ret = fwrite(buf, 1, len, e->fdd[drive]);
			if (ret != len && emulink_reopen_floppy(e, drive) == 0) {
				clearerr(e->fdd[drive]);
				if (pos >= 0 && fseek(e->fdd[drive], pos, SEEK_SET) == 0)
					ret = fwrite(buf, 1, len, e->fdd[drive]);
			}
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
		if (e->args[0] >= 2 || (!e->fdd[e->args[0]] && !e->fdd_bs[e->args[0]]))
			break;
		if (e->argi == 3) {
			int drive = e->args[0];
			int len = size * count;
			if (len > e->dataleft)
				break;
			FLOPPY_ACTIVITY();
			if (e->fdd_buf[drive] &&
			    e->dataoff >= 0 &&
			    e->dataoff + len <= e->fdd_size[drive]) {
				memcpy(buf, e->fdd_buf[drive] + e->dataoff, len);
				e->dataoff += len;
				e->dataleft -= len;
				if (e->dataleft == 0) {
					e->cmd = -1;
					e->status = 0;
					return count;
				}
				return count;
			}
			if (e->fdd_bs[drive]) {
				int ret = emulink_block_read_bytes(e, drive, e->dataoff, buf, len);
				if (ret < 0 && emulink_reopen_floppy_block(e, drive) == 0)
					ret = emulink_block_read_bytes(e, drive, e->dataoff, buf, len);
				if (ret < 0)
					break;
				e->dataoff += len;
				e->dataleft -= len;
				if (e->dataleft == 0) {
					e->cmd = -1;
					e->status = 0;
					return count;
				}
				return count;
			}
			if (!e->fdd[drive])
				break;
			long pos = ftell(e->fdd[drive]);
			clearerr(e->fdd[drive]);
			int ret = fread(buf, 1, len, e->fdd[drive]);
			if (ret != len && emulink_reopen_floppy(e, drive) == 0) {
				clearerr(e->fdd[drive]);
				if (pos >= 0 && fseek(e->fdd[drive], pos, SEEK_SET) == 0)
					ret = fread(buf, 1, len, e->fdd[drive]);
			}
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
