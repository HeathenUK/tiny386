/*
 * Tanmatsu-specific OSD: Custom keyboard-driven menu with file browser
 * For devices without mouse/touch input
 */

#include "tanmatsu_osd.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "../../vga.h"  // For vga_frame_skip_max

// VGA 8x16 bitmap font for clean text rendering
#include "vga_font_8x16.h"

#define FONT_WIDTH 8
#define FONT_HEIGHT 16
#include <dirent.h>
#include <sys/stat.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common.h"
#include "bsp/display.h"
#include "bsp/audio.h"

#include "../../vga.h"
#include "../../misc.h"
#include "../../ide.h"
#include "toast.h"
#include "mouse_emu.h"
#include "../../pc.h"

// View modes
typedef enum {
	VIEW_MAIN_MENU,
	VIEW_MOUNTING,
	VIEW_AUDIOVISUAL,
	VIEW_SYSTEM,
	VIEW_STATUS,
	VIEW_HELP,
	VIEW_FILEBROWSER
} ViewMode;

// Main menu items
typedef enum {
	MAIN_MOUNTING = 0,
	MAIN_AUDIOVISUAL,
	MAIN_SYSTEM,
	MAIN_STATUS,
	MAIN_HELP,
	MAIN_SEP1,
	MAIN_BOOT_ORDER,
	MAIN_SAVE_INI,
	MAIN_SEP2,
	MAIN_CTRLALTDEL,
	MAIN_RESET,
	MAIN_SEP3,
	MAIN_EXIT,
	MAIN_COUNT
} MainMenuItem;

// Mounting submenu items
typedef enum {
	MOUNT_FDA = 0,
	MOUNT_FDB,
	MOUNT_SEP1,
	MOUNT_CDA,
	MOUNT_CDB,
	MOUNT_CDC,
	MOUNT_CDD,
	MOUNT_SEP2,
	MOUNT_BACK,
	MOUNT_COUNT
} MountMenuItem;

// Audio/Visual submenu items
typedef enum {
	AV_BRIGHTNESS = 0,
	AV_VOLUME,
	AV_FRAME_SKIP,
	AV_DOUBLE_BUFFER,
	AV_SEP1,
	AV_BACK,
	AV_COUNT
} AVMenuItem;

// System Settings submenu items
typedef enum {
	SYS_CPU_GEN = 0,
	SYS_FPU,
	SYS_MEMSIZE,
	SYS_BATCH,
	SYS_MOUSE_SPEED,
	SYS_SEP1,
	SYS_CREATE_HDD,
	SYS_SEP2,
	SYS_BACK,
	SYS_COUNT
} SysMenuItem;

// HDD size presets (in MB) - IDE limit is ~504MB without LBA, 2GB practical max
static const int hdd_size_presets[] = { 104, 256, 504, 1024, 2048 };
static const int hdd_size_preset_count = 5;

// File entry for browser
#define MAX_FILES 256
#define MAX_FILENAME 64
#define MAX_PATH_LEN 256

typedef struct {
	char name[MAX_FILENAME];
	int is_dir;  // 1=directory, 0=file, -1=special (eject)
} FileEntry;

// Special file entry type
#define FILE_TYPE_EJECT -1

struct OSD {
	EMULINK *emulink;
	IDEIFState *ide, *ide2;
	PC *pc;  // For CMOS access (boot order)
	void *console;

	// Current view
	ViewMode view;

	// Menu selection state
	int main_sel;
	int mount_sel;
	int av_sel;
	int sys_sel;
	int hdd_size_sel;  // Index into hdd_size_presets for HDD creation

	// Drive paths
	char drive_paths[6][MAX_PATH_LEN];  // FDA, FDB, CDA-CDD

	// Audio/Visual settings
	int brightness;  // 0-100
	int volume;      // 0-100
	int frame_skip;  // 0-10 max frames to skip
	int double_buffer; // 0=disabled, 1=enabled

	// System settings (require restart to take effect)
	int cpu_gen;     // 3=386, 4=486, 5=586
	int fpu;         // 0=disabled, 1=enabled
	int mem_size_mb; // Memory in MB (1 to 24)
	int batch_size;  // 0=auto, or fixed value (512-4096 in 256 increments)
	int mouse_speed; // 1-10, mouse emulation speed

	// File browser state
	int browser_target;  // Which drive slot we're browsing for
	char browser_path[MAX_PATH_LEN];
	FileEntry files[MAX_FILES];
	int file_count;
	int file_sel;
	int file_scroll;
	int files_visible;
};

// Scancode defines
#define SC_UP    0x48
#define SC_DOWN  0x50
#define SC_LEFT  0x4B
#define SC_RIGHT 0x4D
#define SC_ENTER 0x1c
#define SC_ESC   0x01
#define SC_BACKSPACE 0x0e

// Colors (RGB565) - clean color scheme inspired by trackmatsu
#define COLOR_BG      0x0000  // Black
#define COLOR_PANEL   0x18E3  // Dark blue-gray panel background
#define COLOR_BORDER  0x4A69  // Medium gray border
#define COLOR_TEXT    0xFFFF  // White
#define COLOR_HILITE  0x04FF  // Bright blue highlight
#define COLOR_SEP     0x39E7  // Medium gray separator
#define COLOR_DIR     0x07E0  // Green for directories
#define COLOR_TITLE   0xFFE0  // Yellow for titles
#define COLOR_EJECT   0xFB20  // Orange-red for eject
#define COLOR_VALUE   0x87FF  // Light cyan for values
#define COLOR_DIM     0x8410  // Dim gray for inactive items
#define COLOR_MUTED   0x6B4D  // Muted gray for help text

// Forward declarations
static void populate_drive_paths(OSD *osd);
static void scan_directory(OSD *osd);

// Separator checks for each menu
static int is_main_separator(int item) {
	return item == MAIN_SEP1 || item == MAIN_SEP2 || item == MAIN_SEP3;
}

static int is_mount_separator(int item) {
	return item == MOUNT_SEP1 || item == MOUNT_SEP2;
}

static int is_av_separator(int item) {
	return item == AV_SEP1;
}

static int is_sys_separator(int item) {
	return item == SYS_SEP1 || item == SYS_SEP2;
}

// Get just the filename from a path
static const char *basename_ptr(const char *path)
{
	if (!path || !path[0]) return "[empty]";
	const char *p = strrchr(path, '/');
	return p ? p + 1 : path;
}

// Populate drive paths from current state
static void populate_drive_paths(OSD *osd)
{
	// Floppy drives
	for (int i = 0; i < 2; i++) {
		const char *path = osd->emulink ? emulink_get_floppy_path(osd->emulink, i) : NULL;
		if (path && path[0]) {
			strncpy(osd->drive_paths[i], path, MAX_PATH_LEN - 1);
			osd->drive_paths[i][MAX_PATH_LEN - 1] = '\0';
		} else {
			osd->drive_paths[i][0] = '\0';
		}
	}
	// CD drives
	for (int i = 0; i < 4; i++) {
		IDEIFState *ide = (i < 2) ? osd->ide : osd->ide2;
		const char *path = ide ? ide_get_cd_path(ide, i % 2) : NULL;
		if (path && path[0]) {
			strncpy(osd->drive_paths[2 + i], path, MAX_PATH_LEN - 1);
			osd->drive_paths[2 + i][MAX_PATH_LEN - 1] = '\0';
		} else {
			osd->drive_paths[2 + i][0] = '\0';
		}
	}
}

// Directory scanning for file browser
static int compare_files(const void *a, const void *b)
{
	const FileEntry *fa = (const FileEntry *)a;
	const FileEntry *fb = (const FileEntry *)b;
	// Directories first, then alphabetical
	if (fa->is_dir != fb->is_dir) {
		return fb->is_dir - fa->is_dir;
	}
	return strcasecmp(fa->name, fb->name);
}

static void scan_directory(OSD *osd)
{
	osd->file_count = 0;
	osd->file_sel = 0;
	osd->file_scroll = 0;

	// Add "eject" option at top for drives that support it
	if (osd->browser_target >= 0 && osd->browser_target < 6) {
		strcpy(osd->files[osd->file_count].name, "[Eject]");
		osd->files[osd->file_count].is_dir = FILE_TYPE_EJECT;
		osd->file_count++;
	}

	// Add parent directory option
	if (strlen(osd->browser_path) > 1) {
		strcpy(osd->files[osd->file_count].name, "..");
		osd->files[osd->file_count].is_dir = 1;
		osd->file_count++;
	}

	DIR *dir = opendir(osd->browser_path);
	if (!dir) return;

	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL && osd->file_count < MAX_FILES) {
		// Skip hidden files and . / ..
		if (ent->d_name[0] == '.') continue;

		strncpy(osd->files[osd->file_count].name, ent->d_name, MAX_FILENAME - 1);
		osd->files[osd->file_count].name[MAX_FILENAME - 1] = '\0';

		// Check if directory
		char fullpath[MAX_PATH_LEN + MAX_FILENAME];
		snprintf(fullpath, sizeof(fullpath), "%s/%s", osd->browser_path, ent->d_name);
		struct stat st;
		if (stat(fullpath, &st) == 0) {
			osd->files[osd->file_count].is_dir = S_ISDIR(st.st_mode) ? 1 : 0;
		} else {
			osd->files[osd->file_count].is_dir = 0;
		}
		osd->file_count++;
	}
	closedir(dir);

	// Sort (keep eject and .. at top)
	int sort_start = (osd->browser_target >= 0) ? 1 : 0;
	if (strlen(osd->browser_path) > 1) sort_start++;
	if (osd->file_count > sort_start) {
		qsort(&osd->files[sort_start], osd->file_count - sort_start,
		      sizeof(FileEntry), compare_files);
	}
}

// Get font bitmap for a character
static const uint8_t *get_font_char(int c)
{
	if (c >= 32 && c <= 126) {
		return vga_font_8x16_data[c - 32];
	} else if (c >= 128 && c <= 133) {
		return custom_glyphs[c - 128];
	}
	return vga_font_8x16_data['?' - 32];
}

// Text rendering using VGA 8x16 font
static int get_text_width(const char *text)
{
	int width = 0;
	while (*text) {
		unsigned char c = *text++;
		if (c >= 32 && c <= 126) width += FONT_WIDTH;
		else if (c >= 128 && c <= 133) width += FONT_WIDTH;
	}
	return width;
}

// Portrait mode: OSD renders to 480x800 portrait buffer but uses 800x480 logical coords
// User views display rotated 90 CCW, so we rotate OSD 90 CCW to compensate
// Transform: logical (x, y) in 800x480 -> buffer (479-y, x) in 480x800
static int osd_portrait_mode = 0;
static int osd_phys_w = 0;  // Physical buffer width
static int osd_phys_h = 0;  // Physical buffer height
static int osd_phys_pitch = 0;  // Physical buffer pitch
static uint8_t *osd_pixels = NULL;

static inline void put_pixel(int x, int y, uint16_t color)
{
	int bx, by;
	if (osd_portrait_mode) {
		// Rotate 90 CCW for user viewing display rotated 90 CCW
		// Logical (x,y) in 800x480 -> buffer (phys_w-1-y, x) in 480x800
		bx = osd_phys_w - 1 - y;
		by = x;
	} else {
		bx = x;
		by = y;
	}
	if (bx < 0 || bx >= osd_phys_w || by < 0 || by >= osd_phys_h) return;
	uint16_t *p = (uint16_t *)(osd_pixels + by * osd_phys_pitch + bx * 2);
	*p = color;
}

static int draw_text(uint8_t *pixels, int w, int h, int pitch,
                     int x, int y, const char *text, uint16_t color, int max_w)
{
	(void)pixels; (void)pitch;  // Using globals instead
	int start_x = x;
	// w and h are logical dimensions (swapped by osd_render in portrait mode)

	while (*text) {
		unsigned char c = *text++;
		if (c < 32) continue;

		const uint8_t *glyph = get_font_char(c);
		if (max_w > 0 && (x - start_x + FONT_WIDTH) > max_w) break;

		// Draw character from VGA font (1-bit bitmap, MSB is left pixel)
		for (int dy = 0; dy < FONT_HEIGHT && (y + dy) < h; dy++) {
			if (y + dy < 0) continue;
			uint8_t row = glyph[dy];
			for (int dx = 0; dx < FONT_WIDTH && (x + dx) < w; dx++) {
				if (x + dx < 0) continue;
				if (row & (0x80 >> dx)) {
					put_pixel(x + dx, y + dy, color);
				}
			}
		}
		x += FONT_WIDTH;
	}
	return x - start_x;
}

static void draw_rect(uint8_t *pixels, int w, int h, int pitch,
                      int rx, int ry, int rw, int rh, uint16_t color)
{
	(void)pixels; (void)pitch;  // Using globals instead
	// w and h are logical dimensions (swapped by osd_render in portrait mode)

	for (int y = ry; y < ry + rh && y < h; y++) {
		if (y < 0) continue;
		for (int x = rx; x < rx + rw && x < w; x++) {
			if (x < 0) continue;
			put_pixel(x, y, color);
		}
	}
}

// Draw a horizontal bar (for brightness/volume)
static void draw_bar(uint8_t *pixels, int w, int h, int pitch,
                     int x, int y, int bar_w, int bar_h, int value, int max_val)
{
	// Background
	draw_rect(pixels, w, h, pitch, x, y, bar_w, bar_h, COLOR_BORDER);
	// Filled portion
	int fill_w = (bar_w - 2) * value / max_val;
	draw_rect(pixels, w, h, pitch, x + 1, y + 1, fill_w, bar_h - 2, COLOR_VALUE);
}

// Generic menu rendering
typedef struct {
	const char *label;
	int is_separator;
	int is_submenu;  // Shows ">" arrow
	const char *value;  // Optional value to show on right
} MenuEntry;

static void render_generic_menu(uint8_t *pixels, int w, int h, int pitch,
                                const char *title, MenuEntry *entries, int count,
                                int selection, const char *help_text)
{
	int line_h = 20;      // Slightly tighter for 16px font
	int title_h = 28;     // Title area height
	int help_h = 22;      // Help text area
	int padding = 6;

	// Calculate menu height
	int num_separators = 0;
	int num_items = 0;
	for (int i = 0; i < count; i++) {
		if (entries[i].is_separator)
			num_separators++;
		else
			num_items++;
	}
	int content_h = title_h + (num_items * line_h) + (num_separators * line_h / 2) + help_h;
	int menu_h = content_h + padding * 2;
	// Adapt menu width to available screen space
	int menu_w = 380;
	if (menu_w > w - 10) menu_w = w - 10;
	if (menu_w < 100) menu_w = 100;  // Minimum usable width
	int menu_x = (w - menu_w) / 2;
	if (menu_x < 0) menu_x = 0;
	int menu_y = (h - menu_h) / 2;
	if (menu_y < 0) menu_y = 0;

	// Background with panel color
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, menu_w, menu_h, COLOR_PANEL);

	// Border (1 pixel)
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, menu_w, 1, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x, menu_y + menu_h - 1, menu_w, 1, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, 1, menu_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x + menu_w - 1, menu_y, 1, menu_h, COLOR_BORDER);

	// Title
	draw_text(pixels, w, h, pitch, menu_x + 10, menu_y + 8, title, COLOR_TITLE, 0);

	// Menu items
	int y = menu_y + title_h;
	for (int i = 0; i < count; i++) {
		if (entries[i].is_separator) {
			draw_rect(pixels, w, h, pitch, menu_x + 10, y + 8, menu_w - 20, 1, COLOR_SEP);
			y += line_h / 2;
		} else {
			// Highlight selected item
			if (i == selection) {
				draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
			}

			// Draw label
			draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, entries[i].label, COLOR_TEXT, menu_w - 80);

			// Draw value or submenu arrow on right
			if (entries[i].is_submenu) {
				draw_text(pixels, w, h, pitch, menu_x + menu_w - 20, y + 2, ">", COLOR_TEXT, 0);
			} else if (entries[i].value) {
				int vw = get_text_width(entries[i].value);
				draw_text(pixels, w, h, pitch, menu_x + menu_w - 10 - vw, y + 2,
				          entries[i].value, COLOR_VALUE, 0);
			}
			y += line_h;
		}
	}

	// Help text
	draw_text(pixels, w, h, pitch, menu_x + 10, menu_y + menu_h - 20, help_text, COLOR_SEP, 0);
}

// Render main menu
static void render_main_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	// Get current boot order name
	const char *boot_order_val = "Unknown";
	if (osd->pc && osd->pc->cmos) {
		int order = cmos_get_boot_order(osd->pc->cmos);
		if (order >= 0 && order < BOOT_ORDER_COUNT)
			boot_order_val = boot_order_names[order];
	}

	MenuEntry entries[MAIN_COUNT] = {
		{ "Mounting", 0, 1, NULL },
		{ "Audio/Visual", 0, 1, NULL },
		{ "System", 0, 1, NULL },
		{ "Status", 0, 1, NULL },
		{ "Help", 0, 1, NULL },
		{ NULL, 1, 0, NULL },  // SEP1
		{ "Boot Order:", 0, 0, boot_order_val },
		{ "Save Settings", 0, 0, NULL },
		{ NULL, 1, 0, NULL },  // SEP2
		{ "Ctrl+Alt+Del", 0, 0, NULL },
		{ "Exit to Launcher", 0, 0, NULL },
		{ NULL, 1, 0, NULL },  // SEP3
		{ "Exit", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "tiny386", entries, MAIN_COUNT,
	                    osd->main_sel, "Up/Down:Select  Enter:Open  Meta:Close");
}

// Render mounting submenu
static void render_mounting_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	char fda_val[32], fdb_val[32];
	char cda_val[32], cdb_val[32], cdc_val[32], usb_val[32];

	snprintf(fda_val, sizeof(fda_val), "%.20s", basename_ptr(osd->drive_paths[0]));
	snprintf(fdb_val, sizeof(fdb_val), "%.20s", basename_ptr(osd->drive_paths[1]));
	snprintf(cda_val, sizeof(cda_val), "%.20s", basename_ptr(osd->drive_paths[2]));
	snprintf(cdb_val, sizeof(cdb_val), "%.20s", basename_ptr(osd->drive_paths[3]));
	snprintf(cdc_val, sizeof(cdc_val), "%.20s", basename_ptr(osd->drive_paths[4]));

	// USB storage status (read from globals - managed by usb_host.c)
	if (globals.usb_storage_connected) {
		snprintf(usb_val, sizeof(usb_val), "Connected");
	} else {
		snprintf(usb_val, sizeof(usb_val), "(empty)");
	}

	MenuEntry entries[MOUNT_COUNT] = {
		{ "Floppy A:", 0, 0, fda_val },
		{ "Floppy B:", 0, 0, fdb_val },
		{ NULL, 1, 0, NULL },  // SEP1
		{ "CD-ROM A:", 0, 0, cda_val },
		{ "CD-ROM B:", 0, 0, cdb_val },
		{ "CD-ROM C:", 0, 0, cdc_val },
		{ "USB Storage:", 0, 0, usb_val },
		{ NULL, 1, 0, NULL },  // SEP2
		{ "< Back", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "Mounting", entries, MOUNT_COUNT,
	                    osd->mount_sel, "Enter:Browse  Esc:Back");
}

// Render audio/visual submenu
static void render_av_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	char bright_val[16], vol_val[16], skip_val[16], dblbuf_val[16];
	snprintf(bright_val, sizeof(bright_val), "%d%%", osd->brightness);
	snprintf(vol_val, sizeof(vol_val), "%d%%", osd->volume);
	if (osd->frame_skip == 0) {
		snprintf(skip_val, sizeof(skip_val), "Off");
	} else {
		snprintf(skip_val, sizeof(skip_val), "Max %d", osd->frame_skip);
	}
	snprintf(dblbuf_val, sizeof(dblbuf_val), osd->double_buffer ? "On" : "Off");

	MenuEntry entries[AV_COUNT] = {
		{ "Brightness:", 0, 0, bright_val },
		{ "Volume:", 0, 0, vol_val },
		{ "Frame Skip:", 0, 0, skip_val },
		{ "Double Buffer:", 0, 0, dblbuf_val },
		{ NULL, 1, 0, NULL },  // SEP1
		{ "< Back", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "Audio/Visual", entries, AV_COUNT,
	                    osd->av_sel, "Left/Right:Adjust  Esc:Back");
}

// CPU generation names
static const char *cpu_gen_names[] = { "i386", "i486", "i586" };

// Render system settings submenu
static void render_sys_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	char cpu_val[16], fpu_val[16], mem_val[16], batch_val[16], mouse_val[16], hdd_val[32];

	// CPU generation (3=386, 4=486, 5=586)
	int gen_idx = osd->cpu_gen - 3;
	if (gen_idx < 0) gen_idx = 0;
	if (gen_idx > 2) gen_idx = 2;
	snprintf(cpu_val, sizeof(cpu_val), "%s", cpu_gen_names[gen_idx]);

	// FPU
	snprintf(fpu_val, sizeof(fpu_val), "%s", osd->fpu ? "Enabled" : "Disabled");

	// Memory size
	snprintf(mem_val, sizeof(mem_val), "%d MB", osd->mem_size_mb);

	// Batch size
	if (osd->batch_size == 0) {
		snprintf(batch_val, sizeof(batch_val), "Auto");
	} else {
		snprintf(batch_val, sizeof(batch_val), "%d", osd->batch_size);
	}

	// Mouse speed (1-10)
	snprintf(mouse_val, sizeof(mouse_val), "%d", osd->mouse_speed);

	// HDD size for creation
	int size_mb = hdd_size_presets[osd->hdd_size_sel];
	if (size_mb >= 1024) {
		snprintf(hdd_val, sizeof(hdd_val), "%d GB", size_mb / 1024);
	} else {
		snprintf(hdd_val, sizeof(hdd_val), "%d MB", size_mb);
	}

	MenuEntry entries[SYS_COUNT] = {
		{ "CPU:", 0, 0, cpu_val },
		{ "FPU:", 0, 0, fpu_val },
		{ "Memory:", 0, 0, mem_val },
		{ "Batch:", 0, 0, batch_val },
		{ "Mouse Speed:", 0, 0, mouse_val },
		{ NULL, 1, 0, NULL },  // SEP1
		{ "Create HDD:", 0, 0, hdd_val },
		{ NULL, 1, 0, NULL },  // SEP2
		{ "< Back (restart to apply)", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "System", entries, SYS_COUNT,
	                    osd->sys_sel, "L/R:Adjust Enter:Create Esc:Back");
}

// Render status panel (emulator stats)
static void render_status(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	char line[64];

	// Panel dimensions
	int panel_w = (w > 350) ? 320 : w - 20;
	int panel_h = 220;
	int panel_x = (w - panel_w) / 2;
	int panel_y = (h - panel_h) / 2;
	int line_y = panel_y + 10;
	int line_h = 22;

	// Background
	draw_rect(pixels, w, h, pitch, panel_x, panel_y, panel_w, panel_h, COLOR_BG);

	// Border
	draw_rect(pixels, w, h, pitch, panel_x, panel_y, panel_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, panel_x, panel_y + panel_h - 2, panel_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, panel_x, panel_y, 2, panel_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, panel_x + panel_w - 2, panel_y, 2, panel_h, COLOR_BORDER);

	// Title
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, "SYSTEM STATUS", COLOR_TITLE, 0);
	line_y += line_h + 8;

	// Separator
	draw_rect(pixels, w, h, pitch, panel_x + 8, line_y - 4, panel_w - 16, 1, COLOR_SEP);

	// VGA Mode
	snprintf(line, sizeof(line), "VGA: %dx%d", globals.vga_mode_width, globals.vga_mode_height);
	if (globals.vga_pixel_double > 1) {
		char extra[16];
		snprintf(extra, sizeof(extra), " (x%d)", globals.vga_pixel_double);
		strncat(line, extra, sizeof(line) - strlen(line) - 1);
	}
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, line, COLOR_TEXT, 0);
	line_y += line_h;

	// CPU/Peripheral time split
	snprintf(line, sizeof(line), "CPU: %d%%   Periph: %d%%",
	         globals.emu_cpu_percent, globals.emu_periph_percent);
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, line, COLOR_TEXT, 0);
	line_y += line_h;

	// Batch size and calls/sec
	snprintf(line, sizeof(line), "Batch: %d   Calls/s: %lu",
	         globals.emu_batch_size, (unsigned long)globals.emu_calls_per_sec);
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, line, COLOR_TEXT, 0);
	line_y += line_h;

	// Frame skip
	if (osd->frame_skip == 0) {
		snprintf(line, sizeof(line), "Frame Skip: Off");
	} else {
		snprintf(line, sizeof(line), "Frame Skip: Max %d", osd->frame_skip);
	}
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, line, COLOR_TEXT, 0);
	line_y += line_h + 8;

	// Help text
	draw_text(pixels, w, h, pitch, panel_x + 10, panel_y + panel_h - 24,
	          "[Esc] Back", COLOR_MUTED, 0);
}

// Render help screen (keyboard shortcuts)
static void render_help(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	// Panel dimensions
	int panel_w = (w > 380) ? 360 : w - 20;
	int panel_h = 320;
	int panel_x = (w - panel_w) / 2;
	int panel_y = (h - panel_h) / 2;
	int line_y = panel_y + 10;
	int line_h = 20;

	// Background
	draw_rect(pixels, w, h, pitch, panel_x, panel_y, panel_w, panel_h, COLOR_BG);

	// Border
	draw_rect(pixels, w, h, pitch, panel_x, panel_y, panel_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, panel_x, panel_y + panel_h - 2, panel_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, panel_x, panel_y, 2, panel_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, panel_x + panel_w - 2, panel_y, 2, panel_h, COLOR_BORDER);

	// Title
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, "KEYBOARD SHORTCUTS", COLOR_TITLE, 0);
	line_y += line_h + 8;

	// Separator
	draw_rect(pixels, w, h, pitch, panel_x + 8, line_y - 4, panel_w - 16, 1, COLOR_SEP);

	// Global shortcuts
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, "Global:", COLOR_VALUE, 0);
	line_y += line_h;

	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "META          Toggle OSD", COLOR_TEXT, 0);
	line_y += line_h;
	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "META+Up/Down  Volume", COLOR_TEXT, 0);
	line_y += line_h;
	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "META+L/R      Brightness", COLOR_TEXT, 0);
	line_y += line_h + 10;

	// OSD shortcuts
	draw_text(pixels, w, h, pitch, panel_x + 10, line_y, "In OSD:", COLOR_VALUE, 0);
	line_y += line_h;

	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "Up/Down       Navigate", COLOR_TEXT, 0);
	line_y += line_h;
	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "Enter         Select", COLOR_TEXT, 0);
	line_y += line_h;
	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "Escape        Back/Close", COLOR_TEXT, 0);
	line_y += line_h;
	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "Left/Right    Adjust value", COLOR_TEXT, 0);
	line_y += line_h;

	// Help text at bottom
	draw_text(pixels, w, h, pitch, panel_x + 10, panel_y + panel_h - 24,
	          "[Esc] Back", COLOR_MUTED, 0);
}

// Render file browser
static void render_browser(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	int line_h = 18;
	int header_h = 32;
	int help_h = 25;
	int padding = 8;

	int browser_w = (w > 400) ? 380 : w - 20;
	if (browser_w < 100) browser_w = 100;  // Minimum usable width
	int browser_h = h - 40;
	if (browser_h > 400) browser_h = 400;
	if (browser_h < 100) browser_h = 100;  // Minimum usable height
	int browser_x = (w - browser_w) / 2;
	if (browser_x < 0) browser_x = 0;
	int browser_y = (h - browser_h) / 2;
	if (browser_y < 0) browser_y = 0;

	// Background
	draw_rect(pixels, w, h, pitch, browser_x, browser_y, browser_w, browser_h, COLOR_BG);

	// Border
	draw_rect(pixels, w, h, pitch, browser_x, browser_y, browser_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, browser_x, browser_y + browser_h - 2, browser_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, browser_x, browser_y, 2, browser_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, browser_x + browser_w - 2, browser_y, 2, browser_h, COLOR_BORDER);

	// Path header
	draw_text(pixels, w, h, pitch, browser_x + 8, browser_y + 6, osd->browser_path, COLOR_TITLE, browser_w - 16);

	// Separator
	draw_rect(pixels, w, h, pitch, browser_x + 4, browser_y + 26, browser_w - 8, 1, COLOR_SEP);

	// Calculate visible area
	int list_y = browser_y + header_h;
	int list_h = browser_h - header_h - help_h - padding;
	osd->files_visible = list_h / line_h;

	// Adjust scroll to keep selection visible
	if (osd->file_sel < osd->file_scroll) {
		osd->file_scroll = osd->file_sel;
	}
	if (osd->file_sel >= osd->file_scroll + osd->files_visible) {
		osd->file_scroll = osd->file_sel - osd->files_visible + 1;
	}

	// Draw file list
	int y = list_y;
	for (int i = osd->file_scroll; i < osd->file_count && i < osd->file_scroll + osd->files_visible; i++) {
		// Highlight selected
		if (i == osd->file_sel) {
			draw_rect(pixels, w, h, pitch, browser_x + 4, y, browser_w - 8, line_h, COLOR_HILITE);
		}

		// Format entry with icon
		char line[MAX_FILENAME + 4];
		uint16_t color = COLOR_TEXT;
		if (osd->files[i].is_dir == FILE_TYPE_EJECT) {
			snprintf(line, sizeof(line), "  %s", osd->files[i].name);
			color = COLOR_EJECT;
		} else if (osd->files[i].is_dir) {
			snprintf(line, sizeof(line), "[%s]", osd->files[i].name);
			color = COLOR_DIR;
		} else {
			snprintf(line, sizeof(line), "  %s", osd->files[i].name);
		}

		draw_text(pixels, w, h, pitch, browser_x + 8, y + 1, line, color, browser_w - 16);
		y += line_h;
	}

	// Scroll indicator
	if (osd->file_count > osd->files_visible) {
		int scroll_h = (browser_h - header_h - help_h) * osd->files_visible / osd->file_count;
		int scroll_y = list_y + (list_h - scroll_h) * osd->file_scroll / (osd->file_count - osd->files_visible);
		draw_rect(pixels, w, h, pitch, browser_x + browser_w - 8, scroll_y, 4, scroll_h, COLOR_SEP);
	}

	// Help text
	draw_text(pixels, w, h, pitch, browser_x + 8, browser_y + browser_h - 20,
	          "Enter:Select  Backspace:Up  Esc:Cancel", COLOR_SEP, 0);
}

// Apply brightness setting
static void apply_brightness(OSD *osd)
{
	globals.brightness = osd->brightness;  // Sync to globals
	bsp_display_set_backlight_brightness(osd->brightness);
}

// Apply volume setting
static void apply_volume(OSD *osd)
{
	globals.volume = osd->volume;  // Sync to globals
	bsp_audio_set_volume((float)osd->volume);
}

// Handle file selection
static int handle_file_select(OSD *osd)
{
	if (osd->file_sel < 0 || osd->file_sel >= osd->file_count) return 0;

	FileEntry *f = &osd->files[osd->file_sel];

	if (f->is_dir == FILE_TYPE_EJECT) {
		// Eject the drive
		const char *drive_names[] = {"A:", "B:", "CD0:", "CD1:", "CD2:", "CD3:"};
		if (osd->browser_target < 2) {
			// Floppy
			if (osd->emulink) emulink_attach_floppy(osd->emulink, osd->browser_target, NULL);
		} else {
			// CD-ROM
			IDEIFState *ide = (osd->browser_target < 4) ? osd->ide : osd->ide2;
			if (ide) ide_change_cd(ide, (osd->browser_target - 2) % 2, "");
		}
		char msg[32];
		snprintf(msg, sizeof(msg), "%s ejected", drive_names[osd->browser_target]);
		toast_show(msg);
		populate_drive_paths(osd);
		osd->view = VIEW_MOUNTING;
		return 0;
	}

	if (f->is_dir) {
		// Navigate into directory
		if (strcmp(f->name, "..") == 0) {
			char *slash = strrchr(osd->browser_path, '/');
			if (slash && slash != osd->browser_path) {
				*slash = '\0';
			}
		} else {
			if (strlen(osd->browser_path) + strlen(f->name) + 2 < MAX_PATH_LEN) {
				strcat(osd->browser_path, "/");
				strcat(osd->browser_path, f->name);
			}
		}
		scan_directory(osd);
		return 0;
	}

	// Mount the file
	char fullpath[MAX_PATH_LEN];
	snprintf(fullpath, sizeof(fullpath), "%s/%s", osd->browser_path, f->name);

	if (osd->browser_target < 2) {
		// Floppy
		if (osd->emulink) emulink_attach_floppy(osd->emulink, osd->browser_target, fullpath);
	} else {
		// CD-ROM
		IDEIFState *ide = (osd->browser_target < 4) ? osd->ide : osd->ide2;
		if (ide) ide_change_cd(ide, (osd->browser_target - 2) % 2, fullpath);
	}

	// Show toast with filename (truncate if too long)
	char msg[48];
	const char *filename = f->name;
	if (strlen(filename) > 28) {
		snprintf(msg, sizeof(msg), "Mounted ...%s", filename + strlen(filename) - 25);
	} else {
		snprintf(msg, sizeof(msg), "Mounted %s", filename);
	}
	toast_show(msg);

	populate_drive_paths(osd);
	osd->view = VIEW_MOUNTING;
	return 0;
}

// Handle mounting menu selection
static int handle_mount_select(OSD *osd)
{
	switch (osd->mount_sel) {
	case MOUNT_FDA:
	case MOUNT_FDB:
		osd->browser_target = osd->mount_sel;  // 0 or 1
		strcpy(osd->browser_path, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		break;
	case MOUNT_CDA:
	case MOUNT_CDB:
	case MOUNT_CDC:
		osd->browser_target = (osd->mount_sel - MOUNT_CDA) + 2;  // 2-4
		strcpy(osd->browser_path, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		break;
	case MOUNT_CDD:
		// USB Storage - slot 5 is managed by USB host, not user-mountable
		// Do nothing - just show current status
		break;
	case MOUNT_BACK:
		osd->view = VIEW_MAIN_MENU;
		break;
	}
	return 0;
}

// Handle main menu selection
static int handle_main_select(OSD *osd)
{
	switch (osd->main_sel) {
	case MAIN_MOUNTING:
		osd->view = VIEW_MOUNTING;
		osd->mount_sel = MOUNT_FDA;
		break;
	case MAIN_AUDIOVISUAL:
		osd->view = VIEW_AUDIOVISUAL;
		osd->av_sel = AV_BRIGHTNESS;
		break;
	case MAIN_SYSTEM:
		osd->view = VIEW_SYSTEM;
		osd->sys_sel = SYS_CPU_GEN;
		break;
	case MAIN_STATUS:
		osd->view = VIEW_STATUS;
		globals.stats_collecting = true;  // Start collecting stats
		break;
	case MAIN_HELP:
		osd->view = VIEW_HELP;
		break;
	case MAIN_BOOT_ORDER:
		// Cycle boot order
		if (osd->pc && osd->pc->cmos) {
			int order = cmos_get_boot_order(osd->pc->cmos);
			order = (order + 1) % BOOT_ORDER_COUNT;
			cmos_set_boot_order(osd->pc->cmos, order);
		}
		break;
	case MAIN_SAVE_INI:
		// Save all current settings to ini file
		if (osd->pc && osd->pc->ini_path) {
			int boot_order = osd->pc->cmos ? cmos_get_boot_order(osd->pc->cmos) : 0;
			save_settings_to_ini(osd->pc->ini_path, boot_order,
			                     osd->drive_paths[0], osd->drive_paths[1],
			                     osd->drive_paths[2], osd->drive_paths[3],
			                     osd->drive_paths[4], osd->drive_paths[5],
			                     osd->cpu_gen, osd->fpu, osd_get_mem_size_bytes(osd),
			                     osd->brightness, osd->volume, osd->frame_skip,
			                     osd->double_buffer, osd->batch_size, osd->mouse_speed);
			toast_show("Settings saved");
		}
		break;
	case MAIN_CTRLALTDEL:
		toast_show("Ctrl+Alt+Del sent");
		globals.reset_pending = true;  // Signal soft reset
		return 1;  // Close OSD
	case MAIN_RESET:
		esp_restart();
		break;
	case MAIN_EXIT:
		return 1;  // Close OSD
	}
	return 0;
}

// Handle A/V adjustment with left/right
static void handle_av_adjust(OSD *osd, int delta)
{
	switch (osd->av_sel) {
	case AV_BRIGHTNESS:
		osd->brightness += delta * 5;
		if (osd->brightness < 5) osd->brightness = 5;
		if (osd->brightness > 100) osd->brightness = 100;
		apply_brightness(osd);
		break;
	case AV_VOLUME:
		osd->volume += delta * 5;
		if (osd->volume < 0) osd->volume = 0;
		if (osd->volume > 100) osd->volume = 100;
		apply_volume(osd);
		break;
	case AV_FRAME_SKIP:
		osd->frame_skip += delta;
		if (osd->frame_skip < 0) osd->frame_skip = 0;
		if (osd->frame_skip > 10) osd->frame_skip = 10;
		vga_frame_skip_max = osd->frame_skip;
		break;
	case AV_DOUBLE_BUFFER:
		osd->double_buffer = !osd->double_buffer;  // Toggle on any left/right
		vga_double_buffer = osd->double_buffer;
		break;
	}
}

// Create sparse HDD image file
// Returns 0 on success, -1 on error
static int create_hdd_image(int size_mb)
{
	// Generate filename based on size
	char filename[64];
	if (size_mb >= 1024) {
		snprintf(filename, sizeof(filename), "/sdcard/hdd_%dgb.img", size_mb / 1024);
	} else {
		snprintf(filename, sizeof(filename), "/sdcard/hdd_%dmb.img", size_mb);
	}

	// Check if file already exists
	struct stat st;
	if (stat(filename, &st) == 0) {
		// File exists - try with numbered suffix
		for (int i = 2; i <= 99; i++) {
			if (size_mb >= 1024) {
				snprintf(filename, sizeof(filename), "/sdcard/hdd_%dgb_%d.img", size_mb / 1024, i);
			} else {
				snprintf(filename, sizeof(filename), "/sdcard/hdd_%dmb_%d.img", size_mb, i);
			}
			if (stat(filename, &st) != 0) break;
		}
	}

	// Create sparse file using truncate (instant, no actual disk write)
	FILE *f = fopen(filename, "wb");
	if (!f) {
		return -1;
	}

	// Calculate size in bytes
	long long size_bytes = (long long)size_mb * 1024 * 1024;

	// Use fseek + fwrite to create sparse file
	// This creates a file with the specified size but only allocates
	// disk blocks as they're actually written
	if (fseek(f, size_bytes - 1, SEEK_SET) != 0) {
		fclose(f);
		return -1;
	}

	// Write a single byte to establish file size
	if (fwrite("", 1, 1, f) != 1) {
		fclose(f);
		return -1;
	}

	fclose(f);

	// Show success toast with filename
	char msg[64];
	snprintf(msg, sizeof(msg), "Created %s", filename + 8);  // Skip "/sdcard/"
	toast_show(msg);

	return 0;
}

// Handle system settings adjustment with left/right
static void handle_sys_adjust(OSD *osd, int delta)
{
	switch (osd->sys_sel) {
	case SYS_CPU_GEN:
		osd->cpu_gen += delta;
		if (osd->cpu_gen < 3) osd->cpu_gen = 5;  // Wrap
		if (osd->cpu_gen > 5) osd->cpu_gen = 3;
		break;
	case SYS_FPU:
		osd->fpu = !osd->fpu;
		break;
	case SYS_MEMSIZE:
		// Adjust in 1MB increments, max 24MB, min 1MB
		osd->mem_size_mb += delta;
		if (osd->mem_size_mb < 1) osd->mem_size_mb = 1;
		if (osd->mem_size_mb > 24) osd->mem_size_mb = 24;
		break;
	case SYS_BATCH:
		// Cycle: Auto(0), 512, 768, 1024, ... 4096 in 256 increments
		if (osd->batch_size == 0) {
			// Auto -> first fixed value or wrap to max
			osd->batch_size = (delta > 0) ? 512 : 4096;
		} else {
			osd->batch_size += delta * 256;
			if (osd->batch_size < 512) osd->batch_size = 0;  // Wrap to Auto
			if (osd->batch_size > 4096) osd->batch_size = 0; // Wrap to Auto
		}
		// Apply immediately (takes effect next pc_step)
		pc_batch_size_setting = osd->batch_size;
		break;
	case SYS_MOUSE_SPEED:
		osd->mouse_speed += delta;
		if (osd->mouse_speed < 1) osd->mouse_speed = 10;  // Wrap
		if (osd->mouse_speed > 10) osd->mouse_speed = 1;
		// Apply immediately
		mouse_emu_set_speed(osd->mouse_speed);
		globals.mouse_speed = osd->mouse_speed;
		break;
	case SYS_CREATE_HDD:
		// Cycle HDD size presets
		osd->hdd_size_sel += delta;
		if (osd->hdd_size_sel < 0) osd->hdd_size_sel = hdd_size_preset_count - 1;
		if (osd->hdd_size_sel >= hdd_size_preset_count) osd->hdd_size_sel = 0;
		break;
	}
}

// Initialize OSD
OSD *osd_init(void)
{
	OSD *osd = calloc(1, sizeof(OSD));
	if (!osd) return NULL;

	osd->view = VIEW_MAIN_MENU;
	osd->main_sel = MAIN_MOUNTING;
	osd->mount_sel = MOUNT_FDA;
	osd->av_sel = AV_BRIGHTNESS;
	osd->sys_sel = SYS_CPU_GEN;
	osd->hdd_size_sel = 2;  // Default to 504 MB (index 2)
	// Load brightness/volume from globals (set from INI in esp_main.c)
	osd->brightness = globals.brightness;
	osd->volume = globals.volume;
	osd->frame_skip = vga_frame_skip_max;  // Load from config
	osd->double_buffer = vga_double_buffer;  // Load from config
	osd->batch_size = pc_batch_size_setting;  // Load from config (0=auto)
	osd->mouse_speed = globals.mouse_speed > 0 ? globals.mouse_speed : 5;  // Load from config, default 5

	// Default system settings
	osd->cpu_gen = 4;      // i486
	osd->fpu = 1;          // Enabled
	osd->mem_size_mb = 16; // 16MB

	strcpy(osd->browser_path, "/sdcard");

	return osd;
}

void osd_attach_emulink(OSD *osd, void *emulink)
{
	osd->emulink = emulink;
	populate_drive_paths(osd);
}

void osd_attach_ide(OSD *osd, void *ide, void *ide2)
{
	osd->ide = ide;
	osd->ide2 = ide2;
	populate_drive_paths(osd);
}

void osd_attach_pc(OSD *osd, void *pc)
{
	osd->pc = pc;
}

void osd_attach_console(OSD *osd, void *console)
{
	osd->console = console;
}

void osd_set_system_config(OSD *osd, int cpu_gen, int fpu, long mem_size)
{
	osd->cpu_gen = cpu_gen;
	osd->fpu = fpu;
	// Convert bytes to MB
	osd->mem_size_mb = (int)(mem_size / (1024 * 1024));
	if (osd->mem_size_mb < 1) osd->mem_size_mb = 1;
	if (osd->mem_size_mb > 24) osd->mem_size_mb = 24;
}

// Get system memory size in bytes for saving
long osd_get_mem_size_bytes(OSD *osd)
{
	return (long)osd->mem_size_mb * 1024 * 1024;
}

void osd_refresh(OSD *osd)
{
	populate_drive_paths(osd);
	// Sync brightness/volume from globals (may have been changed via META+arrow)
	osd->brightness = globals.brightness;
	osd->volume = globals.volume;
}

void osd_handle_mouse_motion(OSD *osd, int x, int y)
{
	(void)osd; (void)x; (void)y;
}

void osd_handle_mouse_button(OSD *osd, int x, int y, int down, int btn)
{
	(void)osd; (void)x; (void)y; (void)down; (void)btn;
}

// Returns 1 if OSD should close
int osd_handle_key(OSD *osd, int keycode, int down)
{
	if (!down) return 0;

	switch (osd->view) {
	case VIEW_MAIN_MENU:
		switch (keycode) {
		case SC_UP:
			do {
				osd->main_sel--;
				if (osd->main_sel < 0) osd->main_sel = MAIN_COUNT - 1;
			} while (is_main_separator(osd->main_sel));
			break;
		case SC_DOWN:
			do {
				osd->main_sel++;
				if (osd->main_sel >= MAIN_COUNT) osd->main_sel = 0;
			} while (is_main_separator(osd->main_sel));
			break;
		case SC_ENTER:
		case SC_RIGHT:
			return handle_main_select(osd);
		case SC_ESC:
			return 1;
		}
		break;

	case VIEW_MOUNTING:
		switch (keycode) {
		case SC_UP:
			do {
				osd->mount_sel--;
				if (osd->mount_sel < 0) osd->mount_sel = MOUNT_COUNT - 1;
			} while (is_mount_separator(osd->mount_sel));
			break;
		case SC_DOWN:
			do {
				osd->mount_sel++;
				if (osd->mount_sel >= MOUNT_COUNT) osd->mount_sel = 0;
			} while (is_mount_separator(osd->mount_sel));
			break;
		case SC_ENTER:
		case SC_RIGHT:
			return handle_mount_select(osd);
		case SC_ESC:
		case SC_LEFT:
			osd->view = VIEW_MAIN_MENU;
			break;
		}
		break;

	case VIEW_AUDIOVISUAL:
		switch (keycode) {
		case SC_UP:
			do {
				osd->av_sel--;
				if (osd->av_sel < 0) osd->av_sel = AV_COUNT - 1;
			} while (is_av_separator(osd->av_sel));
			break;
		case SC_DOWN:
			do {
				osd->av_sel++;
				if (osd->av_sel >= AV_COUNT) osd->av_sel = 0;
			} while (is_av_separator(osd->av_sel));
			break;
		case SC_LEFT:
			if (osd->av_sel == AV_BACK) {
				osd->view = VIEW_MAIN_MENU;
			} else {
				handle_av_adjust(osd, -1);
			}
			break;
		case SC_RIGHT:
			if (osd->av_sel == AV_BACK) {
				// Do nothing
			} else {
				handle_av_adjust(osd, +1);
			}
			break;
		case SC_ENTER:
			if (osd->av_sel == AV_BACK) {
				osd->view = VIEW_MAIN_MENU;
			}
			break;
		case SC_ESC:
			osd->view = VIEW_MAIN_MENU;
			break;
		}
		break;

	case VIEW_SYSTEM:
		switch (keycode) {
		case SC_UP:
			do {
				osd->sys_sel--;
				if (osd->sys_sel < 0) osd->sys_sel = SYS_COUNT - 1;
			} while (is_sys_separator(osd->sys_sel));
			break;
		case SC_DOWN:
			do {
				osd->sys_sel++;
				if (osd->sys_sel >= SYS_COUNT) osd->sys_sel = 0;
			} while (is_sys_separator(osd->sys_sel));
			break;
		case SC_LEFT:
			if (osd->sys_sel == SYS_BACK) {
				osd->view = VIEW_MAIN_MENU;
			} else {
				handle_sys_adjust(osd, -1);
			}
			break;
		case SC_RIGHT:
			if (osd->sys_sel == SYS_BACK) {
				// Do nothing
			} else {
				handle_sys_adjust(osd, +1);
			}
			break;
		case SC_ENTER:
			if (osd->sys_sel == SYS_BACK) {
				osd->view = VIEW_MAIN_MENU;
			} else if (osd->sys_sel == SYS_CREATE_HDD) {
				// Create the HDD image
				int size_mb = hdd_size_presets[osd->hdd_size_sel];
				if (create_hdd_image(size_mb) != 0) {
					toast_show("Error creating HDD image");
				}
			}
			break;
		case SC_ESC:
			osd->view = VIEW_MAIN_MENU;
			break;
		}
		break;

	case VIEW_STATUS:
		switch (keycode) {
		case SC_ESC:
		case SC_LEFT:
		case SC_ENTER:
			globals.stats_collecting = false;  // Stop collecting stats
			osd->view = VIEW_MAIN_MENU;
			break;
		}
		break;

	case VIEW_HELP:
		switch (keycode) {
		case SC_ESC:
		case SC_LEFT:
		case SC_ENTER:
			osd->view = VIEW_MAIN_MENU;
			break;
		}
		break;

	case VIEW_FILEBROWSER:
		switch (keycode) {
		case SC_UP:
			if (osd->file_sel > 0) osd->file_sel--;
			break;
		case SC_DOWN:
			if (osd->file_sel < osd->file_count - 1) osd->file_sel++;
			break;
		case SC_ENTER:
			return handle_file_select(osd);
		case SC_BACKSPACE:
			{
				char *slash = strrchr(osd->browser_path, '/');
				if (slash && slash != osd->browser_path) {
					*slash = '\0';
					scan_directory(osd);
				}
			}
			break;
		case SC_ESC:
		case SC_LEFT:
			osd->view = VIEW_MOUNTING;
			break;
		}
		break;
	}

	return 0;
}

void osd_render(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	// Store physical buffer info for put_pixel
	osd_pixels = pixels;
	osd_phys_w = w;
	osd_phys_h = h;
	osd_phys_pitch = pitch;

	// Enable portrait mode if buffer is taller than wide
	// User views display rotated 90 CCW, so OSD uses 800x480 logical coords
	if (h > w) {
		osd_portrait_mode = 1;
		// Swap dimensions for logical landscape coordinates
		int tmp = w;
		w = h;  // Logical width = 800
		h = tmp;  // Logical height = 480
	} else {
		osd_portrait_mode = 0;
	}

	switch (osd->view) {
	case VIEW_MAIN_MENU:
		render_main_menu(osd, pixels, w, h, pitch);
		break;
	case VIEW_MOUNTING:
		render_mounting_menu(osd, pixels, w, h, pitch);
		break;
	case VIEW_AUDIOVISUAL:
		render_av_menu(osd, pixels, w, h, pitch);
		break;
	case VIEW_SYSTEM:
		render_sys_menu(osd, pixels, w, h, pitch);
		break;
	case VIEW_STATUS:
		render_status(osd, pixels, w, h, pitch);
		break;
	case VIEW_HELP:
		render_help(osd, pixels, w, h, pitch);
		break;
	case VIEW_FILEBROWSER:
		render_browser(osd, pixels, w, h, pitch);
		break;
	}
}
