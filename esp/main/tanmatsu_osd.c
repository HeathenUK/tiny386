/*
 * Tanmatsu-specific OSD: Custom keyboard-driven menu with file browser
 * For devices without mouse/touch input
 */

#include "tanmatsu_osd.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "../../vga.h"  // For vga_frame_skip_max

// VGA 8x16 bitmap font for clean text rendering
#include "vga_font_8x16.h"

#define FONT_WIDTH 8
#define FONT_HEIGHT 16
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common.h"
#include "bsp/display.h"
#include "bsp/audio.h"
#include "bsp/power.h"

#include "../../vga.h"
#include "../../misc.h"
#include "../../ide.h"
#include "toast.h"
#include "mouse_emu.h"
#include "usb_host.h"
#include "../../pc.h"
#include "esp_vfs_fat.h"
#include "led_activity.h"
#include "ini_selector.h"

// Audio shutdown for clean exit
extern void i2s_shutdown(void);

// View modes
typedef enum {
	VIEW_MAIN_MENU,
	VIEW_MOUNTING,
	VIEW_AUDIOVISUAL,
	VIEW_SYSTEM,
	VIEW_HELP,
	VIEW_FILEBROWSER,
	VIEW_DELETE_CONFIRM,
	VIEW_CREATE_HDD,
	VIEW_COPY_PROGRESS
} ViewMode;

// Main menu items
typedef enum {
	MAIN_MOUNTING = 0,
	MAIN_AUDIOVISUAL,
	MAIN_SYSTEM,
	MAIN_HELP,
	MAIN_SEP1,
	MAIN_BOOT_ORDER,
	MAIN_SAVE_INI,
	MAIN_SWITCH_INI,
	MAIN_SEP2,
	MAIN_CTRLALTDEL,
	MAIN_RESTART_EMU,
	MAIN_EXIT_LAUNCHER,
	MAIN_COUNT
} MainMenuItem;

// Mounting submenu items
typedef enum {
	MOUNT_FDA = 0,
	MOUNT_FDB,
	MOUNT_SEP1,
	MOUNT_HDA,     // Hard drive (requires restart)
	MOUNT_CREATE_HDD,  // Create new HDD image
	MOUNT_CD,      // Single CD-ROM slot (primary slave)
	MOUNT_USB,     // USB storage (secondary master) - read-only status
	MOUNT_USB_PASSTHRU, // Enable/disable USB passthrough (requires restart)
	MOUNT_SEP2,
	MOUNT_BACK,
	MOUNT_COUNT
} MountMenuItem;

// Special browser_target values
#define BROWSER_TARGET_HDA 101
#define BROWSER_TARGET_CREATE_HDD 102

// Audio/Visual submenu items
typedef enum {
	AV_BRIGHTNESS = 0,
	AV_VOLUME,
	AV_FRAME_SKIP,
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
	SYS_PIT_BURST,
	SYS_ACCURACY,
	SYS_MOUSE_SPEED,
	SYS_STATS_BAR,
	SYS_CPU_DEBUG,
	SYS_SEP1,
	SYS_BACK,
	SYS_COUNT
} SysMenuItem;

// File entry for browser
#define MAX_FILES 256
#define MAX_FILENAME 64
#define MAX_PATH_LEN 256

typedef struct {
	char name[MAX_FILENAME];
	int is_dir;  // 1=directory, 0=file, -1=special (eject), -2=special (switch storage)
} FileEntry;

// Special file entry types
#define FILE_TYPE_EJECT -1
#define FILE_TYPE_SWITCH_STORAGE -2
#define FILE_TYPE_CREATE_HERE -3

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
	// Drive paths
	char drive_paths[6][MAX_PATH_LEN];  // FDA, FDB, CDA-CDD
	char hda_path[MAX_PATH_LEN];        // Hard drive (from INI, changeable)

	// Audio/Visual settings
	int brightness;  // 0-110
	int volume;      // 0-100
	int frame_skip;  // 0-10 max frames to skip
	// System settings (require restart to take effect)
	int cpu_gen;     // 3=386, 4=486, 5=586
	int fpu;         // 0=disabled, 1=enabled
	int mem_size_mb; // Memory in MB (1 to 24)
	int batch_size;  // 0=auto, or fixed value (512-4096 in 256 increments)
	int pit_burst;   // 1=enabled, 0=disabled
	int accuracy;    // 0=full, 1=fast
	int boot_accuracy; // accuracy at boot (core switch requires reboot)
	int mouse_speed; // 1-10, mouse emulation speed
	int usb_passthru; // 1=enabled (default), 0=disabled (requires restart)

	// File browser state
	int browser_target;  // Which drive slot we're browsing for
	char browser_path[MAX_PATH_LEN];
	FileEntry files[MAX_FILES];
	int file_count;
	int file_sel;
	int file_scroll;
	int files_visible;

	// Delete confirmation state
	char delete_path[MAX_PATH_LEN];

	// Create HDD state
	char create_path[MAX_PATH_LEN];  // Directory to create image in (VFS path)
	char create_name[33];     // Filename (without .img extension)
	int create_name_len;      // Current name length
	int create_size_idx;      // Index into hdd_sizes array
	int create_fs_type;       // 0=FAT16, 1=FAT32
	int create_sel;           // 0=name, 1=size, 2=partition, 3=create, 4=cancel

	// Copy/paste state
	char copy_src_path[MAX_PATH_LEN];  // Full path of file marked for copy (empty = nothing)
	volatile int copy_progress_pct;     // 0-100, updated by copy task
	volatile bool copy_done;            // Set true when copy completes or fails
	volatile bool copy_cancel;          // Set true to request cancellation
	char copy_dst_path[MAX_PATH_LEN];  // Full destination path
	char copy_error[64];                // Error message (empty = success)
};

// Scancode defines
#define SC_UP    0x48
#define SC_DOWN  0x50
#define SC_LEFT  0x4B
#define SC_RIGHT 0x4D
#define SC_ENTER 0x1c
#define SC_ESC   0x01
#define SC_BACKSPACE 0x0e
#define SC_D     0x20
#define SC_Y     0x15
#define SC_N     0x31
#define SC_C     0x2e
#define SC_V     0x2f

// Convert scancode to uppercase character for filename input (FAT-compatible)
static char scancode_to_char(int sc) {
	if (sc >= 0x02 && sc <= 0x0b) return '0' + (sc == 0x0b ? 0 : sc - 1);  // 1-9, 0
	if (sc == 0x0c) return '-';
	if (sc >= 0x10 && sc <= 0x19) return "QWERTYUIOP"[sc - 0x10];
	if (sc >= 0x1e && sc <= 0x26) return "ASDFGHJKL"[sc - 0x1e];
	if (sc >= 0x2c && sc <= 0x32) return "ZXCVBNM"[sc - 0x2c];
	return 0;
}

// HDD size presets for Create HDD dialog
static const struct {
	const char *label;
	int64_t bytes;
	int heads;
	int spt;
} hdd_sizes[] = {
	{ "64 MB",   64*1024*1024LL,   16, 63 },
	{ "128 MB",  128*1024*1024LL,  16, 63 },
	{ "256 MB",  256*1024*1024LL,  16, 63 },
	{ "504 MB",  1023*16*63*512LL, 16, 63 },
	{ "1024 MB", 1024*1024*1024LL, 255, 63 },
	{ "2048 MB", 2048*1024*1024LL, 255, 63 },
};
#define HDD_SIZE_COUNT 6

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
static void render_browser(OSD *osd, uint8_t *pixels, int w, int h, int pitch);

// Separator checks for each menu
static int is_main_separator(int item) {
	return item == MAIN_SEP1 || item == MAIN_SEP2;
}

static int is_mount_separator(int item) {
	return item == MOUNT_SEP1 || item == MOUNT_SEP2;
}

static int is_av_separator(int item) {
	return item == AV_SEP1;
}

static int is_sys_separator(int item) {
#ifdef CPU_DIAG
	return item == SYS_SEP1;
#else
	return item == SYS_SEP1 || item == SYS_CPU_DEBUG;
#endif
}

// Get just the filename from a path
static const char *basename_ptr(const char *path)
{
	if (!path || !path[0]) return "[empty]";
	const char *p = strrchr(path, '/');
	return p ? p + 1 : path;
}

/* Set browser start dir to the parent directory of the current mounted image.
 * Falls back to default_path if no image is mounted or directory is unavailable. */
static void set_browser_start_dir(OSD *osd, int target, const char *default_path)
{
	const char *mounted_path = NULL;
	char candidate[MAX_PATH_LEN];
	struct stat st;

	if (target >= 0 && target < 6) {
		if (osd->drive_paths[target][0] != '\0')
			mounted_path = osd->drive_paths[target];
	} else if (target == BROWSER_TARGET_HDA) {
		if (osd->hda_path[0] != '\0')
			mounted_path = osd->hda_path;
	}

	if (mounted_path && mounted_path[0]) {
		strncpy(candidate, mounted_path, sizeof(candidate) - 1);
		candidate[sizeof(candidate) - 1] = '\0';

		char *slash = strrchr(candidate, '/');
		if (slash) {
			if (slash == candidate) {
				candidate[1] = '\0';
			} else {
				*slash = '\0';
			}
			if (stat(candidate, &st) == 0 && S_ISDIR(st.st_mode)) {
				strncpy(osd->browser_path, candidate, MAX_PATH_LEN - 1);
				osd->browser_path[MAX_PATH_LEN - 1] = '\0';
				return;
			}
		}
	}

	strncpy(osd->browser_path, default_path, MAX_PATH_LEN - 1);
	osd->browser_path[MAX_PATH_LEN - 1] = '\0';
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

	// Add "eject" option at top if drive has something mounted
	if (osd->browser_target >= 0 && osd->browser_target < 6 &&
	    osd->drive_paths[osd->browser_target][0] != '\0') {
		// Show [Eject filename] with truncated filename
		const char *filename = basename_ptr(osd->drive_paths[osd->browser_target]);
		snprintf(osd->files[osd->file_count].name, MAX_FILENAME, "[Eject %.50s]", filename);
		osd->files[osd->file_count].is_dir = FILE_TYPE_EJECT;
		osd->file_count++;
	}

	// Add storage switch option when USB is mounted and at root of a mount point
	if (globals.usb_vfs_mounted) {
		if (strcmp(osd->browser_path, "/sdcard") == 0) {
			strcpy(osd->files[osd->file_count].name, "[USB Storage]");
			osd->files[osd->file_count].is_dir = FILE_TYPE_SWITCH_STORAGE;
			osd->file_count++;
		} else if (strcmp(osd->browser_path, "/usb") == 0) {
			strcpy(osd->files[osd->file_count].name, "[SD Card]");
			osd->files[osd->file_count].is_dir = FILE_TYPE_SWITCH_STORAGE;
			osd->file_count++;
		}
	}

	// Add "create image here" option when browsing for HDD creation location
	if (osd->browser_target == BROWSER_TARGET_CREATE_HDD) {
		strcpy(osd->files[osd->file_count].name, "[Create Image Here]");
		osd->files[osd->file_count].is_dir = FILE_TYPE_CREATE_HERE;
		osd->file_count++;
	}

	// Add parent directory option
	if (strlen(osd->browser_path) > 1 &&
	    strcmp(osd->browser_path, "/sdcard") != 0 &&
	    strcmp(osd->browser_path, "/usb") != 0) {
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

	// Sort (keep special entries at top: eject, storage switch, ..)
	int sort_start = 0;
	// Count special entries we added
	if (osd->browser_target >= 0 && osd->browser_target < 6 &&
	    osd->drive_paths[osd->browser_target][0] != '\0') sort_start++;  // [Eject ...]
	if (globals.usb_vfs_mounted &&
	    (strcmp(osd->browser_path, "/sdcard") == 0 || strcmp(osd->browser_path, "/usb") == 0)) {
		sort_start++;  // [USB Storage] or [SD Card]
	}
	// ".." was only added if not at mount point root
	if (strlen(osd->browser_path) > 1 &&
	    strcmp(osd->browser_path, "/sdcard") != 0 &&
	    strcmp(osd->browser_path, "/usb") != 0) {
		sort_start++;  // ".."
	}
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

	// Battery percentage (right-aligned in title bar, refreshed every 5s)
	{
		static uint8_t batt_pct = 0;
		static bool batt_charging = false;
		static bool batt_available = false;
		static uint32_t batt_update_time = 0;
		uint32_t now = esp_system_get_time();
		if (now - batt_update_time > 5000000) {  // 5 seconds
			batt_update_time = now;
			bsp_power_battery_information_t info;
			if (bsp_power_get_battery_information(&info) == ESP_OK) {
				batt_available = info.battery_available;
				batt_pct = info.remaining_percentage;
				batt_charging = info.battery_charging;
			}
		}
		if (batt_available) {
			char batt_str[16];
			snprintf(batt_str, sizeof(batt_str), batt_charging ? "%d%%+" : "%d%%", batt_pct);
			int bw = get_text_width(batt_str);
			draw_text(pixels, w, h, pitch, menu_x + menu_w - 10 - bw, menu_y + 8,
			          batt_str, COLOR_VALUE, 0);
		}
	}

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

	/* Show the currently loaded INI filename for the Switch INI item */
	const char *cur_ini = basename_ptr(globals.current_ini_path);
	char switch_val[32];
	snprintf(switch_val, sizeof(switch_val), "%.20s", cur_ini[0] ? cur_ini : "(none)");

	MenuEntry entries[MAIN_COUNT] = {
		{ "Mounting", 0, 1, NULL },
		{ "Audio/Visual", 0, 1, NULL },
		{ "System", 0, 1, NULL },
		{ "Help", 0, 1, NULL },
		{ NULL, 1, 0, NULL },  // SEP1
		{ "Boot Order:", 0, 0, boot_order_val },
		{ "Save Settings", 0, 0, NULL },
		{ "Switch INI:", 0, 0, switch_val },
		{ NULL, 1, 0, NULL },  // SEP2
		{ "Ctrl+Alt+Del", 0, 0, NULL },
		{ "Restart Emulator", 0, 0, NULL },
		{ "Exit to Launcher", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "tiny386", entries, MAIN_COUNT,
	                    osd->main_sel, "Up/Down:Select  Enter:Open  Meta:Close");
}

// Render mounting submenu
static void render_mounting_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	char fda_val[32], fdb_val[32], hda_val[32], cd_val[32], usb_val[32], usb_pass_val[32];

	snprintf(fda_val, sizeof(fda_val), "%.20s", basename_ptr(osd->drive_paths[0]));
	snprintf(fdb_val, sizeof(fdb_val), "%.20s", basename_ptr(osd->drive_paths[1]));
	snprintf(hda_val, sizeof(hda_val), "%.20s", basename_ptr(osd->hda_path));
	snprintf(cd_val, sizeof(cd_val), "%.20s", basename_ptr(osd->drive_paths[3]));  // slot 1 = primary slave

	// USB storage status - shows connection state only
	if (globals.usb_storage_connected) {
		snprintf(usb_val, sizeof(usb_val), "Connected");
	} else {
		snprintf(usb_val, sizeof(usb_val), "(empty)");
	}

	// USB passthrough setting
	snprintf(usb_pass_val, sizeof(usb_pass_val), "%s", osd->usb_passthru ? "Enabled" : "Disabled");

	MenuEntry entries[MOUNT_COUNT] = {
		{ "Floppy A:", 0, 0, fda_val },
		{ "Floppy B:", 0, 0, fdb_val },
		{ NULL, 1, 0, NULL },  // SEP1
		{ "HDD:", 0, 0, hda_val },
		{ "Create HDD...", 0, 0, NULL },
		{ "CD-ROM:", 0, 0, cd_val },
		{ "USB Storage:", 0, 0, usb_val },
		{ "USB Passthru:", 0, 0, usb_pass_val },
		{ NULL, 1, 0, NULL },  // SEP2
		{ "< Back", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "Mounting", entries, MOUNT_COUNT,
	                    osd->mount_sel, "Enter:Select L/R:Adjust Esc:Back");
}

// Render audio/visual submenu
static void render_av_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	char bright_val[16], vol_val[16], skip_val[16];
	snprintf(bright_val, sizeof(bright_val), "%d%%", osd->brightness);
	snprintf(vol_val, sizeof(vol_val), "%d%%", osd->volume);
	if (osd->frame_skip == 0) {
		snprintf(skip_val, sizeof(skip_val), "Off");
	} else {
		snprintf(skip_val, sizeof(skip_val), "Max %d", osd->frame_skip);
	}

	MenuEntry entries[AV_COUNT] = {
		{ "Brightness:", 0, 0, bright_val },
		{ "Volume:", 0, 0, vol_val },
		{ "Frame Skip:", 0, 0, skip_val },
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
	char cpu_val[16], fpu_val[16], mem_val[16], batch_val[16], pitburst_val[16], accuracy_val[16], mouse_val[16], statsbar_val[16], cpudbg_val[16];

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

	// PIT burst catch-up
	snprintf(pitburst_val, sizeof(pitburst_val), "%s", osd->pit_burst ? "On" : "Off");

	// Accuracy mode (show reboot indicator when changed from boot-time value)
	if (osd->accuracy != osd->boot_accuracy)
		snprintf(accuracy_val, sizeof(accuracy_val), "%s (reboot)", osd->accuracy ? "Fast" : "Full");
	else
		snprintf(accuracy_val, sizeof(accuracy_val), "%s", osd->accuracy ? "Fast" : "Full");

	// Mouse speed (1-10)
	snprintf(mouse_val, sizeof(mouse_val), "%d", osd->mouse_speed);

	// Stats bar
	snprintf(statsbar_val, sizeof(statsbar_val), "%s", globals.stats_bar_visible ? "On" : "Off");

#ifdef CPU_DIAG
	// Debug toggle (only available when CPU_DIAG is compiled in)
	snprintf(cpudbg_val, sizeof(cpudbg_val), "%s", globals.cpu_debug_enabled ? "On" : "Off");
#endif

	MenuEntry entries[SYS_COUNT] = {
		{ "CPU:", 0, 0, cpu_val },
		{ "FPU:", 0, 0, fpu_val },
		{ "Memory:", 0, 0, mem_val },
		{ "Batch:", 0, 0, batch_val },
		{ "PIT Burst:", 0, 0, pitburst_val },
		{ "Accuracy:", 0, 0, accuracy_val },
		{ "Mouse Speed:", 0, 0, mouse_val },
		{ "Stats Bar:", 0, 0, statsbar_val },
#ifdef CPU_DIAG
		{ "Debug:", 0, 0, cpudbg_val },
#else
		{ NULL, 1, 0, NULL },  // CPU_DIAG not compiled: hide Debug toggle
#endif
		{ NULL, 1, 0, NULL },  // SEP1
		{ "< Back (restart to apply)", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, "System", entries, SYS_COUNT,
	                    osd->sys_sel, "L/R:Adjust Esc:Back");
}

// Copy file task — runs on core 0 as a FreeRTOS task
static void copy_file_task(void *arg)
{
	OSD *osd = (OSD *)arg;
	printf("[copy] src=%s dst=%s\n", osd->copy_src_path, osd->copy_dst_path);
	{
		uint64_t total, free_bytes;
		if (esp_vfs_fat_info("/sdcard", &total, &free_bytes) == ESP_OK)
			printf("[copy] /sdcard: total=%lluMB free=%lluMB\n",
			       (unsigned long long)(total/1024/1024),
			       (unsigned long long)(free_bytes/1024/1024));
		if (esp_vfs_fat_info("/usb", &total, &free_bytes) == ESP_OK)
			printf("[copy] /usb: total=%lluMB free=%lluMB\n",
			       (unsigned long long)(total/1024/1024),
			       (unsigned long long)(free_bytes/1024/1024));
	}

	FILE *src = fopen(osd->copy_src_path, "rb");
	FILE *dst = fopen(osd->copy_dst_path, "wb");
	printf("[copy] fopen: src=%p dst=%p\n", src, dst);
	if (!src || !dst) {
		snprintf(osd->copy_error, sizeof(osd->copy_error),
		         src ? "Can't create file!" : "Can't read source!");
		if (src) fclose(src);
		if (dst) { fclose(dst); unlink(osd->copy_dst_path); }
		osd->copy_done = true;
		vTaskDelete(NULL);
		return;
	}

	// Get file size
	fseek(src, 0, SEEK_END);
	long file_size = ftell(src);
	fseek(src, 0, SEEK_SET);
	printf("[copy] file_size=%ld\n", file_size);
	if (file_size <= 0) file_size = 1;  // avoid div-by-zero

	uint8_t *buf = malloc(32768);
	if (!buf) {
		printf("[copy] malloc failed!\n");
		snprintf(osd->copy_error, sizeof(osd->copy_error), "Out of memory!");
		fclose(src);
		fclose(dst);
		unlink(osd->copy_dst_path);
		osd->copy_done = true;
		vTaskDelete(NULL);
		return;
	}

	long copied = 0;
	while (!osd->copy_cancel) {
		size_t n = fread(buf, 1, 32768, src);
		if (n == 0) break;
		size_t wr = fwrite(buf, 1, n, dst);
		if (wr != n) {
			printf("[copy] write error: wrote %u of %u, errno=%d ferror=%d\n",
			       (unsigned)wr, (unsigned)n, errno, ferror(dst));
			snprintf(osd->copy_error, sizeof(osd->copy_error),
			         errno == ENOSPC ? "Disk full!" : "Write error (e%d)!", errno);
			break;
		}
		copied += n;
		osd->copy_progress_pct = (int)((copied * 100) / file_size);
	}

	free(buf);
	fclose(src);
	fclose(dst);

	if (osd->copy_cancel || osd->copy_error[0]) {
		unlink(osd->copy_dst_path);
		if (osd->copy_cancel && !osd->copy_error[0])
			snprintf(osd->copy_error, sizeof(osd->copy_error), "Cancelled");
	}
	printf("[copy] done, error='%s'\n", osd->copy_error);
	osd->copy_done = true;
	vTaskDelete(NULL);
}

// Render copy progress dialog
static void render_copy_progress(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	// Check for completion every frame (key handler only runs on keypress)
	if (osd->copy_done) {
		if (osd->copy_error[0]) {
			toast_show(osd->copy_error);
		} else {
			toast_show("File copied!");
		}
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		render_browser(osd, pixels, w, h, pitch);
		return;
	}

	int dialog_w = 300, dialog_h = 100;
	int dx = (w - dialog_w) / 2, dy = (h - dialog_h) / 2;

	// Background + border
	draw_rect(pixels, w, h, pitch, dx, dy, dialog_w, dialog_h, COLOR_PANEL);
	draw_rect(pixels, w, h, pitch, dx, dy, dialog_w, 1, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, dx, dy + dialog_h - 1, dialog_w, 1, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, dx, dy, 1, dialog_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, dx + dialog_w - 1, dy, 1, dialog_h, COLOR_BORDER);

	// Title
	draw_text(pixels, w, h, pitch, dx + 10, dy + 8, "Copying...", COLOR_TITLE, 0);

	// Filename (truncated)
	const char *filename = strrchr(osd->copy_dst_path, '/');
	filename = filename ? filename + 1 : osd->copy_dst_path;
	draw_text(pixels, w, h, pitch, dx + 10, dy + 28, filename, COLOR_TEXT, dialog_w - 20);

	// Progress bar
	int bar_x = dx + 10, bar_y = dy + 50;
	int bar_w = dialog_w - 20, bar_h = 16;
	draw_rect(pixels, w, h, pitch, bar_x, bar_y, bar_w, bar_h, COLOR_BORDER);
	int fill_w = (bar_w - 2) * osd->copy_progress_pct / 100;
	if (fill_w > 0)
		draw_rect(pixels, w, h, pitch, bar_x + 1, bar_y + 1, fill_w, bar_h - 2, COLOR_VALUE);

	// Percentage text
	char pct[8];
	snprintf(pct, sizeof(pct), "%d%%", osd->copy_progress_pct);
	draw_text(pixels, w, h, pitch, dx + dialog_w / 2 - 16, dy + 52, pct, COLOR_TEXT, 0);

	// Help text
	draw_text(pixels, w, h, pitch, dx + 10, dy + dialog_h - 18, "Esc:Cancel", COLOR_SEP, 0);
}

// Render delete confirmation dialog
static void render_delete_confirm(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	// Get just the filename from the path
	const char *filename = strrchr(osd->delete_path, '/');
	if (filename) filename++;
	else filename = osd->delete_path;

	char title[64];
	snprintf(title, sizeof(title), "Delete File?");

	// Truncate filename if too long
	char name_display[40];
	if (strlen(filename) > 35) {
		snprintf(name_display, sizeof(name_display), "...%s", filename + strlen(filename) - 32);
	} else {
		snprintf(name_display, sizeof(name_display), "%s", filename);
	}

	MenuEntry entries[4] = {
		{ name_display, 0, 0, NULL },
		{ NULL, 1, 0, NULL },  // separator
		{ "Press Y to delete", 0, 0, NULL },
		{ "Press N or Esc to cancel", 0, 0, NULL },
	};

	render_generic_menu(pixels, w, h, pitch, title, entries, 4,
	                    -1, "Y:Delete N/Esc:Cancel");
}

/* Minimal MBR bootstrap: finds active partition and chain-loads its VBR
 * at 0000:7C00 via BIOS INT 13h CHS read. */
static const uint8_t tiny386_mbr_bootstrap[] = {
	0xfa, 0x31, 0xc0, 0x8e, 0xd8, 0x8e, 0xc0, 0x8e, 0xd0, 0xbc, 0x00, 0x7c,
	0xfb, 0xbe, 0xbe, 0x07, 0xb9, 0x04, 0x00, 0x80, 0x3c, 0x80, 0x74, 0x07,
	0x83, 0xc6, 0x10, 0xe2, 0xf6, 0xeb, 0x26, 0xbf, 0x00, 0x06, 0x56, 0xb9,
	0x08, 0x00, 0xf3, 0xa5, 0x5e, 0xbe, 0x00, 0x06, 0xbb, 0x00, 0x7c, 0x8a,
	0x74, 0x01, 0x8a, 0x4c, 0x02, 0x8a, 0x6c, 0x03, 0xb4, 0x02, 0xb0, 0x01,
	0xcd, 0x13, 0x72, 0x05, 0xea, 0x00, 0x7c, 0x00, 0x00, 0xf4, 0xeb, 0xfd,
};

// Build MBR with bootstrap and a single partition spanning the disk
static void build_mbr(uint8_t *mbr, int64_t total_bytes, int fs_type, int heads, int spt)
{
	memset(mbr, 0, 512);
	memcpy(mbr, tiny386_mbr_bootstrap, sizeof(tiny386_mbr_bootstrap));

	uint32_t total_sectors = (uint32_t)(total_bytes / 512);
	uint32_t cyl_size = heads * spt;
	uint32_t lba_start = (uint32_t)spt;  // Start at second track (H=1,S=1,C=0)

	// Align partition end to cylinder boundary for reliable CHS geometry detection
	uint32_t total_cyls = total_sectors / cyl_size;
	if (total_cyls < 1) total_cyls = 1;
	uint32_t lba_size = total_cyls * cyl_size - lba_start;

	// CHS start: H=1, S=1, C=0
	uint8_t start_h = 1, start_s = 1, start_c = 0;

	// CHS end: last sector of last cylinder (cylinder-aligned)
	uint32_t c = total_cyls - 1;
	uint32_t h = heads - 1;
	uint32_t s = spt;
	if (c > 1023) { c = 1023; h = heads - 1; s = spt; }

	uint8_t end_h = (uint8_t)h;
	uint8_t end_s = (uint8_t)((s & 0x3F) | ((c >> 2) & 0xC0));
	uint8_t end_c = (uint8_t)(c & 0xFF);

	// Encode CHS start
	uint8_t start_s_enc = (uint8_t)((start_s & 0x3F) | ((start_c >> 2) & 0xC0));
	uint8_t start_c_enc = (uint8_t)(start_c & 0xFF);

	// Partition entry at 0x1BE
	uint8_t *pe = &mbr[0x1BE];
	pe[0] = 0x80;          // Status: active (bootable)
	pe[1] = start_h;       // CHS start head
	pe[2] = start_s_enc;   // CHS start sector + cyl high
	pe[3] = start_c_enc;   // CHS start cylinder low
	pe[4] = fs_type ? 0x0B : 0x06;  // Type: FAT32 CHS or FAT16B
	pe[5] = end_h;         // CHS end head
	pe[6] = end_s;         // CHS end sector + cyl high
	pe[7] = end_c;         // CHS end cylinder low
	// LBA start (little-endian)
	pe[8]  = (uint8_t)(lba_start);
	pe[9]  = (uint8_t)(lba_start >> 8);
	pe[10] = (uint8_t)(lba_start >> 16);
	pe[11] = (uint8_t)(lba_start >> 24);
	// LBA size (little-endian)
	pe[12] = (uint8_t)(lba_size);
	pe[13] = (uint8_t)(lba_size >> 8);
	pe[14] = (uint8_t)(lba_size >> 16);
	pe[15] = (uint8_t)(lba_size >> 24);

	// Boot signature
	mbr[0x1FE] = 0x55;
	mbr[0x1FF] = 0xAA;
}

// Determine the VFS base path ("/sdcard" or "/usb") from a full path
static const char *get_base_path(const char *path)
{
	if (strncmp(path, "/usb", 4) == 0 && (path[4] == '/' || path[4] == '\0'))
		return "/usb";
	return "/sdcard";
}

// Create a new HDD image file
static void do_create_hdd(OSD *osd)
{
	if (osd->create_name_len == 0) {
		toast_show("Enter a filename!");
		return;
	}

	// Build full VFS path: <create_path>/<name>.img
	char path[MAX_PATH_LEN];
	snprintf(path, sizeof(path), "%s/%s.img", osd->create_path, osd->create_name);

	// Check if file already exists
	struct stat st;
	if (stat(path, &st) == 0) {
		toast_show("File already exists!");
		return;
	}

	int64_t size = hdd_sizes[osd->create_size_idx].bytes;
	const char *base = get_base_path(osd->create_path);

	// Allocate file space via VFS (handles FatFS drive mapping internally)
	// Try contiguous first, fall back to non-contiguous
	esp_err_t err = esp_vfs_fat_create_contiguous_file(base, path, (uint64_t)size, true);
	if (err != ESP_OK)
		err = esp_vfs_fat_create_contiguous_file(base, path, (uint64_t)size, false);
	if (err != ESP_OK) {
		unlink(path);  // Clean up partial file if any
		toast_show("Not enough space!");
		return;
	}

	// Write MBR at file offset 0 via standard I/O
	uint8_t mbr[512];
	build_mbr(mbr, size, osd->create_fs_type,
	          hdd_sizes[osd->create_size_idx].heads,
	          hdd_sizes[osd->create_size_idx].spt);
	FILE *f = fopen(path, "r+b");
	if (f) {
		fwrite(mbr, 1, 512, f);
		fclose(f);
	}

	// Auto-set as HDA path
	strncpy(osd->hda_path, path, MAX_PATH_LEN - 1);
	osd->hda_path[MAX_PATH_LEN - 1] = '\0';
	toast_show("HDD image created!");
	osd->view = VIEW_MOUNTING;
}

// Render Create HDD dialog
static void render_create_hdd(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	int line_h = 20;
	int title_h = 28;
	int help_h = 22;
	int padding = 6;
	int menu_w = 380;
	if (menu_w > w - 10) menu_w = w - 10;
	if (menu_w < 100) menu_w = 100;

	// Path info row + 5 selectable rows + separator
	int content_h = title_h + (6 * line_h) + (line_h / 2) + help_h;
	int menu_h = content_h + padding * 2;
	int menu_x = (w - menu_w) / 2;
	if (menu_x < 0) menu_x = 0;
	int menu_y = (h - menu_h) / 2;
	if (menu_y < 0) menu_y = 0;

	// Background
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, menu_w, menu_h, COLOR_PANEL);

	// Border
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, menu_w, 1, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x, menu_y + menu_h - 1, menu_w, 1, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, 1, menu_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x + menu_w - 1, menu_y, 1, menu_h, COLOR_BORDER);

	// Title
	draw_text(pixels, w, h, pitch, menu_x + 10, menu_y + 8, "Create HDD Image", COLOR_TITLE, 0);

	int y = menu_y + title_h;

	// Path info (not selectable)
	draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, "Path:", COLOR_DIM, 0);
	{
		// Show truncated path on the right
		const char *display_path = osd->create_path;
		int path_len = strlen(display_path);
		if (path_len > 28) display_path = display_path + path_len - 28;
		int vw = get_text_width(display_path);
		draw_text(pixels, w, h, pitch, menu_x + menu_w - 10 - vw, y + 2,
		          display_path, COLOR_DIM, 0);
	}
	y += line_h;

	// Row 0: Name field
	if (osd->create_sel == 0)
		draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
	draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, "Name:", COLOR_TEXT, 0);
	{
		char name_display[42];
		snprintf(name_display, sizeof(name_display), "[%s_].img", osd->create_name);
		int vw = get_text_width(name_display);
		draw_text(pixels, w, h, pitch, menu_x + menu_w - 10 - vw, y + 2,
		          name_display, COLOR_VALUE, 0);
	}
	y += line_h;

	// Row 1: Size
	if (osd->create_sel == 1)
		draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
	draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, "Size:", COLOR_TEXT, 0);
	{
		char size_display[24];
		snprintf(size_display, sizeof(size_display), "< %s >", hdd_sizes[osd->create_size_idx].label);
		int vw = get_text_width(size_display);
		draw_text(pixels, w, h, pitch, menu_x + menu_w - 10 - vw, y + 2,
		          size_display, COLOR_VALUE, 0);
	}
	y += line_h;

	// Row 2: Partition type
	if (osd->create_sel == 2)
		draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
	draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, "Type:", COLOR_TEXT, 0);
	{
		const char *type_str = osd->create_fs_type ? "< FAT32 >" : "< FAT16 >";
		int vw = get_text_width(type_str);
		draw_text(pixels, w, h, pitch, menu_x + menu_w - 10 - vw, y + 2,
		          type_str, COLOR_VALUE, 0);
	}
	y += line_h;

	// Separator
	draw_rect(pixels, w, h, pitch, menu_x + 10, y + 8, menu_w - 20, 1, COLOR_SEP);
	y += line_h / 2;

	// Row 3: Create button
	if (osd->create_sel == 3)
		draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
	draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, "[Create]", COLOR_TEXT, 0);
	y += line_h;

	// Row 4: Cancel button
	if (osd->create_sel == 4)
		draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
	draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, "[Cancel]", COLOR_TEXT, 0);

	// Help text
	draw_text(pixels, w, h, pitch, menu_x + 10, menu_y + menu_h - 20,
	          "Type name, L/R:Adjust, Enter:Select", COLOR_SEP, 0);
}

// Handle keys in Create HDD view. Returns 1 to close OSD.
static int handle_create_hdd_key(OSD *osd, int keycode)
{
	switch (keycode) {
	case SC_UP:
		if (osd->create_sel > 0) osd->create_sel--;
		break;
	case SC_DOWN:
		if (osd->create_sel < 4) osd->create_sel++;
		break;
	case SC_LEFT:
		if (osd->create_sel == 1) {
			// Cycle size left
			if (osd->create_size_idx > 0)
				osd->create_size_idx--;
			else
				osd->create_size_idx = HDD_SIZE_COUNT - 1;
		} else if (osd->create_sel == 2) {
			osd->create_fs_type = !osd->create_fs_type;
		}
		break;
	case SC_RIGHT:
		if (osd->create_sel == 1) {
			// Cycle size right
			if (osd->create_size_idx < HDD_SIZE_COUNT - 1)
				osd->create_size_idx++;
			else
				osd->create_size_idx = 0;
		} else if (osd->create_sel == 2) {
			osd->create_fs_type = !osd->create_fs_type;
		}
		break;
	case SC_ENTER:
		if (osd->create_sel == 3) {
			do_create_hdd(osd);
		} else if (osd->create_sel == 4) {
			osd->view = VIEW_MOUNTING;
		}
		break;
	case SC_ESC:
		osd->view = VIEW_MOUNTING;
		break;
	case SC_BACKSPACE:
		if (osd->create_sel == 0 && osd->create_name_len > 0) {
			osd->create_name_len--;
			osd->create_name[osd->create_name_len] = '\0';
		}
		break;
	default:
		// Type characters into name field
		if (osd->create_sel == 0) {
			char ch = scancode_to_char(keycode);
			if (ch && osd->create_name_len < 32) {
				osd->create_name[osd->create_name_len++] = ch;
				osd->create_name[osd->create_name_len] = '\0';
			}
		}
		break;
	}
	return 0;
}

// Render help screen (keyboard shortcuts)
static void render_help(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	// Panel dimensions
	int panel_w = (w > 380) ? 360 : w - 20;
	int panel_h = 340;
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
	line_y += line_h;
	draw_text(pixels, w, h, pitch, panel_x + 20, line_y, "META+Ctrl     Mouse Mode", COLOR_TEXT, 0);
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
		if (osd->files[i].is_dir == FILE_TYPE_CREATE_HERE) {
			snprintf(line, sizeof(line), "  %s", osd->files[i].name);
			color = COLOR_TITLE;  // Yellow for action
		} else if (osd->files[i].is_dir == FILE_TYPE_EJECT) {
			snprintf(line, sizeof(line), "  %s", osd->files[i].name);
			color = COLOR_EJECT;
		} else if (osd->files[i].is_dir == FILE_TYPE_SWITCH_STORAGE) {
			snprintf(line, sizeof(line), "  %s", osd->files[i].name);
			color = COLOR_VALUE;  // Light cyan for storage switch
		} else if (osd->files[i].is_dir) {
			snprintf(line, sizeof(line), "[%s]", osd->files[i].name);
			color = COLOR_DIR;
		} else {
			snprintf(line, sizeof(line), "  %s", osd->files[i].name);
		}

		// Highlight file marked for copy in yellow
		if (osd->copy_src_path[0] && osd->files[i].is_dir == 0) {
			char full[MAX_PATH_LEN];
			snprintf(full, MAX_PATH_LEN, "%s/%s", osd->browser_path, osd->files[i].name);
			if (strcmp(full, osd->copy_src_path) == 0)
				color = COLOR_TITLE;
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
	          "Enter:Sel D:Del C:Copy V:Paste Bk:Up Esc:Back", COLOR_SEP, 0);
}

// Apply brightness setting
static void apply_brightness(OSD *osd)
{
	osd->brightness = clamp_brightness(osd->brightness);
	globals.brightness = osd->brightness;  // Sync to globals
	bsp_display_set_backlight_brightness(brightness_to_bsp_percent(osd->brightness));
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
			if (osd->emulink)
				emulink_attach_floppy(osd->emulink, osd->browser_target, NULL);
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

	if (f->is_dir == FILE_TYPE_CREATE_HERE) {
		// Use current browser_path as save location → open create dialog
		strncpy(osd->create_path, osd->browser_path, MAX_PATH_LEN - 1);
		osd->create_path[MAX_PATH_LEN - 1] = '\0';
		strcpy(osd->create_name, "DISK");
		osd->create_name_len = 4;
		osd->create_size_idx = 3;  // Default: 504 MB
		osd->create_fs_type = 1;   // Default: FAT32
		osd->create_sel = 0;
		osd->view = VIEW_CREATE_HDD;
		return 0;
	}

	if (f->is_dir == FILE_TYPE_SWITCH_STORAGE) {
		// Switch between /sdcard and /usb
		if (strcmp(osd->browser_path, "/sdcard") == 0) {
			strcpy(osd->browser_path, "/usb");
		} else {
			strcpy(osd->browser_path, "/sdcard");
		}
		scan_directory(osd);
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

	// If browser_target is -1, we're just browsing (e.g., USB in VFS mode)
	// Show the path but don't mount - user needs to go to a specific drive to mount
	if (osd->browser_target < 0) {
		char msg[48];
		if (strlen(f->name) > 30) {
			snprintf(msg, sizeof(msg), "...%s", f->name + strlen(f->name) - 27);
		} else {
			snprintf(msg, sizeof(msg), "%s", f->name);
		}
		toast_show(msg);
		return 0;  // Stay in browser
	}

	if (osd->browser_target == BROWSER_TARGET_HDA) {
		// Hard drive selection - save path, user can use Restart Emulator to apply
		strncpy(osd->hda_path, fullpath, MAX_PATH_LEN - 1);
		osd->hda_path[MAX_PATH_LEN - 1] = '\0';

		char msg[64];
		const char *filename = f->name;
		if (strlen(filename) > 24) {
			snprintf(msg, sizeof(msg), "HDD: ...%s", filename + strlen(filename) - 21);
		} else {
			snprintf(msg, sizeof(msg), "HDD: %s", filename);
		}
		toast_show(msg);
		osd->view = VIEW_MOUNTING;
		return 0;
	} else if (osd->browser_target < 2) {
		// Floppy
		if (osd->emulink) {
			if (emulink_attach_floppy(osd->emulink, osd->browser_target, fullpath) != 0) {
				toast_show("Mount failed");
				populate_drive_paths(osd);
				osd->view = VIEW_MOUNTING;
				return 0;
			}
		}
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
	populate_drive_paths(osd);

	switch (osd->mount_sel) {
	case MOUNT_FDA:
	case MOUNT_FDB:
		osd->browser_target = osd->mount_sel;  // 0 or 1
		set_browser_start_dir(osd, osd->browser_target, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		break;
	case MOUNT_HDA:
		// Hard drive - browse for .img files (requires restart)
		osd->browser_target = BROWSER_TARGET_HDA;
		set_browser_start_dir(osd, osd->browser_target, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		break;
	case MOUNT_CREATE_HDD:
		// Open file browser to pick save location, then create dialog
		osd->browser_target = BROWSER_TARGET_CREATE_HDD;
		set_browser_start_dir(osd, BROWSER_TARGET_HDA, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		break;
	case MOUNT_CD:
		// CD-ROM is on slot 1 (primary slave), stored at drive_paths[3]
		// browser_target 3 maps to: ide primary, drive (3-2)%2 = 1 (slave)
		osd->browser_target = 3;
		set_browser_start_dir(osd, osd->browser_target, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		break;
	case MOUNT_USB:
		// USB Storage - managed by USB host, not user-mountable via browser
		// Just show status (or could browse USB contents if mounted)
		if (globals.usb_vfs_mounted) {
			// Browse USB storage contents
			strcpy(osd->browser_path, "/usb");
			osd->browser_target = -1;  // Browse-only mode
			scan_directory(osd);
			osd->view = VIEW_FILEBROWSER;
		}
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
			int rc = save_settings_to_ini(osd->pc->ini_path, boot_order,
			                              osd->hda_path,
			                              osd->drive_paths[0], osd->drive_paths[1],
			                              osd->drive_paths[2], osd->drive_paths[3],
			                              osd->drive_paths[4], osd->drive_paths[5],
			                              osd->cpu_gen, osd->fpu, osd_get_mem_size_bytes(osd),
			                              osd->brightness, osd->volume, osd->frame_skip,
			                              osd->batch_size, osd->pit_burst, osd->mouse_speed,
			                              osd->usb_passthru, osd->accuracy);
			if (rc == 0) {
				// Sync restart-only settings to globals for next restart
				globals.accuracy = osd->accuracy;
				toast_show("Settings saved");
			} else {
				toast_show("Save failed");
			}
		}
		break;
	case MAIN_SWITCH_INI:
		// Close OSD and activate the INI selector (no auto-boot countdown)
		toast_show("Select a configuration...");
		led_activity_off();
		ini_selector_start(false);  // No countdown for manual switch
		return 1;  // Close OSD — selector takes over the screen
	case MAIN_CTRLALTDEL:
		toast_show("Ctrl+Alt+Del sent");
		globals.reset_pending = true;  // Signal soft reset
		return 1;  // Close OSD
	case MAIN_RESTART_EMU:
		toast_show("Restarting emulator...");
		led_activity_off();  // Clear activity LEDs
		// Sync OSD settings to globals so they persist across restart
		globals.accuracy = osd->accuracy;
		// Use full INI switch path (teardown + re-parse) so saved
		// settings like cpu_gen, accuracy, mem_size take effect.
		// Copy current INI path as the "new" INI to reload.
		if (osd->pc && osd->pc->ini_path) {
			strncpy(globals.ini_switch_path, osd->pc->ini_path,
				sizeof(globals.ini_switch_path) - 1);
			globals.ini_switch_path[sizeof(globals.ini_switch_path) - 1] = '\0';
		}
		globals.ini_switch_pending = true;
		return 1;  // Close OSD
	case MAIN_EXIT_LAUNCHER:
		led_activity_off();  // Clear activity LEDs
		i2s_shutdown();  // Silence audio before restart
		esp_restart();
		break;
	}
	return 0;
}

// Handle A/V adjustment with left/right
static void handle_av_adjust(OSD *osd, int delta)
{
	switch (osd->av_sel) {
	case AV_BRIGHTNESS:
		osd->brightness += delta * 5;
		if (osd->brightness < 0) osd->brightness = 0;
		if (osd->brightness > BRIGHTNESS_MAX) osd->brightness = BRIGHTNESS_MAX;
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
	}
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
	case SYS_PIT_BURST:
		osd->pit_burst = !osd->pit_burst;
		// Apply immediately (takes effect next pc_step)
		pc_pit_burst_setting = osd->pit_burst;
		break;
	case SYS_ACCURACY:
		osd->accuracy = !osd->accuracy;
		break;
	case SYS_MOUSE_SPEED:
		osd->mouse_speed += delta;
		if (osd->mouse_speed < 1) osd->mouse_speed = 10;  // Wrap
		if (osd->mouse_speed > 10) osd->mouse_speed = 1;
		// Apply immediately
		mouse_emu_set_speed(osd->mouse_speed);
		globals.mouse_speed = osd->mouse_speed;
		break;
	case SYS_STATS_BAR:
		globals.stats_bar_visible = !globals.stats_bar_visible;
		globals.stats_collecting = globals.stats_bar_visible;
		break;
#ifdef CPU_DIAG
	case SYS_CPU_DEBUG:
		globals.cpu_debug_enabled = !globals.cpu_debug_enabled;
		break;
#endif
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
	// Load brightness/volume from globals (set from INI in esp_main.c)
	osd->brightness = clamp_brightness(globals.brightness);
	osd->volume = globals.volume;
	osd->frame_skip = vga_frame_skip_max;  // Load from config
	osd->batch_size = pc_batch_size_setting;  // Load from config (0=auto)
	osd->pit_burst = pc_pit_burst_setting ? 1 : 0;  // Load from config (1=enabled)
	osd->accuracy = pc_accuracy_setting ? 1 : 0;  // Load from config (0=full)
	osd->boot_accuracy = osd->accuracy;  // Remember boot-time value for reboot indicator
	osd->mouse_speed = globals.mouse_speed > 0 ? globals.mouse_speed : 5;  // Load from config, default 5
	osd->usb_passthru = globals.usb_passthru >= 0 ? globals.usb_passthru : 1;  // Default enabled

	// System settings from config (globals set by esp_main.c after INI parse)
	osd->cpu_gen = globals.cpu_gen > 0 ? globals.cpu_gen : 4;
	osd->fpu = globals.fpu;
	osd->mem_size_mb = globals.mem_size_mb > 0 ? globals.mem_size_mb : 16;

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
	// Initialize hda_path from PC config if not already set by user
	if (osd->hda_path[0] == '\0') {
		PC *p = (PC *)pc;
		if (p->hda_path && p->hda_path[0]) {
			strncpy(osd->hda_path, p->hda_path, MAX_PATH_LEN - 1);
			osd->hda_path[MAX_PATH_LEN - 1] = '\0';
		}
	}
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

void osd_set_hda_path(OSD *osd, const char *hda_path)
{
	if (hda_path && hda_path[0]) {
		strncpy(osd->hda_path, hda_path, MAX_PATH_LEN - 1);
		osd->hda_path[MAX_PATH_LEN - 1] = '\0';
	} else {
		osd->hda_path[0] = '\0';
	}
}

// Get system memory size in bytes for saving
long osd_get_mem_size_bytes(OSD *osd)
{
	return (long)osd->mem_size_mb * 1024 * 1024;
}

void osd_refresh(OSD *osd)
{
	populate_drive_paths(osd);
	// Sync settings from globals/runtime (INI values set by i386_task after boot)
	if (globals.cpu_gen > 0) osd->cpu_gen = globals.cpu_gen;
	osd->fpu = globals.fpu;
	if (globals.mem_size_mb > 0) osd->mem_size_mb = globals.mem_size_mb;
	osd->brightness = clamp_brightness(globals.brightness);
	osd->volume = globals.volume;
	osd->frame_skip = vga_frame_skip_max;
	osd->batch_size = pc_batch_size_setting;
	osd->pit_burst = pc_pit_burst_setting ? 1 : 0;
	osd->accuracy = pc_accuracy_setting ? 1 : 0;
	osd->boot_accuracy = osd->accuracy;  // INI reload = full restart, core re-selected
	osd->mouse_speed = globals.mouse_speed > 0 ? globals.mouse_speed : osd->mouse_speed;
	if (globals.usb_passthru >= 0)
		osd->usb_passthru = globals.usb_passthru;
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
		case SC_LEFT:
			if (osd->mount_sel == MOUNT_BACK) {
				osd->view = VIEW_MAIN_MENU;
			} else if (osd->mount_sel == MOUNT_USB_PASSTHRU) {
				osd->usb_passthru = !osd->usb_passthru;
				globals.usb_passthru = osd->usb_passthru;
			}
			break;
		case SC_RIGHT:
			if (osd->mount_sel == MOUNT_USB_PASSTHRU) {
				osd->usb_passthru = !osd->usb_passthru;
				globals.usb_passthru = osd->usb_passthru;
			} else {
				return handle_mount_select(osd);
			}
			break;
		case SC_ENTER:
			if (osd->mount_sel == MOUNT_USB_PASSTHRU) {
				osd->usb_passthru = !osd->usb_passthru;
				globals.usb_passthru = osd->usb_passthru;
			} else {
				return handle_mount_select(osd);
			}
			break;
		case SC_ESC:
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
				} else if (osd->sys_sel == SYS_PIT_BURST) {
					osd->pit_burst = !osd->pit_burst;
					pc_pit_burst_setting = osd->pit_burst;
				} else if (osd->sys_sel == SYS_ACCURACY) {
					osd->accuracy = !osd->accuracy;
				} else if (osd->sys_sel == SYS_STATS_BAR) {
					globals.stats_bar_visible = !globals.stats_bar_visible;
					globals.stats_collecting = globals.stats_bar_visible;
	#ifdef CPU_DIAG
			} else if (osd->sys_sel == SYS_CPU_DEBUG) {
				globals.cpu_debug_enabled = !globals.cpu_debug_enabled;
#endif
			}
			break;
		case SC_ESC:
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
		case SC_D:
			// Delete selected file (not directories)
			if (osd->file_sel >= 0 && osd->file_sel < osd->file_count) {
				FileEntry *f = &osd->files[osd->file_sel];
				if (!f->is_dir) {
					snprintf(osd->delete_path, MAX_PATH_LEN, "%s/%s",
					         osd->browser_path, f->name);
					osd->view = VIEW_DELETE_CONFIRM;
				}
			}
			break;
		case SC_C:
			// Mark selected file for copy
			if (osd->file_sel >= 0 && osd->file_sel < osd->file_count) {
				FileEntry *f = &osd->files[osd->file_sel];
				if (f->is_dir == 0) {
					snprintf(osd->copy_src_path, MAX_PATH_LEN, "%s/%s",
					         osd->browser_path, f->name);
					char msg[48];
					snprintf(msg, sizeof(msg), "Copied: %.38s", f->name);
					toast_show(msg);
				}
			}
			break;
		case SC_V:
			// Paste copied file into current directory
			if (osd->copy_src_path[0]) {
				const char *fname = strrchr(osd->copy_src_path, '/');
				fname = fname ? fname + 1 : osd->copy_src_path;

				snprintf(osd->copy_dst_path, MAX_PATH_LEN, "%s/%s",
				         osd->browser_path, fname);

				if (strcmp(osd->copy_src_path, osd->copy_dst_path) == 0) {
					toast_show("Same location!");
					break;
				}

				struct stat st;
				if (stat(osd->copy_src_path, &st) != 0) {
					toast_show("Source not found!");
					break;
				}
				if (st.st_size > (1024*1024*1024L)) {
					toast_show("File too large (>1GB)!");
					break;
				}
				if (stat(osd->copy_dst_path, &st) == 0) {
					toast_show("File already exists!");
					break;
				}

				// Check free space on destination filesystem
				{
					const char *base = "/sdcard";
					if (strncmp(osd->browser_path, "/usb", 4) == 0) base = "/usb";
					uint64_t total, free_bytes;
					if (esp_vfs_fat_info(base, &total, &free_bytes) == ESP_OK) {
						if (free_bytes < (uint64_t)st.st_size) {
							char msg[48];
							snprintf(msg, sizeof(msg), "No space! %uMB free, need %uMB",
							         (unsigned)(free_bytes / (1024*1024)),
							         (unsigned)((st.st_size + 1024*1024 - 1) / (1024*1024)));
							toast_show(msg);
							break;
						}
					}
				}

				osd->copy_progress_pct = 0;
				osd->copy_done = false;
				osd->copy_cancel = false;
				osd->copy_error[0] = '\0';
				osd->view = VIEW_COPY_PROGRESS;
				if (xTaskCreatePinnedToCore(copy_file_task, "filecopy", 8192,
				                            osd, 5, NULL, 0) != pdPASS) {
					toast_show("Can't start copy!");
					osd->view = VIEW_FILEBROWSER;
				}
			} else {
				toast_show("Nothing to paste!");
			}
			break;
		case SC_ESC:
		case SC_LEFT:
			osd->view = VIEW_MOUNTING;
			break;
		}
		break;

	case VIEW_DELETE_CONFIRM:
		switch (keycode) {
		case SC_Y:
			// Delete the file
			if (unlink(osd->delete_path) == 0) {
				toast_show("File deleted");
			} else {
				toast_show("Delete failed!");
			}
			// Refresh directory and return to browser
			scan_directory(osd);
			osd->view = VIEW_FILEBROWSER;
			break;
		case SC_N:
		case SC_ESC:
			// Cancel - return to browser
			osd->view = VIEW_FILEBROWSER;
			break;
		}
		break;

	case VIEW_CREATE_HDD:
		return handle_create_hdd_key(osd, keycode);

	case VIEW_COPY_PROGRESS:
		if (keycode == SC_ESC) {
			osd->copy_cancel = true;
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
	case VIEW_HELP:
		render_help(osd, pixels, w, h, pitch);
		break;
	case VIEW_FILEBROWSER:
		render_browser(osd, pixels, w, h, pitch);
		break;
	case VIEW_DELETE_CONFIRM:
		render_delete_confirm(osd, pixels, w, h, pitch);
		break;
	case VIEW_CREATE_HDD:
		render_create_hdd(osd, pixels, w, h, pitch);
		break;
	case VIEW_COPY_PROGRESS:
		render_copy_progress(osd, pixels, w, h, pitch);
		break;
	}
}
