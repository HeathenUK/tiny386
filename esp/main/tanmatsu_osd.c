/*
 * Tanmatsu-specific OSD: Custom keyboard-driven menu with file browser
 * For devices without mouse/touch input
 */

#include "tanmatsu_osd.h"
#include <stdlib.h>
#include <stdio.h>

// Define types needed by atlas.inl (normally from microui.h)
typedef struct { int x, y, w, h; } mu_Rect;
enum {
	MU_ICON_CLOSE = 1,
	MU_ICON_CHECK,
	MU_ICON_COLLAPSED,
	MU_ICON_EXPANDED,
	MU_ICON_MAX
};

// Include the atlas data (font texture and glyph rects)
#include "../../osd/atlas.inl"
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common.h"

#include "../../vga.h"
#include "../../misc.h"
#include "../../ide.h"
#include "../../pc.h"

// Menu items
typedef enum {
	MENU_FDA = 0,
	MENU_FDB,
	MENU_CDA,
	MENU_CDB,
	MENU_CDC,
	MENU_CDD,
	MENU_SEP1,
	MENU_BOOT_ORDER,
	MENU_SAVE_INI,
	MENU_SEP2,
	MENU_CTRLALTDEL,
	MENU_RESET,
	MENU_SEP3,
	MENU_EXIT,
	MENU_COUNT
} MenuItem;

// View modes
typedef enum {
	VIEW_MENU,
	VIEW_FILEBROWSER
} ViewMode;

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

	// Main menu state
	int menu_sel;
	char drive_paths[6][MAX_PATH_LEN];  // FDA, FDB, CDA-CDD

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

// Colors (RGB565)
#define COLOR_BG      0x0000  // Black
#define COLOR_BORDER  0x4208  // Dark gray
#define COLOR_TEXT    0xFFFF  // White
#define COLOR_HILITE  0x001F  // Blue
#define COLOR_SEP     0x4208  // Gray
#define COLOR_DIR     0x07E0  // Green
#define COLOR_TITLE   0xFFE0  // Yellow
#define COLOR_EJECT   0xF800  // Red

// Forward declarations
static void populate_drive_paths(OSD *osd);
static void scan_directory(OSD *osd);

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

// Compare function for sorting files (directories first, then alphabetical)
static int file_compare(const void *a, const void *b)
{
	const FileEntry *fa = a;
	const FileEntry *fb = b;
	// ".." always first
	if (strcmp(fa->name, "..") == 0) return -1;
	if (strcmp(fb->name, "..") == 0) return 1;
	// Directories before files
	if (fa->is_dir != fb->is_dir) return fb->is_dir - fa->is_dir;
	// Alphabetical (case-insensitive)
	return strcasecmp(fa->name, fb->name);
}

// Scan current directory and populate file list
static void scan_directory(OSD *osd)
{
	osd->file_count = 0;
	osd->file_sel = 0;
	osd->file_scroll = 0;

	// Add "[Eject]" option if drive currently has an image mounted
	int drive_idx = osd->browser_target;
	if (drive_idx >= MENU_FDA && drive_idx <= MENU_CDD) {
		int path_idx = drive_idx;  // FDA=0, FDB=1, CDA=2, etc.
		if (osd->drive_paths[path_idx][0] != '\0') {
			strcpy(osd->files[osd->file_count].name, "[Eject]");
			osd->files[osd->file_count].is_dir = FILE_TYPE_EJECT;
			osd->file_count++;
		}
	}

	DIR *dir = opendir(osd->browser_path);
	if (!dir) {
		// Try falling back to root
		strcpy(osd->browser_path, "/sdcard");
		dir = opendir(osd->browser_path);
		if (!dir) return;
	}

	// Add ".." for parent directory (unless at root)
	if (strcmp(osd->browser_path, "/") != 0 && strcmp(osd->browser_path, "/sdcard") != 0) {
		strcpy(osd->files[osd->file_count].name, "..");
		osd->files[osd->file_count].is_dir = 1;
		osd->file_count++;
	}

	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL && osd->file_count < MAX_FILES) {
		// Skip hidden files and . entry
		if (ent->d_name[0] == '.') continue;

		strncpy(osd->files[osd->file_count].name, ent->d_name, MAX_FILENAME - 1);
		osd->files[osd->file_count].name[MAX_FILENAME - 1] = '\0';

		// Check if directory
		char fullpath[MAX_PATH_LEN + MAX_FILENAME];
		snprintf(fullpath, sizeof(fullpath), "%s/%s", osd->browser_path, ent->d_name);
		struct stat st;
		if (stat(fullpath, &st) == 0) {
			osd->files[osd->file_count].is_dir = S_ISDIR(st.st_mode);
		} else {
			osd->files[osd->file_count].is_dir = 0;
		}
		osd->file_count++;
	}
	closedir(dir);

	// Sort: directories first, then alphabetical
	if (osd->file_count > 0) {
		qsort(osd->files, osd->file_count, sizeof(FileEntry), file_compare);
	}
}

// Text rendering using atlas
static void draw_char(uint8_t *pixels, int w, int h, int pitch,
                      int x, int y, char c, uint16_t color)
{
	int chr = (unsigned char)c;
	if (chr > 127) chr = '?';

	int srcx = atlas[ATLAS_FONT + chr].x;
	int srcy = atlas[ATLAS_FONT + chr].y;
	int srcw = atlas[ATLAS_FONT + chr].w;
	int srch = atlas[ATLAS_FONT + chr].h;

	for (int dy = 0; dy < srch && y + dy < h; dy++) {
		if (y + dy < 0) continue;
		for (int dx = 0; dx < srcw && x + dx < w; dx++) {
			if (x + dx < 0) continue;
			uint8_t alpha = atlas_texture[(srcy + dy) * 128 + (srcx + dx)];
			if (alpha > 128) {
#if BPP == 16
				*(uint16_t *)&pixels[(y + dy) * pitch + 2 * (x + dx)] = color;
#elif BPP == 32
				uint32_t c32 = (color & 0x1F) << 3 | ((color >> 5) & 0x3F) << 10 |
				               ((color >> 11) & 0x1F) << 19 | 0xFF000000;
				*(uint32_t *)&pixels[(y + dy) * pitch + 4 * (x + dx)] = c32;
#endif
			}
		}
	}
}

static int draw_text(uint8_t *pixels, int w, int h, int pitch,
                     int x, int y, const char *text, uint16_t color, int max_width)
{
	int startx = x;
	for (const char *p = text; *p && (max_width == 0 || x - startx < max_width - 8); p++) {
		if ((*p & 0xc0) == 0x80) continue;  // Skip UTF-8 continuation bytes
		draw_char(pixels, w, h, pitch, x, y, *p, color);
		int chr = (unsigned char)*p;
		if (chr > 127) chr = '?';
		x += atlas[ATLAS_FONT + chr].w;
	}
	return x - startx;
}

static void draw_rect(uint8_t *pixels, int w, int h, int pitch,
                      int x, int y, int rw, int rh, uint16_t color)
{
	for (int dy = 0; dy < rh && y + dy < h; dy++) {
		if (y + dy < 0) continue;
		for (int dx = 0; dx < rw && x + dx < w; dx++) {
			if (x + dx < 0) continue;
#if BPP == 16
			*(uint16_t *)&pixels[(y + dy) * pitch + 2 * (x + dx)] = color;
#elif BPP == 32
			uint32_t c32 = (color & 0x1F) << 3 | ((color >> 5) & 0x3F) << 10 |
			               ((color >> 11) & 0x1F) << 19 | 0xFF000000;
			*(uint32_t *)&pixels[(y + dy) * pitch + 4 * (x + dx)] = c32;
#endif
		}
	}
}

// Check if menu item is a separator
static int is_separator(int item)
{
	return item == MENU_SEP1 || item == MENU_SEP2 || item == MENU_SEP3;
}

// Get menu item label
static const char *get_menu_label(OSD *osd, int item)
{
	static char buf[80];
	switch (item) {
	case MENU_FDA:
		snprintf(buf, sizeof(buf), "FDA: %s", basename_ptr(osd->drive_paths[0]));
		return buf;
	case MENU_FDB:
		snprintf(buf, sizeof(buf), "FDB: %s", basename_ptr(osd->drive_paths[1]));
		return buf;
	case MENU_CDA:
		snprintf(buf, sizeof(buf), "CDA: %s", basename_ptr(osd->drive_paths[2]));
		return buf;
	case MENU_CDB:
		snprintf(buf, sizeof(buf), "CDB: %s", basename_ptr(osd->drive_paths[3]));
		return buf;
	case MENU_CDC:
		snprintf(buf, sizeof(buf), "CDC: %s", basename_ptr(osd->drive_paths[4]));
		return buf;
	case MENU_CDD:
		snprintf(buf, sizeof(buf), "CDD: %s", basename_ptr(osd->drive_paths[5]));
		return buf;
	case MENU_BOOT_ORDER:
		{
			const char *order_str = boot_order_names[0];
			if (osd->pc && osd->pc->cmos) {
				int order = cmos_get_boot_order(osd->pc->cmos);
				if (order >= 0 && order < BOOT_ORDER_COUNT)
					order_str = boot_order_names[order];
			}
			snprintf(buf, sizeof(buf), "Boot: %s", order_str);
			return buf;
		}
	case MENU_SAVE_INI: return "Save Settings to INI";
	case MENU_CTRLALTDEL: return "Send Ctrl+Alt+Del";
	case MENU_RESET: return "Reset Emulator";
	case MENU_EXIT: return "Reboot ESP32";
	default: return "";
	}
}

// Render main menu
static void render_menu(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	int menu_x = 40;
	int menu_y = 40;
	int menu_w = 360;
	int menu_h = 280;
	int line_h = 22;

	// Background
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, menu_w, menu_h, COLOR_BG);

	// Border
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, menu_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x, menu_y + menu_h - 2, menu_w, 2, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x, menu_y, 2, menu_h, COLOR_BORDER);
	draw_rect(pixels, w, h, pitch, menu_x + menu_w - 2, menu_y, 2, menu_h, COLOR_BORDER);

	// Title
	draw_text(pixels, w, h, pitch, menu_x + 10, menu_y + 8, "tiny386", COLOR_TITLE, 0);

	// Menu items
	int y = menu_y + 32;
	for (int i = 0; i < MENU_COUNT; i++) {
		if (is_separator(i)) {
			// Draw separator line
			draw_rect(pixels, w, h, pitch, menu_x + 10, y + 8, menu_w - 20, 1, COLOR_SEP);
			y += line_h / 2;
		} else {
			// Highlight selected item
			if (i == osd->menu_sel) {
				draw_rect(pixels, w, h, pitch, menu_x + 4, y, menu_w - 8, line_h, COLOR_HILITE);
			}

			// Draw label
			const char *label = get_menu_label(osd, i);
			draw_text(pixels, w, h, pitch, menu_x + 10, y + 2, label, COLOR_TEXT, menu_w - 20);
			y += line_h;
		}
	}

	// Help text
	draw_text(pixels, w, h, pitch, menu_x + 10, menu_y + menu_h - 20,
	          "Up/Down:Select  Enter:Open  Meta:Close", COLOR_SEP, 0);
}

// Render file browser
static void render_browser(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	int browser_x = 30;
	int browser_y = 30;
	int browser_w = 380;
	int browser_h = 300;
	int line_h = 18;

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
	int list_y = browser_y + 32;
	int list_h = browser_h - 60;
	osd->files_visible = list_h / line_h;

	// Adjust scroll to keep selection visible
	if (osd->file_sel < osd->file_scroll) {
		osd->file_scroll = osd->file_sel;
	} else if (osd->file_sel >= osd->file_scroll + osd->files_visible) {
		osd->file_scroll = osd->file_sel - osd->files_visible + 1;
	}

	// Draw file list
	int y = list_y;
	for (int i = osd->file_scroll; i < osd->file_count && i < osd->file_scroll + osd->files_visible; i++) {
		// Highlight selected
		if (i == osd->file_sel) {
			draw_rect(pixels, w, h, pitch, browser_x + 4, y, browser_w - 8, line_h, COLOR_HILITE);
		}

		// Icon/prefix for directories and special entries
		uint16_t color;
		const char *prefix;
		if (osd->files[i].is_dir == FILE_TYPE_EJECT) {
			color = COLOR_EJECT;
			prefix = "";
		} else if (osd->files[i].is_dir == 1) {
			color = COLOR_DIR;
			prefix = "[D] ";
		} else {
			color = COLOR_TEXT;
			prefix = "    ";
		}

		char line[MAX_FILENAME + 8];
		snprintf(line, sizeof(line), "%s%s", prefix, osd->files[i].name);
		draw_text(pixels, w, h, pitch, browser_x + 8, y + 1, line, color, browser_w - 16);

		y += line_h;
	}

	// Scroll indicator
	if (osd->file_count > osd->files_visible) {
		int scroll_h = list_h * osd->files_visible / osd->file_count;
		if (scroll_h < 10) scroll_h = 10;
		int scroll_y = list_y + (list_h - scroll_h) * osd->file_scroll / (osd->file_count - osd->files_visible);
		draw_rect(pixels, w, h, pitch, browser_x + browser_w - 8, scroll_y, 4, scroll_h, COLOR_SEP);
	}

	// Help text
	draw_text(pixels, w, h, pitch, browser_x + 8, browser_y + browser_h - 20,
	          "Enter:Select  Backspace:Up  Esc:Cancel", COLOR_SEP, 0);
}

// External function to send keypresses to emulator (defined in input_bsp.c)
extern void console_send_kbd(void *opaque, int keypress, int keycode);

// Send Ctrl+Alt+Del
static void send_ctrl_alt_del(OSD *osd)
{
	(void)osd;
	console_send_kbd(NULL, 1, 0x1d);  // Ctrl down
	vTaskDelay(pdMS_TO_TICKS(10));
	console_send_kbd(NULL, 1, 0x38);  // Alt down
	vTaskDelay(pdMS_TO_TICKS(10));
	// Extended Delete key (E0 53)
	console_send_kbd(NULL, 1, 0xe0);  // E0 prefix
	vTaskDelay(pdMS_TO_TICKS(5));
	console_send_kbd(NULL, 1, 0x53);  // Del down
	vTaskDelay(pdMS_TO_TICKS(10));
	console_send_kbd(NULL, 1, 0xe0);  // E0 prefix
	vTaskDelay(pdMS_TO_TICKS(5));
	console_send_kbd(NULL, 0, 0x53);  // Del up
	vTaskDelay(pdMS_TO_TICKS(10));
	console_send_kbd(NULL, 0, 0x38);  // Alt up
	vTaskDelay(pdMS_TO_TICKS(10));
	console_send_kbd(NULL, 0, 0x1d);  // Ctrl up
}

// Soft reset the emulator (not full ESP32 reboot)
static void do_reset(OSD *osd)
{
	(void)osd;
	ESP_LOGI("OSD", "Soft reset requested");
	globals.reset_pending = true;
}

// Handle menu selection
static int handle_menu_select(OSD *osd)
{
	switch (osd->menu_sel) {
	case MENU_FDA:
	case MENU_FDB:
	case MENU_CDA:
	case MENU_CDB:
	case MENU_CDC:
	case MENU_CDD:
		// Open file browser for this drive
		osd->browser_target = osd->menu_sel;
		strcpy(osd->browser_path, "/sdcard");
		scan_directory(osd);
		osd->view = VIEW_FILEBROWSER;
		return 0;

	case MENU_BOOT_ORDER:
		// Cycle through all boot order permutations
		ESP_LOGI("OSD", "Boot order: pc=%p cmos=%p", osd->pc, osd->pc ? osd->pc->cmos : NULL);
		if (osd->pc && osd->pc->cmos) {
			int order = cmos_get_boot_order(osd->pc->cmos);
			ESP_LOGI("OSD", "Current boot order: %d", order);
			order = (order + 1) % BOOT_ORDER_COUNT;
			cmos_set_boot_order(osd->pc->cmos, order);
			ESP_LOGI("OSD", "New boot order: %d", order);
		} else {
			ESP_LOGE("OSD", "Cannot change boot order: pc or cmos is NULL");
		}
		return 0;  // Stay in OSD to see the change

	case MENU_SAVE_INI:
		// Save all current settings to ini file
		ESP_LOGI("OSD", "Save INI: pc=%p ini_path=%s", osd->pc, osd->pc ? osd->pc->ini_path : "NULL");
		if (osd->pc && osd->pc->ini_path) {
			int boot_order = osd->pc->cmos ? cmos_get_boot_order(osd->pc->cmos) : 0;
			ESP_LOGI("OSD", "Saving: boot_order=%d", boot_order);
			int ret = save_settings_to_ini(osd->pc->ini_path, boot_order,
			                     osd->drive_paths[0], osd->drive_paths[1],
			                     osd->drive_paths[2], osd->drive_paths[3],
			                     osd->drive_paths[4], osd->drive_paths[5]);
			ESP_LOGI("OSD", "save_settings_to_ini returned %d", ret);
		} else {
			ESP_LOGE("OSD", "Cannot save: pc or ini_path is NULL");
		}
		return 0;  // Stay in OSD

	case MENU_CTRLALTDEL:
		ESP_LOGI("OSD", "Sending Ctrl+Alt+Del");
		send_ctrl_alt_del(osd);
		ESP_LOGI("OSD", "Ctrl+Alt+Del sent");
		return 1;  // Close OSD

	case MENU_RESET:
		do_reset(osd);
		return 1;  // Close OSD after reset

	case MENU_EXIT:
		ESP_LOGI("OSD", "Full ESP32 reboot");
		esp_restart();
		return 1;  // Won't reach here
	}
	return 0;
}

// Handle file selection
static int handle_file_select(OSD *osd)
{
	if (osd->file_sel < 0 || osd->file_sel >= osd->file_count) return 0;

	FileEntry *sel = &osd->files[osd->file_sel];

	if (sel->is_dir == FILE_TYPE_EJECT) {
		// Eject/unmount the current image
		switch (osd->browser_target) {
		case MENU_FDA:
			if (osd->emulink) emulink_attach_floppy(osd->emulink, 0, NULL);
			break;
		case MENU_FDB:
			if (osd->emulink) emulink_attach_floppy(osd->emulink, 1, NULL);
			break;
		case MENU_CDA:
			if (osd->ide) ide_change_cd(osd->ide, 0, "");
			break;
		case MENU_CDB:
			if (osd->ide) ide_change_cd(osd->ide, 1, "");
			break;
		case MENU_CDC:
			if (osd->ide2) ide_change_cd(osd->ide2, 0, "");
			break;
		case MENU_CDD:
			if (osd->ide2) ide_change_cd(osd->ide2, 1, "");
			break;
		}
		// Update drive paths and return to menu
		populate_drive_paths(osd);
		osd->view = VIEW_MENU;
		return 0;
	} else if (sel->is_dir == 1) {
		// Navigate into directory
		if (strcmp(sel->name, "..") == 0) {
			// Go up
			char *slash = strrchr(osd->browser_path, '/');
			if (slash && slash != osd->browser_path) {
				*slash = '\0';
			}
		} else {
			// Go into subdirectory
			int len = strlen(osd->browser_path);
			if (len < MAX_PATH_LEN - MAX_FILENAME - 2) {
				strcat(osd->browser_path, "/");
				strcat(osd->browser_path, sel->name);
			}
		}
		scan_directory(osd);
		return 0;
	} else {
		// Select this file
		char fullpath[MAX_PATH_LEN + MAX_FILENAME];
		snprintf(fullpath, sizeof(fullpath), "%s/%s", osd->browser_path, sel->name);

		// Mount the file
		switch (osd->browser_target) {
		case MENU_FDA:
			if (osd->emulink) emulink_attach_floppy(osd->emulink, 0, fullpath);
			break;
		case MENU_FDB:
			if (osd->emulink) emulink_attach_floppy(osd->emulink, 1, fullpath);
			break;
		case MENU_CDA:
			if (osd->ide) ide_change_cd(osd->ide, 0, fullpath);
			break;
		case MENU_CDB:
			if (osd->ide) ide_change_cd(osd->ide, 1, fullpath);
			break;
		case MENU_CDC:
			if (osd->ide2) ide_change_cd(osd->ide2, 0, fullpath);
			break;
		case MENU_CDD:
			if (osd->ide2) ide_change_cd(osd->ide2, 1, fullpath);
			break;
		}

		// Update drive paths and return to menu
		populate_drive_paths(osd);
		osd->view = VIEW_MENU;
		return 0;
	}
}

// Public API
OSD *osd_init(void)
{
	OSD *osd = calloc(1, sizeof(OSD));
	osd->view = VIEW_MENU;
	osd->menu_sel = MENU_FDA;
	strcpy(osd->browser_path, "/sdcard");
	return osd;
}

void osd_attach_emulink(OSD *osd, void *emulink)
{
	osd->emulink = emulink;
}

void osd_attach_ide(OSD *osd, void *ide, void *ide2)
{
	osd->ide = ide;
	osd->ide2 = ide2;
}

void osd_attach_pc(OSD *osd, void *pc)
{
	osd->pc = pc;
}

void osd_attach_console(OSD *osd, void *console)
{
	osd->console = console;
}

void osd_refresh(OSD *osd)
{
	populate_drive_paths(osd);
}

void osd_handle_mouse_motion(OSD *osd, int x, int y)
{
	(void)osd; (void)x; (void)y;
	// Not used on Tanmatsu
}

void osd_handle_mouse_button(OSD *osd, int x, int y, int down, int btn)
{
	(void)osd; (void)x; (void)y; (void)down; (void)btn;
	// Not used on Tanmatsu
}

// Returns 1 if OSD should close
int osd_handle_key(OSD *osd, int keycode, int down)
{
	if (!down) return 0;  // Only handle key press, not release

	if (osd->view == VIEW_MENU) {
		switch (keycode) {
		case SC_UP:
			do {
				osd->menu_sel--;
				if (osd->menu_sel < 0) osd->menu_sel = MENU_COUNT - 1;
			} while (is_separator(osd->menu_sel));
			break;

		case SC_DOWN:
			do {
				osd->menu_sel++;
				if (osd->menu_sel >= MENU_COUNT) osd->menu_sel = 0;
			} while (is_separator(osd->menu_sel));
			break;

		case SC_ENTER:
			return handle_menu_select(osd);

		case SC_ESC:
			return 1;  // Close OSD
		}
	} else if (osd->view == VIEW_FILEBROWSER) {
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
			// Go up a directory
			{
				char *slash = strrchr(osd->browser_path, '/');
				if (slash && slash != osd->browser_path) {
					*slash = '\0';
					scan_directory(osd);
				}
			}
			break;

		case SC_ESC:
			// Back to menu
			osd->view = VIEW_MENU;
			break;
		}
	}

	return 0;
}

void osd_render(OSD *osd, uint8_t *pixels, int w, int h, int pitch)
{
	if (osd->view == VIEW_MENU) {
		render_menu(osd, pixels, w, h, pitch);
	} else if (osd->view == VIEW_FILEBROWSER) {
		render_browser(osd, pixels, w, h, pitch);
	}
}
