/*
 * INI File Selector Screen
 * Scans /sdcard/tiny386 for .ini files, displays them sorted by modification time
 * (newest first), and shows a settings preview panel for the highlighted file.
 * Supports 5-second auto-boot countdown at startup.
 */

#include "ini_selector.h"
#include "common.h"
#include "../../ini.h"
#include "vga_font_8x16.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ini_sel";

/* ── Layout Constants (logical 800×480, rotated to 480×800 physical) ── */
#define FONT_W 8
#define FONT_H 16

/* Physical display is 480 wide × 800 tall (portrait).
 * We render in logical 800×480 landscape and rotate 90° CCW, same as OSD. */
#define LOG_W 800
#define LOG_H 480

/* Panels */
#define LEFT_X     10
#define LEFT_W    300
#define RIGHT_X   320
#define RIGHT_W   470
#define TITLE_Y    10
#define LIST_Y     36
#define FOOTER_Y  455

/* Colours (RGB565) */
#define C_BG      0x0000
#define C_PANEL   0x18E3
#define C_BORDER  0x4A69
#define C_TEXT    0xFFFF
#define C_HILITE  0x04FF
#define C_TITLE   0xFFE0
#define C_VALUE   0x87FF
#define C_DIM     0x8410
#define C_SEP     0x39E7

/* ── File list ───────────────────────────────────────────────────────── */
#define MAX_INI_FILES 32
#define MAX_INI_NAME  64

typedef struct {
	char name[MAX_INI_NAME];    /* filename only */
	char path[256];             /* full VFS path */
	time_t mtime;
	/* Preview fields (parsed lazily) */
	bool parsed;
	bool valid;                 /* ini_parse succeeded */
	char cpu[16];
	char fpu[12];
	char mem[16];
	char hda[52];
	char cda[52];
	char fda[52];
	char bios[52];
	char vgabios[52];
	char boot[52];
} IniFileEntry;

/* ── Selector state (file-scoped) ─────────────────────────────────── */
static IniFileEntry s_files[MAX_INI_FILES];
static int  s_count;
static int  s_sel;
static int  s_scroll;
static bool s_auto_boot;
static int  s_countdown;            /* seconds remaining (5→0, -1 = cancelled) */
static uint32_t s_countdown_tick;   /* timestamp of last countdown update */

/* How many list entries fit on screen */
#define VISIBLE_LINES ((FOOTER_Y - LIST_Y) / (FONT_H + 2))

/* ── Portrait coordinate transform (same as toast.c) ────────────── */
static uint16_t *s_fb;
static int s_phys_w, s_phys_h;

static inline void sel_put_pixel(int lx, int ly, uint16_t color)
{
	/* Rotate 90 CCW: logical (x,y) in 800×480 → buffer (phys_w-1-y, x) */
	int bx = s_phys_w - 1 - ly;
	int by = lx;
	if (bx < 0 || bx >= s_phys_w || by < 0 || by >= s_phys_h) return;
	s_fb[by * s_phys_w + bx] = color;
}

static void sel_fill_rect(int x, int y, int w, int h, uint16_t color)
{
	for (int dy = 0; dy < h; dy++) {
		int ly = y + dy;
		if (ly < 0 || ly >= LOG_H) continue;
		for (int dx = 0; dx < w; dx++) {
			int lx = x + dx;
			if (lx < 0 || lx >= LOG_W) continue;
			sel_put_pixel(lx, ly, color);
		}
	}
}

static int sel_draw_text(int x, int y, const char *text, uint16_t color, int max_w)
{
	int start_x = x;
	while (*text) {
		unsigned char c = *text++;
		if (c < 32) continue;
		if (c > 126) c = '?';
		if (max_w > 0 && (x - start_x + FONT_W) > max_w) break;
		const uint8_t *glyph = vga_font_8x16_data[c - 32];
		for (int dy = 0; dy < FONT_H; dy++) {
			int ly = y + dy;
			if (ly < 0 || ly >= LOG_H) continue;
			uint8_t row = glyph[dy];
			for (int dx = 0; dx < FONT_W; dx++) {
				if (row & (0x80 >> dx)) {
					int lx = x + dx;
					if (lx >= 0 && lx < LOG_W)
						sel_put_pixel(lx, ly, color);
				}
			}
		}
		x += FONT_W;
	}
	return x - start_x;
}

/* ── INI preview parsing ─────────────────────────────────────────── */
typedef struct {
	IniFileEntry *entry;
} PreviewCtx;

static int preview_handler(void *user, const char *section,
                           const char *name, const char *value)
{
	PreviewCtx *ctx = user;
	IniFileEntry *e = ctx->entry;
#define SEC(a) (strcmp(section, a) == 0)
#define NM(a)  (strcmp(name, a) == 0)

	if (SEC("pc")) {
		if (NM("bios")) {
			const char *p = strrchr(value, '/');
			snprintf(e->bios, sizeof(e->bios), "%s", p ? p + 1 : value);
		} else if (NM("vga_bios")) {
			const char *p = strrchr(value, '/');
			snprintf(e->vgabios, sizeof(e->vgabios), "%s", p ? p + 1 : value);
		} else if (NM("mem_size")) {
			snprintf(e->mem, sizeof(e->mem), "%s", value);
		} else if (NM("hda")) {
			const char *p = strrchr(value, '/');
			snprintf(e->hda, sizeof(e->hda), "%s", p ? p + 1 : value);
		} else if (NM("cda")) {
			const char *p = strrchr(value, '/');
			snprintf(e->cda, sizeof(e->cda), "%s", p ? p + 1 : value);
		} else if (NM("fda")) {
			const char *p = strrchr(value, '/');
			snprintf(e->fda, sizeof(e->fda), "%s", p ? p + 1 : value);
		} else if (NM("boot_order")) {
			snprintf(e->boot, sizeof(e->boot), "%s", value);
		}
	} else if (SEC("cpu")) {
		if (NM("gen")) {
			int g = atoi(value);
			const char *names[] = { "i386", "i486", "i586", "i586-ext" };
			int idx = g - 3;
			if (idx < 0) idx = 0;
			if (idx > 3) idx = 3;
			snprintf(e->cpu, sizeof(e->cpu), "%s", names[idx]);
		} else if (NM("fpu")) {
			snprintf(e->fpu, sizeof(e->fpu), "%s", atoi(value) ? "On" : "Off");
		}
	}
#undef SEC
#undef NM
	return 1;
}

static void parse_preview(IniFileEntry *e)
{
	if (e->parsed) return;
	e->parsed = true;

	/* Defaults */
	strcpy(e->cpu, "i486");
	strcpy(e->fpu, "Off");
	strcpy(e->mem, "8M");
	e->hda[0] = e->cda[0] = e->fda[0] = 0;
	strcpy(e->bios, "(default)");
	strcpy(e->vgabios, "(default)");
	e->boot[0] = 0;

	PreviewCtx ctx = { .entry = e };
	int ret = ini_parse(e->path, preview_handler, &ctx);
	e->valid = (ret == 0);
	if (!e->valid) {
		ESP_LOGW(TAG, "Failed to parse %s (line %d)", e->path, ret);
	}
}

/* ── Directory scanning ──────────────────────────────────────────── */
static int cmp_by_mtime_desc(const void *a, const void *b)
{
	const IniFileEntry *ea = (const IniFileEntry *)a;
	const IniFileEntry *eb = (const IniFileEntry *)b;
	if (eb->mtime > ea->mtime) return 1;
	if (eb->mtime < ea->mtime) return -1;
	return strcasecmp(ea->name, eb->name);
}

static bool s_dir_missing;  /* true when /sdcard/tiny386 doesn't exist */

static void scan_ini_files(void)
{
	s_count = 0;
	s_sel = 0;
	s_scroll = 0;
	s_dir_missing = false;

	DIR *dir = opendir("/sdcard/tiny386");
	if (!dir) {
		ESP_LOGE(TAG, "Failed to open /sdcard/tiny386 — directory missing");
		s_dir_missing = true;
		return;
	}

	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL && s_count < MAX_INI_FILES) {
		if (ent->d_name[0] == '.') continue;

		/* Check for .ini extension (case-insensitive) */
		size_t len = strlen(ent->d_name);
		if (len < 5) continue;
		const char *ext = ent->d_name + len - 4;
		if (strcasecmp(ext, ".ini") != 0) continue;

		char fullpath[256];
		snprintf(fullpath, sizeof(fullpath), "/sdcard/tiny386/%s", ent->d_name);

		struct stat st;
		if (stat(fullpath, &st) != 0) continue;
		if (S_ISDIR(st.st_mode)) continue;

		IniFileEntry *e = &s_files[s_count];
		strncpy(e->name, ent->d_name, MAX_INI_NAME - 1);
		e->name[MAX_INI_NAME - 1] = '\0';
		strncpy(e->path, fullpath, sizeof(e->path) - 1);
		e->path[sizeof(e->path) - 1] = '\0';
		e->mtime = st.st_mtime;
		e->parsed = false;
		e->valid = false;
		s_count++;
	}
	closedir(dir);

	/* Sort by modification time, newest first */
	if (s_count > 1) {
		qsort(s_files, s_count, sizeof(IniFileEntry), cmp_by_mtime_desc);
	}

	/* Eagerly parse the first (selected) entry for immediate preview */
	if (s_count > 0) {
		parse_preview(&s_files[0]);
	}
}

/* ── Public API ──────────────────────────────────────────────────── */

void ini_selector_start(bool auto_boot)
{
	scan_ini_files();
	s_auto_boot = auto_boot;
	if (auto_boot && s_count > 0) {
		s_countdown = 5;
		s_countdown_tick = esp_log_timestamp();
	} else {
		s_countdown = -1;
	}

	globals.ini_selector_done = false;
	globals.ini_selected_path[0] = '\0';
	globals.ini_selector_active = true;

	ESP_LOGI(TAG, "INI selector started (%d files, auto_boot=%d)", s_count, auto_boot);
}

static void finish_selection(int idx)
{
	const char *path = "";
	if (idx >= 0 && idx < s_count)
		path = s_files[idx].path;

	if (s_auto_boot) {
		/* Boot-time: i386_task is waiting for ini_selector_done */
		strncpy(globals.ini_selected_path, path,
		        sizeof(globals.ini_selected_path) - 1);
		globals.ini_selected_path[sizeof(globals.ini_selected_path) - 1] = '\0';
		globals.ini_selector_done = true;
	} else {
		/* Runtime: trigger an INI switch via the emulator loop */
		if (path[0]) {
			strncpy(globals.ini_switch_path, path,
			        sizeof(globals.ini_switch_path) - 1);
			globals.ini_switch_path[sizeof(globals.ini_switch_path) - 1] = '\0';
			globals.ini_switch_pending = true;
		}
		/* If path is empty (Esc pressed), just deactivate selector */
	}

	globals.ini_selector_active = false;
	ESP_LOGI(TAG, "Selected: %s (auto_boot=%d)", path, s_auto_boot);
}

int ini_selector_handle_key(int scancode)
{
	/* Any keypress cancels auto-boot countdown */
	if (s_countdown >= 0) {
		s_countdown = -1;
	}

	if (s_count == 0) {
		/* No files — Enter or Esc finishes with empty path (will create default) */
		if (scancode == 0x1c || scancode == 0x01) {
			finish_selection(-1);
			return 1;
		}
		return 0;
	}

	switch (scancode) {
	case 0x48: /* Up */
		if (s_sel > 0) s_sel--;
		/* Lazy-parse the newly selected entry */
		parse_preview(&s_files[s_sel]);
		break;
	case 0x50: /* Down */
		if (s_sel < s_count - 1) s_sel++;
		parse_preview(&s_files[s_sel]);
		break;
	case 0x1c: /* Enter */
		finish_selection(s_sel);
		return 1;
	case 0x01: /* Escape — in auto_boot mode, just cancel countdown.
	            * In switch mode (no auto_boot), cancel and go back. */
		if (!s_auto_boot) {
			globals.ini_selector_done = true;
			globals.ini_selector_active = false;
			return 1;
		}
		break;
	}
	return 0;
}

/* ── Rendering ───────────────────────────────────────────────────── */

static void draw_preview_line(int x, int *y, const char *label,
                              const char *value, uint16_t vc)
{
	sel_draw_text(x, *y, label, C_DIM, 0);
	int lw = strlen(label) * FONT_W;
	sel_draw_text(x + lw, *y, value[0] ? value : "(none)", vc, RIGHT_W - lw - 10);
	*y += FONT_H + 2;
}

void ini_selector_render(uint16_t *fb, int phys_w, int phys_h, int pitch)
{
	(void)pitch;
	s_fb = fb;
	s_phys_w = phys_w;
	s_phys_h = phys_h;

	/* ── Update countdown ──────────────────────────────────────── */
	if (s_countdown >= 0) {
		uint32_t now = esp_log_timestamp();
		if (now - s_countdown_tick >= 1000) {
			s_countdown_tick = now;
			s_countdown--;
			if (s_countdown < 0) {
				/* Auto-boot the first file */
				finish_selection(0);
				return;
			}
		}
	}

	/* ── Clear framebuffer ─────────────────────────────────────── */
	memset(fb, 0, phys_w * phys_h * sizeof(uint16_t));

	/* ── Title bar ─────────────────────────────────────────────── */
	sel_fill_rect(0, 0, LOG_W, 28, C_PANEL);
	sel_draw_text(LEFT_X, 6, "tiny386 - Select Configuration", C_TITLE, 0);

	/* Divider between title and content */
	sel_fill_rect(0, 28, LOG_W, 1, C_BORDER);

	/* ── Left panel: file list ─────────────────────────────────── */
	sel_fill_rect(LEFT_X - 2, LIST_Y - 2, LEFT_W + 4, FOOTER_Y - LIST_Y + 4, C_PANEL);
	/* Border */
	sel_fill_rect(LEFT_X - 2, LIST_Y - 2, LEFT_W + 4, 1, C_BORDER);
	sel_fill_rect(LEFT_X - 2, FOOTER_Y + 2, LEFT_W + 4, 1, C_BORDER);
	sel_fill_rect(LEFT_X - 2, LIST_Y - 2, 1, FOOTER_Y - LIST_Y + 5, C_BORDER);
	sel_fill_rect(LEFT_X + LEFT_W + 1, LIST_Y - 2, 1, FOOTER_Y - LIST_Y + 5, C_BORDER);

	if (s_count == 0) {
		if (s_dir_missing) {
			sel_draw_text(LEFT_X + 8, LIST_Y + 20, "ERROR: /sdcard/tiny386", 0xF800, 0);
			sel_draw_text(LEFT_X + 8, LIST_Y + 38, "directory not found!", 0xF800, 0);
			sel_draw_text(LEFT_X + 8, LIST_Y + 68, "Press Enter to create", C_TEXT, 0);
			sel_draw_text(LEFT_X + 8, LIST_Y + 86, "it with a default INI.", C_TEXT, 0);
		} else {
			sel_draw_text(LEFT_X + 8, LIST_Y + 20, "No .ini files found in", C_DIM, 0);
			sel_draw_text(LEFT_X + 8, LIST_Y + 40, "/sdcard/tiny386/", C_DIM, 0);
			sel_draw_text(LEFT_X + 8, LIST_Y + 70, "Press Enter to create", C_TEXT, 0);
			sel_draw_text(LEFT_X + 8, LIST_Y + 88, "a default config.", C_TEXT, 0);
		}
	} else {
		/* Adjust scroll to keep selection visible */
		if (s_sel < s_scroll) s_scroll = s_sel;
		if (s_sel >= s_scroll + VISIBLE_LINES) s_scroll = s_sel - VISIBLE_LINES + 1;

		int y = LIST_Y;
		int line_h = FONT_H + 2;
		for (int i = s_scroll; i < s_count && i < s_scroll + VISIBLE_LINES; i++) {
			if (i == s_sel) {
				sel_fill_rect(LEFT_X, y, LEFT_W, line_h, C_HILITE);
			}
			/* Strip .ini extension for display */
			char display[MAX_INI_NAME];
			strncpy(display, s_files[i].name, sizeof(display) - 1);
			display[sizeof(display) - 1] = '\0';
			size_t dlen = strlen(display);
			if (dlen > 4 && strcasecmp(display + dlen - 4, ".ini") == 0)
				display[dlen - 4] = '\0';

			uint16_t color = C_TEXT;
			/* Mark invalid files */
			if (s_files[i].parsed && !s_files[i].valid)
				color = C_DIM;

			sel_draw_text(LEFT_X + 4, y + 1, display, color, LEFT_W - 8);
			y += line_h;
		}

		/* Scroll indicator */
		if (s_count > VISIBLE_LINES) {
			int track_h = FOOTER_Y - LIST_Y;
			int thumb_h = track_h * VISIBLE_LINES / s_count;
			if (thumb_h < 8) thumb_h = 8;
			int thumb_y = LIST_Y + (track_h - thumb_h) * s_scroll /
			              (s_count - VISIBLE_LINES);
			sel_fill_rect(LEFT_X + LEFT_W - 4, thumb_y, 3, thumb_h, C_SEP);
		}
	}

	/* ── Right panel: settings preview ─────────────────────────── */
	sel_fill_rect(RIGHT_X - 2, LIST_Y - 2, RIGHT_W + 4, FOOTER_Y - LIST_Y + 4, C_PANEL);
	/* Border */
	sel_fill_rect(RIGHT_X - 2, LIST_Y - 2, RIGHT_W + 4, 1, C_BORDER);
	sel_fill_rect(RIGHT_X - 2, FOOTER_Y + 2, RIGHT_W + 4, 1, C_BORDER);
	sel_fill_rect(RIGHT_X - 2, LIST_Y - 2, 1, FOOTER_Y - LIST_Y + 5, C_BORDER);
	sel_fill_rect(RIGHT_X + RIGHT_W + 1, LIST_Y - 2, 1, FOOTER_Y - LIST_Y + 5, C_BORDER);

	if (s_count > 0 && s_sel >= 0 && s_sel < s_count) {
		IniFileEntry *e = &s_files[s_sel];
		parse_preview(e);  /* no-op if already parsed */

		sel_draw_text(RIGHT_X + 8, LIST_Y + 2, "Settings Preview", C_TITLE, 0);
		sel_fill_rect(RIGHT_X + 8, LIST_Y + 20, RIGHT_W - 16, 1, C_SEP);

		int py = LIST_Y + 26;
		int px = RIGHT_X + 8;

		if (!e->valid) {
			sel_draw_text(px, py, "Parse error - invalid INI file", 0xF800, 0);
			py += FONT_H + 8;
			sel_draw_text(px, py, e->path, C_DIM, RIGHT_W - 16);
		} else {
			draw_preview_line(px, &py, "File:     ", e->name, C_VALUE);
			py += 4; /* extra spacing */
			draw_preview_line(px, &py, "CPU:      ", e->cpu, C_VALUE);
			draw_preview_line(px, &py, "FPU:      ", e->fpu, C_VALUE);
			draw_preview_line(px, &py, "Memory:   ", e->mem, C_VALUE);
			py += 4;
			draw_preview_line(px, &py, "BIOS:     ", e->bios, C_VALUE);
			draw_preview_line(px, &py, "VGA BIOS: ", e->vgabios, C_VALUE);
			py += 4;
			draw_preview_line(px, &py, "HDA:      ",
			                  e->hda[0] ? e->hda : "(none)", C_VALUE);
			draw_preview_line(px, &py, "CD-ROM:   ",
			                  e->cda[0] ? e->cda : "(none)", C_VALUE);
			draw_preview_line(px, &py, "Floppy A: ",
			                  e->fda[0] ? e->fda : "(none)", C_VALUE);
			if (e->boot[0]) {
				py += 4;
				draw_preview_line(px, &py, "Boot:     ", e->boot, C_VALUE);
			}
		}
	} else {
		sel_draw_text(RIGHT_X + 8, LIST_Y + 20, "No file selected", C_DIM, 0);
	}

	/* ── Footer ────────────────────────────────────────────────── */
	sel_fill_rect(0, FOOTER_Y - 4, LOG_W, 1, C_BORDER);

	if (s_countdown >= 0 && s_count > 0) {
		char msg[64];
		snprintf(msg, sizeof(msg), "Auto-boot \"%s\" in %d...",
		         s_files[0].name, s_countdown);
		sel_draw_text(LEFT_X, FOOTER_Y + 2, msg, C_TITLE, LOG_W - 20);
	} else {
		sel_draw_text(LEFT_X, FOOTER_Y + 2,
		              "Up/Down:Navigate  Enter:Select  Esc:Cancel",
		              C_SEP, 0);
	}
}
