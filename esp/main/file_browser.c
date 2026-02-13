#include "file_browser.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static void copy_str(char *dst, size_t dst_sz, const char *src)
{
    if (!dst || dst_sz == 0) return;
    if (!src) src = "";
    size_t n = strlen(src);
    if (n >= dst_sz) n = dst_sz - 1;
    if (n > 0) memcpy(dst, src, n);
    dst[n] = '\0';
}

static int compare_files(const void *a, const void *b)
{
    const FBEntry *fa = (const FBEntry *)a;
    const FBEntry *fb = (const FBEntry *)b;
    if (fa->is_dir != fb->is_dir) {
        return fb->is_dir - fa->is_dir;
    }
    return strcasecmp(fa->name, fb->name);
}

void fb_scan_directory(char *path, size_t path_sz,
                       FBEntry *entries, int max_entries,
                       int *count, int *sel, int *scroll,
                       int usb_vfs_mounted,
                       const FBCustomEntry *custom, int custom_count,
                       int add_parent)
{
    (void)path_sz;
    if (!path || !entries || !count || !sel || !scroll || max_entries <= 0) return;

    *count = 0;
    *sel = 0;
    *scroll = 0;

    for (int i = 0; i < custom_count && *count < max_entries; i++) {
        copy_str(entries[*count].name, sizeof(entries[*count].name), custom[i].label);
        entries[*count].is_dir = custom[i].type;
        (*count)++;
    }

    if (usb_vfs_mounted && *count < max_entries) {
        if (strcmp(path, "/sdcard") == 0) {
            copy_str(entries[*count].name, sizeof(entries[*count].name), "[USB Storage]");
            entries[*count].is_dir = FB_TYPE_SWITCH_STORAGE;
            (*count)++;
        } else if (strcmp(path, "/usb") == 0) {
            copy_str(entries[*count].name, sizeof(entries[*count].name), "[SD Card]");
            entries[*count].is_dir = FB_TYPE_SWITCH_STORAGE;
            (*count)++;
        }
    }

    if (add_parent && *count < max_entries &&
        strlen(path) > 1 && strcmp(path, "/sdcard") != 0 && strcmp(path, "/usb") != 0) {
        copy_str(entries[*count].name, sizeof(entries[*count].name), "..");
        entries[*count].is_dir = FB_TYPE_DIR;
        (*count)++;
    }

    DIR *dir = opendir(path);
    if (!dir) return;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && *count < max_entries) {
        if (ent->d_name[0] == '.') continue;

        copy_str(entries[*count].name, sizeof(entries[*count].name), ent->d_name);

        char fullpath[FB_MAX_PATH + FB_MAX_FILENAME];
        size_t pl = strlen(path);
        size_t nl = strlen(ent->d_name);
        if (pl + 1 + nl + 1 > sizeof(fullpath)) continue;
        memcpy(fullpath, path, pl);
        fullpath[pl] = '/';
        memcpy(fullpath + pl + 1, ent->d_name, nl + 1);

        struct stat st;
        if (stat(fullpath, &st) == 0) {
            entries[*count].is_dir = S_ISDIR(st.st_mode) ? FB_TYPE_DIR : FB_TYPE_FILE;
        } else {
            entries[*count].is_dir = FB_TYPE_FILE;
        }
        (*count)++;
    }
    closedir(dir);

    int sort_start = custom_count;
    if (usb_vfs_mounted && (strcmp(path, "/sdcard") == 0 || strcmp(path, "/usb") == 0)) {
        sort_start++;
    }
    if (add_parent && strlen(path) > 1 && strcmp(path, "/sdcard") != 0 && strcmp(path, "/usb") != 0) {
        sort_start++;
    }

    if (*count > sort_start) {
        qsort(&entries[sort_start], *count - sort_start, sizeof(FBEntry), compare_files);
    }
}

void fb_go_parent(char *path, size_t path_sz)
{
    if (!path || path_sz == 0) return;
    if (strlen(path) <= 1 || strcmp(path, "/sdcard") == 0 || strcmp(path, "/usb") == 0) return;

    char *slash = strrchr(path, '/');
    if (slash && slash != path) {
        *slash = '\0';
    } else {
        copy_str(path, path_sz, "/sdcard");
    }
}

int fb_select_entry(char *path, size_t path_sz,
                    const FBEntry *entry,
                    int usb_vfs_mounted,
                    char *out_fullpath, size_t out_fullpath_sz,
                    int *out_type)
{
    if (!path || !entry) return 0;

    if (out_fullpath && out_fullpath_sz > 0) out_fullpath[0] = '\0';
    if (out_type) *out_type = 0;

    if (entry->is_dir == FB_TYPE_SWITCH_STORAGE) {
        if (usb_vfs_mounted) {
            if (strcmp(path, "/sdcard") == 0) copy_str(path, path_sz, "/usb");
            else copy_str(path, path_sz, "/sdcard");
        }
        return 0;
    }

    if (entry->is_dir == FB_TYPE_DIR) {
        if (strcmp(entry->name, "..") == 0) {
            fb_go_parent(path, path_sz);
        } else {
            size_t pl = strlen(path);
            size_t nl = strlen(entry->name);
            if (pl + 1 + nl + 1 < path_sz) {
                path[pl] = '/';
                memcpy(path + pl + 1, entry->name, nl + 1);
            }
        }
        return 0;
    }

    if (entry->is_dir < 0) {
        if (out_type) *out_type = entry->is_dir;
        return -1;
    }

    if (out_fullpath && out_fullpath_sz > 0) {
        size_t pl = strlen(path);
        size_t nl = strlen(entry->name);
        if (pl + 1 + nl + 1 > out_fullpath_sz) return 0;
        memcpy(out_fullpath, path, pl);
        out_fullpath[pl] = '/';
        memcpy(out_fullpath + pl + 1, entry->name, nl + 1);
    }
    return 1;
}
