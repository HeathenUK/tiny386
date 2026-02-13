#ifndef FILE_BROWSER_H
#define FILE_BROWSER_H

#include <stddef.h>

#define FB_MAX_FILES 256
#define FB_MAX_FILENAME 64
#define FB_MAX_PATH 256

#define FB_TYPE_FILE 0
#define FB_TYPE_DIR 1
#define FB_TYPE_SWITCH_STORAGE -2

typedef struct {
    char name[FB_MAX_FILENAME];
    int is_dir; /* 1=dir, 0=file, negative=special */
} FBEntry;

typedef struct {
    const char *label;
    int type;
} FBCustomEntry;

void fb_scan_directory(char *path, size_t path_sz,
                       FBEntry *entries, int max_entries,
                       int *count, int *sel, int *scroll,
                       int usb_vfs_mounted,
                       const FBCustomEntry *custom, int custom_count,
                       int add_parent);

void fb_go_parent(char *path, size_t path_sz);

/* Returns 1 when a file path was selected into out_fullpath.
 * Returns 0 when browser state changed (dir navigation/storage switch).
 * Returns -1 when a custom entry (<0 and != switch) was selected (out_type set).
 */
int fb_select_entry(char *path, size_t path_sz,
                    const FBEntry *entry,
                    int usb_vfs_mounted,
                    char *out_fullpath, size_t out_fullpath_sz,
                    int *out_type);

#endif
