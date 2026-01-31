/*
 * Tanmatsu-specific OSD header
 * Custom keyboard-driven menu with file browser for devices without mouse/touch
 */
#ifndef TANMATSU_OSD_H
#define TANMATSU_OSD_H

#include <stdint.h>

typedef struct OSD OSD;

OSD *osd_init(void);
void osd_attach_emulink(OSD *osd, void *emulink);
void osd_attach_ide(OSD *osd, void *ide, void *ide2);
void osd_attach_pc(OSD *osd, void *pc);
void osd_attach_console(OSD *osd, void *console);
void osd_refresh(OSD *osd);

// Input handlers
void osd_handle_mouse_motion(OSD *osd, int x, int y);  // No-op on Tanmatsu
void osd_handle_mouse_button(OSD *osd, int x, int y, int down, int btn);  // No-op on Tanmatsu
int osd_handle_key(OSD *osd, int keycode, int down);  // Returns 1 if OSD should close

// Rendering
void osd_render(OSD *osd, uint8_t *pixels, int w, int h, int pitch);

#endif /* TANMATSU_OSD_H */
