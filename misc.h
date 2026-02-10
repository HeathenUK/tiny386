#ifndef MISC_H
#define MISC_H

#include <stdint.h>

typedef struct U8250 U8250;
U8250 *u8250_init(int irq, void *pic, void (*set_irq)(void *pic, int irq, int level));
uint8_t u8250_reg_read(U8250 *uart, int off);
void u8250_reg_write(U8250 *uart, int off, uint8_t val);
void u8250_update(U8250 *uart);
void CaptureKeyboardInput();

typedef struct CMOS CMOS;
CMOS *cmos_init(long mem_size, int irq, void *pic, void (*set_irq)(void *pic, int irq, int level));
void cmos_update_irq(CMOS *s);
uint8_t cmos_ioport_read(CMOS *cmos, int addr);
void cmos_ioport_write(CMOS *cmos, int addr, uint8_t val);

uint8_t cmos_set(void *cmos, int addr, uint8_t val);

// Boot order presets (indexes into boot_orders array)
// Each is a 3-device sequence: {first, second, third}
// Device codes: 1=Floppy, 2=HDD, 3=CD
#define BOOT_ORDER_COUNT 6
extern const int boot_orders[BOOT_ORDER_COUNT][3];
extern const char *boot_order_names[BOOT_ORDER_COUNT];

void cmos_set_boot_order(CMOS *cmos, int order_index);
int cmos_get_boot_order(CMOS *cmos);

// Save all settings to ini file
// Sound device constants
#define SOUND_DEVICE_SB16 0
#define SOUND_DEVICE_GUS  1
extern const char *sound_device_names[];

int save_settings_to_ini(const char *ini_path, int boot_order,
                         const char *hda,
                         const char *fda, const char *fdb,
                         const char *cda, const char *cdb,
                         const char *cdc, const char *cdd,
                         int cpu_gen, int fpu, long mem_size,
                         int brightness, int volume, int frame_skip,
                         int batch_size, int mouse_speed,
                         int usb_passthru, int sound_device);

typedef struct EMULINK EMULINK;
EMULINK *emulink_init();
void emulink_close(EMULINK *e);

int emulink_attach_floppy(EMULINK *e, int i, const char *filename);
const char *emulink_get_floppy_path(EMULINK *e, int i);
uint32_t emulink_status_read(void *s);
void emulink_cmd_write(void *s, uint32_t val);
void emulink_data_write(void *s, uint32_t val);
int emulink_data_write_string(void *s, uint8_t *buf, int size, int count);
int emulink_data_read_string(void *s, uint8_t *buf, int size, int count);

#endif /* MISC_H */
