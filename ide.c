/*
 * IDE emulation
 * 
 * Copyright (c) 2003-2016 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#define _FILE_OFFSET_BITS 64
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include <unistd.h>

//#include "cutils.h"
#include "ide.h"

//#define DEBUG_IDE
#include "led_activity.h"
#include "usb_host.h"
#include "esp_heap_caps.h"
#include "ff.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#define IDE_ACTIVITY() led_activity_hdd()

#define SDMMC_RETRY_COUNT 8
#define SDMMC_RETRY_DELAY_MS 2
static SemaphoreHandle_t sdmmc_io_mutex = NULL;

static esp_err_t sdmmc_read_sectors_retry(sdmmc_card_t *card, void *dst,
                                          size_t start_sector, size_t sector_count)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= SDMMC_RETRY_COUNT; attempt++) {
        if (sdmmc_io_mutex)
            xSemaphoreTake(sdmmc_io_mutex, portMAX_DELAY);
        err = sdmmc_read_sectors(card, dst, start_sector, sector_count);
        if (sdmmc_io_mutex)
            xSemaphoreGive(sdmmc_io_mutex);
        if (err == ESP_OK)
            return err;
        if (attempt < SDMMC_RETRY_COUNT)
            vTaskDelay(pdMS_TO_TICKS(SDMMC_RETRY_DELAY_MS));
    }
    fprintf(stderr, "sdmmc_read retry exhausted: sector=%u count=%u err=0x%x\n",
            (unsigned)start_sector, (unsigned)sector_count, (unsigned)err);
    return err;
}

static esp_err_t sdmmc_write_sectors_retry(sdmmc_card_t *card, const void *src,
                                           size_t start_sector, size_t sector_count)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= SDMMC_RETRY_COUNT; attempt++) {
        if (sdmmc_io_mutex)
            xSemaphoreTake(sdmmc_io_mutex, portMAX_DELAY);
        err = sdmmc_write_sectors(card, src, start_sector, sector_count);
        if (sdmmc_io_mutex)
            xSemaphoreGive(sdmmc_io_mutex);
        if (err == ESP_OK)
            return err;
        if (attempt < SDMMC_RETRY_COUNT)
            vTaskDelay(pdMS_TO_TICKS(SDMMC_RETRY_DELAY_MS));
    }
    fprintf(stderr, "sdmmc_write retry exhausted: sector=%u count=%u err=0x%x\n",
            (unsigned)start_sector, (unsigned)sector_count, (unsigned)err);
    return err;
}

//#define DEBUG_IDE_ATAPI

/* Bits of HD_STATUS */
#define ERR_STAT		0x01
#define INDEX_STAT		0x02
#define ECC_STAT		0x04	/* Corrected error */
#define DRQ_STAT		0x08
#define SEEK_STAT		0x10
#define SRV_STAT		0x10
#define WRERR_STAT		0x20
#define READY_STAT		0x40
#define BUSY_STAT		0x80

/* Bits for HD_ERROR */
#define MARK_ERR		0x01	/* Bad address mark */
#define TRK0_ERR		0x02	/* couldn't find track 0 */
#define ABRT_ERR		0x04	/* Command aborted */
#define MCR_ERR			0x08	/* media change request */
#define ID_ERR			0x10	/* ID field not found */
#define MC_ERR			0x20	/* media changed */
#define ECC_ERR			0x40	/* Uncorrectable ECC error */
#define BBD_ERR			0x80	/* pre-EIDE meaning:  block marked bad */
#define ICRC_ERR		0x80	/* new meaning:  CRC error during transfer */

/* Bits of HD_NSECTOR */
#define CD			0x01
#define IO			0x02
#define REL			0x04
#define TAG_MASK		0xf8

#define IDE_CMD_RESET           0x04
#define IDE_CMD_DISABLE_IRQ     0x02

/* ATA/ATAPI Commands pre T13 Spec */
#define WIN_NOP				0x00
/*
 *	0x01->0x02 Reserved
 */
#define CFA_REQ_EXT_ERROR_CODE		0x03 /* CFA Request Extended Error Code */
/*
 *	0x04->0x07 Reserved
 */
#define WIN_SRST			0x08 /* ATAPI soft reset command */
#define WIN_DEVICE_RESET		0x08
/*
 *	0x09->0x0F Reserved
 */
#define WIN_RECAL			0x10
#define WIN_RESTORE			WIN_RECAL
/*
 *	0x10->0x1F Reserved
 */
#define WIN_READ			0x20 /* 28-Bit */
#define WIN_READ_ONCE			0x21 /* 28-Bit without retries */
#define WIN_READ_LONG			0x22 /* 28-Bit */
#define WIN_READ_LONG_ONCE		0x23 /* 28-Bit without retries */
#define WIN_READ_EXT			0x24 /* 48-Bit */
#define WIN_READDMA_EXT			0x25 /* 48-Bit */
#define WIN_READDMA_QUEUED_EXT		0x26 /* 48-Bit */
#define WIN_READ_NATIVE_MAX_EXT		0x27 /* 48-Bit */
/*
 *	0x28
 */
#define WIN_MULTREAD_EXT		0x29 /* 48-Bit */
/*
 *	0x2A->0x2F Reserved
 */
#define WIN_WRITE			0x30 /* 28-Bit */
#define WIN_WRITE_ONCE			0x31 /* 28-Bit without retries */
#define WIN_WRITE_LONG			0x32 /* 28-Bit */
#define WIN_WRITE_LONG_ONCE		0x33 /* 28-Bit without retries */
#define WIN_WRITE_EXT			0x34 /* 48-Bit */
#define WIN_WRITEDMA_EXT		0x35 /* 48-Bit */
#define WIN_WRITEDMA_QUEUED_EXT		0x36 /* 48-Bit */
#define WIN_SET_MAX_EXT			0x37 /* 48-Bit */
#define CFA_WRITE_SECT_WO_ERASE		0x38 /* CFA Write Sectors without erase */
#define WIN_MULTWRITE_EXT		0x39 /* 48-Bit */
/*
 *	0x3A->0x3B Reserved
 */
#define WIN_WRITE_VERIFY		0x3C /* 28-Bit */
/*
 *	0x3D->0x3F Reserved
 */
#define WIN_VERIFY			0x40 /* 28-Bit - Read Verify Sectors */
#define WIN_VERIFY_ONCE			0x41 /* 28-Bit - without retries */
#define WIN_VERIFY_EXT			0x42 /* 48-Bit */
/*
 *	0x43->0x4F Reserved
 */
#define WIN_FORMAT			0x50
/*
 *	0x51->0x5F Reserved
 */
#define WIN_INIT			0x60
/*
 *	0x61->0x5F Reserved
 */
#define WIN_SEEK			0x70 /* 0x70-0x7F Reserved */
#define CFA_TRANSLATE_SECTOR		0x87 /* CFA Translate Sector */
#define WIN_DIAGNOSE			0x90
#define WIN_SPECIFY			0x91 /* set drive geometry translation */
#define WIN_DOWNLOAD_MICROCODE		0x92
#define WIN_STANDBYNOW2			0x94
#define WIN_STANDBY2			0x96
#define WIN_SETIDLE2			0x97
#define WIN_CHECKPOWERMODE2		0x98
#define WIN_SLEEPNOW2			0x99
/*
 *	0x9A VENDOR
 */
#define WIN_PACKETCMD			0xA0 /* Send a packet command. */
#define WIN_PIDENTIFY			0xA1 /* identify ATAPI device	*/
#define WIN_QUEUED_SERVICE		0xA2
#define WIN_SMART			0xB0 /* self-monitoring and reporting */
#define CFA_ERASE_SECTORS       	0xC0
#define WIN_MULTREAD			0xC4 /* read sectors using multiple mode*/
#define WIN_MULTWRITE			0xC5 /* write sectors using multiple mode */
#define WIN_SETMULT			0xC6 /* enable/disable multiple mode */
#define WIN_READDMA_QUEUED		0xC7 /* read sectors using Queued DMA transfers */
#define WIN_READDMA			0xC8 /* read sectors using DMA transfers */
#define WIN_READDMA_ONCE		0xC9 /* 28-Bit - without retries */
#define WIN_WRITEDMA			0xCA /* write sectors using DMA transfers */
#define WIN_WRITEDMA_ONCE		0xCB /* 28-Bit - without retries */
#define WIN_WRITEDMA_QUEUED		0xCC /* write sectors using Queued DMA transfers */
#define CFA_WRITE_MULTI_WO_ERASE	0xCD /* CFA Write multiple without erase */
#define WIN_GETMEDIASTATUS		0xDA	
#define WIN_ACKMEDIACHANGE		0xDB /* ATA-1, ATA-2 vendor */
#define WIN_POSTBOOT			0xDC
#define WIN_PREBOOT			0xDD
#define WIN_DOORLOCK			0xDE /* lock door on removable drives */
#define WIN_DOORUNLOCK			0xDF /* unlock door on removable drives */
#define WIN_STANDBYNOW1			0xE0
#define WIN_IDLEIMMEDIATE		0xE1 /* force drive to become "ready" */
#define WIN_STANDBY             	0xE2 /* Set device in Standby Mode */
#define WIN_SETIDLE1			0xE3
#define WIN_READ_BUFFER			0xE4 /* force read only 1 sector */
#define WIN_CHECKPOWERMODE1		0xE5
#define WIN_SLEEPNOW1			0xE6
#define WIN_FLUSH_CACHE			0xE7
#define WIN_WRITE_BUFFER		0xE8 /* force write only 1 sector */
#define WIN_WRITE_SAME			0xE9 /* read ata-2 to use */
	/* SET_FEATURES 0x22 or 0xDD */
#define WIN_FLUSH_CACHE_EXT		0xEA /* 48-Bit */
#define WIN_IDENTIFY			0xEC /* ask drive to identify itself	*/
#define WIN_MEDIAEJECT			0xED
#define WIN_IDENTIFY_DMA		0xEE /* same as WIN_IDENTIFY, but DMA */
#define WIN_SETFEATURES			0xEF /* set special drive features */
#define EXABYTE_ENABLE_NEST		0xF0
#define WIN_SECURITY_SET_PASS		0xF1
#define WIN_SECURITY_UNLOCK		0xF2
#define WIN_SECURITY_ERASE_PREPARE	0xF3
#define WIN_SECURITY_ERASE_UNIT		0xF4
#define WIN_SECURITY_FREEZE_LOCK	0xF5
#define WIN_SECURITY_DISABLE		0xF6
#define WIN_READ_NATIVE_MAX		0xF8 /* return the native maximum address */
#define WIN_SET_MAX			0xF9
#define DISABLE_SEAGATE			0xFB

/* ATAPI defines */

#define ATAPI_PACKET_SIZE 12

/* The generic packet command opcodes for CD/DVD Logical Units,
 * From Table 57 of the SFF8090 Ver. 3 (Mt. Fuji) draft standard. */
#define GPCMD_BLANK			    0xa1
#define GPCMD_CLOSE_TRACK		    0x5b
#define GPCMD_FLUSH_CACHE		    0x35
#define GPCMD_FORMAT_UNIT		    0x04
#define GPCMD_GET_CONFIGURATION		    0x46
#define GPCMD_GET_EVENT_STATUS_NOTIFICATION 0x4a
#define GPCMD_GET_PERFORMANCE		    0xac
#define GPCMD_INQUIRY			    0x12
#define GPCMD_LOAD_UNLOAD		    0xa6
#define GPCMD_MECHANISM_STATUS		    0xbd
#define GPCMD_MODE_SELECT_10		    0x55
#define GPCMD_MODE_SENSE_10		    0x5a
#define GPCMD_PAUSE_RESUME		    0x4b
#define GPCMD_PLAY_AUDIO_10		    0x45
#define GPCMD_PLAY_AUDIO_MSF		    0x47
#define GPCMD_PLAY_AUDIO_TI		    0x48
#define GPCMD_PLAY_CD			    0xbc
#define GPCMD_PREVENT_ALLOW_MEDIUM_REMOVAL  0x1e
#define GPCMD_READ_10			    0x28
#define GPCMD_READ_12			    0xa8
#define GPCMD_READ_CDVD_CAPACITY	    0x25
#define GPCMD_READ_CD			    0xbe
#define GPCMD_READ_CD_MSF		    0xb9
#define GPCMD_READ_DISC_INFO		    0x51
#define GPCMD_READ_DVD_STRUCTURE	    0xad
#define GPCMD_READ_FORMAT_CAPACITIES	    0x23
#define GPCMD_READ_HEADER		    0x44
#define GPCMD_READ_TRACK_RZONE_INFO	    0x52
#define GPCMD_READ_SUBCHANNEL		    0x42
#define GPCMD_READ_TOC_PMA_ATIP		    0x43
#define GPCMD_REPAIR_RZONE_TRACK	    0x58
#define GPCMD_REPORT_KEY		    0xa4
#define GPCMD_REQUEST_SENSE		    0x03
#define GPCMD_RESERVE_RZONE_TRACK	    0x53
#define GPCMD_SCAN			    0xba
#define GPCMD_SEEK			    0x2b
#define GPCMD_SEND_DVD_STRUCTURE	    0xad
#define GPCMD_SEND_EVENT		    0xa2
#define GPCMD_SEND_KEY			    0xa3
#define GPCMD_SEND_OPC			    0x54
#define GPCMD_SET_READ_AHEAD		    0xa7
#define GPCMD_SET_STREAMING		    0xb6
#define GPCMD_START_STOP_UNIT		    0x1b
#define GPCMD_STOP_PLAY_SCAN		    0x4e
#define GPCMD_TEST_UNIT_READY		    0x00
#define GPCMD_VERIFY_10			    0x2f
#define GPCMD_WRITE_10			    0x2a
#define GPCMD_WRITE_AND_VERIFY_10	    0x2e
/* This is listed as optional in ATAPI 2.6, but is (curiously)
 * missing from Mt. Fuji, Table 57.  It _is_ mentioned in Mt. Fuji
 * Table 377 as an MMC command for SCSi devices though...  Most ATAPI
 * drives support it. */
#define GPCMD_SET_SPEED			    0xbb
/* This seems to be a SCSI specific CD-ROM opcode
 * to play data at track/index */
#define GPCMD_PLAYAUDIO_TI		    0x48
/*
 * From MS Media Status Notification Support Specification. For
 * older drives only.
 */
#define GPCMD_GET_MEDIA_STATUS		    0xda
#define GPCMD_MODE_SENSE_6		    0x1a

/* Mode page codes for mode sense/set */
#define GPMODE_R_W_ERROR_PAGE		0x01
#define GPMODE_WRITE_PARMS_PAGE		0x05
#define GPMODE_AUDIO_CTL_PAGE		0x0e
#define GPMODE_POWER_PAGE		0x1a
#define GPMODE_FAULT_FAIL_PAGE		0x1c
#define GPMODE_TO_PROTECT_PAGE		0x1d
#define GPMODE_CAPABILITIES_PAGE	0x2a
#define GPMODE_ALL_PAGES		0x3f
/* Not in Mt. Fuji, but in ATAPI 2.6 -- depricated now in favor
 * of MODE_SENSE_POWER_PAGE */
#define GPMODE_CDROM_PAGE		0x0d

/* Some generally useful CD-ROM information */
#define CD_MINS                       80 /* max. minutes per CD */
#define CD_SECS                       60 /* seconds per minute */
#define CD_FRAMES                     75 /* frames per second */
#define CD_FRAMESIZE                2048 /* bytes per frame, "cooked" mode */
#define CD_MAX_BYTES       (CD_MINS * CD_SECS * CD_FRAMES * CD_FRAMESIZE)
#define CD_MAX_SECTORS     (CD_MAX_BYTES / 512)

/* Profile list from MMC-6 revision 1 table 91 */
#define MMC_PROFILE_NONE                0x0000
#define MMC_PROFILE_CD_ROM              0x0008
#define MMC_PROFILE_DVD_ROM             0x0010

#define ATAPI_INT_REASON_CD             0x01 /* 0 = data transfer */
#define ATAPI_INT_REASON_IO             0x02 /* 1 = transfer to the host */
#define ATAPI_INT_REASON_REL            0x04
#define ATAPI_INT_REASON_TAG            0xf8
#define ASC_ILLEGAL_OPCODE                   0x20
#define ASC_LOGICAL_BLOCK_OOR                0x21
#define ASC_INV_FIELD_IN_CMD_PACKET          0x24
#define ASC_MEDIUM_MAY_HAVE_CHANGED          0x28
#define ASC_INCOMPATIBLE_FORMAT              0x30
#define ASC_MEDIUM_NOT_PRESENT               0x3a
#define ASC_SAVING_PARAMETERS_NOT_SUPPORTED  0x39
#define ASC_MEDIA_REMOVAL_PREVENTED          0x53
#define SENSE_NONE            0
#define SENSE_NOT_READY       2
#define SENSE_ILLEGAL_REQUEST 5
#define SENSE_UNIT_ATTENTION  6

#define MAX_MULT_SECTORS 16 /* 512 * 16 == 8192 */

void *pcmalloc(long size);

typedef void BlockDeviceCompletionFunc(void *opaque, int ret);

struct BlockDevice {
    int64_t (*get_sector_count)(BlockDevice *bs);
    int (*get_chs)(BlockDevice *bs, int *cylinders, int *heads, int *sectors);
    int (*read_async)(BlockDevice *bs,
                      uint64_t sector_num, uint8_t *buf, int n,
                      BlockDeviceCompletionFunc *cb, void *opaque);
    int (*write_async)(BlockDevice *bs,
                       uint64_t sector_num, const uint8_t *buf, int n,
                       BlockDeviceCompletionFunc *cb, void *opaque);
    void *opaque;
};

/* Async read state — shared between ide_status_read (poll) and io_task (producer).
 * Declared early so ide_status_read can reference it. */
static struct {
    int pending;                        /* atomic access via __atomic builtins */
    int done;                           /* atomic access via __atomic builtins */
    int result;
    void *bf;               /* BlockDeviceFile* (void* to avoid forward decl) */
    uint8_t *buf;
    uint64_t sector_num;
    int count;
    int start_offset;
    BlockDeviceCompletionFunc *cb;
    void *cb_opaque;
} async_read;

typedef struct IDEState IDEState;

typedef void EndTransferFunc(IDEState *);

struct IDEState {
    IDEIFState *ide_if;
    BlockDevice *bs;
    enum {
        IDE_HD, IDE_CD
    } drive_kind;
    int cylinders, heads, sectors;
    int mult_sectors;
    int64_t nb_sectors;

    /* ide regs */
    uint8_t feature;
    uint8_t error;
    uint16_t nsector; /* 0 is 256 to ease computations */
    uint8_t sector;
    uint8_t lcyl;
    uint8_t hcyl;
    uint8_t select;
    uint8_t status;

    /* ATAPI specific */
    uint8_t sense_key;
    uint8_t asc;
    uint8_t cdrom_changed;
    int packet_transfer_size;
    int elementary_transfer_size;
    int io_buffer_index;
    int lba;
    int cd_sector_size;

    int io_nb_sectors;
    int req_nb_sectors;
    EndTransferFunc *end_transfer_func;
    
    int data_index;
    int data_end;
    uint8_t io_buffer[MAX_MULT_SECTORS*512 + 4];
};

struct IDEIFState {
    int irq;
    void *pic;
    void (*set_irq)(void *pic, int irq, int level);

    IDEState *cur_drive;
    IDEState *drives[2];
    /* 0x3f6 command */
    uint8_t cmd;
};

static void ide_sector_read_cb(void *opaque, int ret);
static void ide_sector_read_cb_end(IDEState *s);
static void ide_sector_write_cb2(void *opaque, int ret);

static void padstr(char *str, const char *src, int len)
{
    int i, v;
    for(i = 0; i < len; i++) {
        if (*src)
            v = *src++;
        else
            v = ' ';
        *(char *)((uintptr_t)str ^ 1) = v;
        str++;
    }
}

static void padstr8(uint8_t *buf, int buf_size, const char *src)
{
    int i;
    for(i = 0; i < buf_size; i++) {
        if (*src)
            buf[i] = *src++;
        else
            buf[i] = ' ';
    }
}

/* little endian assume */
static void stw(uint16_t *buf, int v)
{
    *buf = v;
}

static void ide_identify(IDEState *s)
{
    uint16_t *tab;
    uint32_t oldsize;
    uint32_t cyls, heads, secs;

    tab = (uint16_t *)s->io_buffer;

    memset(tab, 0, 512 * 2);

    /* Use geometry that doesn't exceed actual disk size */
    cyls = s->cylinders;
    heads = s->heads;
    secs = s->sectors;

    /* Reduce cylinders if CHS product exceeds actual sectors */
    while (cyls > 1 && (uint64_t)cyls * heads * secs > s->nb_sectors) {
        cyls--;
    }
    oldsize = cyls * heads * secs;

    stw(tab + 0, 0x0040);
    stw(tab + 1, cyls);
    stw(tab + 3, heads);
    stw(tab + 4, 512 * secs); /* sectors */
    stw(tab + 5, 512); /* sector size */
    stw(tab + 6, secs);
    stw(tab + 20, 3); /* buffer type */
    stw(tab + 21, 512); /* cache size in sectors */
    stw(tab + 22, 4); /* ecc bytes */
    padstr((char *)(tab + 27), "TINY386 HARDDISK", 40);
    stw(tab + 47, 0x8000 | MAX_MULT_SECTORS);
    stw(tab + 48, 1); /* dword I/O */
    stw(tab + 49, (1 << 9)); /* LBA supported, no DMA (no Bus Master) */
    stw(tab + 51, 0x200); /* PIO transfer cycle */
    stw(tab + 52, 0x200); /* DMA transfer cycle */
    /* No DMA modes advertised — Bus Master DMA not emulated */
    stw(tab + 54, cyls);
    stw(tab + 55, heads);
    stw(tab + 56, secs);
    stw(tab + 57, oldsize);
    stw(tab + 58, oldsize >> 16);
    if (s->mult_sectors)
        stw(tab + 59, 0x100 | s->mult_sectors);
    stw(tab + 60, s->nb_sectors);
    stw(tab + 61, s->nb_sectors >> 16);
    stw(tab + 80, (1 << 1) | (1 << 2));
    stw(tab + 82, (1 << 14) | (1 << 3)); /* power management supported */
    stw(tab + 83, (1 << 14) | (1 << 12)); /* flush cache supported */
    stw(tab + 84, (1 << 14));
    stw(tab + 85, (1 << 14) | (1 << 3)); /* power management enabled */
    stw(tab + 86, (1 << 12)); /* flush cache enabled */
    stw(tab + 87, (1 << 14));
}

static void ide_atapi_identify(IDEState *s)
{
    uint16_t *tab;
    tab = (uint16_t *)s->io_buffer;
    memset(tab, 0, 512 * 2);

    /* Removable CDROM, 50us response, 12 byte packets */
    stw(tab + 0, (2 << 14) | (5 << 8) | (1 << 7) | (2 << 5) | (0 << 0));
    stw(tab + 20, 3); /* buffer type */
    stw(tab + 21, 512); /* cache size in sectors */
    stw(tab + 22, 4); /* ecc bytes */
    padstr((char *)(tab + 27), "TINY386 CD-ROM", 40); /* model */
    stw(tab + 48, 1); /* dword I/O (XXX: should not be set on CDROM) */
    stw(tab + 49, 1 << 9); /* LBA supported, no DMA */
    stw(tab + 53, 3); /* words 64-70, 54-58 valid */
    stw(tab + 63, 0x0); /* no DMA modes supported (no bus master DMA) */
    stw(tab + 64, 1); /* PIO modes */
    stw(tab + 65, 0xb4); /* minimum DMA multiword tx cycle time */
    stw(tab + 66, 0xb4); /* recommended DMA multiword tx cycle time */
    stw(tab + 67, 0x12c); /* minimum PIO cycle time without flow control */
    stw(tab + 68, 0xb4); /* minimum PIO cycle time with IORDY flow control */

    stw(tab + 71, 30); /* in ns */
    stw(tab + 72, 30); /* in ns */

    stw(tab + 80, 0x1e); /* support up to ATA/ATAPI-4 */
}

static void ide_set_signature(IDEState *s) 
{
    s->select &= 0xf0;
    s->nsector = 1;
    s->sector = 1;
    if (s->drive_kind == IDE_CD) {
        s->lcyl = 0x14;
        s->hcyl = 0xeb;
    } else {
        s->lcyl = 0;
        s->hcyl = 0;
    }
}

static void ide_abort_command(IDEState *s) 
{
    s->status = READY_STAT | ERR_STAT;
    s->error = ABRT_ERR;
}

static void ide_set_irq(IDEState *s)
{
    IDEIFState *ide_if = s->ide_if;
    if (!(ide_if->cmd & IDE_CMD_DISABLE_IRQ)) {
        /* Ensure a clean 0→1 edge for the PIC's edge-triggered detection.
           Without lowering first, back-to-back ide_set_irq() calls (e.g.
           multi-sector PIO where the host reads 0x3F6 instead of 0x1F7)
           leave last_irr high and the PIC silently drops the interrupt. */
        ide_if->set_irq(ide_if->pic, ide_if->irq, 0);
        ide_if->set_irq(ide_if->pic, ide_if->irq, 1);
    }
}

/* prepare data transfer and tell what to do after */
static void ide_transfer_start(IDEState *s, int size,
                               EndTransferFunc *end_transfer_func)
{
    if (size > (int)sizeof(s->io_buffer))
        size = sizeof(s->io_buffer);
    s->end_transfer_func = end_transfer_func;
    s->data_index = 0;
    s->data_end = size;
}

static void ide_transfer_stop(IDEState *s)
{
    s->end_transfer_func = ide_transfer_stop;
    s->data_index = 0;
    s->data_end = 0;
}

static void ide_transfer_start2(IDEState *s, int off, int size,
                               EndTransferFunc *end_transfer_func)
{
    int end = off + size;
    if (end > (int)sizeof(s->io_buffer))
        end = sizeof(s->io_buffer);
    if (off > end)
        off = end;
    s->end_transfer_func = end_transfer_func;
    s->data_index = off;
    s->data_end = end;
    if (!(s->status & ERR_STAT))
        s->status |= DRQ_STAT;
}

static void ide_transfer_stop2(IDEState *s)
{
    s->end_transfer_func = ide_transfer_stop2;
    s->data_index = 0;
    s->data_end = 0;
    s->status &= ~DRQ_STAT;
}

static int64_t ide_get_sector(IDEState *s)
{
    int64_t sector_num;
    if (s->select & 0x40) {
        /* lba */
        sector_num = ((s->select & 0x0f) << 24) | (s->hcyl << 16) |
            (s->lcyl << 8) | s->sector;
    } else {
        sector_num = ((s->hcyl << 8) | s->lcyl) * 
            s->heads * s->sectors +
            (s->select & 0x0f) * s->sectors + (s->sector - 1);
    }
    return sector_num;
}

static void ide_set_sector(IDEState *s, int64_t sector_num)
{
    unsigned int cyl, r;
    if (s->select & 0x40) {
        s->select = (s->select & 0xf0) | ((sector_num >> 24) & 0x0f);
        s->hcyl = (sector_num >> 16) & 0xff;
        s->lcyl = (sector_num >> 8) & 0xff;
        s->sector = sector_num & 0xff;
    } else {
        cyl = sector_num / (s->heads * s->sectors);
        r = sector_num % (s->heads * s->sectors);
        s->hcyl = (cyl >> 8) & 0xff;
        s->lcyl = cyl & 0xff;
        s->select = (s->select & 0xf0) | ((r / s->sectors) & 0x0f);
        s->sector = (r % s->sectors) + 1;
    }
}

static void ide_sector_read(IDEState *s)
{
    int64_t sector_num;
    int ret, n;

    sector_num = ide_get_sector(s);
    n = s->nsector;
    if (n == 0)
        n = 256;
    if (n > s->req_nb_sectors)
        n = s->req_nb_sectors;
#if defined(DEBUG_IDE)
    printf("read sector=%" PRId64 " count=%d\n", sector_num, n);
#endif
#if defined(DEBUG_USB_IDE)
    /* Debug USB drive reads - check if this is the USB drive (secondary master) */
    if (s->ide_if && s->ide_if->irq == 15 && s == s->ide_if->drives[0]) {
        fprintf(stderr, "USB IDE read: sector=%lld count=%d nb_sectors=%lld\n",
                (long long)sector_num, n, (long long)s->nb_sectors);
    }
#endif
    s->io_nb_sectors = n;
    IDE_ACTIVITY();
    ret = s->bs->read_async(s->bs, sector_num, s->io_buffer, n,
                            ide_sector_read_cb, s);
    if (ret < 0) {
        /* error */
        ide_abort_command(s);
        ide_set_irq(s);
    } else if (ret == 0) {
        /* synchronous case (needed for performance) */
        ide_sector_read_cb(s, 0);
    } else {
        /* async case */
        s->status = READY_STAT | SEEK_STAT | BUSY_STAT;
        s->error = 0; /* not needed by IDE spec, but needed by Windows */
    }
}

static void ide_sector_read_cb(void *opaque, int ret)
{
    IDEState *s = opaque;
    int n;
    EndTransferFunc *func;

    n = s->io_nb_sectors;
#if defined(DEBUG_USB_IDE)
    /* For USB drive, dump data when reading from USB */
    if (s->ide_if && s->ide_if->irq == 15 && s == s->ide_if->drives[0]) {
        int64_t cur_sector = ide_get_sector(s);  /* sector we just read */
        uint8_t *buf = s->io_buffer;
        fprintf(stderr, "USB read done sector=%lld: %02x %02x %02x %02x %02x %02x %02x %02x ... %02x %02x\n",
                (long long)cur_sector,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                buf[510], buf[511]);
        if (cur_sector == 0) {
            /* Dump MBR partition table entry 1 */
            fprintf(stderr, "  MBR part1: boot=%02x type=%02x LBA_start=%u size=%u\n",
                    buf[446+0], buf[446+4],
                    buf[446+8] | (buf[446+9]<<8) | (buf[446+10]<<16) | (buf[446+11]<<24),
                    buf[446+12] | (buf[446+13]<<8) | (buf[446+14]<<16) | (buf[446+15]<<24));
        }
    }
#endif
    ide_set_sector(s, ide_get_sector(s) + n);
    s->nsector = (s->nsector - n) & 0xff;
    if (s->nsector == 0)
        func = ide_sector_read_cb_end;
    else
        func = ide_sector_read;
    ide_transfer_start(s, 512 * n, func);
    ide_set_irq(s);
    s->status = READY_STAT | SEEK_STAT | DRQ_STAT;
    s->error = 0; /* not needed by IDE spec, but needed by Windows */
}

static void ide_sector_read_cb_end(IDEState *s)
{
    /* no more sector to read from disk */
    s->status = READY_STAT | SEEK_STAT;
    s->error = 0; /* not needed by IDE spec, but needed by Windows */
    ide_transfer_stop(s);
}

static void ide_sector_write_cb1(IDEState *s)
{
    int64_t sector_num;
    int ret;

    ide_transfer_stop(s);
    sector_num = ide_get_sector(s);
#if defined(DEBUG_IDE)
    printf("write sector=%" PRId64 "  count=%d\n",
           sector_num, s->io_nb_sectors);
#endif
    IDE_ACTIVITY();
    ret = s->bs->write_async(s->bs, sector_num, s->io_buffer, s->io_nb_sectors,
                             ide_sector_write_cb2, s);
    if (ret < 0) {
        /* error */
        fprintf(stderr, "IDE: write FAILED sector=%lld\n", (long long)sector_num);
        ide_abort_command(s);
        ide_set_irq(s);
    } else if (ret == 0) {
        /* synchronous case (needed for performance) */
        ide_sector_write_cb2(s, 0);
    } else {
        /* async case */
        s->status = READY_STAT | SEEK_STAT | BUSY_STAT;
    }
}

static void ide_sector_write_cb2(void *opaque, int ret)
{
    IDEState *s = opaque;
    int n;

    n = s->io_nb_sectors;
    ide_set_sector(s, ide_get_sector(s) + n);
    s->nsector = (s->nsector - n) & 0xff;
    if (s->nsector == 0) {
        /* no more sectors to write */
        s->status = READY_STAT | SEEK_STAT;
    } else {
        n = s->nsector;
        if (n > s->req_nb_sectors)
            n = s->req_nb_sectors;
        s->io_nb_sectors = n;
        ide_transfer_start(s, 512 * n, ide_sector_write_cb1);
        s->status = READY_STAT | SEEK_STAT | DRQ_STAT;
    }
    ide_set_irq(s);
}

static void ide_sector_write(IDEState *s)
{
    int n;
    n = s->nsector;
    if (n == 0)
        n = 256;
    if (n > s->req_nb_sectors)
        n = s->req_nb_sectors;
    s->io_nb_sectors = n;
    ide_transfer_start(s, 512 * n, ide_sector_write_cb1);
    s->status = READY_STAT | SEEK_STAT | DRQ_STAT;
}

static void ide_identify_cb(IDEState *s)
{
    ide_transfer_stop(s);
    s->status = READY_STAT;
}

static void ide_exec_cmd(IDEState *s, int val)
{
#if defined(DEBUG_IDE)
    printf("ide: exec_cmd=0x%02x\n", (unsigned)val);
#endif
    switch(val) {
    case WIN_IDENTIFY:
        ide_identify(s);
        s->status = READY_STAT | SEEK_STAT | DRQ_STAT;
        ide_transfer_start(s, 512, ide_identify_cb);
        ide_set_irq(s);
        break;
    case WIN_SPECIFY:
    case WIN_RECAL:
        s->error = 0;
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_SETMULT:
        if (s->nsector > MAX_MULT_SECTORS || 
            (s->nsector & (s->nsector - 1)) != 0) {
            ide_abort_command(s);
        } else {
            s->mult_sectors = s->nsector;
#if defined(DEBUG_IDE)
            printf("ide: setmult=%d\n", s->mult_sectors);
#endif
            s->status = READY_STAT;
        }
        ide_set_irq(s);
        break;
    case WIN_READ:
    case WIN_READ_ONCE:
        s->req_nb_sectors = 1;
        ide_sector_read(s);
        break;
    case WIN_WRITE:
    case WIN_WRITE_ONCE:
        s->req_nb_sectors = 1;
        ide_sector_write(s);
        break;
    case WIN_MULTREAD:
        if (!s->mult_sectors) {
            ide_abort_command(s);
            ide_set_irq(s);
        } else {
            s->req_nb_sectors = s->mult_sectors;
            ide_sector_read(s);
        }
        break;
    case WIN_MULTWRITE:
        if (!s->mult_sectors) {
            ide_abort_command(s);
            ide_set_irq(s);
        } else {
            s->req_nb_sectors = s->mult_sectors;
            ide_sector_write(s);
        }
        break;
    case WIN_READ_NATIVE_MAX:
        ide_set_sector(s, s->nb_sectors - 1);
        s->status = READY_STAT;
        ide_set_irq(s);
        break;
    case WIN_READDMA:
    case WIN_READDMA_ONCE:
        /* DMA not emulated — fall back to PIO multi-sector read.
         * Clamp to MAX_MULT_SECTORS to prevent io_buffer overflow. */
        s->req_nb_sectors = MAX_MULT_SECTORS;
        ide_sector_read(s);
        break;
    case WIN_WRITEDMA:
    case WIN_WRITEDMA_ONCE:
        /* DMA not emulated — fall back to PIO multi-sector write.
         * Clamp to MAX_MULT_SECTORS to prevent io_buffer overflow. */
        s->req_nb_sectors = MAX_MULT_SECTORS;
        ide_sector_write(s);
        break;
    case WIN_VERIFY:
    case WIN_VERIFY_ONCE:
        /* Verify sectors — advance position like a real read, no data transfer */
        { int n = s->nsector ? s->nsector : 256;
          ide_set_sector(s, ide_get_sector(s) + n);
          s->nsector = 0; }
        s->error = 0;
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_DIAGNOSE:
        /* Execute drive diagnostics — set device signature and return no error */
        ide_set_signature(s);
        s->error = 0x01;
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_FLUSH_CACHE:
    case WIN_FLUSH_CACHE_EXT:
        /* Flush cache — data is always written through, just succeed */
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_SETFEATURES:
        if (s->feature == 0x03) {
            /* Set transfer mode — reject DMA/UDMA (nsector >> 3 >= 2) */
            if ((s->nsector >> 3) >= 2) {
                ide_abort_command(s);
                ide_set_irq(s);
                break;
            }
        }
        s->error = 0;
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_SEEK:
        /* Seek — just report success */
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_NOP:
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_CHECKPOWERMODE1:
    case WIN_CHECKPOWERMODE2:
        s->nsector = 0xFF;  /* 0xFF = device is active/idle */
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    case WIN_STANDBYNOW1:
    case WIN_STANDBYNOW2:
    case WIN_IDLEIMMEDIATE:
    case WIN_SETIDLE1:
    case WIN_SETIDLE2:
    case WIN_SLEEPNOW1:
    case WIN_SLEEPNOW2:
    case WIN_STANDBY:
    case WIN_STANDBY2:
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s);
        break;
    default:
        ide_abort_command(s);
        ide_set_irq(s);
        break;
    }
}

static void lba_to_msf(uint8_t *buf, int lba)
{
    lba += 150;
    buf[0] = (lba / 75) / 60;
    buf[1] = (lba / 75) % 60;
    buf[2] = lba % 75;
}

static void cpu_to_be32wu(uint32_t *o, uint32_t v)
{
    *o = __builtin_bswap32(v);
}

static void cpu_to_be16wu(uint16_t *o, uint16_t v)
{
    *o = __builtin_bswap16(v);
}

/* same toc as bochs. Return -1 if error or the toc length */
/* XXX: check this */
static int cdrom_read_toc(int nb_sectors, uint8_t *buf, int msf, int start_track)
{
    uint8_t *q;
    int len;

    if (start_track > 1 && start_track != 0xaa)
        return -1;
    q = buf + 2;
    *q++ = 1; /* first session */
    *q++ = 1; /* last session */
    if (start_track <= 1) {
        *q++ = 0; /* reserved */
        *q++ = 0x14; /* ADR, control */
        *q++ = 1;    /* track number */
        *q++ = 0; /* reserved */
        if (msf) {
            *q++ = 0; /* reserved */
            lba_to_msf(q, 0);
            q += 3;
        } else {
            /* sector 0 */
            cpu_to_be32wu((uint32_t *)q, 0);
            q += 4;
        }
    }
    /* lead out track */
    *q++ = 0; /* reserved */
    *q++ = 0x16; /* ADR, control */
    *q++ = 0xaa; /* track number */
    *q++ = 0; /* reserved */
    if (msf) {
        *q++ = 0; /* reserved */
        lba_to_msf(q, nb_sectors);
        q += 3;
    } else {
        cpu_to_be32wu((uint32_t *)q, nb_sectors);
        q += 4;
    }
    len = q - buf;
    cpu_to_be16wu((uint16_t *)buf, len - 2);
    return len;
}

/* mostly same info as PearPc */
static int cdrom_read_toc_raw(int nb_sectors, uint8_t *buf, int msf, int session_num)
{
    uint8_t *q;
    int len;

    q = buf + 2;
    *q++ = 1; /* first session */
    *q++ = 1; /* last session */

    *q++ = 1; /* session number */
    *q++ = 0x14; /* data track */
    *q++ = 0; /* track number */
    *q++ = 0xa0; /* lead-in */
    *q++ = 0; /* min */
    *q++ = 0; /* sec */
    *q++ = 0; /* frame */
    *q++ = 0;
    *q++ = 1; /* first track */
    *q++ = 0x00; /* disk type */
    *q++ = 0x00;

    *q++ = 1; /* session number */
    *q++ = 0x14; /* data track */
    *q++ = 0; /* track number */
    *q++ = 0xa1;
    *q++ = 0; /* min */
    *q++ = 0; /* sec */
    *q++ = 0; /* frame */
    *q++ = 0;
    *q++ = 1; /* last track */
    *q++ = 0x00;
    *q++ = 0x00;

    *q++ = 1; /* session number */
    *q++ = 0x14; /* data track */
    *q++ = 0; /* track number */
    *q++ = 0xa2; /* lead-out */
    *q++ = 0; /* min */
    *q++ = 0; /* sec */
    *q++ = 0; /* frame */
    if (msf) {
        *q++ = 0; /* reserved */
        lba_to_msf(q, nb_sectors);
        q += 3;
    } else {
        cpu_to_be32wu((uint32_t *)q, nb_sectors);
        q += 4;
    }

    *q++ = 1; /* session number */
    *q++ = 0x14; /* ADR, control */
    *q++ = 0;    /* track number */
    *q++ = 1;    /* point */
    *q++ = 0; /* min */
    *q++ = 0; /* sec */
    *q++ = 0; /* frame */
    if (msf) {
        *q++ = 0;
        lba_to_msf(q, 0);
        q += 3;
    } else {
        *q++ = 0;
        *q++ = 0;
        *q++ = 0;
        *q++ = 0;
    }

    len = q - buf;
    cpu_to_be16wu((uint16_t *)buf, len - 2);
    return len;
}

/* XXX: DVDs that could fit on a CD will be reported as a CD */
static inline int media_present(IDEState *s)
{
    return (s->bs != NULL && s->nb_sectors > 0);
}

/* Safe sector count that returns 0 if no media */
static inline uint64_t cd_get_sector_count(IDEState *s)
{
    if (!s->bs) return 0;
    return s->bs->get_sector_count(s->bs);
}

static inline int media_is_dvd(IDEState *s)
{
    return (media_present(s) && s->nb_sectors > CD_MAX_SECTORS);
}

static inline int media_is_cd(IDEState *s)
{
    return (media_present(s) && s->nb_sectors <= CD_MAX_SECTORS);
}

static void ide_atapi_cmd_ok(IDEState *s)
{
    s->error = 0;
    s->status = READY_STAT | SEEK_STAT;
    s->nsector = (s->nsector & ~7) | ATAPI_INT_REASON_IO | ATAPI_INT_REASON_CD;
    ide_set_irq(s);
}

static void ide_atapi_cmd_error(IDEState *s, int sense_key, int asc)
{
#ifdef DEBUG_IDE_ATAPI
    printf("atapi_cmd_error: sense=0x%x asc=0x%x\n", sense_key, asc);
#endif
    s->error = sense_key << 4;
    s->status = READY_STAT | ERR_STAT;
    s->nsector = (s->nsector & ~7) | ATAPI_INT_REASON_IO | ATAPI_INT_REASON_CD;
    s->sense_key = sense_key;
    s->asc = asc;
    ide_set_irq(s);
}

static void ide_atapi_cmd_check_status(IDEState *s)
{
#ifdef DEBUG_IDE_ATAPI
    printf("atapi_cmd_check_status\n");
#endif
    s->error = MC_ERR | (SENSE_UNIT_ATTENTION << 4);
    s->status = ERR_STAT;
    s->nsector = 0;
    ide_set_irq(s);
}

static inline void cpu_to_ube16(uint8_t *buf, int val)
{
    buf[0] = val >> 8;
    buf[1] = val;
}

static inline void cpu_to_ube32(uint8_t *buf, unsigned int val)
{
    buf[0] = val >> 24;
    buf[1] = val >> 16;
    buf[2] = val >> 8;
    buf[3] = val;
}

static inline int ube16_to_cpu(const uint8_t *buf)
{
    return (buf[0] << 8) | buf[1];
}

static inline int ube32_to_cpu(const uint8_t *buf)
{
    return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

static int cd_read_sector(BlockDevice *bs, int lba, uint8_t *buf,
                           int sector_size)
{
    int ret;

    if (!bs) return -1;  // No disc inserted

    IDE_ACTIVITY();
    switch(sector_size) {
    case 2048:
        ret = bs->read_async(bs, (int64_t)lba << 2, buf, 4,
                             NULL, NULL); // XXX
        break;
    default:
        ret = -1;
        break;
    }
    return ret;
}

static void ide_atapi_io_error(IDEState *s, int ret)
{
    /* XXX: handle more errors */
    ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                        ASC_LOGICAL_BLOCK_OOR);
}

/* The whole ATAPI transfer logic is handled in this function */
static void ide_atapi_cmd_reply_end(IDEState *s)
{
    int byte_count_limit, size, ret;
#ifdef DEBUG_IDE_ATAPI
    printf("reply: tx_size=%d elem_tx_size=%d index=%d\n",
           s->packet_transfer_size,
           s->elementary_transfer_size,
           s->io_buffer_index);
#endif
    if (s->packet_transfer_size <= 0) {
        /* end of transfer */
        ide_transfer_stop2(s);
        s->status = READY_STAT | SEEK_STAT;
        s->nsector = (s->nsector & ~7) | ATAPI_INT_REASON_IO | ATAPI_INT_REASON_CD;
        ide_set_irq(s);
#ifdef DEBUG_IDE_ATAPI
        printf("status=0x%x\n", s->status);
#endif
    } else {
        /* see if a new sector must be read */
        if (s->lba != -1 && s->io_buffer_index >= s->cd_sector_size) {
            ret = cd_read_sector(s->bs, s->lba, s->io_buffer, s->cd_sector_size);
            if (ret < 0) {
                ide_transfer_stop2(s);
                ide_atapi_io_error(s, ret);
                return;
            }
            s->lba++;
            s->io_buffer_index = 0;
        }
        if (s->elementary_transfer_size > 0) {
            /* there are some data left to transmit in this elementary
               transfer */
            size = s->cd_sector_size - s->io_buffer_index;
            if (size > s->elementary_transfer_size)
                size = s->elementary_transfer_size;
            ide_transfer_start2(s, s->io_buffer_index,
                               size, ide_atapi_cmd_reply_end);
            s->packet_transfer_size -= size;
            s->elementary_transfer_size -= size;
            s->io_buffer_index += size;
        } else {
            /* a new transfer is needed */
            s->nsector = (s->nsector & ~7) | ATAPI_INT_REASON_IO;
            byte_count_limit = s->lcyl | (s->hcyl << 8);
#ifdef DEBUG_IDE_ATAPI
            printf("byte_count_limit=%d\n", byte_count_limit);
            printf("status=0x%x\n", s->status);
#endif
            if (byte_count_limit == 0xffff)
                byte_count_limit--;
            size = s->packet_transfer_size;
            if (size > byte_count_limit) {
                /* byte count limit must be even if this case */
                if (byte_count_limit & 1)
                    byte_count_limit--;
                size = byte_count_limit;
            }
            s->lcyl = size;
            s->hcyl = size >> 8;
            s->elementary_transfer_size = size;
            /* we cannot transmit more than one sector at a time */
            if (s->lba != -1) {
                if (size > (s->cd_sector_size - s->io_buffer_index))
                    size = (s->cd_sector_size - s->io_buffer_index);
            }
            ide_transfer_start2(s, s->io_buffer_index,
                               size, ide_atapi_cmd_reply_end);
            s->packet_transfer_size -= size;
            s->elementary_transfer_size -= size;
            s->io_buffer_index += size;
            ide_set_irq(s);
        }
    }
}

/* send a reply of 'size' bytes in s->io_buffer to an ATAPI command */
static void ide_atapi_cmd_reply(IDEState *s, int size, int max_size)
{
    if (size > max_size)
        size = max_size;
    s->lba = -1; /* no sector read */
    s->packet_transfer_size = size;
//    s->io_buffer_size = size;    /* dma: send the reply data as one chunk */
    s->elementary_transfer_size = 0;
    s->io_buffer_index = 0;

    s->status = READY_STAT | SEEK_STAT;
    ide_atapi_cmd_reply_end(s);
}

/* start a CD-CDROM read command */
static void ide_atapi_cmd_read_pio(IDEState *s, int lba, int nb_sectors,
                                   int sector_size)
{
    s->lba = lba;
    s->packet_transfer_size = nb_sectors * sector_size;
    s->elementary_transfer_size = 0;
    s->io_buffer_index = sector_size;
    s->cd_sector_size = sector_size;

    s->status = READY_STAT | SEEK_STAT;
    ide_atapi_cmd_reply_end(s);
}

static void ide_atapi_cmd_read(IDEState *s, int lba, int nb_sectors,
                               int sector_size)
{
#ifdef DEBUG_IDE_ATAPI
    printf("read %s: LBA=%d nb_sectors=%d\n", 0 ? "dma" : "pio",
           lba, nb_sectors);
#endif
    ide_atapi_cmd_read_pio(s, lba, nb_sectors, sector_size);
}

static inline uint8_t ide_atapi_set_profile(uint8_t *buf, uint8_t *index,
                                            uint16_t profile)
{
    uint8_t *buf_profile = buf + 12; /* start of profiles */

    buf_profile += ((*index) * 4); /* start of indexed profile */
    cpu_to_ube16 (buf_profile, profile);
    buf_profile[2] = ((buf_profile[0] == buf[6]) && (buf_profile[1] == buf[7]));

    /* each profile adds 4 bytes to the response */
    (*index)++;
    buf[11] += 4; /* Additional Length */

    return 4;
}

static void ide_atapi_cmd(IDEState *s)
{
    const uint8_t *packet;
    uint8_t *buf;
    int max_len;

    packet = s->io_buffer;
    buf = s->io_buffer;
#ifdef DEBUG_IDE_ATAPI
    {
        int i;
        printf("ATAPI limit=0x%x packet:", s->lcyl | (s->hcyl << 8));
        for(i = 0; i < ATAPI_PACKET_SIZE; i++) {
            printf(" %02x", packet[i]);
        }
        printf("\n");
    }
#endif
    /* If there's a UNIT_ATTENTION condition pending, only
       REQUEST_SENSE and INQUIRY commands are allowed to complete. */
    if (s->sense_key == SENSE_UNIT_ATTENTION &&
        s->io_buffer[0] != GPCMD_REQUEST_SENSE &&
        s->io_buffer[0] != GPCMD_INQUIRY) {
        ide_atapi_cmd_check_status(s);
        return;
    }
    switch(s->io_buffer[0]) {
    case GPCMD_TEST_UNIT_READY:
        if (!s->cdrom_changed) {
            ide_atapi_cmd_ok(s);
        } else {
            s->cdrom_changed = 0;
            ide_atapi_cmd_error(s, SENSE_NOT_READY,
                                ASC_MEDIUM_NOT_PRESENT);
        }
        break;
    case GPCMD_MODE_SENSE_6:
    case GPCMD_MODE_SENSE_10:
        {
            int action, code;
            if (packet[0] == GPCMD_MODE_SENSE_10)
                max_len = ube16_to_cpu(packet + 7);
            else
                max_len = packet[4];
            action = packet[2] >> 6;
            code = packet[2] & 0x3f;
            switch(action) {
            case 0: /* current values */
                switch(code) {
                case 0x01: /* error recovery */
                    cpu_to_ube16(&buf[0], 16 + 6);
                    buf[2] = 0x70;
                    buf[3] = 0;
                    buf[4] = 0;
                    buf[5] = 0;
                    buf[6] = 0;
                    buf[7] = 0;

                    buf[8] = 0x01;
                    buf[9] = 0x06;
                    buf[10] = 0x00;
                    buf[11] = 0x05;
                    buf[12] = 0x00;
                    buf[13] = 0x00;
                    buf[14] = 0x00;
                    buf[15] = 0x00;
                    ide_atapi_cmd_reply(s, 16, max_len);
                    break;
                case 0x2a:
                    cpu_to_ube16(&buf[0], 28 + 6);
                    buf[2] = 0x70;
                    buf[3] = 0;
                    buf[4] = 0;
                    buf[5] = 0;
                    buf[6] = 0;
                    buf[7] = 0;

                    buf[8] = 0x2a;
                    buf[9] = 0x12;
                    buf[10] = 0x00;
                    buf[11] = 0x00;

                    /* Claim PLAY_AUDIO capability (0x01) since some Linux
                       code checks for this to automount media. */
                    buf[12] = 0x71;
                    buf[13] = 3 << 5;
                    buf[14] = (1 << 0) | (1 << 3) | (1 << 5);
//                    if (bdrv_is_locked(s->bs))
//                        buf[6] |= 1 << 1;
                    buf[15] = 0x00;
                    cpu_to_ube16(&buf[16], 706);
                    buf[18] = 0;
                    buf[19] = 2;
                    cpu_to_ube16(&buf[20], 512);
                    cpu_to_ube16(&buf[22], 706);
                    buf[24] = 0;
                    buf[25] = 0;
                    buf[26] = 0;
                    buf[27] = 0;
                    ide_atapi_cmd_reply(s, 28, max_len);
                    break;
                default:
                    goto error_cmd;
                }
                break;
            case 1: /* changeable values */
                goto error_cmd;
            case 2: /* default values */
                goto error_cmd;
            default:
            case 3: /* saved values */
                ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                                    ASC_SAVING_PARAMETERS_NOT_SUPPORTED);
                break;
            }
        }
        break;
    case GPCMD_REQUEST_SENSE:
        max_len = packet[4];
        memset(buf, 0, 18);
        buf[0] = 0x70 | (1 << 7);
        buf[2] = s->sense_key;
        buf[7] = 10;
        buf[12] = s->asc;
        if (s->sense_key == SENSE_UNIT_ATTENTION)
            s->sense_key = SENSE_NONE;
        ide_atapi_cmd_reply(s, 18, max_len);
        break;
    case GPCMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        ide_atapi_cmd_ok(s);
        break;
    case GPCMD_READ_10:
    case GPCMD_READ_12:
        {
            int nb_sectors, lba;

            if (packet[0] == GPCMD_READ_10)
                nb_sectors = ube16_to_cpu(packet + 7);
            else
                nb_sectors = ube32_to_cpu(packet + 6);
            lba = ube32_to_cpu(packet + 2);
            if (nb_sectors == 0) {
                ide_atapi_cmd_ok(s);
                break;
            }
            ide_atapi_cmd_read(s, lba, nb_sectors, 2048);
        }
        break;
    case GPCMD_READ_CD:
        {
            int nb_sectors, lba, transfer_request;

            nb_sectors = (packet[6] << 16) | (packet[7] << 8) | packet[8];
            lba = ube32_to_cpu(packet + 2);
            if (nb_sectors == 0) {
                ide_atapi_cmd_ok(s);
                break;
            }
            transfer_request = packet[9];
            switch(transfer_request & 0xf8) {
            case 0x00:
                /* nothing */
                ide_atapi_cmd_ok(s);
                break;
            case 0x10:
                /* normal read */
                ide_atapi_cmd_read(s, lba, nb_sectors, 2048);
                break;
            case 0xf8:
                /* read all data */
                ide_atapi_cmd_read(s, lba, nb_sectors, 2352);
                break;
            default:
                ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                                    ASC_INV_FIELD_IN_CMD_PACKET);
                break;
            }
        }
        break;
    case GPCMD_SEEK:
        {
            unsigned int lba;
            uint64_t total_sectors;

            total_sectors = cd_get_sector_count(s);
            total_sectors >>= 2;
            if (total_sectors == 0) {
                ide_atapi_cmd_error(s, SENSE_NOT_READY,
                                    ASC_MEDIUM_NOT_PRESENT);
                break;
            }
            lba = ube32_to_cpu(packet + 2);
            if (lba >= total_sectors) {
                ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                                    ASC_LOGICAL_BLOCK_OOR);
                break;
            }
            ide_atapi_cmd_ok(s);
        }
        break;
    case GPCMD_START_STOP_UNIT:
        {
            ide_atapi_cmd_ok(s);
        }
        break;
    case GPCMD_MECHANISM_STATUS:
        {
            max_len = ube16_to_cpu(packet + 8);
            cpu_to_ube16(buf, 0);
            /* no current LBA */
            buf[2] = 0;
            buf[3] = 0;
            buf[4] = 0;
            buf[5] = 1;
            cpu_to_ube16(buf + 6, 0);
            ide_atapi_cmd_reply(s, 8, max_len);
        }
        break;
    case GPCMD_READ_TOC_PMA_ATIP:
        {
            int format, msf, start_track, len;
            uint64_t total_sectors;

            total_sectors = cd_get_sector_count(s);
            total_sectors >>= 2;
            if (total_sectors == 0) {
                ide_atapi_cmd_error(s, SENSE_NOT_READY,
                                    ASC_MEDIUM_NOT_PRESENT);
                break;
            }
            max_len = ube16_to_cpu(packet + 7);
            format = packet[9] >> 6;
            msf = (packet[1] >> 1) & 1;
            start_track = packet[6];
            switch(format) {
            case 0:
                len = cdrom_read_toc(total_sectors, buf, msf, start_track);
                if (len < 0)
                    goto error_cmd;
                ide_atapi_cmd_reply(s, len, max_len);
                break;
            case 1:
                /* multi session : only a single session defined */
                memset(buf, 0, 12);
                buf[1] = 0x0a;
                buf[2] = 0x01;
                buf[3] = 0x01;
                ide_atapi_cmd_reply(s, 12, max_len);
                break;
            case 2:
                len = cdrom_read_toc_raw(total_sectors, buf, msf, start_track);
                if (len < 0)
                    goto error_cmd;
                ide_atapi_cmd_reply(s, len, max_len);
                break;
            default:
            error_cmd:
                ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                                    ASC_INV_FIELD_IN_CMD_PACKET);
                break;
            }
        }
        break;
    case GPCMD_READ_CDVD_CAPACITY:
        {
            uint64_t total_sectors;

            total_sectors = cd_get_sector_count(s);
            total_sectors >>= 2;
            if (total_sectors == 0) {
                ide_atapi_cmd_error(s, SENSE_NOT_READY,
                                    ASC_MEDIUM_NOT_PRESENT);
                break;
            }
            /* NOTE: it is really the number of sectors minus 1 */
            cpu_to_ube32(buf, total_sectors - 1);
            cpu_to_ube32(buf + 4, 2048);
            ide_atapi_cmd_reply(s, 8, 8);
        }
        break;
    case GPCMD_SET_SPEED:
        ide_atapi_cmd_ok(s);
        break;
    case GPCMD_INQUIRY:
        max_len = packet[4];
        buf[0] = 0x05; /* CD-ROM */
        buf[1] = 0x80; /* removable */
        buf[2] = 0x00; /* ISO */
        buf[3] = 0x21; /* ATAPI-2 (XXX: put ATAPI-4 ?) */
        buf[4] = 31; /* additional length */
        buf[5] = 0; /* reserved */
        buf[6] = 0; /* reserved */
        buf[7] = 0; /* reserved */
        padstr8(buf + 8, 8, "TINY386");
        padstr8(buf + 16, 16, "TINY386 CD-ROM");
        padstr8(buf + 32, 4, "0.1");
        ide_atapi_cmd_reply(s, 36, max_len);
        break;
    case GPCMD_GET_CONFIGURATION:
        {
            uint32_t len;
            uint8_t index = 0;

            /* only feature 0 is supported */
            if (packet[2] != 0 || packet[3] != 0) {
                ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                                    ASC_INV_FIELD_IN_CMD_PACKET);
                break;
            }

            /* XXX: could result in alignment problems in some architectures */
            max_len = ube16_to_cpu(packet + 7);

            /*
             * XXX: avoid overflow for io_buffer if max_len is bigger than
             *      the size of that buffer (dimensioned to max number of
             *      sectors to transfer at once)
             *
             *      Only a problem if the feature/profiles grow.
             */
            if (max_len > 512) /* XXX: assume 1 sector */
                max_len = 512;

            memset(buf, 0, max_len);
            /* 
             * the number of sectors from the media tells us which profile
             * to use as current.  0 means there is no media
             */
            if (media_is_dvd(s))
                cpu_to_ube16(buf + 6, MMC_PROFILE_DVD_ROM);
            else if (media_is_cd(s))
                cpu_to_ube16(buf + 6, MMC_PROFILE_CD_ROM);

            buf[10] = 0x02 | 0x01; /* persistent and current */
            len = 12; /* headers: 8 + 4 */
            len += ide_atapi_set_profile(buf, &index, MMC_PROFILE_DVD_ROM);
            len += ide_atapi_set_profile(buf, &index, MMC_PROFILE_CD_ROM);
            cpu_to_ube32(buf, len - 4); /* data length */

            ide_atapi_cmd_reply(s, len, max_len);
            break;
        }
    default:
        ide_atapi_cmd_error(s, SENSE_ILLEGAL_REQUEST,
                            ASC_ILLEGAL_OPCODE);
        break;
    }
}

static void idecd_exec_cmd(IDEState *s, int val)
{
#if defined(DEBUG_IDE)
    printf("idecd: exec_cmd=0x%02x\n", (unsigned)val);
#endif
    switch(val) {
    case WIN_DEVICE_RESET:
        //ide_transfer_halt(s);
        //ide_cancel_dma_sync(s);
        //ide_reset(s);
        ide_set_signature(s);
        s->status = 0x00;
        break;
    case WIN_PACKETCMD:
        s->status = READY_STAT | SEEK_STAT;
        s->nsector = 1;
        ide_transfer_start2(s, 0, ATAPI_PACKET_SIZE, ide_atapi_cmd);
        ide_set_irq(s);
        break;
    case WIN_PIDENTIFY:
        ide_atapi_identify(s);
        s->status = READY_STAT | SEEK_STAT;
        ide_transfer_start2(s, 0, 512, ide_transfer_stop2);
        ide_set_irq(s);
        break;
    case WIN_IDENTIFY:
    case WIN_READ:
        ide_set_signature(s);
        /* fall through */
    default:
        ide_abort_command(s);
        ide_set_irq(s);
        break;
    }
}

void ide_ioport_write(void *opaque, uint32_t addr, uint32_t val)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    
#ifdef DEBUG_IDE
    printf("ide: write addr=0x%02x val=0x%02x\n", (unsigned)addr, (unsigned)val);
#endif
    switch(addr) {
    case 0:
        break;
    case 1:
        if (s) {
            s->feature = val;
        }
        break;
    case 2:
        if (s) {
            s->nsector = val;
        }
        break;
    case 3:
        if (s) {
            s->sector = val;
        }
        break;
    case 4:
        if (s) {
            s->lcyl = val;
        }
        break;
    case 5:
        if (s) {
            s->hcyl = val;
        }
        break;
    case 6:
        /* select drive */
        s = s1->cur_drive = s1->drives[(val >> 4) & 1];
        if (s) {
            s->select = val;
        }
        break;
    default:
    case 7:
        /* command */
        if (s) {
            if (s->drive_kind != IDE_CD)
                ide_exec_cmd(s, val);
            else
                idecd_exec_cmd(s, val);
        }
        break;
    }
}

uint32_t ide_ioport_read(void *opaque, uint32_t addr)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    int ret;

    if (!s) {
        ret = 0x00;
    } else {
        switch(addr) {
        case 0:
           ret = 0xff;
           break;
        case 1:
            ret = s->error;
            break;
        case 2:
            ret = s->nsector;
            break;
        case 3:
            ret = s->sector;
            break;
        case 4:
            ret = s->lcyl;
            break;
        case 5:
            ret = s->hcyl;
            break;
        case 6:
            ret = s->select;
            break;
        default:
        case 7:
            ret = s->status;
            s1->set_irq(s1->pic, s1->irq, 0);
            break;
        }
    }
#ifdef DEBUG_IDE
    printf("ide: read addr=0x%02x val=0x%02x\n", (unsigned)addr, (unsigned)ret);
#endif
    return ret;
}

uint32_t ide_status_read(void *opaque)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    int ret;

    /* Phase 3: complete async read if done */
    if (s && (s->status & BUSY_STAT) &&
        __atomic_load_n(&async_read.done, __ATOMIC_ACQUIRE)) {
        __atomic_store_n(&async_read.done, 0, __ATOMIC_RELAXED);
        async_read.cb(async_read.cb_opaque, async_read.result);
    }

    if (s) {
        ret = s->status;
    } else {
        ret = 0;
    }
#ifdef DEBUG_IDE
    printf("ide: read status=0x%02x\n", ret);
#endif
    return ret;
}

void ide_cmd_write(void *opaque, uint32_t val)
{
    IDEIFState *s1 = opaque;
    IDEState *s;
    int i;
    
#ifdef DEBUG_IDE
    printf("ide: cmd write=0x%02x\n", (unsigned)val);
#endif
    if (!(s1->cmd & IDE_CMD_RESET) && (val & IDE_CMD_RESET)) {
        /* low to high */
        for(i = 0; i < 2; i++) {
            s = s1->drives[i];
            if (s) {
                s->status = BUSY_STAT | SEEK_STAT;
                s->error = 0x01;
            }
        }
    } else if ((s1->cmd & IDE_CMD_RESET) && !(val & IDE_CMD_RESET)) {
        /* high to low */
        for(i = 0; i < 2; i++) {
            s = s1->drives[i];
            if (s) {
                s->status = READY_STAT | SEEK_STAT;
                ide_set_signature(s);
            }
        }
    }
    s1->cmd = val;
}

void ide_data_writew(void *opaque, uint32_t val)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    int p;
    uint8_t *tab;
    
    if (!s)
        return;
    p = s->data_index;
    if (p + 2 > s->data_end)
        return;
    tab = s->io_buffer;
    tab[p] = val & 0xff;
    tab[p + 1] = (val >> 8) & 0xff;
    p += 2;
    s->data_index = p;
    if (p >= s->data_end)
        s->end_transfer_func(s);
}

uint32_t ide_data_readw(void *opaque)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    int p, ret;
    uint8_t *tab;
    
    if (!s) {
        ret = 0;
    } else {
        p = s->data_index;
        if (p + 2 > s->data_end)
            return 0;
        tab = s->io_buffer;
        ret = tab[p] | (tab[p + 1] << 8);
        p += 2;
        s->data_index = p;
        if (p >= s->data_end)
            s->end_transfer_func(s);
    }
    return ret;
}

void ide_data_writel(void *opaque, uint32_t val)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    int p;
    uint8_t *tab;
    
    if (!s)
        return;
    p = s->data_index;
    if (p + 4 > s->data_end)
        return;
    tab = s->io_buffer;
    tab[p] = val & 0xff;
    tab[p + 1] = (val >> 8) & 0xff;
    tab[p + 2] = (val >> 16) & 0xff;
    tab[p + 3] = (val >> 24) & 0xff;
    p += 4;
    s->data_index = p;
    if (p >= s->data_end)
        s->end_transfer_func(s);
}

uint32_t ide_data_readl(void *opaque)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    int p, ret;
    uint8_t *tab;
    
    if (!s) {
        ret = 0;
    } else {
        p = s->data_index;
        if (p + 4 > s->data_end)
            return 0;
        tab = s->io_buffer;
        ret = tab[p] | (tab[p + 1] << 8) | (tab[p + 2] << 16) | (tab[p + 3] << 24);
        p += 4;
        s->data_index = p;
        if (p >= s->data_end)
            s->end_transfer_func(s);
    }
    return ret;
}

int ide_data_write_string(void *opaque, uint8_t *buf, int size, int count)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    if (!s)
        return 0;

    int avail = s->data_end - s->data_index;
    if (avail <= 0)
        return 0;
    int len1 = size * count;
    if (len1 > avail)
        len1 = avail;
    len1 -= len1 % size;
    if (len1 > 0) {
        memcpy(s->io_buffer + s->data_index, buf, len1);
        s->data_index += len1;
    }
    if (s->data_index >= s->data_end)
        s->end_transfer_func(s);
    return len1 / size;
}

int ide_data_read_string(void *opaque, uint8_t *buf, int size, int count)
{
    IDEIFState *s1 = opaque;
    IDEState *s = s1->cur_drive;
    if (!s)
        return 0;

    int avail = s->data_end - s->data_index;
    if (avail <= 0)
        return 0;
    int len1 = size * count;
    if (len1 > avail)
        len1 = avail;
    len1 -= len1 % size;
    if (len1 > 0) {
        memcpy(buf, s->io_buffer + s->data_index, len1);
        s->data_index += len1;
    }
    if (s->data_index >= s->data_end)
        s->end_transfer_func(s);
    return len1 / size;
}

void ide_set_geometry(IDEIFState *s, int drive, int heads, int spt)
{
    IDEState *d = s->drives[drive];
    if (!d || d->drive_kind != IDE_HD)
        return;
    d->heads = heads;
    d->sectors = spt;
    uint32_t cylinders = d->nb_sectors / (heads * spt);
    if (cylinders > 16383) cylinders = 16383;
    else if (cylinders < 2) cylinders = 2;
    d->cylinders = cylinders;
}

static IDEState *ide_hddrive_init(IDEIFState *ide_if, BlockDevice *bs)
{
    IDEState *s;
    uint32_t cylinders;
    uint64_t nb_sectors;

#ifndef USE_RAWSD
    s = pcmalloc(sizeof(*s));
#else
    s = malloc(sizeof(*s));
#endif
    memset(s, 0, sizeof(*s));

    s->ide_if = ide_if;
    s->bs = bs;
    s->drive_kind = IDE_HD;

    nb_sectors = s->bs->get_sector_count(s->bs);
    /* Use 255H/63S for disks >504MB (matches QEMU/real BIOS behavior),
       16H/63S for smaller DOS-era disks */
    if (nb_sectors > (uint64_t)16 * 63 * 1024) {
        s->heads = 255;
        s->sectors = 63;
    } else {
        s->heads = 16;
        s->sectors = 63;
    }
    cylinders = nb_sectors / (s->heads * s->sectors);
    if (cylinders > 16383)
        cylinders = 16383;
    else if (cylinders < 2)
        cylinders = 2;
    s->cylinders = cylinders;
    s->nb_sectors = nb_sectors;
    if (s->bs->get_chs)
        s->bs->get_chs(s->bs, &s->cylinders, &s->heads, &s->sectors);

    s->mult_sectors = MAX_MULT_SECTORS;
    /* ide regs */
    s->feature = 0;
    s->error = 0;
    s->nsector = 0;
    s->sector = 0;
    s->lcyl = 0;
    s->hcyl = 0;
    s->select = 0xa0;
    s->status = READY_STAT | SEEK_STAT;

    /* init I/O buffer */
    s->data_index = 0;
    s->data_end = 0;
    s->end_transfer_func = ide_transfer_stop;

    s->req_nb_sectors = 0; /* temp for read/write */
    s->io_nb_sectors = 0; /* temp for read/write */
    return s;
}

static IDEState *ide_cddrive_init(IDEIFState *ide_if, BlockDevice *bs)
{
    IDEState *s;

#ifndef USE_RAWSD
    s = pcmalloc(sizeof(*s));
#else
    s = malloc(sizeof(*s));
#endif
    memset(s, 0, sizeof(*s));

    s->ide_if = ide_if;
    s->bs = bs;
    s->drive_kind = IDE_CD;

    /* ide regs */
    s->feature = 0;
    s->error = 0;
    s->nsector = 0;
    s->sector = 0;
    s->lcyl = 0;
    s->hcyl = 0;
    s->select = 0xa0;
    s->status = READY_STAT | SEEK_STAT;

    ide_set_signature(s);

    /* init I/O buffer */
    s->data_index = 0;
    s->data_end = 0;
    s->end_transfer_func = ide_transfer_stop2;

    s->req_nb_sectors = 0; /* temp for read/write */
    s->io_nb_sectors = 0; /* temp for read/write */
    return s;
}

// old img format
const uint8_t ide_magic[8] = {
  '1','D','E','D','1','5','C','0'
};

typedef enum {
    BF_MODE_RO,
    BF_MODE_RW,
    BF_MODE_SNAPSHOT,
} BlockDeviceModeEnum;

#define SECTOR_SIZE 512

/* Run-length encoded mapping: contiguous file sectors → SD card sectors */
typedef struct {
    uint32_t file_sector;  /* Starting sector within the disk image file */
    uint32_t sd_sector;    /* Starting sector on the SD card */
    uint32_t count;        /* Number of contiguous 512-byte sectors */
} DiskExtent;

typedef struct BlockDeviceFile BlockDeviceFile;

struct BlockDeviceFile {
    FILE *f;               /* stdio handle (NULL when using raw SD) */
    int start_offset;
    int cylinders, heads, sectors;
    int64_t nb_sectors;
    BlockDeviceModeEnum mode;
    uint8_t **sector_table;
    char path[256];  // Store filename for OSD display

    /* Raw SD acceleration (bypasses VFS/FatFS) */
    sdmmc_card_t *card;    /* Raw SD card handle (NULL = use stdio) */
    DiskExtent *extents;   /* Extent map (NULL = use stdio) */
    int num_extents;

    /* Fallback stdio cache (used when raw SD not available) */
    uint8_t *cache_buf;        /* Cache buffer in PSRAM */
    int64_t cache_start;       /* First sector in cache */
    int cache_count;           /* Number of valid sectors in cache */
#define DISK_CACHE_SECTORS 64  /* Cache 64 sectors = 32KB per disk */
    int dirty;                 /* Set on write, cleared on sync */
};

static int64_t bf_get_sector_count(BlockDevice *bs)
{
    BlockDeviceFile *bf = bs->opaque;
    return bf->nb_sectors;
}

static int bf_get_chs(BlockDevice *bs, int *cylinders, int *heads, int *sectors)
{
    BlockDeviceFile *bf = bs->opaque;
    *cylinders = bf->cylinders;
    *heads = bf->heads;
    *sectors = bf->sectors;
    return 0;
}

/*
 * Build a raw SD sector extent map for a file on the FAT32 SD card.
 * This allows bypassing VFS/FatFS entirely for disk image I/O,
 * avoiding the expensive cluster-chain walk on every fseeko().
 *
 * Returns 0 on success (bf->extents/num_extents populated), -1 on failure.
 */
extern void *rawsd;

/* ── Write-behind cache + async I/O infrastructure ── */
#define WCACHE_SLOTS 512  /* 256KB ring buffer */
#define WCACHE_INDEX_SIZE 4096u
#define WCACHE_INDEX_MASK (WCACHE_INDEX_SIZE - 1u)

typedef struct {
    BlockDeviceFile *bf;
    uint64_t disk_sector;
    uint32_t seq;   /* enqueue sequence number (maps to ring slot via % WCACHE_SLOTS) */
} WCacheIndexEntry;

static struct {
    uint8_t *data;                      /* PSRAM buffer: WCACHE_SLOTS * 512 bytes */
    struct {
        BlockDeviceFile *bf;
        uint64_t disk_sector;           /* sector_num in disk image space */
        uint32_t sd_sector;
    } meta[WCACHE_SLOTS];
    WCacheIndexEntry *index;            /* 2-choice hash: latest write per (bf,sector) */
    uint32_t head;                      /* producer index (atomic access) */
    uint32_t tail;                      /* consumer index (atomic access) */
    SemaphoreHandle_t not_empty;
    SemaphoreHandle_t sd_mutex;         /* protects all sdmmc_read/write calls */
    sdmmc_card_t *card;
    int active;

    /* Lookup telemetry */
    uint64_t lookup_calls;
    uint64_t lookup_hits;
    uint64_t lookup_index_hits;
    uint64_t lookup_index_stale;
    uint64_t lookup_index_out_of_range;
    uint64_t lookup_fallback_hits;
    uint64_t lookup_fallback_misses;
    uint64_t lookup_scan_steps;
    uint64_t lookup_reported_calls;
} wcache;

static uint32_t extent_lookup(BlockDeviceFile *bf, uint32_t file_sector, int *contig);
static int wcache_lookup(BlockDeviceFile *bf, uint64_t sector_num, uint8_t *buf);
static void wcache_report_stats(void);

static inline uint32_t wcache_mix64(uint64_t x)
{
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return (uint32_t)x;
}

static inline uint32_t wcache_hash1(const BlockDeviceFile *bf, uint64_t sector)
{
    uint64_t x = sector ^ ((uint64_t)(uintptr_t)bf * 0x9e3779b97f4a7c15ULL);
    return wcache_mix64(x) & WCACHE_INDEX_MASK;
}

static inline uint32_t wcache_hash2(const BlockDeviceFile *bf, uint64_t sector)
{
    uint64_t x = (sector << 1) ^ ((uint64_t)(uintptr_t)bf * 0xd6e8feb86659fd93ULL) ^
                 0xa0761d6478bd642fULL;
    return wcache_mix64(x) & WCACHE_INDEX_MASK;
}

/* True if seq is currently in [tail, head) with wrap-safe unsigned arithmetic. */
static inline bool wcache_seq_in_window(uint32_t seq, uint32_t tail, uint32_t head)
{
    return (uint32_t)(seq - tail) < (uint32_t)(head - tail);
}

static inline void wcache_index_store(WCacheIndexEntry *e, BlockDeviceFile *bf,
                                      uint64_t sector, uint32_t seq)
{
    e->bf = bf;
    e->disk_sector = sector;
    e->seq = seq;
}

/* Update latest write location for (bf, sector) with 2-choice hashing. */
static inline void wcache_index_update(BlockDeviceFile *bf, uint64_t sector, uint32_t seq)
{
    if (!wcache.index)
        return;

    uint32_t i1 = wcache_hash1(bf, sector);
    uint32_t i2 = wcache_hash2(bf, sector);
    WCacheIndexEntry *e1 = &wcache.index[i1];
    WCacheIndexEntry *e2 = &wcache.index[i2];

    if (i1 == i2) {
        wcache_index_store(e1, bf, sector, seq);
        return;
    }
    if (e1->bf == bf && e1->disk_sector == sector) {
        e1->seq = seq;
        return;
    }
    if (e2->bf == bf && e2->disk_sector == sector) {
        e2->seq = seq;
        return;
    }
    if (!e1->bf) {
        wcache_index_store(e1, bf, sector, seq);
        return;
    }
    if (!e2->bf) {
        wcache_index_store(e2, bf, sector, seq);
        return;
    }

    /* Replace the older candidate (age relative to current seq). */
    uint32_t age1 = seq - e1->seq;
    uint32_t age2 = seq - e2->seq;
    if (age1 >= age2)
        wcache_index_store(e1, bf, sector, seq);
    else
        wcache_index_store(e2, bf, sector, seq);
}

/* Try O(1) lookup via index; validates slot metadata before returning data. */
static inline int wcache_lookup_index(BlockDeviceFile *bf, uint64_t sector_num,
                                      uint8_t *buf, uint32_t tail, uint32_t head)
{
    if (!wcache.index)
        return 0;

    uint32_t idx[2];
    idx[0] = wcache_hash1(bf, sector_num);
    idx[1] = wcache_hash2(bf, sector_num);

    int checks = (idx[0] == idx[1]) ? 1 : 2;
    for (int k = 0; k < checks; k++) {
        WCacheIndexEntry *e = &wcache.index[idx[k]];
        if (e->bf != bf || e->disk_sector != sector_num)
            continue;

        uint32_t seq = e->seq;
        if (!wcache_seq_in_window(seq, tail, head)) {
            wcache.lookup_index_out_of_range++;
            continue;
        }

        uint32_t slot = seq % WCACHE_SLOTS;
        if (wcache.meta[slot].bf == bf &&
            wcache.meta[slot].disk_sector == sector_num) {
            memcpy(buf, wcache.data + slot * SECTOR_SIZE, SECTOR_SIZE);
            wcache.lookup_index_hits++;
            return 1;
        }

        wcache.lookup_index_stale++;
    }

    return 0;
}

static void io_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(wcache.not_empty, portMAX_DELAY);

        /* Phase 3: async reads have priority over writes */
        if (__atomic_load_n(&async_read.pending, __ATOMIC_ACQUIRE)) {
            BlockDeviceFile *bf = async_read.bf;
            uint8_t *buf = async_read.buf;
            uint64_t sector_num = async_read.sector_num;
            int n = async_read.count;
            int start_offset = async_read.start_offset;
            int result = 0;

            uint32_t file_sector = (uint32_t)((sector_num * SECTOR_SIZE + start_offset) / SECTOR_SIZE);
            while (n > 0) {
                /* Check write cache first — pending writes not yet on SD */
                if (wcache_lookup(bf, sector_num, buf)) {
                    buf += SECTOR_SIZE;
                    file_sector++;
                    sector_num++;
                    n--;
                    continue;
                }
                int contig;
                uint32_t sd_sector = extent_lookup(bf, file_sector, &contig);
                if (sd_sector == (uint32_t)-1) {
                    memset(buf, 0, n * SECTOR_SIZE);
                    break;
                }
                int chunk = (n < contig) ? n : contig;
                esp_err_t err = sdmmc_read_sectors_retry(bf->card, buf, sd_sector, chunk);
                if (err != ESP_OK) {
                    result = -1;
                    break;
                }
                /* Overlay any write-cached sectors over stale SD data */
                for (int c = 0; c < chunk; c++)
                    wcache_lookup(bf, sector_num + c, buf + c * SECTOR_SIZE);
                buf += chunk * SECTOR_SIZE;
                file_sector += chunk;
                sector_num += chunk;
                n -= chunk;
            }

            async_read.result = result;
            __atomic_store_n(&async_read.done, 1, __ATOMIC_RELEASE);
            __atomic_store_n(&async_read.pending, 0, __ATOMIC_RELEASE);
            continue;  /* re-check semaphore */
        }

        /* Drain writes: batch adjacent SD sectors */
        {
            uint32_t local_head = __atomic_load_n(&wcache.head, __ATOMIC_ACQUIRE);
            uint32_t local_tail = wcache.tail;  /* only io_task writes tail */
            while (local_tail != local_head) {
                uint32_t t = local_tail % WCACHE_SLOTS;
                uint32_t sd_start = wcache.meta[t].sd_sector;
                BlockDeviceFile *start_bf = wcache.meta[t].bf;
                uint32_t batch = 1;

                /* Coalesce adjacent sectors: same device, sequential SD sectors,
                 * cap at 16, don't cross ring buffer wrap boundary */
                uint32_t max_batch = WCACHE_SLOTS - t;
                if (max_batch > 16) max_batch = 16;
                while (batch < max_batch && local_tail + batch < local_head) {
                    uint32_t next = (local_tail + batch) % WCACHE_SLOTS;
                    if (wcache.meta[next].bf != start_bf ||
                        wcache.meta[next].sd_sector != sd_start + batch)
                        break;
                    batch++;
                }

                esp_err_t werr = sdmmc_write_sectors_retry(wcache.card,
                                                           wcache.data + t * SECTOR_SIZE,
                                                           sd_start, batch);
                if (werr != ESP_OK) {
                    fprintf(stderr, "wcache drain write failed, preserving queue tail=%u\n",
                            local_tail);
                    xSemaphoreGive(wcache.not_empty);
                    break;
                }

                local_tail += batch;
                __atomic_store_n(&wcache.tail, local_tail, __ATOMIC_RELEASE);
            }
        }
    }
}

static void wcache_init(void)
{
    if (wcache.active)
        return;
    if ((WCACHE_INDEX_SIZE & (WCACHE_INDEX_SIZE - 1u)) != 0) {
        fprintf(stderr, "wcache: WCACHE_INDEX_SIZE must be a power of two\n");
        return;
    }
    wcache.data = heap_caps_malloc(WCACHE_SLOTS * SECTOR_SIZE,
                                   MALLOC_CAP_SPIRAM);
    if (!wcache.data) {
        fprintf(stderr, "wcache: failed to allocate %dKB PSRAM\n",
                WCACHE_SLOTS * SECTOR_SIZE / 1024);
        return;
    }
    wcache.index = heap_caps_calloc(WCACHE_INDEX_SIZE, sizeof(WCacheIndexEntry),
                                    MALLOC_CAP_SPIRAM);
    if (!wcache.index) {
        fprintf(stderr, "wcache: index alloc failed, using reverse-scan fallback only\n");
    }
    wcache.sd_mutex = xSemaphoreCreateMutex();
    sdmmc_io_mutex = wcache.sd_mutex;
    wcache.not_empty = xSemaphoreCreateBinary();
    wcache.card = (sdmmc_card_t *)rawsd;
    wcache.head = 0;
    wcache.tail = 0;
    wcache.active = 1;
    xTaskCreatePinnedToCore(io_task, "ide_io", 4096, NULL, 2, NULL, 0);
    fprintf(stderr, "wcache: %dKB ring buffer on core 0\n",
            WCACHE_SLOTS * SECTOR_SIZE / 1024);
}

static void wcache_enqueue(BlockDeviceFile *bf, uint64_t disk_sector,
                           uint32_t sd_sector, const uint8_t *buf, int count)
{
    for (int i = 0; i < count; i++) {
        /* Backpressure: spin if ring full */
        while (wcache.head - __atomic_load_n(&wcache.tail, __ATOMIC_ACQUIRE) >= WCACHE_SLOTS)
            vTaskDelay(1);

        uint32_t seq = wcache.head;
        uint32_t slot = seq % WCACHE_SLOTS;
        memcpy(wcache.data + slot * SECTOR_SIZE, buf + i * SECTOR_SIZE, SECTOR_SIZE);
        wcache.meta[slot].bf = bf;
        wcache.meta[slot].disk_sector = disk_sector + i;
        wcache.meta[slot].sd_sector = sd_sector + i;
        wcache_index_update(bf, disk_sector + i, seq);
        __atomic_store_n(&wcache.head, seq + 1, __ATOMIC_RELEASE);
    }
    xSemaphoreGive(wcache.not_empty);
}

/* Read-through: check if a sector is pending in the write cache.
 * Fast path: index lookup + metadata validation.
 * Fallback: reverse scan head→tail so the newest write wins immediately. */
static int wcache_lookup(BlockDeviceFile *bf, uint64_t sector_num, uint8_t *buf)
{
    wcache.lookup_calls++;
    uint32_t t = __atomic_load_n(&wcache.tail, __ATOMIC_ACQUIRE);
    uint32_t h = __atomic_load_n(&wcache.head, __ATOMIC_ACQUIRE);

    /* Fast path: if write queue is empty, nothing to look up */
    if (t == h) {
        wcache.lookup_fallback_misses++;
        return 0;
    }

    if (wcache_lookup_index(bf, sector_num, buf, t, h)) {
        wcache.lookup_hits++;
        return 1;
    }

    for (uint32_t i = h; i > t; ) {
        i--;
        uint32_t slot = i % WCACHE_SLOTS;
        wcache.lookup_scan_steps++;
        if (wcache.meta[slot].bf == bf &&
            wcache.meta[slot].disk_sector == sector_num) {
            memcpy(buf, wcache.data + slot * SECTOR_SIZE, SECTOR_SIZE);
            wcache.lookup_hits++;
            wcache.lookup_fallback_hits++;
            return 1;
        }
    }

    wcache.lookup_fallback_misses++;
    return 0;
}

static void wcache_flush(void)
{
    if (!wcache.active)
        return;
    /* Wake the IO task in case it's sleeping */
    xSemaphoreGive(wcache.not_empty);
    /* Spin until drained */
    while (__atomic_load_n(&wcache.tail, __ATOMIC_ACQUIRE) !=
           __atomic_load_n(&wcache.head, __ATOMIC_ACQUIRE))
        vTaskDelay(1);
}

static void wcache_report_stats(void)
{
    if (!wcache.active)
        return;
    uint64_t calls = wcache.lookup_calls;
    if (calls == 0 || calls == wcache.lookup_reported_calls)
        return;

    double hit_pct = 100.0 * (double)wcache.lookup_hits / (double)calls;
    double idx_pct = 100.0 * (double)wcache.lookup_index_hits / (double)calls;
    uint64_t fallback_calls = calls - wcache.lookup_index_hits;
    double avg_scan = fallback_calls ?
        (double)wcache.lookup_scan_steps / (double)fallback_calls : 0.0;

    fprintf(stderr,
            "wcache_lookup: calls=%llu hits=%llu (%.1f%%) idx=%llu (%.1f%%) "
            "fallback_hits=%llu misses=%llu avg_scan=%.2f stale=%llu oor=%llu\n",
            (unsigned long long)calls,
            (unsigned long long)wcache.lookup_hits, hit_pct,
            (unsigned long long)wcache.lookup_index_hits, idx_pct,
            (unsigned long long)wcache.lookup_fallback_hits,
            (unsigned long long)wcache.lookup_fallback_misses,
            avg_scan,
            (unsigned long long)wcache.lookup_index_stale,
            (unsigned long long)wcache.lookup_index_out_of_range);

    wcache.lookup_reported_calls = calls;
}

/*
 * Read the next cluster number from the FAT via raw SD reads.
 * Handles FAT16 (2 bytes/entry) and FAT32/exFAT (4 bytes/entry).
 * Caches one FAT sector at a time to avoid re-reading for consecutive clusters.
 * Returns the next cluster, or 0 on read error.
 */
static DWORD fat_next_cluster(sdmmc_card_t *card, DWORD cluster,
                              int fs_type, LBA_t fat_base,
                              uint8_t *fat_buf, uint32_t *cached_fat_sector)
{
    uint32_t entry_size = (fs_type == FS_FAT16) ? 2 : 4;
    uint32_t fat_sector = fat_base + (cluster * entry_size) / SECTOR_SIZE;

    if (fat_sector != *cached_fat_sector) {
        if (sdmmc_read_sectors_retry(card, fat_buf, fat_sector, 1) != ESP_OK)
            return 0;  /* read error */
        *cached_fat_sector = fat_sector;
    }

    uint32_t byte_offset = (cluster * entry_size) % SECTOR_SIZE;
    if (fs_type == FS_FAT16) {
        uint16_t val;
        memcpy(&val, fat_buf + byte_offset, 2);
        return (val >= 0xFFF8) ? 0x0FFFFFFF : (DWORD)val;
    } else {
        uint32_t val;
        memcpy(&val, fat_buf + byte_offset, 4);
        /* FAT32: top 4 bits reserved, mask them off.
         * exFAT: full 32-bit entry, no mask needed. */
        return (fs_type == FS_FAT32) ? (val & 0x0FFFFFFF) : val;
    }
}

static int build_extent_map(BlockDeviceFile *bf)
{
    sdmmc_card_t *card = rawsd;
    if (!card)
        return -1;

    /* Convert VFS path "/sdcard/foo.img" to FatFS path "0:/foo.img" */
    const char *prefix = "/sdcard/";
    size_t pfxlen = strlen(prefix);
    if (strncmp(bf->path, prefix, pfxlen) != 0)
        return -1;  /* Not on SD card */

    char fatfs_path[256];
    snprintf(fatfs_path, sizeof(fatfs_path), "0:/%s", bf->path + pfxlen);

    /* Open via FatFS to get starting cluster and filesystem geometry.
     * FIL is heap-allocated because it contains a 4KB buf[FF_MAX_SS]
     * which would overflow the 8KB task stack. */
    FIL *fil = malloc(sizeof(FIL));
    if (!fil)
        return -1;
    FRESULT fres = f_open(fil, fatfs_path, FA_READ);
    if (fres != FR_OK) {
        fprintf(stderr, "extent_map: f_open(%s) failed: %d\n", fatfs_path, fres);
        free(fil);
        return -1;
    }

    DWORD start_cluster = fil->obj.sclust;
    FATFS *fs = fil->obj.fs;
    int fs_type = fs->fs_type;
    int sectors_per_cluster = fs->csize;
    LBA_t fat_base = fs->fatbase;
    LBA_t data_base = fs->database;
    DWORD n_fatent = fs->n_fatent;  /* number of clusters + 2 */
#if FF_FS_EXFAT
    int exfat_contiguous = (fs_type == FS_EXFAT) && (fil->obj.stat & 2);
    FSIZE_t file_obj_size = fil->obj.objsize;
#endif
    f_close(fil);
    free(fil);

    if (start_cluster < 2 || sectors_per_cluster == 0) {
        fprintf(stderr, "extent_map: invalid file (sclust=%lu spc=%d)\n",
                (unsigned long)start_cluster, sectors_per_cluster);
        return -1;
    }

    if (fs_type != FS_FAT16 && fs_type != FS_FAT32
#if FF_FS_EXFAT
        && fs_type != FS_EXFAT
#endif
    ) {
        fprintf(stderr, "extent_map: unsupported fs_type %d\n", fs_type);
        return -1;
    }

    const char *fs_name = (fs_type == FS_FAT16) ? "FAT16" :
                          (fs_type == FS_FAT32) ? "FAT32" : "exFAT";

    /* Calculate total clusters in file (including start_offset sectors) */
    int64_t total_file_bytes = (int64_t)bf->nb_sectors * SECTOR_SIZE + bf->start_offset;
    uint32_t total_clusters = (total_file_bytes + (int64_t)sectors_per_cluster * SECTOR_SIZE - 1)
                              / ((int64_t)sectors_per_cluster * SECTOR_SIZE);

#if FF_FS_EXFAT
    /* exFAT contiguous files: no FAT walk needed — one extent from sclust */
    if (exfat_contiguous) {
        DiskExtent *extents = malloc(sizeof(DiskExtent));
        if (!extents)
            return -1;
        extents[0].file_sector = 0;
        extents[0].sd_sector = data_base + (start_cluster - 2) * sectors_per_cluster;
        extents[0].count = total_clusters * sectors_per_cluster;
        bf->card = card;
        bf->extents = extents;
        bf->num_extents = 1;
        fprintf(stderr, "extent_map: %s → 1 extent (exFAT contiguous), %u clusters, spc=%d\n",
                bf->path, total_clusters, sectors_per_cluster);
        return 0;
    }
#endif

    /* Walk the FAT chain via raw SD reads.
     * Cache one FAT sector at a time to minimize I/O. */
    uint8_t *fat_buf = malloc(SECTOR_SIZE);
    if (!fat_buf)
        return -1;
    uint32_t cached_fat_sector = (uint32_t)-1;

    /* First pass: count extents (contiguous cluster runs) */
    int num_extents = 0;
    DWORD cluster = start_cluster;
    DWORD prev_cluster = 0;
    uint32_t clusters_walked = 0;

    while (cluster >= 2 && cluster < n_fatent && clusters_walked < total_clusters) {
        if (prev_cluster == 0 || cluster != prev_cluster + 1)
            num_extents++;
        prev_cluster = cluster;
        clusters_walked++;

        cluster = fat_next_cluster(card, cluster, fs_type, fat_base,
                                   fat_buf, &cached_fat_sector);
        if (cluster == 0) {  /* read error */
            fprintf(stderr, "extent_map: FAT read error at cluster %lu\n",
                    (unsigned long)prev_cluster);
            free(fat_buf);
            return -1;
        }
    }

    if (num_extents == 0) {
        free(fat_buf);
        return -1;
    }

    /* Allocate extent array */
    DiskExtent *extents = malloc(num_extents * sizeof(DiskExtent));
    if (!extents) {
        free(fat_buf);
        return -1;
    }

    /* Second pass: fill extent array */
    cluster = start_cluster;
    cached_fat_sector = (uint32_t)-1;
    int ext_idx = -1;
    uint32_t file_sector = 0;
    clusters_walked = 0;

    while (cluster >= 2 && cluster < n_fatent && clusters_walked < total_clusters) {
        uint32_t sd_sector = data_base + (cluster - 2) * sectors_per_cluster;

        if (ext_idx < 0 ||
            sd_sector != extents[ext_idx].sd_sector + extents[ext_idx].count) {
            /* Start new extent */
            ext_idx++;
            extents[ext_idx].file_sector = file_sector;
            extents[ext_idx].sd_sector = sd_sector;
            extents[ext_idx].count = sectors_per_cluster;
        } else {
            /* Extend current extent */
            extents[ext_idx].count += sectors_per_cluster;
        }
        file_sector += sectors_per_cluster;
        clusters_walked++;

        cluster = fat_next_cluster(card, cluster, fs_type, fat_base,
                                   fat_buf, &cached_fat_sector);
        if (cluster == 0) {
            free(fat_buf);
            free(extents);
            return -1;
        }
    }

    free(fat_buf);

    bf->card = card;
    bf->extents = extents;
    bf->num_extents = ext_idx + 1;

    fprintf(stderr, "extent_map: %s → %d extents, %u clusters, spc=%d (%s)\n",
            bf->path, bf->num_extents, clusters_walked, sectors_per_cluster, fs_name);

    return 0;
}

/*
 * Translate a file-relative sector number to an SD card sector
 * using the extent map. Returns the SD sector, or (uint32_t)-1 on miss.
 * Also returns the number of contiguous sectors available via *contig.
 */
static uint32_t extent_lookup(BlockDeviceFile *bf, uint32_t file_sector, int *contig)
{
    /* Binary search through sorted extents */
    int lo = 0, hi = bf->num_extents - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        DiskExtent *e = &bf->extents[mid];
        if (file_sector < e->file_sector) {
            hi = mid - 1;
        } else if (file_sector >= e->file_sector + e->count) {
            lo = mid + 1;
        } else {
            uint32_t offset = file_sector - e->file_sector;
            *contig = e->count - offset;
            return e->sd_sector + offset;
        }
    }
    *contig = 0;
    return (uint32_t)-1;
}

//#define DUMP_BLOCK_READ

static int bf_read_async(BlockDevice *bs,
                         uint64_t sector_num, uint8_t *buf, int n,
                         BlockDeviceCompletionFunc *cb, void *opaque)
{
    BlockDeviceFile *bf = bs->opaque;
    //    printf("bf_read_async: sector_num=%" PRId64 " n=%d\n", sector_num, n);

    /* Bounds checking: return zeros for sectors beyond disk size */
    if (sector_num >= (uint64_t)bf->nb_sectors) {
        memset(buf, 0, n * SECTOR_SIZE);
        return 0;
    }
    /* Clamp read count to not exceed disk size */
    if (sector_num + n > (uint64_t)bf->nb_sectors) {
        int valid_sectors = bf->nb_sectors - sector_num;
        /* Zero the out-of-bounds portion */
        memset(buf + valid_sectors * SECTOR_SIZE, 0, (n - valid_sectors) * SECTOR_SIZE);
        n = valid_sectors;
    }

    /* Raw SD path: bypass VFS/FatFS entirely */
    if (bf->extents) {
        if (bf->cache_buf) {
            /* Phase 1: read-ahead cache for RO raw SD (CD-ROMs) */
            uint32_t file_sector = (uint32_t)((sector_num * SECTOR_SIZE + bf->start_offset) / SECTOR_SIZE);
            while (n > 0) {
                /* Check if sector is in cache */
                if (file_sector >= (uint32_t)bf->cache_start &&
                    file_sector < (uint32_t)(bf->cache_start + bf->cache_count)) {
                    int cache_offset = (file_sector - (uint32_t)bf->cache_start) * SECTOR_SIZE;
                    memcpy(buf, bf->cache_buf + cache_offset, SECTOR_SIZE);
                    buf += SECTOR_SIZE;
                    file_sector++;
                    sector_num++;
                    n--;
                } else {
                    /* Cache miss — read ahead DISK_CACHE_SECTORS from SD */
                    int contig;
                    uint32_t sd_sector = extent_lookup(bf, file_sector, &contig);
                    if (sd_sector == (uint32_t)-1) {
                        memset(buf, 0, n * SECTOR_SIZE);
                        break;
                    }
                    int read_count = DISK_CACHE_SECTORS;
                    if (read_count > contig) read_count = contig;
                    esp_err_t err = sdmmc_read_sectors_retry(bf->card, bf->cache_buf,
                                                             sd_sector, read_count);
                    if (err != ESP_OK)
                        return -1;
                    bf->cache_start = file_sector;
                    bf->cache_count = read_count;
                    /* Loop will hit cache on next iteration */
                }
            }
            return 0;
        }
        /* RW raw SD path — write-cache read-through + async */
        {
            /* First check if any sectors are in the write cache */
            int all_cached = 1;
            if (wcache.active) {
                uint64_t sn = sector_num;
                uint8_t *b = buf;
                int remaining = n;
                while (remaining > 0) {
                    if (!wcache_lookup(bf, sn, b)) {
                        all_cached = 0;
                        break;
                    }
                    b += SECTOR_SIZE;
                    sn++;
                    remaining--;
                }
                if (all_cached)
                    return 0;
            }

            /* Phase 3: async read — post to IO task if callback available */
            if (wcache.active && cb &&
                !__atomic_load_n(&async_read.pending, __ATOMIC_ACQUIRE) &&
                !__atomic_load_n(&async_read.done, __ATOMIC_ACQUIRE)) {
                async_read.bf = bf;
                async_read.buf = buf;
                async_read.sector_num = sector_num;
                async_read.count = n;
                async_read.start_offset = bf->start_offset;
                async_read.cb = cb;
                async_read.cb_opaque = opaque;
                __atomic_store_n(&async_read.pending, 1, __ATOMIC_RELEASE);
                xSemaphoreGive(wcache.not_empty);
                return 1;  /* async — caller sets BUSY_STAT */
            }

            /* Synchronous fallback */
            uint32_t file_sector = (uint32_t)((sector_num * SECTOR_SIZE + bf->start_offset) / SECTOR_SIZE);
            while (n > 0) {
                if (wcache.active && wcache_lookup(bf, sector_num, buf)) {
                    buf += SECTOR_SIZE;
                    file_sector++;
                    sector_num++;
                    n--;
                    continue;
                }
                int contig;
                uint32_t sd_sector = extent_lookup(bf, file_sector, &contig);
                if (sd_sector == (uint32_t)-1) {
                    memset(buf, 0, n * SECTOR_SIZE);
                    break;
                }
                int chunk = (n < contig) ? n : contig;
                esp_err_t err = sdmmc_read_sectors_retry(bf->card, buf, sd_sector, chunk);
                if (err != ESP_OK)
                    return -1;
                buf += chunk * SECTOR_SIZE;
                file_sector += chunk;
                sector_num += chunk;
                n -= chunk;
            }
            return 0;
        }
    }

    if (!bf->f)
        return -1;

    if (bf->mode == BF_MODE_SNAPSHOT) {
        int i;
        for(i = 0; i < n; i++) {
            if (!bf->sector_table[sector_num]) {
                fseeko(bf->f, sector_num * SECTOR_SIZE, SEEK_SET);
                fread(buf, 1, SECTOR_SIZE, bf->f);
            } else {
                memcpy(buf, bf->sector_table[sector_num], SECTOR_SIZE);
            }
            sector_num++;
            buf += SECTOR_SIZE;
        }
    } else {
        /* Use read-ahead cache for faster sequential access */
        if (bf->cache_buf) {
            while (n > 0) {
                /* Check if sector is in cache */
                if (sector_num >= bf->cache_start &&
                    sector_num < bf->cache_start + bf->cache_count) {
                    /* Cache hit - copy from cache */
                    int cache_offset = (sector_num - bf->cache_start) * SECTOR_SIZE;
                    memcpy(buf, bf->cache_buf + cache_offset, SECTOR_SIZE);
                    sector_num++;
                    buf += SECTOR_SIZE;
                    n--;
                } else {
                    /* Cache miss - read ahead into cache */
                    int64_t read_start = sector_num;
                    int read_count = DISK_CACHE_SECTORS;
                    /* Don't read past end of disk */
                    if (read_start + read_count > bf->nb_sectors)
                        read_count = bf->nb_sectors - read_start;
                    if (read_count > 0) {
                        fseeko(bf->f, bf->start_offset + read_start * SECTOR_SIZE, SEEK_SET);
                        fread(bf->cache_buf, 1, read_count * SECTOR_SIZE, bf->f);
                        bf->cache_start = read_start;
                        bf->cache_count = read_count;
                    } else {
                        /* Past end of disk */
                        break;
                    }
                }
            }
        } else
        {
            fseeko(bf->f, bf->start_offset + sector_num * SECTOR_SIZE, SEEK_SET);
            fread(buf, 1, n * SECTOR_SIZE, bf->f);
        }
    }
    /* synchronous read */
    return 0;
}

static int bf_write_async(BlockDevice *bs,
                          uint64_t sector_num, const uint8_t *buf, int n,
                          BlockDeviceCompletionFunc *cb, void *opaque)
{
    BlockDeviceFile *bf = bs->opaque;
    int ret;

    /* Raw SD path: bypass VFS/FatFS entirely */
    if (bf->extents) {
        if (bf->mode == BF_MODE_RO)
            return -1;
        uint32_t file_sector = (uint32_t)((sector_num * SECTOR_SIZE + bf->start_offset) / SECTOR_SIZE);
        if (wcache.active) {
            /* Write-behind: enqueue to ring buffer, IO task drains async */
            while (n > 0) {
                int contig;
                uint32_t sd_sector = extent_lookup(bf, file_sector, &contig);
                if (sd_sector == (uint32_t)-1)
                    return -1;
                int chunk = (n < contig) ? n : contig;
                wcache_enqueue(bf, sector_num, sd_sector, buf, chunk);
                buf += chunk * SECTOR_SIZE;
                file_sector += chunk;
                sector_num += chunk;
                n -= chunk;
            }
        } else {
            /* Fallback: direct synchronous write */
            while (n > 0) {
                int contig;
                uint32_t sd_sector = extent_lookup(bf, file_sector, &contig);
                if (sd_sector == (uint32_t)-1)
                    return -1;
                int chunk = (n < contig) ? n : contig;
                if (sdmmc_write_sectors_retry(bf->card, buf, sd_sector, chunk) != ESP_OK)
                    return -1;
                buf += chunk * SECTOR_SIZE;
                file_sector += chunk;
                n -= chunk;
            }
        }
        return 0;
    }

    /* Invalidate cache on write - simple approach, just clear it */
    if (bf->cache_buf) {
        bf->cache_count = 0;
    }

    switch(bf->mode) {
    case BF_MODE_RO:
        ret = -1; /* error */
        break;
    case BF_MODE_RW:
        fseeko(bf->f, bf->start_offset + sector_num * SECTOR_SIZE, SEEK_SET);
        fwrite(buf, 1, n * SECTOR_SIZE, bf->f);
        bf->dirty = 1;
        ret = 0;
        break;
    case BF_MODE_SNAPSHOT:
        {
            int i;
            if ((sector_num + n) > bf->nb_sectors)
                return -1;
            for(i = 0; i < n; i++) {
                if (!bf->sector_table[sector_num]) {
                    bf->sector_table[sector_num] = pcmalloc(SECTOR_SIZE);
                }
                memcpy(bf->sector_table[sector_num], buf, SECTOR_SIZE);
                sector_num++;
                buf += SECTOR_SIZE;
            }
            ret = 0;
        }
        break;
    default:
        abort();
    }

    return ret;
}


static BlockDevice *block_device_init(const char *filename,
                                      BlockDeviceModeEnum mode)
{
    BlockDevice *bs;
    BlockDeviceFile *bf;
    int64_t file_size;
    FILE *f;
    const char *mode_str;

    if (mode == BF_MODE_RW) {
        mode_str = "r+b";
    } else {
        mode_str = "rb";
    }
    
    f = fopen(filename, mode_str);
    if (!f) {
        perror(filename);
        return NULL;
    }
    char buf[8];
    int start_offset = 0;
    if (mode == BF_MODE_RO || mode == BF_MODE_RW) {
        fread(buf, 1, 8, f);
        if (memcmp(buf, ide_magic, 8) == 0)
            start_offset = 1024;
    }
    fseeko(f, 0, SEEK_END);
    /* ftello returns off_t which is 32-bit signed on ESP32-P4.
       Files >= 2GB overflow to negative. Cast through uint32_t
       to get the correct unsigned size (FAT32 max is 4GB-1). */
    {
        off_t raw = ftello(f);
        file_size = (raw >= 0) ? (int64_t)raw : (int64_t)(uint32_t)raw;
    }
    file_size -= start_offset;

    /* off_t is 32-bit signed on ESP32-P4, so fseeko can only address
       up to 2GB-1 bytes.  Cap file_size to avoid seek overflow. */
    if (file_size > (int64_t)0x7FFFFFFF) {
        fprintf(stderr, "Warning: %s is %lldMB, capping to 2047MB (off_t limit)\n",
                filename, (long long)(file_size / (1024 * 1024)));
        file_size = (int64_t)0x7FFFFE00;  /* 2GB - 512, aligned to sector */
    }

    bs = pcmalloc(sizeof(*bs));
    bf = pcmalloc(sizeof(*bf));
    memset(bs, 0, sizeof(*bs));
    memset(bf, 0, sizeof(*bf));

    bf->mode = mode;
    bf->nb_sectors = file_size / 512;
    bf->f = f;
    bf->start_offset = start_offset;
    if (mode == BF_MODE_RW)
        setvbuf(f, NULL, _IOFBF, 32768);
    strncpy(bf->path, filename, sizeof(bf->path) - 1);
    bf->path[sizeof(bf->path) - 1] = '\0';

    if (mode == BF_MODE_SNAPSHOT) {
        bf->sector_table = pcmalloc(sizeof(bf->sector_table[0]) *
                                    bf->nb_sectors);
        memset(bf->sector_table, 0,
               sizeof(bf->sector_table[0]) * bf->nb_sectors);
    }

    bs->opaque = bf;
    bs->get_sector_count = bf_get_sector_count;
    bs->get_chs = NULL;
    if (start_offset) {
        fseeko(f, 512, SEEK_SET);
        unsigned char buf[7 * 2];
        fread(buf, 1, 7 * 2, f);
        bf->cylinders = buf[2] | (buf[3] << 8);
        bf->heads = buf[6] | (buf[7] << 8);
        bf->sectors = buf[12] | (buf[13] << 8);
        bs->get_chs = bf_get_chs;
        fseeko(f, 0, SEEK_END);
    } else if (mode == BF_MODE_RW || mode == BF_MODE_RO) {
        /* Auto-detect CHS geometry from MBR partition table */
        unsigned char mbr[512];
        fseeko(f, 0, SEEK_SET);
        if (fread(mbr, 1, 512, f) == 512 &&
            mbr[510] == 0x55 && mbr[511] == 0xAA) {
            static const int pe_offsets[4] = {0x1BE, 0x1CE, 0x1DE, 0x1EE};
            for (int i = 0; i < 4; i++) {
                const unsigned char *pe = mbr + pe_offsets[i];
                if (pe[4] == 0) continue; /* empty partition */
                int end_head = pe[5];
                int end_sec = pe[6] & 0x3F;
                int start_head = pe[1];
                int start_sec = pe[2] & 0x3F;
                int start_cyl = pe[3] | ((pe[2] & 0xC0) << 2);
                uint32_t lba_start = pe[8] | (pe[9] << 8) |
                                     (pe[10] << 16) | (pe[11] << 24);
                int spt = end_sec;
                int heads = end_head + 1;
                if (spt < 1 || spt > 63 || heads < 1 || heads > 255)
                    continue;
                if (start_sec < 1 || start_sec > spt)
                    continue;
                uint32_t calc_lba = (uint32_t)start_cyl * heads * spt +
                                    (uint32_t)start_head * spt +
                                    (start_sec - 1);
                if (calc_lba == lba_start) {
                    bf->sectors = spt;
                    bf->heads = heads;
                    bf->cylinders = bf->nb_sectors / (heads * spt);
                    if (bf->cylinders > 16383) bf->cylinders = 16383;
                    else if (bf->cylinders < 2) bf->cylinders = 2;
                    bs->get_chs = bf_get_chs;
                    break;
                }
            }
        }
    }
    bs->read_async = bf_read_async;
    bs->write_async = bf_write_async;

    /* Try raw SD acceleration for /sdcard/ files (non-snapshot mode).
     * On success: close stdio FILE, skip cache — all I/O goes direct to SD.
     * On failure: fall back to stdio + read-ahead cache. */
    if (mode != BF_MODE_SNAPSHOT && build_extent_map(bf) == 0) {
        /* Raw SD active — close stdio handle */
        fclose(f);
        bf->f = NULL;
        /* Remove 2GB cap — raw SD reads use 32-bit sector numbers directly,
         * no off_t limitation. Recalculate from original file size.
         * FIL heap-allocated to avoid stack overflow (4KB buf inside). */
        {
            FIL *fil2 = malloc(sizeof(FIL));
            if (fil2) {
                char fatfs_path[256];
                snprintf(fatfs_path, sizeof(fatfs_path), "0:/%s", filename + strlen("/sdcard/"));
                if (f_open(fil2, fatfs_path, FA_READ) == FR_OK) {
                    int64_t real_size = (int64_t)f_size(fil2) - start_offset;
                    f_close(fil2);
                    if (real_size > 0) {
                        bf->nb_sectors = real_size / SECTOR_SIZE;
                        fprintf(stderr, "extent_map: full size %lldMB (off_t cap bypassed)\n",
                                (long long)(real_size / (1024 * 1024)));
                    }
                }
                free(fil2);
            }
        }
        /* Initialize write-behind cache infrastructure (idempotent) */
        wcache_init();
        /* Allocate read-ahead cache for RO raw SD (CD-ROMs) */
        if (mode == BF_MODE_RO) {
            bf->cache_buf = malloc(DISK_CACHE_SECTORS * SECTOR_SIZE);
            bf->cache_start = -1;
            bf->cache_count = 0;
        }
    } else {
        /* Fallback: allocate read-ahead cache for stdio path */
        fprintf(stderr, "disk: %s using stdio (mode=%d, no raw SD)\n",
                filename, mode);
        bf->cache_buf = malloc(DISK_CACHE_SECTORS * SECTOR_SIZE);
        bf->cache_start = -1;
        bf->cache_count = 0;
    }

    return bs;
}

/**
 * Create an empty read-only BlockDevice (for CD-ROM without disc)
 * Allocates structures from pcram but cache from heap to save pcram space.
 */
static BlockDevice *block_device_init_empty_ro(void)
{
    BlockDevice *bs;
    BlockDeviceFile *bf;

    bs = pcmalloc(sizeof(*bs));
    bf = pcmalloc(sizeof(*bf));
    memset(bs, 0, sizeof(*bs));
    memset(bf, 0, sizeof(*bf));

    bf->mode = BF_MODE_RO;
    bf->nb_sectors = 0;
    bf->f = NULL;
    bf->start_offset = 0;
    bf->path[0] = '\0';
    /* Allocate cache from heap (not pcram) since CD is read-only and
     * heap/PSRAM is abundant on ESP32-P4 */
    bf->cache_buf = malloc(DISK_CACHE_SECTORS * SECTOR_SIZE);
    bf->cache_start = -1;
    bf->cache_count = 0;

    bs->opaque = bf;
    bs->get_sector_count = bf_get_sector_count;
    bs->get_chs = NULL;
    bs->read_async = bf_read_async;
    bs->write_async = bf_write_async;
    return bs;
}

/* Create empty RW block device for HDD hot-mounting */
static BlockDevice *block_device_init_empty_rw(void)
{
    BlockDevice *bs;
    BlockDeviceFile *bf;

    bs = pcmalloc(sizeof(*bs));
    bf = pcmalloc(sizeof(*bf));
    memset(bs, 0, sizeof(*bs));
    memset(bf, 0, sizeof(*bf));

    bf->mode = BF_MODE_RW;
    bf->nb_sectors = 0;
    bf->f = NULL;
    bf->start_offset = 0;
    bf->path[0] = '\0';
    bf->cache_buf = malloc(DISK_CACHE_SECTORS * SECTOR_SIZE);
    bf->cache_start = -1;
    bf->cache_count = 0;

    bs->opaque = bf;
    bs->get_sector_count = bf_get_sector_count;
    bs->get_chs = bf_get_chs;  /* HDD needs CHS geometry */
    bs->read_async = bf_read_async;
    bs->write_async = bf_write_async;
    return bs;
}

// === Sector Cache and Prefetch for ESP32 ===
// Cache recently read sectors to avoid repeated SD card access
// Prefetch ahead when sequential reads are detected

#define CACHE_SIZE 128          // Number of sectors to cache (64KB)
#define PREFETCH_SIZE 16        // Sectors to prefetch on sequential read
#define CACHE_LINE_SECTORS 8    // Read this many sectors at once

typedef struct {
    uint64_t sector_num;        // Sector number (-1 = invalid)
    uint8_t data[512];          // Sector data
    uint8_t valid;              // Entry is valid
    uint32_t access_count;      // LRU counter
} CacheEntry;

typedef struct {
    CacheEntry entries[CACHE_SIZE];
    uint32_t access_counter;    // Global access counter for LRU
    uint64_t last_sector;       // Last sector read (for sequential detection)
    uint32_t hits;              // Cache hit counter
    uint32_t misses;            // Cache miss counter
} SectorCache;

static SectorCache *sector_cache = NULL;

/* Reset stale pcram pointer so cache_init() will reinitialize.
 * Called during INI switch after pcram pool is zeroed. */
void ide_reset_statics(void)
{
    sector_cache = NULL;
}

static void cache_init(void)
{
    if (sector_cache) return;
    sector_cache = pcmalloc(sizeof(SectorCache));
    memset(sector_cache, 0, sizeof(SectorCache));
    for (int i = 0; i < CACHE_SIZE; i++) {
        sector_cache->entries[i].sector_num = (uint64_t)-1;
        sector_cache->entries[i].valid = 0;
    }
    sector_cache->last_sector = (uint64_t)-1;
#ifdef DEBUG_IDE
    printf("IDE cache initialized: %d sectors (%d KB)\n", CACHE_SIZE, CACHE_SIZE / 2);
#endif
}

static int cache_find(uint64_t sector_num)
{
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (sector_cache->entries[i].valid &&
            sector_cache->entries[i].sector_num == sector_num) {
            return i;
        }
    }
    return -1;
}

static int cache_find_lru(void)
{
    int lru_idx = 0;
    uint32_t lru_count = sector_cache->entries[0].access_count;
    for (int i = 1; i < CACHE_SIZE; i++) {
        if (!sector_cache->entries[i].valid) {
            return i;  // Use empty slot first
        }
        if (sector_cache->entries[i].access_count < lru_count) {
            lru_count = sector_cache->entries[i].access_count;
            lru_idx = i;
        }
    }
    return lru_idx;
}

static void cache_store(uint64_t sector_num, const uint8_t *data)
{
    int idx = cache_find(sector_num);
    if (idx < 0) {
        idx = cache_find_lru();
    }
    sector_cache->entries[idx].sector_num = sector_num;
    memcpy(sector_cache->entries[idx].data, data, 512);
    sector_cache->entries[idx].valid = 1;
    sector_cache->entries[idx].access_count = ++sector_cache->access_counter;
}

static void cache_invalidate(uint64_t sector_num)
{
    int idx = cache_find(sector_num);
    if (idx >= 0) {
        sector_cache->entries[idx].valid = 0;
    }
}

typedef struct BlockDeviceESPSD {
    sdmmc_card_t *card;
    int64_t start_sector;
    int64_t nb_sectors;
} BlockDeviceESPSD;

static int64_t espsd_get_sector_count(BlockDevice *bs)
{
    BlockDeviceESPSD *bf = bs->opaque;
    return bf->nb_sectors;
}

//#define DUMP_BLOCK_READ

static int espsd_read_async(BlockDevice *bs,
                         uint64_t sector_num, uint8_t *buf, int n,
                         BlockDeviceCompletionFunc *cb, void *opaque)
{
    BlockDeviceESPSD *bf = bs->opaque;
    if (!bf->card)
        return -1;

    // Initialize cache on first use
    if (!sector_cache) {
        cache_init();
    }

    esp_err_t ret;
    int sectors_from_cache = 0;
    int sectors_read = 0;

    // Check cache for each requested sector
    for (int i = 0; i < n; i++) {
        uint64_t sect = sector_num + i;
        int cache_idx = cache_find(sect);
        if (cache_idx >= 0) {
            // Cache hit
            memcpy(buf + i * 512, sector_cache->entries[cache_idx].data, 512);
            sector_cache->entries[cache_idx].access_count = ++sector_cache->access_counter;
            sectors_from_cache++;
        } else {
            // Cache miss - need to read from SD
            // Read multiple sectors at once for efficiency
            int read_start = i;
            int read_count = 0;

            // Find contiguous cache misses
            while (i < n && cache_find(sector_num + i) < 0) {
                read_count++;
                i++;
            }
            i--;  // Back up since loop will increment

            // Prefetch: if sequential read detected, read extra sectors
            int prefetch = 0;
            if (sector_cache->last_sector != (uint64_t)-1 &&
                sector_num == sector_cache->last_sector + 1) {
                prefetch = PREFETCH_SIZE;
                // Don't prefetch beyond disk
                if (sector_num + read_count + prefetch > (uint64_t)bf->nb_sectors) {
                    prefetch = bf->nb_sectors - sector_num - read_count;
                    if (prefetch < 0) prefetch = 0;
                }
            }

            // Allocate temp buffer if prefetching
            int total_read = read_count + prefetch;
            uint8_t *read_buf;
            uint8_t *temp_buf = NULL;

            if (prefetch > 0) {
                temp_buf = malloc(total_read * 512);
                if (temp_buf) {
                    read_buf = temp_buf;
                } else {
                    // Fallback to no prefetch
                    total_read = read_count;
                    read_buf = buf + read_start * 512;
                }
            } else {
                read_buf = buf + read_start * 512;
            }

            // Read from SD card (under sd_mutex if wcache active)
            ret = sdmmc_read_sectors_retry(bf->card, read_buf,
                                           bf->start_sector + sector_num + read_start,
                                           total_read);
            if (ret != 0) {
                if (temp_buf) free(temp_buf);
                return -1;
            }

            // Copy requested sectors to output buffer
            if (temp_buf) {
                memcpy(buf + read_start * 512, temp_buf, read_count * 512);
            }

            // Store all read sectors in cache
            for (int j = 0; j < total_read; j++) {
                cache_store(sector_num + read_start + j, read_buf + j * 512);
            }

            if (temp_buf) free(temp_buf);
            sectors_read += read_count;
        }
    }

    // Update last sector for sequential detection
    sector_cache->last_sector = sector_num + n - 1;

    // Update stats
    sector_cache->hits += sectors_from_cache;
    sector_cache->misses += sectors_read;

#ifdef DEBUG_IDE
    static int log_count = 0;
    if (++log_count % 100 == 0) {
        printf("IDE cache: hits=%lu misses=%lu ratio=%.1f%%\n",
               (unsigned long)sector_cache->hits, (unsigned long)sector_cache->misses,
               sector_cache->hits * 100.0 / (sector_cache->hits + sector_cache->misses + 1));
    }
#endif

    return 0;
}

static int espsd_write_async(BlockDevice *bs,
                          uint64_t sector_num, const uint8_t *buf, int n,
                          BlockDeviceCompletionFunc *cb, void *opaque)
{
    BlockDeviceESPSD *bf = bs->opaque;
    if (!bf->card)
        return -1;

    // Invalidate cache entries for written sectors
    if (sector_cache) {
        for (int i = 0; i < n; i++) {
            cache_invalidate(sector_num + i);
        }
    }

    esp_err_t ret;
    ret = sdmmc_write_sectors_retry(bf->card, buf, bf->start_sector + sector_num, n);
    if (ret != 0)
        return -1;
    return 0;
}

static BlockDevice *block_device_init_espsd(int64_t start_sector, int64_t nb_sectors)
{
    sdmmc_card_t *card = rawsd;
    assert(card);
    assert(card->csd.sector_size == 512);
    BlockDevice *bs;
    BlockDeviceESPSD *bf;

    bs = pcmalloc(sizeof(*bs));
    bf = pcmalloc(sizeof(*bf));
    memset(bs, 0, sizeof(*bs));
    memset(bf, 0, sizeof(*bf));
    bf->card = card;
    bf->start_sector = start_sector;
    if (nb_sectors == -1) {
        bf->nb_sectors = card->csd.capacity;
    } else {
        bf->nb_sectors = nb_sectors;
    }
    bs->opaque = bf;
    bs->get_sector_count = espsd_get_sector_count;
    bs->get_chs = NULL;
    bs->read_async = espsd_read_async;
    bs->write_async = espsd_write_async;
    return bs;
}

/*
 * USB Mass Storage BlockDevice
 * Returns 0 sectors when no USB device is connected.
 * Reads return zeros and writes are silently discarded when disconnected.
 */

/*
 * USB MSC block device — all transfers bounce through internal SRAM.
 *
 * The IDE io_buffer lives in PSRAM (via pcmalloc). On ESP32-P4 the USB DMA
 * controller may bypass the CPU data cache, so reading/writing PSRAM directly
 * from USB causes silent corruption. We keep a small bounce buffer in
 * internal SRAM and do one-sector-at-a-time transfers through it.
 */
#define USB_BOUNCE_SECTORS 4  /* 2KB — small enough to always fit in SRAM */

typedef struct BlockDeviceUSB {
    uint8_t *bounce;  /* Internal SRAM bounce buffer — MUST NOT be NULL */
} BlockDeviceUSB;

static BlockDeviceUSB *usb_bf = NULL;

static int64_t usbmsc_get_sector_count(BlockDevice *bs)
{
    (void)bs;
    return usb_host_msc_get_sector_count();
}

static int usbmsc_read_async(BlockDevice *bs,
                              uint64_t sector_num, uint8_t *buf, int n,
                              BlockDeviceCompletionFunc *cb, void *opaque)
{
    (void)cb;
    (void)opaque;
    IDE_ACTIVITY();

    BlockDeviceUSB *bf = bs->opaque;

    if (!usb_host_msc_connected()) {
        memset(buf, 0, n * SECTOR_SIZE);
        return 0;
    }

    int64_t nb_sectors = usb_host_msc_get_sector_count();
    if ((int64_t)sector_num >= nb_sectors) {
        memset(buf, 0, n * SECTOR_SIZE);
        return 0;
    }
    if ((int64_t)(sector_num + n) > nb_sectors) {
        int valid = nb_sectors - sector_num;
        memset(buf + valid * SECTOR_SIZE, 0, (n - valid) * SECTOR_SIZE);
        n = valid;
    }

    /* Read through SRAM bounce buffer, USB_BOUNCE_SECTORS at a time */
    while (n > 0) {
        int chunk = (n > USB_BOUNCE_SECTORS) ? USB_BOUNCE_SECTORS : n;
        int ret = usb_host_msc_read(sector_num, bf->bounce, chunk);
        if (ret != 0)
            return ret;
        memcpy(buf, bf->bounce, chunk * SECTOR_SIZE);
        buf += chunk * SECTOR_SIZE;
        sector_num += chunk;
        n -= chunk;
    }
    return 0;
}

static int usbmsc_write_async(BlockDevice *bs,
                               uint64_t sector_num, const uint8_t *buf, int n,
                               BlockDeviceCompletionFunc *cb, void *opaque)
{
    (void)cb;
    (void)opaque;
    IDE_ACTIVITY();

    BlockDeviceUSB *bf = bs->opaque;

    if (!usb_host_msc_connected())
        return 0;

    /* Write through SRAM bounce buffer */
    while (n > 0) {
        int chunk = (n > USB_BOUNCE_SECTORS) ? USB_BOUNCE_SECTORS : n;
        memcpy(bf->bounce, buf, chunk * SECTOR_SIZE);
        int ret = usb_host_msc_write(sector_num, bf->bounce, chunk);
        if (ret != 0)
            return ret;
        buf += chunk * SECTOR_SIZE;
        sector_num += chunk;
        n -= chunk;
    }
    return 0;
}

/* Read CHS geometry from FAT BPB on USB drive.
 * Uses the SRAM bounce buffer to avoid PSRAM DMA issues. */
static int usbmsc_get_chs(BlockDevice *bs, int *cyls, int *heads, int *secs)
{
    BlockDeviceUSB *bf = bs->opaque;
    uint8_t *bpb = bf->bounce;  /* Use SRAM bounce buffer */

    if (!usb_host_msc_connected())
        return 0;

    if (usb_host_msc_read(0, bpb, 1) != 0)
        return 0;

    /* Check for MBR signature */
    if (bpb[510] == 0x55 && bpb[511] == 0xAA && bpb[0] != 0xEB && bpb[0] != 0xE9) {
        uint32_t part_start = bpb[446+8] | (bpb[446+9]<<8) | (bpb[446+10]<<16) | (bpb[446+11]<<24);
        if (part_start > 0 && usb_host_msc_read(part_start, bpb, 1) != 0)
            return 0;
    }

    if ((bpb[0] != 0xEB && bpb[0] != 0xE9) || bpb[510] != 0x55 || bpb[511] != 0xAA)
        return 0;

    int bpb_secs = bpb[0x18] | (bpb[0x19] << 8);
    int bpb_heads = bpb[0x1A] | (bpb[0x1B] << 8);

    if (bpb_secs > 0 && bpb_secs <= 63 && bpb_heads > 0 && bpb_heads <= 255) {
        *secs = bpb_secs;
        *heads = bpb_heads;
        uint64_t total = usb_host_msc_get_sector_count();
        int c = total / (bpb_heads * bpb_secs);
        if (c > 16383) c = 16383;
        if (c < 2) c = 2;
        *cyls = c;
    }
    return 0;
}

static BlockDevice *block_device_init_usb(void)
{
    BlockDevice *bs;
    BlockDeviceUSB *bf;

    bs = pcmalloc(sizeof(*bs));
    bf = pcmalloc(sizeof(*bf));
    memset(bs, 0, sizeof(*bs));
    memset(bf, 0, sizeof(*bf));

    /* Bounce buffer MUST be in internal SRAM — USB DMA vs CPU cache
     * coherency on ESP32-P4 corrupts data in PSRAM. */
    bf->bounce = heap_caps_malloc(USB_BOUNCE_SECTORS * SECTOR_SIZE,
                                  MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!bf->bounce) {
        fprintf(stderr, "USB IDE: FATAL — cannot allocate %d bytes of internal SRAM for bounce buffer\n",
                USB_BOUNCE_SECTORS * SECTOR_SIZE);
        return NULL;
    }
    fprintf(stderr, "USB IDE: bounce buffer %d bytes at %p (internal SRAM)\n",
            USB_BOUNCE_SECTORS * SECTOR_SIZE, bf->bounce);
    usb_bf = bf;

    bs->opaque = bf;
    bs->get_sector_count = usbmsc_get_sector_count;
    bs->get_chs = usbmsc_get_chs;
    bs->read_async = usbmsc_read_async;
    bs->write_async = usbmsc_write_async;
    return bs;
}

IDEIFState *ide_allocate(int irq, void *pic, void (*set_irq)(void *pic, int irq, int level))
{
    IDEIFState *s;
    
    s = pcmalloc(sizeof(IDEIFState));
    memset(s, 0, sizeof(*s));

    s->irq = irq;
    s->pic = pic;
    s->set_irq = set_irq;

    s->cur_drive = s->drives[0];
    return s;
}

BlockDevice *ide_block_open_rw(const char *filename)
{
    if (!filename || !filename[0])
        return NULL;
    return block_device_init(filename, BF_MODE_RW);
}

int ide_block_reopen_rw(BlockDevice *bs, const char *filename)
{
    (void)bs;
    (void)filename;
    /* Not currently supported for generic RW block devices. */
    return -1;
}

static uint32_t ide_stat_sectors_read;
static uint32_t ide_stat_sectors_written;

int ide_block_read(BlockDevice *bs, uint64_t sector_num, uint8_t *buf, int nsectors)
{
    int ret;
    if (!bs || !buf || nsectors <= 0)
        return -1;
    ret = bs->read_async(bs, sector_num, buf, nsectors, NULL, NULL);
    if (ret == 0) ide_stat_sectors_read += nsectors;
    return (ret <= 0) ? ret : -1;
}

int ide_block_write(BlockDevice *bs, uint64_t sector_num, const uint8_t *buf, int nsectors)
{
    int ret;
    if (!bs || !buf || nsectors <= 0)
        return -1;
    ret = bs->write_async(bs, sector_num, buf, nsectors, NULL, NULL);
    if (ret == 0) ide_stat_sectors_written += nsectors;
    return (ret <= 0) ? ret : -1;
}

void ide_get_io_stats(uint32_t *sectors_read, uint32_t *sectors_written)
{
    *sectors_read = ide_stat_sectors_read;
    *sectors_written = ide_stat_sectors_written;
}

int64_t ide_block_sector_count(BlockDevice *bs)
{
    if (!bs || !bs->get_sector_count)
        return -1;
    return bs->get_sector_count(bs);
}

void ide_block_close(BlockDevice *bs)
{
    if (!bs)
        return;
    /* Only supports standard file-backed block devices for now. */
    if (bs->get_sector_count != bf_get_sector_count)
        return;
    BlockDeviceFile *bf = bs->opaque;
    if (!bf)
        return;
    if (bf->f) {
        fclose(bf->f);
        bf->f = NULL;
    }
    if (bf->cache_buf) {
        free(bf->cache_buf);
        bf->cache_buf = NULL;
        bf->cache_start = -1;
        bf->cache_count = 0;
    }
    if (bf->extents) {
        free(bf->extents);
        bf->extents = NULL;
        bf->num_extents = 0;
    }
}

/* HLE accessors for INT 13h emulation */
BlockDevice *ide_get_drive_bs(IDEIFState *s, int drive)
{
    if (!s || drive < 0 || drive > 1 || !s->drives[drive])
        return NULL;
    return s->drives[drive]->bs;
}

int ide_get_drive_cylinders(IDEIFState *s, int drive)
{
    if (!s || drive < 0 || drive > 1 || !s->drives[drive])
        return 0;
    return s->drives[drive]->cylinders;
}

int ide_get_drive_heads(IDEIFState *s, int drive)
{
    if (!s || drive < 0 || drive > 1 || !s->drives[drive])
        return 0;
    return s->drives[drive]->heads;
}

int ide_get_drive_sectors(IDEIFState *s, int drive)
{
    if (!s || drive < 0 || drive > 1 || !s->drives[drive])
        return 0;
    return s->drives[drive]->sectors;
}

int64_t ide_get_drive_nb_sectors(IDEIFState *s, int drive)
{
    if (!s || drive < 0 || drive > 1 || !s->drives[drive])
        return 0;
    return s->drives[drive]->nb_sectors;
}

int ide_is_drive_cd(IDEIFState *s, int drive)
{
    if (!s || drive < 0 || drive > 1 || !s->drives[drive])
        return 0;
    return s->drives[drive]->drive_kind == IDE_CD;
}

int ide_attach(IDEIFState *s, int drive, const char *filename)
{
    BlockDevice *bs;
    if (strcmp(filename, "/dev/mmcblk0") == 0) {
        bs = block_device_init_espsd(0, -1);
    } else {
        bs = block_device_init(filename, BF_MODE_RW);
    }
    if (!bs) {
        return -1;
    }
    s->drives[drive] = ide_hddrive_init(s, bs);
    return 0;
}

int ide_attach_cd(IDEIFState *s, int drive, const char *filename)
{
    assert(MAX_MULT_SECTORS >= 4);
    BlockDevice *bs;
    if (filename && filename[0]) {
        bs = block_device_init(filename, BF_MODE_RO);
    } else {
        // Always allocate BlockDevice (with cache) at startup
        // This avoids runtime pcram allocation when mounting CDs via OSD
        bs = block_device_init_empty_ro();
    }
    s->drives[drive] = ide_cddrive_init(s, bs);
    return 0;
}

/* Attach HDD, creating empty slot if filename is NULL for hot-mounting */
int ide_attach_hdd(IDEIFState *s, int drive, const char *filename)
{
    BlockDevice *bs;
    if (filename && filename[0]) {
        bs = block_device_init(filename, BF_MODE_RW);
        if (!bs) {
            return -1;
        }
    } else {
        // Create empty HDD slot for hot-mounting via OSD
        bs = block_device_init_empty_rw();
    }
    s->drives[drive] = ide_hddrive_init(s, bs);
    return 0;
}

int ide_attach_usb(IDEIFState *s, int drive)
{
    // Create USB BlockDevice - always present but reports 0 sectors when
    // no USB storage is connected. This avoids IDE hot-attach complexity.
    BlockDevice *bs = block_device_init_usb();
    if (!bs) {
        return -1;
    }
    s->drives[drive] = ide_hddrive_init(s, bs);
#ifdef DEBUG_USB_IDE
    if (s->drives[drive]) {
        fprintf(stderr, "USB IDE attached: nb_sectors=%lld cyl=%d heads=%d sectors=%d\n",
                (long long)s->drives[drive]->nb_sectors,
                s->drives[drive]->cylinders,
                s->drives[drive]->heads,
                s->drives[drive]->sectors);
    }
#endif
    return 0;
}

static void block_device_reinit(BlockDevice *bs, const char *filename)
{
    if (bs->get_sector_count != bf_get_sector_count) {
        fprintf(stderr, "block_device_reinit: not supported device\n");
        return;
    }
    BlockDeviceFile *bf = bs->opaque;

    if (bf->mode != BF_MODE_RO || bs->get_chs) {
        fprintf(stderr, "block_device_reinit: not supported device mode\n");
        return;
    }

    // Handle eject (empty filename)
    if (!filename || !filename[0]) {
        if (bf->f) {
            fclose(bf->f);
            bf->f = NULL;
        }
        bf->nb_sectors = 0;
        bf->start_offset = 0;
        bf->path[0] = '\0';
        bf->cache_start = -1;
        bf->cache_count = 0;
        return;
    }

    int64_t file_size;
    FILE *f;
    const char *mode_str = "rb";

    f = fopen(filename, mode_str);
    if (!f) {
        fprintf(stderr, "block_device_reinit: open failed: %s\n", filename);
        return;
    }

    fseeko(f, 0, SEEK_END);
    file_size = ftello(f);

    bf->nb_sectors = file_size / 512;
    if (bf->f)
        fclose(bf->f);
    bf->f = f;
    bf->start_offset = 0;
    strncpy(bf->path, filename, sizeof(bf->path) - 1);
    bf->path[sizeof(bf->path) - 1] = '\0';
    bf->cache_start = -1;
    bf->cache_count = 0;

    /* Free old extent map if any */
    if (bf->extents) {
        free(bf->extents);
        bf->extents = NULL;
        bf->num_extents = 0;
    }
    /* Try raw SD acceleration for new file */
    if (build_extent_map(bf) == 0) {
        fclose(bf->f);
        bf->f = NULL;
        /* Remove 2GB cap via FatFS */
        FIL *fil2 = malloc(sizeof(FIL));
        if (fil2) {
            char fatfs_path[256];
            snprintf(fatfs_path, sizeof(fatfs_path), "0:/%s",
                     filename + strlen("/sdcard/"));
            if (f_open(fil2, fatfs_path, FA_READ) == FR_OK) {
                int64_t real_size = (int64_t)f_size(fil2);
                f_close(fil2);
                if (real_size > 0)
                    bf->nb_sectors = real_size / SECTOR_SIZE;
            }
            free(fil2);
        }
        wcache_init();
        /* RO raw SD gets read-ahead cache */
        if (!bf->cache_buf) {
            bf->cache_buf = malloc(DISK_CACHE_SECTORS * SECTOR_SIZE);
            bf->cache_start = -1;
            bf->cache_count = 0;
        }
    }
}

void ide_change_cd(IDEIFState *sif, int drive, const char *filename)
{
    fprintf(stderr, "ide_change_cd: sif=%p drive=%d file=%s\n",
            (void*)sif, drive, filename ? filename : "(null)");

    if (!sif) {
        fprintf(stderr, "ide_change_cd: sif is NULL!\n");
        return;
    }

    IDEState *s = sif->drives[drive];
    fprintf(stderr, "ide_change_cd: s=%p\n", (void*)s);

    /* Sanity: back-pointer must match parent (catches stale PSRAM after reflash) */
    if (s && s->ide_if != sif) {
        fprintf(stderr, "ide_change_cd: stale pointer (ide_if=%p != sif=%p), skipping\n",
                (void*)s->ide_if, (void*)sif);
        return;
    }

    if (s && s->drive_kind == IDE_CD && s->bs) {
        fprintf(stderr, "ide_change_cd: drive is CD, bs=%p\n", (void*)s->bs);
        // Reinitialize with new image (or eject if filename is empty)
        block_device_reinit(s->bs, filename);
        fprintf(stderr, "ide_change_cd: sectors=%lld\n",
                (long long)s->bs->get_sector_count(s->bs));
        s->sense_key = SENSE_UNIT_ATTENTION;
        s->asc = ASC_MEDIUM_MAY_HAVE_CHANGED;
        s->cdrom_changed = 1;
        ide_set_irq(s);
        fprintf(stderr, "ide_change_cd: done\n");
    } else {
        fprintf(stderr, "ide_change_cd: not a CD drive (s=%p, kind=%d, bs=%p)\n",
                (void*)s, s ? s->drive_kind : -1, s ? (void*)s->bs : NULL);
    }
}

const char *ide_get_cd_path(IDEIFState *sif, int drive)
{
    if (!sif || drive < 0 || drive > 1)
        return NULL;
    IDEState *s = sif->drives[drive];
    if (!s || s->drive_kind != IDE_CD || !s->bs)
        return NULL;
    if (s->bs->get_sector_count != bf_get_sector_count)
        return NULL;
    BlockDeviceFile *bf = s->bs->opaque;
    return bf->path;
}

const char *ide_get_hdd_path(IDEIFState *sif, int drive)
{
    if (!sif || drive < 0 || drive > 1)
        return NULL;
    IDEState *s = sif->drives[drive];
    if (!s || s->drive_kind != IDE_HD || !s->bs)
        return NULL;
    if (s->bs->get_sector_count != bf_get_sector_count)
        return NULL;
    BlockDeviceFile *bf = s->bs->opaque;
    return bf->path;
}

int ide_change_hdd(IDEIFState *sif, int drive, const char *filename)
{
    fprintf(stderr, "ide_change_hdd: sif=%p drive=%d file=%s\n",
            (void*)sif, drive, filename ? filename : "(null)");

    if (!sif) {
        fprintf(stderr, "ide_change_hdd: sif is NULL!\n");
        return -1;
    }

    IDEState *s = sif->drives[drive];

    /* Sanity: back-pointer must match parent (catches stale PSRAM after reflash) */
    if (s && s->ide_if != sif) {
        fprintf(stderr, "ide_change_hdd: stale pointer (ide_if=%p != sif=%p)\n",
                (void*)s->ide_if, (void*)sif);
        sif->drives[drive] = NULL;
        s = NULL;
    }

    /* If no drive exists yet, create one (for hot-mounting via OSD) */
    if (!s) {
        fprintf(stderr, "ide_change_hdd: creating new HDD drive structure\n");
        BlockDevice *bs = block_device_init_empty_rw();
        if (!bs) {
            fprintf(stderr, "ide_change_hdd: failed to create block device\n");
            return -1;
        }
        sif->drives[drive] = ide_hddrive_init(sif, bs);
        s = sif->drives[drive];
        if (!s) {
            fprintf(stderr, "ide_change_hdd: failed to create drive\n");
            return -1;
        }
    }

    if (s->drive_kind != IDE_HD || !s->bs) {
        fprintf(stderr, "ide_change_hdd: not an HDD drive (s=%p, kind=%d)\n",
                (void*)s, s ? s->drive_kind : -1);
        return -1;
    }

    if (s->bs->get_sector_count != bf_get_sector_count) {
        fprintf(stderr, "ide_change_hdd: unsupported block device type\n");
        return -1;
    }

    BlockDeviceFile *bf = s->bs->opaque;

    /* --- Tear down old state --- */
    if (bf->f) {
        fclose(bf->f);
        bf->f = NULL;
    }
    /* Free old extent map if any */
    if (bf->extents) {
        free(bf->extents);
        bf->extents = NULL;
        bf->num_extents = 0;
    }
    /* Free read-ahead cache — must be NULL for RW raw SD so reads
     * go through the wcache-aware path instead of the CD-ROM path */
    if (bf->cache_buf) {
        free(bf->cache_buf);
        bf->cache_buf = NULL;
    }
    bf->nb_sectors = 0;
    bf->start_offset = 0;
    bf->cache_start = -1;
    bf->cache_count = 0;
    bf->dirty = 0;
    bf->cylinders = 0;
    bf->heads = 0;
    bf->sectors = 0;
    s->bs->get_chs = NULL;

    /* Handle eject (empty filename) */
    if (!filename || !filename[0]) {
        bf->path[0] = '\0';
        s->nb_sectors = 0;
        return 0;
    }

    /* Open new file */
    FILE *f = fopen(filename, "r+b");
    if (!f) {
        fprintf(stderr, "ide_change_hdd: open failed: %s\n", filename);
        bf->path[0] = '\0';
        return -1;
    }

    /* Check for magic header */
    char buf[8];
    int start_offset = 0;
    fread(buf, 1, 8, f);
    if (memcmp(buf, ide_magic, 8) == 0)
        start_offset = 1024;

    /* Get file size (handle 32-bit off_t overflow on ESP32-P4) */
    fseeko(f, 0, SEEK_END);
    int64_t file_size;
    {
        off_t raw = ftello(f);
        file_size = (raw >= 0) ? (int64_t)raw : (int64_t)(uint32_t)raw;
    }
    file_size -= start_offset;

    /* Cap to off_t limit for stdio path (may be lifted by extent map below) */
    if (file_size > (int64_t)0x7FFFFFFF) {
        fprintf(stderr, "ide_change_hdd: %s is %lldMB, capping to 2047MB (off_t limit)\n",
                filename, (long long)(file_size / (1024 * 1024)));
        file_size = (int64_t)0x7FFFFE00;
    }

    bf->f = f;
    bf->mode = BF_MODE_RW;
    bf->nb_sectors = file_size / 512;
    bf->start_offset = start_offset;
    setvbuf(f, NULL, _IOFBF, 32768);
    strncpy(bf->path, filename, sizeof(bf->path) - 1);
    bf->path[sizeof(bf->path) - 1] = '\0';

    /* Read CHS from magic header or auto-detect from MBR */
    if (start_offset) {
        /* Old-format header has geometry at offset 512 */
        unsigned char geobuf[14];
        fseeko(f, 512, SEEK_SET);
        fread(geobuf, 1, 14, f);
        bf->cylinders = geobuf[2] | (geobuf[3] << 8);
        bf->heads = geobuf[6] | (geobuf[7] << 8);
        bf->sectors = geobuf[12] | (geobuf[13] << 8);
        s->bs->get_chs = bf_get_chs;
    } else {
        /* Auto-detect CHS geometry from MBR partition table */
        unsigned char mbr[512];
        fseeko(f, 0, SEEK_SET);
        if (fread(mbr, 1, 512, f) == 512 &&
            mbr[510] == 0x55 && mbr[511] == 0xAA) {
            static const int pe_offsets[4] = {0x1BE, 0x1CE, 0x1DE, 0x1EE};
            for (int i = 0; i < 4; i++) {
                const unsigned char *pe = mbr + pe_offsets[i];
                if (pe[4] == 0) continue;
                int end_head = pe[5];
                int end_sec = pe[6] & 0x3F;
                int start_head = pe[1];
                int start_sec = pe[2] & 0x3F;
                int start_cyl = pe[3] | ((pe[2] & 0xC0) << 2);
                uint32_t lba_start = pe[8] | (pe[9] << 8) |
                                     (pe[10] << 16) | (pe[11] << 24);
                int spt = end_sec;
                int heads = end_head + 1;
                if (spt < 1 || spt > 63 || heads < 1 || heads > 255)
                    continue;
                if (start_sec < 1 || start_sec > spt)
                    continue;
                uint32_t calc_lba = (uint32_t)start_cyl * heads * spt +
                                    (uint32_t)start_head * spt +
                                    (start_sec - 1);
                if (calc_lba == lba_start) {
                    bf->sectors = spt;
                    bf->heads = heads;
                    bf->cylinders = bf->nb_sectors / (heads * spt);
                    if (bf->cylinders > 16383) bf->cylinders = 16383;
                    else if (bf->cylinders < 2) bf->cylinders = 2;
                    s->bs->get_chs = bf_get_chs;
                    break;
                }
            }
        }
    }

    /* Try raw SD acceleration (rebuilds extent map for new file) */
    if (build_extent_map(bf) == 0) {
        /* Raw SD active — close stdio handle */
        fclose(f);
        bf->f = NULL;
        /* Remove 2GB cap via FatFS.
         * FIL heap-allocated to avoid stack overflow (4KB buf inside). */
        FIL *fil2 = malloc(sizeof(FIL));
        if (fil2) {
            char fatfs_path[256];
            snprintf(fatfs_path, sizeof(fatfs_path), "0:/%s", filename + strlen("/sdcard/"));
            if (f_open(fil2, fatfs_path, FA_READ) == FR_OK) {
                int64_t real_size = (int64_t)f_size(fil2) - start_offset;
                f_close(fil2);
                if (real_size > 0)
                    bf->nb_sectors = real_size / SECTOR_SIZE;
            }
            free(fil2);
        }
        wcache_init();
    } else if (!bf->cache_buf) {
        /* Allocate read-ahead cache if not already present */
        bf->cache_buf = malloc(DISK_CACHE_SECTORS * SECTOR_SIZE);
        bf->cache_start = -1;
        bf->cache_count = 0;
    }

    /* Update IDE drive geometry (mirrors ide_hddrive_init logic) */
    s->nb_sectors = bf->nb_sectors;
    if (s->bs->get_chs) {
        s->bs->get_chs(s->bs, &s->cylinders, &s->heads, &s->sectors);
    } else {
        /* Default geometry: 255H/63S for >504MB, 16H/63S for smaller */
        if (s->nb_sectors > (uint64_t)16 * 63 * 1024) {
            s->heads = 255;
            s->sectors = 63;
        } else {
            s->heads = 16;
            s->sectors = 63;
        }
        uint32_t cyls = s->nb_sectors / (s->heads * s->sectors);
        if (cyls > 16383) cyls = 16383;
        else if (cyls < 2) cyls = 2;
        s->cylinders = cyls;
    }

    /* Reset IDE register state for clean BIOS detection */
    s->feature = 0;
    s->error = 0;
    s->nsector = 0;
    s->sector = 0;
    s->lcyl = 0;
    s->hcyl = 0;
    s->select = 0xa0;
    s->status = READY_STAT | SEEK_STAT;
    s->data_index = 0;
    s->data_end = 0;
    s->end_transfer_func = ide_transfer_stop;
    s->req_nb_sectors = 0;
    s->io_nb_sectors = 0;

    fprintf(stderr, "ide_change_hdd: done, sectors=%lld cyl=%d heads=%d secs=%d%s\n",
            (long long)s->nb_sectors, s->cylinders, s->heads, s->sectors,
            bf->extents ? " (raw SD)" : " (stdio)");
    return 0;
}

PCIDevice *piix3_ide_init(PCIBus *pci_bus, int devfn)
{
    PCIDevice *d;
    d = pci_register_device(pci_bus, "PIIX3 IDE", devfn, 0x8086, 0x7010, 0x00, 0x0101);
    pci_device_set_config8(d, 0x09, 0x00); /* ISA IDE ports, PIO only */
    return d;
}

void ide_fill_cmos(IDEIFState *s, void *cmos,
                   uint8_t (*set)(void *cmos, int addr, uint8_t val))
{
    // hard disk type, fixes "MS-DOS compatibility mode" in win9x
    uint8_t d_0x12 = 0;
    if (s->drives[0]) {
        d_0x12 |= 0xf0;
        set(cmos, 0x19, 47);
        set(cmos, 0x1b, set(cmos, 0x21, s->drives[0]->cylinders));
        set(cmos, 0x1c, set(cmos, 0x22, s->drives[0]->cylinders >> 8));
        set(cmos, 0x1d, s->drives[0]->heads);
        set(cmos, 0x1e, 0xff);
        set(cmos, 0x1f, 0xff);
        set(cmos, 0x20, 0xc0 | ((s->drives[0]->heads > 8) << 3));
        set(cmos, 0x23, s->drives[0]->sectors);
    }
    if (s->drives[1]) {
        d_0x12 |= 0x0f;
        set(cmos, 0x1a, 47);
        set(cmos, 0x24, set(cmos, 0x2a, s->drives[1]->cylinders));
        set(cmos, 0x25, set(cmos, 0x2b, s->drives[1]->cylinders >> 8));
        set(cmos, 0x26, s->drives[1]->heads);
        set(cmos, 0x27, 0xff);
        set(cmos, 0x28, 0xff);
        set(cmos, 0x29, 0xc0 | ((s->drives[1]->heads > 8) << 3));
        set(cmos, 0x2c, s->drives[1]->sectors);
    }
    set(cmos, 0x12, d_0x12);
}

/* Poll for completed async reads — call from main emulation loop
 * so interrupt-driven guests that HLT get their completions. */
void ide_poll_async(void)
{
    if (__atomic_load_n(&async_read.done, __ATOMIC_ACQUIRE)) {
        __atomic_store_n(&async_read.done, 0, __ATOMIC_RELAXED);
        async_read.cb(async_read.cb_opaque, async_read.result);
    }
}

/* Sync all open disk files to physical media.
 * Call periodically to limit data loss on unexpected power-off. */
static void ide_sync_if(IDEIFState *s)
{
    if (!s) return;
    for (int i = 0; i < 2; i++) {
        if (!s->drives[i] || !s->drives[i]->bs) continue;
        BlockDeviceFile *bf = s->drives[i]->bs->opaque;
        if (bf && bf->f && bf->mode == BF_MODE_RW && bf->dirty) {
            fflush(bf->f);
            fsync(fileno(bf->f));
            bf->dirty = 0;
        }
    }
}

void ide_sync(IDEIFState *ide, IDEIFState *ide2)
{
    wcache_flush();
    wcache_report_stats();
    ide_sync_if(ide);
    ide_sync_if(ide2);
}

/* Close all open file handles on an IDE interface (for teardown). */
static void ide_close_files_if(IDEIFState *s)
{
    if (!s) return;
    for (int i = 0; i < 2; i++) {
        if (!s->drives[i] || !s->drives[i]->bs) continue;
        BlockDeviceFile *bf = s->drives[i]->bs->opaque;
        if (bf && bf->f) {
            fflush(bf->f);
            fsync(fileno(bf->f));
            fclose(bf->f);
            bf->f = NULL;
        }
    }
}

void ide_close_files(IDEIFState *ide, IDEIFState *ide2)
{
    ide_close_files_if(ide);
    ide_close_files_if(ide2);
}
