# ESP32-P4 Target Review: Functionality Enhancement Analysis

**Document Version**: 1.0
**Date**: 2026-02-03
**Branch**: `claude/review-esp32-p4-el3dg`
**Status**: In Progress - Iterative Review

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Current Implementation Analysis](#2-current-implementation-analysis)
3. [OSD Enhancement Opportunities](#3-osd-enhancement-opportunities)
4. [Sound Card Options](#4-sound-card-options)
5. [USB Host Implementation](#5-usb-host-implementation)
6. [HID Integration](#6-hid-integration)
7. [USB Mass Storage](#7-usb-mass-storage)
8. [Implementation Priority](#8-implementation-priority)
9. [Open Questions](#9-open-questions)
10. [Session Notes](#10-session-notes)

---

## 1. Executive Summary

Tiny386 is a portable x86 PC emulator (~6K LOC CPU core) that boots Windows 9x/NT and Linux on ESP32 microcontrollers. The ESP32-P4 target is primarily supported via the **Tanmatsu** badge platform with these current capabilities:

| Component | Current State | Maturity |
|-----------|---------------|----------|
| CPU | i386/486/586 + optional FPU | Mature |
| Graphics | VGA + Bochs VBE, PPA rotation | Mature |
| Audio | SB16 + AdLib OPL2 + PC Speaker | Mature |
| Input | PS/2 keyboard via badge-bsp | Mature |
| Storage | IDE HDD/CD-ROM, Floppy | Mature |
| OSD | Tanmatsu keyboard-driven menu | Functional |
| Network | NE2000 via libslirp | Functional |
| USB | Not implemented (PR #4 pending) | Missing |

### Key Files for ESP32-P4

```
esp/main/
├── esp_main.c        # Entry point, task creation
├── lcd_bsp.c         # Display with PPA hardware rotation (493 lines)
├── input_bsp.c       # Keyboard via badge-bsp (241 lines)
├── tanmatsu_osd.c    # On-screen display menu (1,139 lines)
├── i2s.c             # Audio output (144 lines)
├── storage.c         # SD card/flash mounting
├── board_tanmatsu.h  # Hardware definitions
└── common.h          # Shared globals
```

---

## 2. Current Implementation Analysis

### 2.1 Display System

**Architecture:**
- Physical display: 480x800 portrait (MIPI DSI)
- VGA framebuffer: 800x480 landscape
- Rotation: PPA (Pixel Processing Accelerator) hardware unit
- Double buffering for rotation output
- Frame skip: configurable 0-10 for performance tuning

**Key Code** (`lcd_bsp.c`):
```c
// PPA handles 270° rotation with stride support
// Fallback to block-based software rotation (32-byte blocks for cache efficiency)
static void ppa_rotate_270(...);
static void software_rotate_block(...);
```

**Performance Optimizations:**
- TEXT_RENDER_OPT flag for incremental text rendering
- VGA glyph caching to avoid repeated character rendering
- Adaptive frame skipping based on VGA mode changes

### 2.2 Audio System

**Architecture:**
```
┌─────────────┬──────────────┬─────────────┐
│ AdLib OPL2  │ SB16 DSP     │ PC Speaker  │
│ (FM synth)  │ (DMA audio)  │ (square)    │
│ fmopl.c     │ sb16.c       │ pcspk.c     │
└──────┬──────┴──────┬───────┴──────┬──────┘
       │             │              │
       └─────────────┼──────────────┘
                     ▼
            mixer_callback() [pc.c:926]
                     │
                     ▼
            I2S Output 44.1kHz stereo [i2s.c]
                     │
                     ▼
            BSP Audio Amplifier
```

**Mixer Implementation** (`pc.c:926-979`):
- AdLib: mono FM synthesis, volume via SB16 mixer FM registers
- SB16: stereo digital audio, voice volume registers
- PC Speaker: bypasses mixer (separate circuit on real hardware)
- Master volume applied to combined FM+voice
- Per-channel L/R volume support

### 2.3 Input System

**Current Path:**
```
Badge-BSP keyboard → input_task() → PS/2 scancode queue → i8042 → CPU
```

**Special Handling** (`input_bsp.c`):
- META key toggles OSD on/off
- META+Arrow: brightness/volume adjustment
- 1500ms overlay display timeout
- Force VGA redraw on OSD close

### 2.4 OSD System

**Menu Structure** (`tanmatsu_osd.c`):
```
Main Menu
├── MOUNTING
│   ├── FDA (Floppy A) → File browser
│   ├── FDB (Floppy B) → File browser
│   ├── CDA-CDD (CD-ROM) → File browser
│   └── [256 file limit per directory]
├── AUDIO/VISUAL
│   ├── Brightness (0-100%)
│   ├── Volume (0-100%)
│   └── Frame Skip (0-10)
├── SYSTEM SETTINGS
│   ├── CPU Generation (3/4/5/6)
│   ├── FPU Enable/Disable
│   └── Memory Size (1-24MB)
├── BOOT ORDER (6 presets)
├── SAVE INI
├── CTRL+ALT+DEL
├── RESET SYSTEM
└── EXIT
```

**Rendering:**
- VGA 8x16 bitmap font (cp437 character set)
- Direct framebuffer drawing
- Semi-transparent overlay background
- Real-time brightness/volume bars

---

## 3. OSD Enhancement Opportunities

### 3.1 Status Information Panel

**Rationale**: Users often want to know what's happening inside the emulator

**Proposed Features:**
- [ ] Current VGA mode (e.g., "Mode 13h 320x200x256", "VESA 101h 640x480")
- [ ] CPU utilization percentage
- [ ] Current batch size (512-4096)
- [ ] Memory usage
- [ ] Disk activity indicator

**Implementation Approach:**
```c
// Add to ViewMode enum in tanmatsu_osd.c
typedef enum {
    VIEW_MAIN,
    VIEW_MOUNTING,
    VIEW_BROWSE,
    VIEW_AUDIOVISUAL,
    VIEW_SYSTEM,
    VIEW_BOOTORDER,
    VIEW_STATUS,      // NEW: System status panel
} ViewMode;

// Data already available in pc.c pc_step() instrumentation:
// - stat_cpu_us, stat_periph_us (CPU/peripheral time split)
// - batch_size (current instruction batch)
// From vga.c: current mode, resolution, color depth
```

**Complexity**: Low
**Impact**: Medium - improves user understanding of system state

### 3.2 Screenshot Capture

**Rationale**: Preserve moments, debug visual issues

**Proposed Implementation:**
```c
// In tanmatsu_osd.c
case MAIN_SCREENSHOT:
    if (save_screenshot()) {
        show_toast("Screenshot saved");
    }
    break;

// New function
bool save_screenshot(void) {
    char filename[64];
    snprintf(filename, sizeof(filename),
             "/sdcard/screenshots/shot_%lu.bmp",
             (unsigned long)time(NULL));

    // Write BMP header + RGB565 framebuffer data
    // globals.fb contains the current frame
    return write_bmp_rgb565(filename, globals.fb,
                            DISPLAY_WIDTH, DISPLAY_HEIGHT);
}
```

**Complexity**: Low
**Impact**: Medium - useful for sharing/debugging

### 3.3 Keyboard Shortcut Help

**Rationale**: Discoverable shortcuts improve UX

**Current shortcuts** (undocumented in OSD):
- META: Toggle OSD
- META+UP/DOWN: Volume
- META+LEFT/RIGHT: Brightness

**Proposed**: Add "HELP" menu item showing keyboard shortcuts

**Complexity**: Low
**Impact**: Low-Medium

### 3.4 Drive Status Indicators

**Rationale**: Know which drives are mounted and active

**Proposed:**
- Show mounted image names in compact form
- Disk activity LED simulation (brief highlight on access)

**Complexity**: Medium (requires IDE callback hooks)
**Impact**: Medium

### 3.5 Network Status (Future)

**Rationale**: Useful when WiFi/NE2000 is active

**Proposed:**
- IP address display
- Connection status
- Packet counters

**Complexity**: Low
**Impact**: Low (only relevant for network-enabled builds)

---

## 4. Sound Card Options

### 4.1 Current: AdLib OPL2 + Sound Blaster 16

**AdLib** (`adlib.c` + `fmopl.c`):
- Yamaha YM3812 (OPL2) FM synthesis
- 9 melody channels OR 6 melody + 5 rhythm
- I/O ports: 0x388-0x389 (also mirrored at 0x220-0x223)
- Well-tested MAME-derived implementation

**Sound Blaster 16** (`sb16.c`):
- DSP version 4.05 emulation
- DMA-based PCM playback (8/16-bit, mono/stereo)
- Auto-init DMA for streaming audio
- Mixer registers for volume control
- I/O ports: 0x220-0x22F

### 4.2 Option: OPL3 Upgrade

**What it adds:**
- 18 channels (vs 9 in OPL2)
- 4-operator mode for richer sounds
- True stereo output (OPL2 is mono)
- New waveforms

**Implementation Path:**
- Replace fmopl.c with MAME's ymf262.c (OPL3)
- Update adlib.c for OPL3 register set
- Minimal changes to I/O port handling

**Complexity**: Medium
**Memory**: ~20KB additional for tables
**Impact**: Medium - many DOS games benefit from OPL3

**Source Reference**: MAME `src/devices/sound/ymf262.cpp`

### 4.3 Option: MPU-401 MIDI Interface

**What it provides:**
- Standard MIDI interface for DOS/Windows
- Games can output MIDI to external synth or software synth
- UART mode (intelligent mode rarely used)

**I/O Ports**: 0x330 (data), 0x331 (status/command)

**Implementation:**
```c
// mpu401.h
typedef struct {
    uint8_t rx_fifo[256];
    uint8_t tx_fifo[256];
    int rx_head, rx_tail;
    int tx_head, tx_tail;
    uint8_t status;
    bool uart_mode;
    bool irq_pending;
    int irq;
    void (*set_irq)(void *, int, int);
    void *pic;
} MPU401State;

// UART mode is simple:
// - Write to 0x330: queue MIDI byte for output
// - Read from 0x330: get MIDI byte from input
// - 0x331 status: bit 6 = output ready, bit 7 = input available
```

**Output Options:**
1. Route to OPL2/3 for FM playback (software GM patch set)
2. Buffer for WiFi transmission to external synth
3. Discard (silent, but games won't hang)

**Complexity**: Medium
**Memory**: ~2KB
**Impact**: Medium - enables MIDI games, MT-32 compatibility layer possible

### 4.4 Option: Gravis Ultrasound (GUS)

**What it provides:**
- 32-voice wavetable synthesis
- 1MB onboard sample RAM
- Hardware volume ramping
- Stereo panning per voice
- Used by many DOS games and demos

**Architecture:**
```
┌────────────────────────────────────────────────┐
│               GUS Hardware Model               │
├────────────────────────────────────────────────┤
│  32 Voices                                     │
│  ┌─────────────────────────────────────────┐   │
│  │ Voice N:                                │   │
│  │  - Start/End/Loop addresses (20-bit)   │   │
│  │  - Current position (fixed-point)      │   │
│  │  - Frequency control (sample rate)     │   │
│  │  - Volume (0-4095 log scale)           │   │
│  │  - Volume ramp (rate, start, end)      │   │
│  │  - Pan position (0-15)                 │   │
│  │  - Control flags (loop, bi-dir, IRQ)   │   │
│  └─────────────────────────────────────────┘   │
│                                                │
│  1MB Sample RAM                                │
│  ┌─────────────────────────────────────────┐   │
│  │ Samples loaded via DMA or direct I/O   │   │
│  │ 8-bit or 16-bit samples                │   │
│  └─────────────────────────────────────────┘   │
│                                                │
│  I/O: 0x240-0x24F (Global/Voice registers)    │
│  IRQ: Configurable (typically IRQ 5 or 11)    │
│  DMA: Channels 1 and 5 (8-bit and 16-bit)     │
└────────────────────────────────────────────────┘
```

**Implementation Challenges:**

1. **Memory**: 1MB RAM requirement is significant on ESP32-P4
   - PSRAM available: 32MB total, 24MB for PC RAM
   - Could allocate from remaining PSRAM

2. **CPU**: Mixing 32 voices at 44.1kHz is expensive
   - Each voice needs: position update, sample fetch, volume/pan
   - Estimate: ~500 cycles per sample per voice
   - At 44.1kHz stereo: 32 * 44100 * 500 = 706M cycles/sec
   - ESP32-P4 at 400MHz: marginal, may need voice limiting

3. **Timing**: GUS has complex interrupt generation
   - Voice position IRQs, volume ramp IRQs, timer IRQs
   - Must be accurate for music timing

**Phased Implementation:**
1. Phase 1: Basic playback (8 voices, no volume ramping)
2. Phase 2: Full 32 voices, volume ramping
3. Phase 3: DMA transfers, IRQ generation

**Complexity**: HIGH
**Memory**: ~1.1MB (RAM + state)
**Impact**: High for demo scene and music software

**Decision Point**: Is there actual demand? GUS was niche even in its era.

### 4.5 Sound Card Comparison Matrix

| Feature | OPL2 (current) | OPL3 | MPU-401 | GUS |
|---------|----------------|------|---------|-----|
| Channels | 9 | 18 | N/A | 32 |
| Synthesis | FM | FM | MIDI routing | Wavetable |
| Stereo | No | Yes | Depends | Yes |
| Memory | ~10KB | ~30KB | ~2KB | ~1.1MB |
| CPU Load | Low | Low | Minimal | High |
| Game Support | Excellent | Excellent | Good | Niche |
| Complexity | Done | Medium | Medium | High |

---

## 5. USB Host Implementation

### 5.1 Current State

**Existing Work:**
- PR #4 by BigFishDev22 adds basic USB HID support
- Status: **Not merged** due to issues:
  - 8KB+ stack allocation causing memory pressure
  - Conflict with WiFi DMA on ESP32-S3
  - Some USB devices not recognized
  - Reset loops if devices connected before BIOS init

**ESP32-P4 Advantages:**
- More RAM than ESP32-S3
- Native USB OTG with better host support
- No WiFi/USB DMA conflict (separate controllers)

### 5.2 Recommended Approach

**Use ESP-IDF USB Host Library** (not TinyUSB for host mode):
- Better ESP32 integration
- Maintained by Espressif
- Documented examples

**Component Dependencies:**
```yaml
# esp/main/idf_component.yml additions
dependencies:
  espressif/usb_host_hid: "^1.0"
  espressif/usb_host_msc: "^1.0"
```

### 5.3 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    USB Host Task (Core 0)                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ HID Driver  │  │ MSC Driver  │  │ Hub Driver (future) │ │
│  │             │  │             │  │                     │ │
│  │ - Keyboard  │  │ - FAT/exFAT │  │ - Multi-device      │ │
│  │ - Mouse     │  │ - VFS mount │  │                     │ │
│  └──────┬──────┘  └──────┬──────┘  └─────────────────────┘ │
│         │                │                                  │
│         ▼                ▼                                  │
│  ┌─────────────┐  ┌─────────────┐                          │
│  │ PS/2        │  │ /usb VFS    │                          │
│  │ Translation │  │ mount point │                          │
│  └──────┬──────┘  └──────┬──────┘                          │
│         │                │                                  │
└─────────┼────────────────┼──────────────────────────────────┘
          │                │
          ▼                ▼
    i8042 (kbd/mouse)    OSD File Browser
```

### 5.4 Configuration

**INI File Options:**
```ini
[usb]
enable_host = 0      # Master enable (default off for safety)
enable_hid = 1       # Enable HID class (keyboard/mouse)
enable_msc = 1       # Enable mass storage class
```

**Why Default Off:**
- Memory overhead (~50KB+)
- Potential compatibility issues
- User should opt-in

### 5.5 Memory Budget

| Component | Estimated Size |
|-----------|----------------|
| USB Host Library | ~20KB |
| HID Class Driver | ~4KB |
| MSC Class Driver | ~8KB |
| Per-device buffers | ~4KB each |
| **Total (1 HID + 1 MSC)** | **~40KB** |

ESP32-P4 PSRAM budget: 32MB total, 24MB for PC RAM, leaves ~8MB for system.
USB overhead is acceptable.

---

## 6. HID Integration

### 6.1 USB HID to PS/2 Translation

**Challenge**: USB HID uses different scancode format than PS/2

**USB HID Boot Protocol** (keyboard):
```
Byte 0: Modifier keys (Ctrl, Shift, Alt, GUI)
Byte 1: Reserved
Bytes 2-7: Up to 6 simultaneous key codes
```

**PS/2 Scan Code Set 1** (used by i8042):
- Make code: single byte (or E0 prefix + byte for extended)
- Break code: make code + 0x80

**Translation Table Required:**
```c
// usb_hid_translate.h
static const uint8_t usb_to_ps2[256] = {
    [0x00] = 0x00,  // No key
    [0x04] = 0x1E,  // A
    [0x05] = 0x30,  // B
    [0x06] = 0x2E,  // C
    [0x07] = 0x20,  // D
    [0x08] = 0x12,  // E
    [0x09] = 0x21,  // F
    [0x0A] = 0x22,  // G
    [0x0B] = 0x23,  // H
    [0x0C] = 0x17,  // I
    [0x0D] = 0x24,  // J
    [0x0E] = 0x25,  // K
    [0x0F] = 0x26,  // L
    [0x10] = 0x32,  // M
    [0x11] = 0x31,  // N
    [0x12] = 0x18,  // O
    [0x13] = 0x19,  // P
    [0x14] = 0x10,  // Q
    [0x15] = 0x13,  // R
    [0x16] = 0x1F,  // S
    [0x17] = 0x14,  // T
    [0x18] = 0x16,  // U
    [0x19] = 0x2F,  // V
    [0x1A] = 0x11,  // W
    [0x1B] = 0x2D,  // X
    [0x1C] = 0x15,  // Y
    [0x1D] = 0x2C,  // Z
    [0x1E] = 0x02,  // 1
    [0x1F] = 0x03,  // 2
    [0x20] = 0x04,  // 3
    [0x21] = 0x05,  // 4
    [0x22] = 0x06,  // 5
    [0x23] = 0x07,  // 6
    [0x24] = 0x08,  // 7
    [0x25] = 0x09,  // 8
    [0x26] = 0x0A,  // 9
    [0x27] = 0x0B,  // 0
    [0x28] = 0x1C,  // Enter
    [0x29] = 0x01,  // Escape
    [0x2A] = 0x0E,  // Backspace
    [0x2B] = 0x0F,  // Tab
    [0x2C] = 0x39,  // Space
    // ... continue for full keyboard
    [0x4F] = 0x4D,  // Right Arrow (needs E0 prefix)
    [0x50] = 0x4B,  // Left Arrow (needs E0 prefix)
    [0x51] = 0x50,  // Down Arrow (needs E0 prefix)
    [0x52] = 0x48,  // Up Arrow (needs E0 prefix)
};

// Keys requiring E0 prefix
static const uint8_t usb_extended_keys[] = {
    0x4F, 0x50, 0x51, 0x52,  // Arrow keys
    0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,  // Ins/Home/PgUp/Del/End/PgDn
    0x46,  // Print Screen
    // ...
};
```

### 6.2 Mouse Integration

**USB HID Boot Protocol** (mouse):
```
Byte 0: Buttons (bit 0=left, bit 1=right, bit 2=middle)
Byte 1: X movement (signed)
Byte 2: Y movement (signed)
Byte 3: Wheel (optional, signed)
```

**PS/2 Mouse Protocol:**
```
Byte 0: [YO XO YS XS 1 MB RB LB]
Byte 1: X movement
Byte 2: Y movement
```

Translation is straightforward - reformat button bits and clamp movement values.

### 6.3 Key Tracking for Make/Break

**Challenge**: USB reports current state; PS/2 needs transitions

```c
typedef struct {
    uint8_t prev_keys[6];      // Previous report keys
    uint8_t prev_modifiers;    // Previous modifier state
} hid_keyboard_state_t;

void process_keyboard_report(uint8_t *report, hid_keyboard_state_t *state) {
    // Find released keys (in prev, not in current)
    for (int i = 0; i < 6; i++) {
        uint8_t key = state->prev_keys[i];
        if (key && !key_in_report(key, report)) {
            send_ps2_break(key);  // Key released
        }
    }

    // Find pressed keys (in current, not in prev)
    for (int i = 2; i < 8; i++) {
        uint8_t key = report[i];
        if (key && !key_in_array(key, state->prev_keys)) {
            send_ps2_make(key);  // Key pressed
        }
    }

    // Update state
    memcpy(state->prev_keys, &report[2], 6);
    state->prev_modifiers = report[0];
}
```

---

## 7. USB Mass Storage

### 7.1 Use Cases

1. **Larger disk images**: SD cards are typically 32GB max FAT32; USB drives can be larger
2. **Easy image transfer**: Mount USB drive, copy image, unmount
3. **Multiple image libraries**: Swap USB drives with different collections

### 7.2 VFS Integration

```c
// Mount USB drive at /usb
esp_vfs_fat_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024,
};

esp_err_t err = msc_host_vfs_register(msc_device, "/usb",
                                       &mount_config, &vfs_handle);
```

### 7.3 OSD File Browser Integration

**Current behavior**: Browse `/sdcard` only

**Proposed change:**
```c
// In tanmatsu_osd.c browse_start()
typedef enum {
    BROWSE_SOURCE_SDCARD,
    BROWSE_SOURCE_USB,
} browse_source_t;

static browse_source_t current_source = BROWSE_SOURCE_SDCARD;

const char* get_browse_path(void) {
    switch (current_source) {
        case BROWSE_SOURCE_USB:
            return "/usb";
        default:
            return "/sdcard";
    }
}

// Add source toggle in browse view
// Show "[SD] filename.img" or "[USB] filename.img"
```

### 7.4 Hot-Plug Handling

```c
// MSC event callback
void msc_event_callback(const msc_host_event_t *event, void *arg) {
    switch (event->event) {
        case MSC_DEVICE_CONNECTED:
            // Mount and notify OSD
            usb_msc_mount(event->device.handle);
            osd_notify_usb_connected();
            break;

        case MSC_DEVICE_DISCONNECTED:
            // Unmount (ensure no open files first!)
            if (current_source == BROWSE_SOURCE_USB) {
                current_source = BROWSE_SOURCE_SDCARD;
                browse_refresh();
            }
            usb_msc_unmount();
            osd_notify_usb_disconnected();
            break;
    }
}
```

**Safety**: Must handle mid-operation disconnect gracefully

---

## 8. Implementation Priority

### Priority Matrix (Updated per Session Decisions)

| Feature | Difficulty | Impact | Memory | Priority | Status |
|---------|------------|--------|--------|----------|--------|
| USB Host Framework | Medium | High | ~20KB | **P1** | Decided |
| USB HID Keyboard | Medium | High | ~10KB | **P1** | Decided |
| USB HID Mouse | Low | High | Included | **P1** | Decided |
| OSD Stats Panel | Low | Medium | ~1KB | **P2** | Decided |
| OSD Help Screen | Low | Low | ~0.5KB | **P2** | Decided |
| Drive Activity LEDs | Low | Medium | ~0.2KB | **P2** | Decided |
| Toast System | Low | Medium | ~0.5KB | **P2** | Decided |
| USB Mass Storage | Medium | High | ~40KB | **P3** | Decided |
| ~~OSD Screenshot~~ | - | - | - | - | **Rejected** |
| ~~OPL3 Upgrade~~ | - | - | - | - | **Rejected** |
| ~~GUS Emulation~~ | - | - | - | - | **Rejected** |

### Implementation Order

**Phase 1: USB Foundation** (enables external keyboard/mouse)
1. USB Host framework initialization
2. USB HID keyboard with PS/2 translation
3. USB HID mouse with PS/2 translation
4. Remove wifikbd

**Phase 2: OSD Enhancements** (polish and usability)
1. Toast notification system (needed by Phase 3)
2. Drive activity LEDs
3. OSD Stats Panel
4. OSD Help Screen

**Phase 3: USB Storage** (advanced feature)
1. BlockDeviceUSB implementation
2. IDE hot-attach for USB drives
3. OSD integration (hide CDD, show USB status)

---

## 9. Open Questions

### For Discussion

1. **GUS Demand**: Is there actual user demand for GUS emulation, or is SB16+OPL2 sufficient for target use cases?

2. **USB Priority**: Should USB HID be enabled by default on Tanmatsu, or remain opt-in?

3. **Memory Tradeoffs**: If GUS is implemented, should PC RAM be reduced to accommodate? (24MB → 23MB)

4. **WiFi Keyboard**: The existing `wifikbd` tool provides network keyboard input. Should USB HID replace it, or coexist?

5. **OPL3 vs OPL2**: Many games auto-detect and prefer OPL3. Should we upgrade, or keep OPL2 for compatibility?

6. **MIDI Output**: If MPU-401 is added, where should MIDI data go?
   - Option A: Route to OPL for FM playback
   - Option B: Buffer for external transmission
   - Option C: Discard (games won't hang)

### Technical Questions

7. **USB Hub Support**: PR #4 mentions hub support. Priority for multi-device scenarios?

8. **Boot Timing**: PR #4 notes issues with USB devices connected at boot. Should USB init be delayed until after BIOS?

---

## 10. Session Notes

### Session 1: 2026-02-03

**Participants**: User + Claude

**Summary**:
- Completed comprehensive review of ESP32-P4 target
- Analyzed current OSD, audio, display, and input implementations
- Evaluated sound card options (OPL3, MPU-401, GUS)
- Reviewed USB host implementation strategies
- Examined PR #4 for existing USB HID work
- Created this document for future reference

**Decisions Made**:

#### OSD Stats Panel - DECIDED
- **Approach**: Option A - Replace fprintf with memory writes, display via OSD
- **Rationale**:
  - Current fprintf blocks ~11ms every 2 seconds (0.5% CPU overhead)
  - Stats collection itself (get_uticks calls) is cheap (~0.04% CPU)
  - t0/t2 timer reads are essential for dynamic batch sizing anyway
  - t1 read (CPU/peripheral split) adds only ~0.01-0.04% overhead
- **Implementation**:
  - Remove fprintf from pc.c:602-607
  - Add stats struct to globals (cpu_pct, periph_pct, batch_size, etc.)
  - Write stats to memory every 2 seconds (same timing, no serial I/O)
  - New OSD VIEW_STATUS panel reads and displays these values
  - Zero cost when not viewing, ~0.5ms render cost when panel open

#### OSD Help Screen - DECIDED
- **Approach**: Simple text menu showing available keyboard shortcuts
- **Content**:
  - META: Toggle OSD
  - META+UP/DOWN: Volume adjust
  - META+LEFT/RIGHT: Brightness adjust
  - Future: META+F1 eject FDA, etc.
- **Implementation**: New VIEW_HELP in tanmatsu_osd.c, accessible from main menu

#### Drive Activity LEDs - DECIDED
- **Approach**: Use hardware RGB LEDs instead of OSD overlay
- **LED Assignment** (from Tanmatsu hardware docs):
  - LED 4 (A): HDD activity - red flash
  - LED 5 (B): Floppy activity - green/yellow flash
  - LEDs 0-3 reserved for system (Power, Radio, Messages, Power Button)
- **Implementation**:
  - Add callbacks to ide.c and emulink.c for read/write operations
  - Set LED on activity, clear after ~50-100ms timeout
  - Use bsp_led_set_pixel_rgb() and bsp_led_send()
  - Call bsp_led_set_mode(false) for manual control of LEDs 4-5

#### Toast Notification System - DECIDED
- **Approach**: Minimal, focused on non-visible confirmations only
- **Use cases** (toast IS appropriate):
  - "Settings saved" / "Save failed" after INI write
  - Hotkey feedback when OSD not open (e.g., META+F1 eject → "Floppy A ejected")
  - External events (USB disconnect, errors)
- **NOT for** (already visible, redundant):
  - Drive mounting via OSD file browser
  - Boot order changes (visible in menu)
  - Brightness/volume (already have bar overlay)
- **Implementation**: Extend existing overlay_type enum, add overlay_text field to globals

#### Screenshot Feature - DECIDED
- **Decision**: NO - not implementing
- **Rationale**: User indicated not needed

#### Sound Card Enhancements - DECIDED
- **Decision**: Keep current audio setup, no changes needed
- **Current state**:
  - SB16: Full PCM/digital audio (8/16-bit, stereo, DMA) - **MIT licensed** ✅
  - OPL2: FM synthesis via fmopl.c - LGPL 2.1 (accepted)
  - PC Speaker: Functional
- **OPL3 upgrade**: Not pursuing
  - Opal (public domain) lacks percussion mode
  - ymfm (BSD) is C++ and performance unknown
  - OPL2 + SB16 digital covers most games adequately
- **GUS**: Not pursuing
  - No MIT/BSD implementation exists
  - 1MB+ RAM cost, high CPU, niche demand
- **Rationale**: SB16 handles digital audio well; games wanting better music use digital samples anyway

#### USB Host - DECIDED
- **Enable State**: Always on (not opt-in)
- **WiFi Keyboard**: Remove wifikbd, USB HID fully replaces it
- **USB-C Port**: Power/UART/JTAG only - configured as USB_SERIAL_JTAG
  - Used for: Console output, JTAG debugging, power input
  - Cannot serve as second USB host port (would require hardware changes)
- **USB-A Port**: Primary USB host via ESP32-P4 USB OTG
  - 5V boost controlled via coprocessor (bsp_power_set_usb_host_boost_enabled)

#### USB HID - DECIDED
- **Keyboard**: Translate USB HID boot protocol to PS/2 scancode set 1
- **Mouse**: Translate USB HID mouse reports to PS/2 mouse protocol
- **OSD Interaction**: META key on USB keyboard toggles OSD
- **Implementation**:
  - Use ESP-IDF USB Host Library (not TinyUSB for host mode)
  - Add espressif/usb_host_hid component
  - Full USB HID to PS/2 translation table (see Section 6)
  - Track key state for make/break codes

#### USB Mass Storage - DECIDED
- **Mode**: Direct guest access (Option B) - USB filesystem visible to guest OS
- **IDE Channel**: Secondary slave (slot 3) - replaces CDD position
  - Current layout: HDA(0), HDB(1), CDC(2), CDD(3)
  - USB takes slot 3, so CDD becomes unavailable when USB attached
  - Acceptable tradeoff: most setups only use one CD-ROM (CDC)
- **Write Access**: Full read/write (not read-only)
- **Implementation**:
  - Add BlockDeviceUSB backend using existing BlockDevice abstraction in ide.c
  - ide_attach on secondary slave channel when USB mass storage detected
  - OSD: Hide CDD option or show "USB" when USB storage attached
  - Toast notification on connect/disconnect ("USB drive attached as D:")
- **Rationale**: IDE abstraction is clean integration point; BlockDevice struct already supports async read/write callbacks

#### USB Hub Support - DECIDED
- **Approach**: Enable hub support via CONFIG_USB_HOST_HUBS_SUPPORTED
- **Rationale**: Allows multi-device setups (keyboard + mouse + storage)

**Next Steps**:
- Finalize priority and implementation order
- Begin implementation based on decisions

**Key Files Examined**:
- `esp/main/tanmatsu_osd.c` - OSD implementation
- `esp/main/input_bsp.c` - Input handling
- `esp/main/lcd_bsp.c` - Display with PPA rotation
- `esp/main/i2s.c` - Audio output
- `pc.c` - Mixer and peripheral integration
- `sb16.c`, `adlib.c`, `fmopl.c` - Current audio
- PR #4 - USB HID work in progress

---

## 11. Detailed Implementation Plans

### 11.1 USB Host Framework

**Goal**: Initialize USB host on ESP32-P4, enable 5V boost, prepare for HID and MSC drivers.

**Files to Modify:**
| File | Changes |
|------|---------|
| `esp/main/idf_component.yml` | Add USB host dependencies |
| `esp/main/CMakeLists.txt` | Add new source files |
| `esp/main/Kconfig.projbuild` | Add USB config options |
| `sdkconfig_tanmatsu` | Enable USB host |
| `esp/main/common.h` | Add USB state to globals |
| **NEW** `esp/main/usb_host.c` | USB host task and initialization |
| **NEW** `esp/main/usb_host.h` | Public USB API |

**Step-by-Step Implementation:**

**Step 1: Add Dependencies** (`esp/main/idf_component.yml`)
```yaml
dependencies:
  espressif/usb_host_hid: "^1.0"
  espressif/usb_host_msc: "^1.0"
  espressif/usb_host_hub: "^1.0"  # For hub support
```

**Step 2: Kconfig Options** (`esp/main/Kconfig.projbuild`)
```
menu "USB Host Configuration"
    config USB_HOST_ENABLED
        bool "Enable USB Host"
        default y
        help
            Enable USB host functionality for keyboard, mouse, and storage.

    config USB_HOST_HID_ENABLED
        bool "Enable USB HID (keyboard/mouse)"
        default y
        depends on USB_HOST_ENABLED

    config USB_HOST_MSC_ENABLED
        bool "Enable USB Mass Storage"
        default y
        depends on USB_HOST_ENABLED
endmenu
```

**Step 3: sdkconfig Changes** (`sdkconfig_tanmatsu`)
```
CONFIG_USB_OTG_SUPPORTED=y
CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=1024
CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED=y
CONFIG_USB_HOST_HUBS_SUPPORTED=y
```

**Step 4: USB Host Header** (`esp/main/usb_host.h`)
```c
#pragma once
#include "esp_err.h"
#include <stdbool.h>

// Initialize USB host subsystem
esp_err_t usb_host_init(void);

// Check if USB keyboard is connected
bool usb_host_keyboard_connected(void);

// Check if USB mouse is connected
bool usb_host_mouse_connected(void);

// Check if USB storage is connected
bool usb_host_storage_connected(void);

// Get USB storage block device (for IDE integration)
struct BlockDevice* usb_host_get_storage_device(void);
```

**Step 5: USB Host Implementation** (`esp/main/usb_host.c`)
```c
#include "usb_host.h"
#include "bsp/power.h"
#include "usb/usb_host.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "USB_HOST";

static TaskHandle_t usb_host_task_handle = NULL;
static bool usb_initialized = false;

// USB Host Library task
static void usb_host_lib_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            // All clients deregistered
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            // All devices freed
        }
    }
}

esp_err_t usb_host_init(void) {
    esp_err_t err;

    // Enable 5V boost for USB-A port
    err = bsp_power_set_usb_host_boost_enabled(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable USB boost: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "USB 5V boost enabled");

    // Small delay for power to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize USB Host Library
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(err));
        return err;
    }

    // Create USB host library task
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host_lib", 4096,
                            NULL, 10, &usb_host_task_handle, 0);

    usb_initialized = true;
    ESP_LOGI(TAG, "USB Host initialized");
    return ESP_OK;
}
```

**Step 6: Integration in esp_main.c**
```c
// In app_main(), after BSP init but before emulator start:
#if CONFIG_USB_HOST_ENABLED
    #include "usb_host.h"
    esp_err_t usb_err = usb_host_init();
    if (usb_err != ESP_OK) {
        ESP_LOGW(TAG, "USB host init failed, continuing without USB");
    }
#endif
```

**Dependencies**: None (this is the foundation)

---

### 11.2 USB HID Implementation

**Goal**: Translate USB keyboard/mouse to PS/2 scancodes for i8042.

**Files to Modify:**
| File | Changes |
|------|---------|
| `esp/main/usb_host.c` | Add HID driver registration |
| **NEW** `esp/main/usb_hid.c` | HID event handling and translation |
| **NEW** `esp/main/usb_hid.h` | HID public API |
| `esp/main/input_bsp.c` | Accept scancodes from USB HID |
| `esp/main/common.h` | Add USB HID state |

**Step 1: USB HID Header** (`esp/main/usb_hid.h`)
```c
#pragma once
#include "esp_err.h"
#include <stdint.h>

// Initialize USB HID driver (call after usb_host_init)
esp_err_t usb_hid_init(void);

// Set callback for keyboard events (PS/2 scancodes)
typedef void (*usb_hid_keyboard_cb_t)(uint8_t scancode, bool is_break);
void usb_hid_set_keyboard_callback(usb_hid_keyboard_cb_t cb);

// Set callback for mouse events (PS/2 packets)
typedef void (*usb_hid_mouse_cb_t)(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel);
void usb_hid_set_mouse_callback(usb_hid_mouse_cb_t cb);

// Which key acts as META for OSD toggle (default: Left GUI / Windows key)
#define USB_HID_META_KEY 0xE3  // Left GUI in USB HID usage
```

**Step 2: Full Translation Table** (`esp/main/usb_hid.c`)
```c
#include "usb_hid.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "USB_HID";

// USB HID to PS/2 Scan Code Set 1 translation
// Index = USB HID usage code, Value = PS/2 make code (0 = no mapping)
// Keys needing E0 prefix have bit 7 set in a separate table
static const uint8_t usb_to_ps2_base[256] = {
    [0x00] = 0x00,  // Reserved (no event)
    [0x01] = 0x00,  // ErrorRollOver
    [0x02] = 0x00,  // POSTFail
    [0x03] = 0x00,  // ErrorUndefined
    [0x04] = 0x1E,  // A
    [0x05] = 0x30,  // B
    [0x06] = 0x2E,  // C
    [0x07] = 0x20,  // D
    [0x08] = 0x12,  // E
    [0x09] = 0x21,  // F
    [0x0A] = 0x22,  // G
    [0x0B] = 0x23,  // H
    [0x0C] = 0x17,  // I
    [0x0D] = 0x24,  // J
    [0x0E] = 0x25,  // K
    [0x0F] = 0x26,  // L
    [0x10] = 0x32,  // M
    [0x11] = 0x31,  // N
    [0x12] = 0x18,  // O
    [0x13] = 0x19,  // P
    [0x14] = 0x10,  // Q
    [0x15] = 0x13,  // R
    [0x16] = 0x1F,  // S
    [0x17] = 0x14,  // T
    [0x18] = 0x16,  // U
    [0x19] = 0x2F,  // V
    [0x1A] = 0x11,  // W
    [0x1B] = 0x2D,  // X
    [0x1C] = 0x15,  // Y
    [0x1D] = 0x2C,  // Z
    [0x1E] = 0x02,  // 1 !
    [0x1F] = 0x03,  // 2 @
    [0x20] = 0x04,  // 3 #
    [0x21] = 0x05,  // 4 $
    [0x22] = 0x06,  // 5 %
    [0x23] = 0x07,  // 6 ^
    [0x24] = 0x08,  // 7 &
    [0x25] = 0x09,  // 8 *
    [0x26] = 0x0A,  // 9 (
    [0x27] = 0x0B,  // 0 )
    [0x28] = 0x1C,  // Enter
    [0x29] = 0x01,  // Escape
    [0x2A] = 0x0E,  // Backspace
    [0x2B] = 0x0F,  // Tab
    [0x2C] = 0x39,  // Space
    [0x2D] = 0x0C,  // - _
    [0x2E] = 0x0D,  // = +
    [0x2F] = 0x1A,  // [ {
    [0x30] = 0x1B,  // ] }
    [0x31] = 0x2B,  // \ |
    [0x32] = 0x2B,  // Non-US # ~ (same as backslash for US)
    [0x33] = 0x27,  // ; :
    [0x34] = 0x28,  // ' "
    [0x35] = 0x29,  // ` ~
    [0x36] = 0x33,  // , <
    [0x37] = 0x34,  // . >
    [0x38] = 0x35,  // / ?
    [0x39] = 0x3A,  // Caps Lock
    [0x3A] = 0x3B,  // F1
    [0x3B] = 0x3C,  // F2
    [0x3C] = 0x3D,  // F3
    [0x3D] = 0x3E,  // F4
    [0x3E] = 0x3F,  // F5
    [0x3F] = 0x40,  // F6
    [0x40] = 0x41,  // F7
    [0x41] = 0x42,  // F8
    [0x42] = 0x43,  // F9
    [0x43] = 0x44,  // F10
    [0x44] = 0x57,  // F11
    [0x45] = 0x58,  // F12
    [0x46] = 0x37,  // Print Screen (special handling needed)
    [0x47] = 0x46,  // Scroll Lock
    [0x48] = 0x00,  // Pause (special handling needed)
    [0x49] = 0x52,  // Insert (E0 prefix)
    [0x4A] = 0x47,  // Home (E0 prefix)
    [0x4B] = 0x49,  // Page Up (E0 prefix)
    [0x4C] = 0x53,  // Delete (E0 prefix)
    [0x4D] = 0x4F,  // End (E0 prefix)
    [0x4E] = 0x51,  // Page Down (E0 prefix)
    [0x4F] = 0x4D,  // Right Arrow (E0 prefix)
    [0x50] = 0x4B,  // Left Arrow (E0 prefix)
    [0x51] = 0x50,  // Down Arrow (E0 prefix)
    [0x52] = 0x48,  // Up Arrow (E0 prefix)
    [0x53] = 0x45,  // Num Lock
    [0x54] = 0x35,  // Keypad /  (E0 prefix)
    [0x55] = 0x37,  // Keypad *
    [0x56] = 0x4A,  // Keypad -
    [0x57] = 0x4E,  // Keypad +
    [0x58] = 0x1C,  // Keypad Enter (E0 prefix)
    [0x59] = 0x4F,  // Keypad 1 End
    [0x5A] = 0x50,  // Keypad 2 Down
    [0x5B] = 0x51,  // Keypad 3 PgDn
    [0x5C] = 0x4B,  // Keypad 4 Left
    [0x5D] = 0x4C,  // Keypad 5
    [0x5E] = 0x4D,  // Keypad 6 Right
    [0x5F] = 0x47,  // Keypad 7 Home
    [0x60] = 0x48,  // Keypad 8 Up
    [0x61] = 0x49,  // Keypad 9 PgUp
    [0x62] = 0x52,  // Keypad 0 Ins
    [0x63] = 0x53,  // Keypad . Del
    [0x64] = 0x56,  // Non-US \ |
    [0x65] = 0x5D,  // Application (E0 prefix) - context menu key
    // Modifier keys (handled separately via modifier byte)
    [0xE0] = 0x1D,  // Left Control
    [0xE1] = 0x2A,  // Left Shift
    [0xE2] = 0x38,  // Left Alt
    [0xE3] = 0x5B,  // Left GUI (Windows) - E0 prefix - THIS IS META
    [0xE4] = 0x1D,  // Right Control (E0 prefix)
    [0xE5] = 0x36,  // Right Shift
    [0xE6] = 0x38,  // Right Alt (E0 prefix)
    [0xE7] = 0x5C,  // Right GUI (Windows) - E0 prefix
};

// Keys that need E0 prefix
static const uint8_t usb_extended_keys[] = {
    0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,  // Ins, Home, PgUp, Del, End, PgDn
    0x4F, 0x50, 0x51, 0x52,              // Arrow keys
    0x54, 0x58,                           // Keypad / and Enter
    0x65,                                 // Application key
    0xE3, 0xE4, 0xE6, 0xE7,              // Right Ctrl, Right Alt, GUI keys
};

static bool needs_e0_prefix(uint8_t usb_code) {
    for (size_t i = 0; i < sizeof(usb_extended_keys); i++) {
        if (usb_extended_keys[i] == usb_code) return true;
    }
    return false;
}

// State tracking
static uint8_t prev_keys[6] = {0};
static uint8_t prev_modifiers = 0;
static usb_hid_keyboard_cb_t keyboard_cb = NULL;
static usb_hid_mouse_cb_t mouse_cb = NULL;

// Send PS/2 scancode via callback
static void send_ps2_key(uint8_t usb_code, bool is_break) {
    if (!keyboard_cb) return;

    uint8_t ps2_code = usb_to_ps2_base[usb_code];
    if (ps2_code == 0) return;

    if (needs_e0_prefix(usb_code)) {
        keyboard_cb(0xE0, false);  // E0 prefix (not a break code)
    }

    if (is_break) {
        keyboard_cb(ps2_code | 0x80, true);  // Break code
    } else {
        keyboard_cb(ps2_code, false);  // Make code
    }
}

// Process modifier byte changes
static void process_modifiers(uint8_t old_mods, uint8_t new_mods) {
    // Bit 0: Left Ctrl, Bit 1: Left Shift, Bit 2: Left Alt, Bit 3: Left GUI
    // Bit 4: Right Ctrl, Bit 5: Right Shift, Bit 6: Right Alt, Bit 7: Right GUI
    static const uint8_t mod_usb_codes[] = {0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7};

    for (int i = 0; i < 8; i++) {
        bool was_pressed = (old_mods >> i) & 1;
        bool is_pressed = (new_mods >> i) & 1;
        if (was_pressed != is_pressed) {
            send_ps2_key(mod_usb_codes[i], !is_pressed);
        }
    }
}

// Check if META key is pressed (for OSD toggle)
bool usb_hid_is_meta_pressed(uint8_t modifiers) {
    // Left GUI (bit 3) or Right GUI (bit 7)
    return (modifiers & 0x88) != 0;
}

// Process keyboard report
void usb_hid_process_keyboard(uint8_t *report) {
    uint8_t modifiers = report[0];
    // report[1] is reserved
    uint8_t *keys = &report[2];

    // Process modifier changes
    process_modifiers(prev_modifiers, modifiers);
    prev_modifiers = modifiers;

    // Find released keys
    for (int i = 0; i < 6; i++) {
        uint8_t key = prev_keys[i];
        if (key == 0) continue;
        bool still_pressed = false;
        for (int j = 0; j < 6; j++) {
            if (keys[j] == key) { still_pressed = true; break; }
        }
        if (!still_pressed) {
            send_ps2_key(key, true);  // Key released
        }
    }

    // Find newly pressed keys
    for (int i = 0; i < 6; i++) {
        uint8_t key = keys[i];
        if (key == 0) continue;
        bool was_pressed = false;
        for (int j = 0; j < 6; j++) {
            if (prev_keys[j] == key) { was_pressed = true; break; }
        }
        if (!was_pressed) {
            send_ps2_key(key, false);  // Key pressed
        }
    }

    memcpy(prev_keys, keys, 6);
}

// Process mouse report
void usb_hid_process_mouse(uint8_t *report, int len) {
    if (!mouse_cb || len < 3) return;

    uint8_t buttons = report[0];
    int8_t dx = (int8_t)report[1];
    int8_t dy = (int8_t)report[2];
    int8_t wheel = (len > 3) ? (int8_t)report[3] : 0;

    mouse_cb(buttons, dx, dy, wheel);
}

void usb_hid_set_keyboard_callback(usb_hid_keyboard_cb_t cb) {
    keyboard_cb = cb;
}

void usb_hid_set_mouse_callback(usb_hid_mouse_cb_t cb) {
    mouse_cb = cb;
}
```

**Step 3: Integration with input_bsp.c**
```c
// Add to input_bsp.c

#include "usb_hid.h"

// Callback from USB HID - inject PS/2 scancode
static void usb_keyboard_event(uint8_t scancode, bool is_break) {
    // Check for META key (GUI/Windows key) to toggle OSD
    // This is handled at USB level before translation
    // ... existing OSD toggle logic ...

    // Queue scancode for i8042
    if (globals.i8042) {
        i8042_queue_scancode(globals.i8042, scancode);
    }
}

static void usb_mouse_event(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel) {
    // Convert to PS/2 mouse packet format
    if (globals.i8042) {
        // PS/2 mouse packet: [status][dx][dy]
        uint8_t status = 0x08;  // Always set bit 3
        status |= (buttons & 0x01);        // Left button
        status |= (buttons & 0x02);        // Right button
        status |= (buttons & 0x04) >> 1;   // Middle button to bit 2
        if (dx < 0) status |= 0x10;        // X sign bit
        if (dy < 0) status |= 0x20;        // Y sign bit

        i8042_queue_mouse_packet(globals.i8042, status, (uint8_t)dx, (uint8_t)(-dy));
    }
}

// In input_init():
#if CONFIG_USB_HOST_HID_ENABLED
    usb_hid_set_keyboard_callback(usb_keyboard_event);
    usb_hid_set_mouse_callback(usb_mouse_event);
#endif
```

**Dependencies**: USB Host Framework (11.1)

---

### 11.3 Toast Notification System

**Goal**: Display brief text notifications for non-visible confirmations.

**Files to Modify:**
| File | Changes |
|------|---------|
| `esp/main/common.h` | Add toast state to globals |
| `esp/main/lcd_bsp.c` | Render toast overlay |
| `esp/main/tanmatsu_osd.c` | Toast trigger function |
| **NEW** `esp/main/toast.h` | Toast API |

**Step 1: Toast Header** (`esp/main/toast.h`)
```c
#pragma once

#define TOAST_MAX_LEN 48
#define TOAST_DURATION_MS 2000

// Show a toast notification
void toast_show(const char *message);

// Clear current toast (if any)
void toast_clear(void);

// Check if toast is currently visible
bool toast_is_visible(void);

// Get current toast message (for rendering)
const char* toast_get_message(void);
```

**Step 2: Add to globals** (`esp/main/common.h`)
```c
// Add to globals struct:
struct {
    char toast_message[TOAST_MAX_LEN];
    int64_t toast_start_time;  // esp_timer_get_time() when shown
    bool toast_active;
} toast;
```

**Step 3: Toast Implementation** (add to `esp/main/tanmatsu_osd.c` or new file)
```c
#include "toast.h"
#include "esp_timer.h"
#include <string.h>

void toast_show(const char *message) {
    strncpy(globals.toast.toast_message, message, TOAST_MAX_LEN - 1);
    globals.toast.toast_message[TOAST_MAX_LEN - 1] = '\0';
    globals.toast.toast_start_time = esp_timer_get_time();
    globals.toast.toast_active = true;
}

void toast_clear(void) {
    globals.toast.toast_active = false;
}

bool toast_is_visible(void) {
    if (!globals.toast.toast_active) return false;

    int64_t elapsed = esp_timer_get_time() - globals.toast.toast_start_time;
    if (elapsed > TOAST_DURATION_MS * 1000) {
        globals.toast.toast_active = false;
        return false;
    }
    return true;
}

const char* toast_get_message(void) {
    return globals.toast.toast_message;
}
```

**Step 4: Render Toast** (add to `esp/main/lcd_bsp.c` in render loop)
```c
// After VGA framebuffer copy, before final display:
if (toast_is_visible() && !globals.osd_enabled) {
    const char *msg = toast_get_message();
    int len = strlen(msg);
    int x = (DISPLAY_WIDTH - len * 8) / 2;  // Center horizontally
    int y = DISPLAY_HEIGHT - 32;            // Near bottom

    // Draw semi-transparent background
    draw_rect_alpha(fb, x - 8, y - 4, len * 8 + 16, 24, 0x0000, 128);

    // Draw text
    draw_text(fb, x, y, msg, 0xFFFF);  // White text
}
```

**Step 5: Usage Examples**
```c
// In tanmatsu_osd.c SAVE_INI handler:
if (save_ini_file() == ESP_OK) {
    toast_show("Settings saved");
} else {
    toast_show("Save failed!");
}

// In usb_host.c on device connect:
toast_show("USB drive attached as D:");

// In usb_host.c on device disconnect:
toast_show("USB drive removed");
```

**Dependencies**: None (can be implemented standalone)

---

### 11.4 Drive Activity LEDs

**Goal**: Flash LEDs 4 and 5 on HDD/Floppy access.

**Files to Modify:**
| File | Changes |
|------|---------|
| `ide.c` | Add activity callback hook |
| `emulink.c` | Add activity callback for floppy |
| `esp/main/common.h` | Add LED state |
| **NEW** `esp/main/led_activity.c` | LED management |
| **NEW** `esp/main/led_activity.h` | LED API |

**Step 1: LED Activity Header** (`esp/main/led_activity.h`)
```c
#pragma once
#include <stdbool.h>

// Initialize LED activity subsystem
void led_activity_init(void);

// Signal drive activity (call from IDE/floppy code)
void led_activity_hdd(void);    // Flash LED 4 (red)
void led_activity_floppy(void); // Flash LED 5 (green)

// Must be called periodically to manage LED timeouts
void led_activity_tick(void);
```

**Step 2: LED Activity Implementation** (`esp/main/led_activity.c`)
```c
#include "led_activity.h"
#include "bsp/led.h"
#include "esp_timer.h"

#define LED_HDD_INDEX    4
#define LED_FLOPPY_INDEX 5
#define LED_TIMEOUT_US   100000  // 100ms

static int64_t hdd_last_activity = 0;
static int64_t floppy_last_activity = 0;
static bool hdd_led_on = false;
static bool floppy_led_on = false;

void led_activity_init(void) {
    // Take manual control of LEDs 4-5
    bsp_led_set_mode(false);

    // Ensure LEDs are off initially
    bsp_led_set_pixel_rgb(LED_HDD_INDEX, 0, 0, 0);
    bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 0, 0);
    bsp_led_send();
}

void led_activity_hdd(void) {
    hdd_last_activity = esp_timer_get_time();
    if (!hdd_led_on) {
        bsp_led_set_pixel_rgb(LED_HDD_INDEX, 255, 0, 0);  // Red
        bsp_led_send();
        hdd_led_on = true;
    }
}

void led_activity_floppy(void) {
    floppy_last_activity = esp_timer_get_time();
    if (!floppy_led_on) {
        bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 255, 0);  // Green
        bsp_led_send();
        floppy_led_on = true;
    }
}

void led_activity_tick(void) {
    int64_t now = esp_timer_get_time();
    bool need_update = false;

    if (hdd_led_on && (now - hdd_last_activity > LED_TIMEOUT_US)) {
        bsp_led_set_pixel_rgb(LED_HDD_INDEX, 0, 0, 0);
        hdd_led_on = false;
        need_update = true;
    }

    if (floppy_led_on && (now - floppy_last_activity > LED_TIMEOUT_US)) {
        bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 0, 0);
        floppy_led_on = false;
        need_update = true;
    }

    if (need_update) {
        bsp_led_send();
    }
}
```

**Step 3: Hook into IDE** (`ide.c`)
```c
// Add near top of file:
#ifdef TANMATSU_BUILD
extern void led_activity_hdd(void);
#define IDE_ACTIVITY() led_activity_hdd()
#else
#define IDE_ACTIVITY() ((void)0)
#endif

// In ide_read_async and ide_write_async, add at start:
IDE_ACTIVITY();
```

**Step 4: Hook into Floppy** (`emulink.c`)
```c
// Similar pattern for floppy operations:
#ifdef TANMATSU_BUILD
extern void led_activity_floppy(void);
#define FLOPPY_ACTIVITY() led_activity_floppy()
#else
#define FLOPPY_ACTIVITY() ((void)0)
#endif
```

**Step 5: Tick from main loop** (`esp/main/esp_main.c`)
```c
// In emulator main loop:
led_activity_tick();  // Call every frame or every few ms
```

**Dependencies**: None

---

### 11.5 OSD Stats Panel

**Goal**: Display CPU%, VGA mode, batch size in OSD panel.

**Files to Modify:**
| File | Changes |
|------|---------|
| `pc.c` | Write stats to memory instead of fprintf |
| `esp/main/common.h` | Add stats struct to globals |
| `esp/main/tanmatsu_osd.c` | Add VIEW_STATUS and rendering |

**Step 1: Stats Structure** (`esp/main/common.h`)
```c
// Add to globals struct:
struct {
    uint8_t cpu_percent;        // 0-100
    uint8_t periph_percent;     // 0-100
    uint16_t batch_size;        // Current batch size
    uint32_t calls_per_sec;     // pc_step calls per second
    char vga_mode[24];          // e.g., "640x480x16 VGA"
} emu_stats;
```

**Step 2: Update pc.c**
Replace fprintf stats output with memory writes:
```c
// In pc_step(), replace the fprintf block (~line 602-607) with:
#ifdef TANMATSU_BUILD
    extern struct globals globals;
    if (stat_calls > 0) {
        uint32_t total = stat_cpu_us + stat_periph_us;
        if (total > 0) {
            globals.emu_stats.cpu_percent = (stat_cpu_us * 100) / total;
            globals.emu_stats.periph_percent = (stat_periph_us * 100) / total;
        }
        globals.emu_stats.batch_size = batch_size;
        globals.emu_stats.calls_per_sec = stat_calls * 1000000 / stat_total_us;
    }
#else
    fprintf(stderr, "pc_step stats: ..."); // Keep for non-Tanmatsu builds
#endif
```

**Step 3: VGA Mode String** (`vga.c`)
```c
// Add function to get mode string:
void vga_get_mode_string(VGAState *s, char *buf, size_t len) {
    int w = vga_get_width(s);
    int h = vga_get_height(s);
    int bpp = vga_get_bpp(s);
    const char *type = s->vbe_enabled ? "VESA" : "VGA";
    snprintf(buf, len, "%dx%dx%d %s", w, h, bpp, type);
}
```

**Step 4: OSD View** (`esp/main/tanmatsu_osd.c`)
```c
// Add to ViewMode enum:
VIEW_STATUS,

// Add menu item:
MAIN_STATUS,

// Add to main menu items array:
{"STATUS", MAIN_STATUS},

// Handle selection:
case MAIN_STATUS:
    osd->view = VIEW_STATUS;
    break;

// Render status view:
static void render_status(OSD *osd, uint8_t *fb, int stride) {
    char line[64];

    draw_text(fb, 16, 40, "SYSTEM STATUS", 0xFFFF, stride);

    snprintf(line, sizeof(line), "VGA Mode: %s", globals.emu_stats.vga_mode);
    draw_text(fb, 16, 80, line, 0xFFFF, stride);

    snprintf(line, sizeof(line), "CPU: %d%%  Periph: %d%%",
             globals.emu_stats.cpu_percent,
             globals.emu_stats.periph_percent);
    draw_text(fb, 16, 104, line, 0xFFFF, stride);

    snprintf(line, sizeof(line), "Batch: %d  Calls/s: %lu",
             globals.emu_stats.batch_size,
             (unsigned long)globals.emu_stats.calls_per_sec);
    draw_text(fb, 16, 128, line, 0xFFFF, stride);

    draw_text(fb, 16, 180, "[ESC] Back", 0x8410, stride);
}
```

**Dependencies**: None

---

### 11.6 OSD Help Screen

**Goal**: Show keyboard shortcuts in OSD.

**Files to Modify:**
| File | Changes |
|------|---------|
| `esp/main/tanmatsu_osd.c` | Add VIEW_HELP and content |

**Implementation:**
```c
// Add to ViewMode enum:
VIEW_HELP,

// Add menu item:
MAIN_HELP,

// Add to main menu items array:
{"HELP", MAIN_HELP},

// Handle selection:
case MAIN_HELP:
    osd->view = VIEW_HELP;
    break;

// Render help view:
static void render_help(OSD *osd, uint8_t *fb, int stride) {
    draw_text(fb, 16, 40, "KEYBOARD SHORTCUTS", 0xFFFF, stride);

    int y = 80;
    draw_text(fb, 16, y, "META           Toggle OSD", 0xFFFF, stride); y += 20;
    draw_text(fb, 16, y, "META+UP/DOWN   Volume", 0xFFFF, stride); y += 20;
    draw_text(fb, 16, y, "META+LEFT/RIGHT  Brightness", 0xFFFF, stride); y += 20;
    y += 20;
    draw_text(fb, 16, y, "IN OSD:", 0x8410, stride); y += 20;
    draw_text(fb, 16, y, "Arrows         Navigate", 0xFFFF, stride); y += 20;
    draw_text(fb, 16, y, "Enter          Select", 0xFFFF, stride); y += 20;
    draw_text(fb, 16, y, "Escape         Back/Close", 0xFFFF, stride); y += 20;

    draw_text(fb, 16, 300, "[ESC] Back", 0x8410, stride);
}
```

**Dependencies**: None

---

### 11.7 USB Mass Storage

**Goal**: Expose USB storage as IDE secondary slave, always present but empty when no USB connected.

**Design Rationale**:
- Drive slot 3 (secondary slave) is **permanently** allocated to USB storage
- CDD (second CD-ROM) is no longer available
- BlockDeviceUSB is **always attached at boot** - no hot-attach needed
- When USB disconnected: reports 0 sectors, reads return zeros, writes silently ignored
- When USB connected: normal operation
- No special drivers needed in DOS/Windows - just a standard IDE HDD
- Reboot required to detect USB drive plugged in after boot (DOS limitation)

**Files to Modify:**
| File | Changes |
|------|---------|
| `ide.c` | Add BlockDeviceUSB with disconnect-safe behavior |
| `esp/main/usb_host.c` | MSC driver, update BlockDevice handle |
| `esp/main/common.h` | USB storage state |
| `esp/main/tanmatsu_osd.c` | Show "USB STORAGE" instead of CDD |

**Step 1: BlockDeviceUSB Structure** (`esp/main/usb_storage.c`)
```c
#include "usb_storage.h"
#include "usb/usb_host.h"
#include "usb_host_msc.h"
#include <string.h>

typedef struct {
    BlockDevice base;
    msc_host_device_handle_t msc_handle;  // NULL when disconnected
    uint32_t block_size;
    uint64_t block_count;
} BlockDeviceUSB;

static BlockDeviceUSB usb_block_device = {0};

static int64_t bd_usb_get_sector_count(BlockDevice *bs) {
    BlockDeviceUSB *s = (BlockDeviceUSB *)bs;
    if (!s->msc_handle) {
        return 0;  // No USB connected - report empty
    }
    return s->block_count;
}

static int bd_usb_read_async(BlockDevice *bs, uint64_t sector,
                              uint8_t *buf, int n,
                              BlockDeviceCompletionFunc *cb, void *opaque) {
    BlockDeviceUSB *s = (BlockDeviceUSB *)bs;

    if (!s->msc_handle) {
        // No USB connected - return zeros silently
        memset(buf, 0, n * 512);
        if (cb) cb(opaque, 0);
        return 0;
    }

    // Normal USB read
    esp_err_t err = msc_host_read_sector(s->msc_handle, sector, buf, n);
    if (err == ESP_OK) {
        if (cb) cb(opaque, 0);
        return 0;
    }
    return -1;
}

static int bd_usb_write_async(BlockDevice *bs, uint64_t sector,
                               uint8_t *buf, int n,
                               BlockDeviceCompletionFunc *cb, void *opaque) {
    BlockDeviceUSB *s = (BlockDeviceUSB *)bs;

    if (!s->msc_handle) {
        // No USB connected - silently discard write
        if (cb) cb(opaque, 0);
        return 0;
    }

    // Normal USB write
    esp_err_t err = msc_host_write_sector(s->msc_handle, sector, buf, n);
    if (err == ESP_OK) {
        if (cb) cb(opaque, 0);
        return 0;
    }
    return -1;
}

// Called once at boot to get the BlockDevice for IDE attachment
BlockDevice *usb_storage_get_block_device(void) {
    usb_block_device.base.get_sector_count = bd_usb_get_sector_count;
    usb_block_device.base.read_async = bd_usb_read_async;
    usb_block_device.base.write_async = bd_usb_write_async;
    usb_block_device.msc_handle = NULL;  // Initially disconnected
    return (BlockDevice *)&usb_block_device;
}

// Called when USB storage connects
void usb_storage_connect(msc_host_device_handle_t handle) {
    msc_host_device_info_t info;
    msc_host_get_device_info(handle, &info);

    usb_block_device.msc_handle = handle;
    usb_block_device.block_size = info.sector_size;
    usb_block_device.block_count = info.sector_count;

    globals.usb_storage_attached = true;
    toast_show("USB drive connected");
}

// Called when USB storage disconnects
void usb_storage_disconnect(void) {
    usb_block_device.msc_handle = NULL;
    usb_block_device.block_count = 0;

    globals.usb_storage_attached = false;
    toast_show("USB drive removed");
}
```

**Step 2: Attach at Boot** (`esp/main/esp_main.c` or `pc.c`)
```c
// During PC initialization, after IDE controller setup:
// Attach USB BlockDevice to secondary slave (slot 3)
// This happens ONCE at boot, not on USB connect/disconnect

BlockDevice *usb_bd = usb_storage_get_block_device();
ide_attach_block_device(pc->ide2, 1, usb_bd);  // drive 1 = slave
```

**Step 3: USB MSC Event Handler** (`esp/main/usb_host.c`)
```c
// Simplified - no hot-attach/detach of IDE, just update the handle
static void msc_event_handler(const msc_host_event_t *event, void *arg) {
    switch (event->event) {
        case MSC_DEVICE_CONNECTED:
            ESP_LOGI(TAG, "USB storage connected");
            usb_storage_connect(event->device.handle);
            break;

        case MSC_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "USB storage disconnected");
            usb_storage_disconnect();
            break;
    }
}
```

**Step 4: OSD Integration** (`esp/main/tanmatsu_osd.c`)
```c
// Replace CDD menu item with USB status
// In render_mounting():
// Instead of "CDD: (empty)" show:

if (globals.usb_storage_attached) {
    draw_text(fb, x, y, "USB: Connected", 0x07E0, stride);  // Green
} else {
    draw_text(fb, x, y, "USB: (empty)", 0x8410, stride);    // Gray
}

// CDD mount option is removed entirely - slot is USB only
// Remove MOUNT_CDD from menu navigation
```

**Behavior Summary:**

| USB State | get_sector_count | read() | write() | DOS sees |
|-----------|------------------|--------|---------|----------|
| Connected | Actual size | Normal data | Normal write | Working D: drive |
| Disconnected | 0 | Returns zeros | Silently ignored | Empty/unformatted D: |
| Unplugged mid-use | 0 | Returns zeros | Silently ignored | Read errors / corrupt data |

**Note**: For best results, users should connect USB before boot. Hot-plugging after DOS has booted won't be detected until reboot.

**Dependencies**: USB Host Framework (11.1), Toast System (11.3)

---

*Document maintained in branch `claude/review-esp32-p4-el3dg`*
