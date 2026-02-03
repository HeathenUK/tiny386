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

### Priority Matrix

| Feature | Difficulty | Impact | Memory | Priority |
|---------|------------|--------|--------|----------|
| OSD Status Panel | Low | Medium | ~1KB | **P1** |
| OSD Screenshot | Low | Medium | ~2KB | **P1** |
| OSD Help Screen | Low | Low | ~1KB | **P2** |
| USB HID Keyboard | Medium | High | ~30KB | **P1** |
| USB HID Mouse | Medium | High | Included | **P1** |
| USB Mass Storage | Medium | High | ~40KB | **P2** |
| OPL3 Upgrade | Medium | Medium | ~20KB | **P3** |
| MPU-401 MIDI | Medium | Medium | ~2KB | **P3** |
| GUS Emulation | High | Medium | ~1.1MB | **P4** |

### Suggested Implementation Order

**Phase 1: Quick Wins + USB HID**
1. OSD Status Panel
2. OSD Screenshot
3. USB Host framework
4. USB HID Keyboard
5. USB HID Mouse

**Phase 2: Storage + Audio**
1. USB Mass Storage
2. OSD USB drive integration
3. OPL3 upgrade (optional)

**Phase 3: Advanced Audio**
1. MPU-401 MIDI interface
2. GUS emulation (if demand exists)

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

**Next Steps**:
- Continue discussing remaining topics (sound, USB)
- Finalize overall implementation plan
- Begin implementation based on priorities

**Key Files Examined**:
- `esp/main/tanmatsu_osd.c` - OSD implementation
- `esp/main/input_bsp.c` - Input handling
- `esp/main/lcd_bsp.c` - Display with PPA rotation
- `esp/main/i2s.c` - Audio output
- `pc.c` - Mixer and peripheral integration
- `sb16.c`, `adlib.c`, `fmopl.c` - Current audio
- PR #4 - USB HID work in progress

---

*Document maintained in branch `claude/review-esp32-p4-el3dg`*
