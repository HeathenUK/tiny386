# Performance Optimization Opportunities

## Critical Issues Found

### 1. **Display Refresh Inefficiency** ⚠️ HIGH IMPACT

**Location**: `main.c:1273-1288` (redraw function)

**Problem**:
- Updates display in 32 chunks (`NN = 32`)
- **900µs sleep between each chunk** = 28.8ms overhead per refresh!
- Calls `vga_step()` during display refresh (inefficient)
- Unnecessary `memcpy` from main framebuffer to temp buffer

**Current code**:
```c
for (int i = 0; i < NN; i++) {
    uint16_t *src = s->fb;
    src += 240 * 320 / NN * i;
    memcpy(s->fb1, src, 240 * 320 / NN * 2);  // Unnecessary copy
    esp_lcd_panel_draw_bitmap(...);
    vga_step(s->pc->vga);  // Called during refresh!
    usleep(900);  // 28.8ms total overhead!
}
```

**Optimizations**:
1. **Reduce chunk count**: Use 4-8 chunks instead of 32
2. **Remove sleep delays**: Use DMA completion callbacks instead
3. **Remove vga_step call**: Already called in vga_task
4. **Direct framebuffer access**: Use fb1 as double buffer, avoid memcpy
5. **Partial updates**: Only refresh changed regions

**Expected improvement**: 20-30ms per refresh → 5-10ms per refresh

### 2. **Low CPU Step Count** ⚠️ MEDIUM IMPACT

**Location**: `main.c:617`

**Problem**:
- ESP32 only executes **512 instructions per step** vs 10240 on desktop
- This causes excessive overhead from peripheral updates
- More frequent task switching

**Current code**:
```c
#ifdef BUILD_ESP32
    cpui386_step(pc->cpu, 512);  // Very low!
#else
    cpui386_step(pc->cpu, 10240);
#endif
```

**Optimization**:
- Increase to **2048-4096** instructions per step
- Test stability and adjust based on interrupt latency requirements

**Expected improvement**: 2-4x reduction in emulation overhead

### 3. **VGA Task Frequency** ⚠️ MEDIUM IMPACT

**Location**: `esp_main.c:233`

**Problem**:
- VGA task runs every **10ms** regardless of refresh needs
- Wakes up even when no refresh is needed

**Current code**:
```c
while (1) {
    if (thepc)
        pc_vga_step(thepc);
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Always 10ms
}
```

**Optimization**:
- Use adaptive delay based on refresh status
- Delay longer when no refresh needed (30-50ms)
- Shorter delay when refresh pending (5ms)

**Expected improvement**: 50-70% reduction in CPU usage when idle

### 4. **Memory Access Patterns** ⚠️ LOW-MEDIUM IMPACT

**Locations**: `vga.c`, `i386.c`

**Problems**:
- VGA memory writes already have `IRAM_ATTR` (good)
- Some hot paths could benefit from cache alignment
- Framebuffer is in PSRAM (slower than IRAM)

**Optimizations**:
1. **Cache-aligned framebuffer chunks**: Use `MALLOC_CAP_8BIT | MALLOC_CAP_DMA`
2. **IRAM for critical paths**: Already done for some functions
3. **Prefetch**: Use prefetch hints for memory access

### 5. **Full Screen Updates** ⚠️ LOW IMPACT

**Location**: `vga.c:1137-1165`

**Problem**:
- `full_update` flag causes entire screen refresh
- Could use dirty rectangle tracking

**Optimization**:
- Implement dirty rectangle tracking
- Only refresh changed regions
- Partial updates for text mode changes

## Recommended Implementation Order

### Phase 1: Quick Wins (Immediate Impact)

1. **Fix display refresh** (Highest priority)
   ```c
   // Reduce chunks, remove sleep, remove vga_step call
   #define NN 4  // Instead of 32
   // Remove usleep(900)
   // Remove vga_step call from redraw()
   ```

2. **Increase CPU step count**
   ```c
   cpui386_step(pc->cpu, 2048);  // 4x increase
   ```

3. **Optimize VGA task delay**
   ```c
   int delay = refresh_needed ? 5 : 30;
   vTaskDelay(delay / portTICK_PERIOD_MS);
   ```

### Phase 2: Medium-Term Optimizations

4. **Eliminate memcpy in display refresh**
   - Use double buffering
   - Swap buffers instead of copying

5. **Partial screen updates**
   - Track dirty regions
   - Only update changed areas

6. **Cache optimization**
   - Align framebuffer chunks
   - Use DMA-capable memory

### Phase 3: Advanced Optimizations

7. **Interrupt-driven display**
   - Use DMA completion callbacks
   - Eliminate polling delays

8. **CPU instruction batching**
   - Batch memory operations
   - Reduce function call overhead

## Code Changes Needed

### Change 1: Optimize Display Refresh

**File**: `main.c`

```c
#define NN 4  // Reduced from 32

static void redraw(void *opaque, int x, int y, int w, int h)
{
    Console *s = opaque;
    if (thepanel) {
        // Update in fewer, larger chunks
        for (int i = 0; i < NN; i++) {
            uint16_t *src = s->fb;
            src += 240 * 320 / NN * i;
            // Direct copy - remove if using double buffering
            memcpy(s->fb1, src, 240 * 320 / NN * 2);
            ESP_ERROR_CHECK(
                esp_lcd_panel_draw_bitmap(
                    thepanel,
                    0, 320 / NN * i,
                    240, 320 / NN * (i + 1),
                    s->fb1));
            // REMOVED: vga_step(s->pc->vga);  // Already called in vga_task
            // REMOVED: usleep(900);  // Use DMA completion instead
        }
    }
}
```

### Change 2: Increase CPU Step Count

**File**: `main.c:617`

```c
#ifdef BUILD_ESP32
    cpui386_step(pc->cpu, 2048);  // Increased from 512
#else
    cpui386_step(pc->cpu, 10240);
#endif
```

### Change 3: Adaptive VGA Task Delay

**File**: `esp_main.c:233`

```c
static void vga_task(void *arg)
{
    // ... initialization ...
    
    int last_refresh = 0;
    while (1) {
        if (thepc) {
            int refresh = pc_vga_step(thepc);
            if (refresh) {
                last_refresh = 1;
            }
        }
        
        // Adaptive delay: shorter when refresh needed
        int delay = last_refresh ? 5 : 30;
        last_refresh = 0;
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}
```

## Performance Monitoring

Add performance counters to measure:
- CPU emulation cycles per second
- Display refresh frequency
- VGA task CPU usage
- Memory bandwidth

## Expected Overall Improvement

- **Display refresh**: 28ms → 5ms (82% improvement)
- **CPU emulation**: 2-4x faster execution
- **Idle CPU usage**: 50-70% reduction
- **Overall FPS**: 2-3x improvement in typical workloads

## Testing Recommendations

1. Test with different CPU step counts (1024, 2048, 4096)
2. Measure display refresh time before/after
3. Monitor task CPU usage with FreeRTOS stats
4. Verify stability with real workloads (Windows boot, etc.)
