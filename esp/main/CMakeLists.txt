set(COMMON_SRCS
    "esp_main.c"
    "../../adlib.c"
    "../../fpu.c"
    "../../i386.c"
    "../../i8042.c"
    "../../i8254.c"
    "../../i8259.c"
    "../../ide.c"
    "../../ini.c"
    "../../pc.c"
    "../../misc.c"
    "../../pci.c"
    "../../vga.c"
    "../../ne2000.c"
    "../../i8257.c"
    "../../sb16.c"
    "../../pcspk.c"
    "i2s.c"
    "storage.c"
)

# ymfm OPL emulation (replaces fmopl.c)
set(YMFM_SRCS
    "../../ymfm_wrapper.cpp"
    "../../thirdparty/ymfm/src/ymfm_opl.cpp"
    "../../thirdparty/ymfm/src/ymfm_adpcm.cpp"
    "../../thirdparty/ymfm/src/ymfm_pcm.cpp"
    "../../thirdparty/ymfm/src/ymfm_ssg.cpp"
)

# Board-specific sources based on DEVICE variable
if(DEVICE STREQUAL "tanmatsu")
    # Tanmatsu build with badge-bsp (physical keyboard, custom OSD)
    # Include PIE assembly for ESP32-P4 SIMD optimization
    set(BOARD_SRCS "lcd_bsp.c" "input_bsp.c" "tanmatsu_osd.c" "toast.c" "led_activity.c" "mouse_emu.c" "vga_pie.S" "usb_host.c")
    set(BOARD_HEADER "board_tanmatsu.h")
elseif(DEVICE STREQUAL "jc4880p433")
    # JC4880P433 build with direct hardware access
    set(BOARD_SRCS "lcd_st7701.c" "wifi.c")
    set(BOARD_HEADER "board_jc4880p433.h")
elseif(DEVICE STREQUAL "jc3248w535")
    # ESP32-S3 build
    set(BOARD_SRCS "lcd_axs15231b.c" "wifi.c")
    set(BOARD_HEADER "board_jc3248w535.h")
else()
    # Fallback based on target
    if(CONFIG_IDF_TARGET_ESP32P4)
        set(BOARD_SRCS "lcd_st7701.c" "wifi.c")
        set(BOARD_HEADER "board_jc4880p433.h")
    else()
        set(BOARD_SRCS "lcd_axs15231b.c" "wifi.c")
        set(BOARD_HEADER "board_jc3248w535.h")
    endif()
endif()

idf_component_register(SRCS ${COMMON_SRCS} ${YMFM_SRCS} ${BOARD_SRCS}
                       INCLUDE_DIRS "." "../../")

add_definitions(-include ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD_HEADER})


# Performance-critical emulation core: -O3 with aggressive optimizations
set_source_files_properties("../../i386.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -funroll-loops")
set_source_files_properties("../../fpu.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -ffast-math -funroll-loops")
set_source_files_properties("../../pc.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../i8254.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../i8259.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")

# Audio: -O3 for OPL/SB16 performance
set_source_files_properties("../../adlib.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../sb16.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../pcspk.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")

# ymfm OPL emulation: -O3 with C++ flags
set_source_files_properties("../../ymfm_wrapper.cpp" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -fno-exceptions -fno-rtti")
set_source_files_properties("../../thirdparty/ymfm/src/ymfm_opl.cpp" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -fno-exceptions -fno-rtti")
set_source_files_properties("../../thirdparty/ymfm/src/ymfm_adpcm.cpp" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -fno-exceptions -fno-rtti")
set_source_files_properties("../../thirdparty/ymfm/src/ymfm_pcm.cpp" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -fno-exceptions -fno-rtti")
set_source_files_properties("../../thirdparty/ymfm/src/ymfm_ssg.cpp" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -fno-exceptions -fno-rtti")

# Graphics: -O3 with fast-math for VGA rendering
set_source_files_properties("../../vga.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3 -funroll-loops -ffast-math")

# I/O subsystems: -O3 for DMA/IDE performance
set_source_files_properties("../../i8257.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../ide.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")

# Other peripherals (less critical, still benefit from -O3)
set_source_files_properties("../../i8042.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../ne2000.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../misc.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../pci.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")

# UI/OSD (not performance critical)
set_source_files_properties("../../osd/osd.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("../../osd/microui.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("tanmatsu_osd.c" PROPERTIES COMPILE_FLAGS "-Wno-all")

# ymfm has no lookup tables to generate, no preparation needed
