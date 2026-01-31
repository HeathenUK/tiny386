set(COMMON_SRCS
    "esp_main.c"
    "../../adlib.c"
    "../../fmopl.c"
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

# Board-specific sources based on DEVICE variable
if(DEVICE STREQUAL "tanmatsu")
    # Tanmatsu build with badge-bsp (physical keyboard, no network for now)
    set(BOARD_SRCS "lcd_bsp.c" "input_bsp.c")
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

idf_component_register(SRCS ${COMMON_SRCS} ${BOARD_SRCS}
                       INCLUDE_DIRS "")

add_definitions(-include ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD_HEADER})


set_source_files_properties("../../adlib.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("../../fmopl.c" PROPERTIES COMPILE_FLAGS "-Wno-all -fsingle-precision-constant")
set_source_files_properties("../../fpu.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("../../i386.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../i8042.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("../../vga.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../pc.c" PROPERTIES COMPILE_FLAGS "-Wno-all -O3")
set_source_files_properties("../../ne2000.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("../../sb16.c" PROPERTIES COMPILE_FLAGS "-Wno-all")
set_source_files_properties("../../i8257.c" PROPERTIES COMPILE_FLAGS "-Wno-all")

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../fmopl.inc")
    message(FATAL_ERROR "Please run `make prepare` first")
endif()
