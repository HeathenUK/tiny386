# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Device selection
if(DEVICE STREQUAL "")
	set(DEVICE "$ENV{TARGET}")
endif()

if(DEVICE STREQUAL "")
	# Default to tanmatsu if not specified
	set(DEVICE "tanmatsu")
	message(STATUS "DEVICE not set, defaulting to tanmatsu")
endif()

# Device configurations
if(DEVICE STREQUAL "tanmatsu")
	set(SDKCONFIG_DEFAULTS "sdkconfigs/general;sdkconfigs/tanmatsu")
	set(IDF_TARGET esp32p4)
	set(SDKCONFIG "sdkconfig_tanmatsu")
elseif(DEVICE STREQUAL "jc4880p433")
	set(SDKCONFIG_DEFAULTS "sdkconfigs/general;sdkconfigs/jc4880p433")
	set(IDF_TARGET esp32p4)
	set(SDKCONFIG "sdkconfig_jc4880p433")
elseif(DEVICE STREQUAL "jc3248w535")
	set(SDKCONFIG_DEFAULTS "sdkconfigs/general;sdkconfigs/jc3248w535")
	set(IDF_TARGET esp32s3)
	set(SDKCONFIG "sdkconfig_jc3248w535")
else()
	message(FATAL_ERROR "Unknown target device ${DEVICE}")
endif()

message(STATUS "Building for device: ${DEVICE}")
message(STATUS "Target: ${IDF_TARGET}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tiny386)
