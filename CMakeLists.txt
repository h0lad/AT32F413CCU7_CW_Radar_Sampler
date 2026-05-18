# SPDX-License-Identifier: AGPL-3.0-or-later
cmake_minimum_required(VERSION 3.24)

project(at32f413_modules LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

set(AT32_DEVICE AT32F413CCU7 CACHE STRING "Target device macro from at32f413.h")
set(AT32_VENDOR_ROOT "${CMAKE_SOURCE_DIR}/drivers/AT32F413_Firmware_Library")
set(AT32_CMSIS_DIR "${AT32_VENDOR_ROOT}/libraries/cmsis")
set(AT32_DRIVER_DIR "${AT32_VENDOR_ROOT}/libraries/drivers")
set(AT32_BOARD_DIR "${AT32_VENDOR_ROOT}/project/at32f413_board")
set(AT32_STARTUP_FILE "${AT32_CMSIS_DIR}/cm4/device_support/startup/gcc/startup_at32f413.s")
set(AT32_LINKER_SCRIPT "${AT32_CMSIS_DIR}/cm4/device_support/startup/gcc/linker/AT32F413xC_FLASH.ld")
set(AT32_SVD_FILE "${AT32_VENDOR_ROOT}/project/at_start_f413/templates/at32_ide/SVD/AT32F413xx_v2.svd")

set(AT32_OPENOCD_SOURCE_DIR "${CMAKE_SOURCE_DIR}/drivers/openocd" CACHE PATH "AT32 OpenOCD source directory")
set(AT32_OPENOCD_BUILD_DIR "${AT32_OPENOCD_SOURCE_DIR}/build-at32")
set(AT32_OPENOCD_BINARY "${AT32_OPENOCD_BUILD_DIR}/src/openocd" CACHE FILEPATH "OpenOCD binary path")
set(AT32_OPENOCD_SCRIPTS_DIR "${AT32_OPENOCD_SOURCE_DIR}/tcl")

if(NOT EXISTS "${AT32_VENDOR_ROOT}/libraries/cmsis")
  message(FATAL_ERROR
    "AT32F413_Firmware_Library submodule not found at ${AT32_VENDOR_ROOT}\n"
    "Run: git submodule update --init drivers/AT32F413_Firmware_Library")
endif()

include(cmake/at32_module.cmake)

add_library(at32_mcu_options INTERFACE)
target_compile_definitions(at32_mcu_options INTERFACE
  ${AT32_DEVICE}
  AT_START_F413_V1
)
target_include_directories(at32_mcu_options INTERFACE
  "${CMAKE_SOURCE_DIR}/platform/include"
  "${AT32_CMSIS_DIR}/cm4/core_support"
  "${AT32_CMSIS_DIR}/cm4/device_support"
  "${AT32_DRIVER_DIR}/inc"
  "${AT32_BOARD_DIR}"
)
target_compile_options(at32_mcu_options INTERFACE
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -ffunction-sections
  -fdata-sections
  -fno-common
  -Wall
  -Wextra
  -Wpedantic
  $<$<CONFIG:Debug>:-Og>
  $<$<CONFIG:Debug>:-g3>
  $<$<CONFIG:Release>:-Os>
)
target_link_options(at32_mcu_options INTERFACE
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  --specs=nano.specs
)

file(GLOB AT32_DRIVER_SOURCES CONFIGURE_DEPENDS
  "${AT32_DRIVER_DIR}/src/*.c"
)

add_library(at32_vendor STATIC
  ${AT32_DRIVER_SOURCES}
  "${AT32_CMSIS_DIR}/cm4/device_support/system_at32f413.c"
  "${AT32_BOARD_DIR}/at32f413_board.c"
  "${CMAKE_SOURCE_DIR}/platform/src/board_clock.c"
)
target_link_libraries(at32_vendor PUBLIC at32_mcu_options)

add_library(at32_runtime OBJECT
  "${AT32_STARTUP_FILE}"
  "${CMAKE_SOURCE_DIR}/platform/src/at32f413_int.c"
  "${CMAKE_SOURCE_DIR}/platform/src/syscalls.c"
)
target_link_libraries(at32_runtime PUBLIC at32_mcu_options)

file(GLOB AT32_MODULE_CMAKES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/module/*/CMakeLists.txt")
if(NOT AT32_MODULE_CMAKES)
  message(FATAL_ERROR "No firmware modules found below ${CMAKE_SOURCE_DIR}/module")
endif()

foreach(module_cmake IN LISTS AT32_MODULE_CMAKES)
  get_filename_component(module_dir "${module_cmake}" DIRECTORY)
  get_filename_component(module_name "${module_dir}" NAME)
  list(APPEND AT32_MODULE_NAMES "${module_name}")
endforeach()

list(SORT AT32_MODULE_NAMES)
message(STATUS "Discovered AT32 modules: ${AT32_MODULE_NAMES}")

foreach(module_name IN LISTS AT32_MODULE_NAMES)
  add_subdirectory("${CMAKE_SOURCE_DIR}/module/${module_name}")
endforeach()

set(AT32_VSCODE_MODULE_OPTIONS "")
foreach(module_name IN LISTS AT32_MODULE_NAMES)
  string(APPEND AT32_VSCODE_MODULE_OPTIONS "        \"${module_name}\",\n")
endforeach()
string(REGEX REPLACE ",\n$" "\n" AT32_VSCODE_MODULE_OPTIONS "${AT32_VSCODE_MODULE_OPTIONS}")

file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/.vscode")

configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/templates/tasks.json.in"
  "${CMAKE_SOURCE_DIR}/.vscode/tasks.json"
  @ONLY
)
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/templates/launch.json.in"
  "${CMAKE_SOURCE_DIR}/.vscode/launch.json"
  @ONLY
)
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/templates/settings.json.in"
  "${CMAKE_SOURCE_DIR}/.vscode/settings.json"
  @ONLY
)
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/templates/extensions.json.in"
  "${CMAKE_SOURCE_DIR}/.vscode/extensions.json"
  @ONLY
)
