cmake_minimum_required(VERSION 3.13)

# -----------------------------------------------------------------------------------
# Toolchain / SDK setup. Must happen before project().
# -----------------------------------------------------------------------------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Board: Raspberry Pi Pico 2 (RP2350). Change if you're targeting different hardware.
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk CACHE STRING "Path to the Pico SDK")

include(pico_sdk_import.cmake)

project(MotionControllerRP C CXX ASM)

pico_sdk_init()

# FreeRTOS-Kernel must be the raspberrypi/FreeRTOS-Kernel fork (not the upstream
# FreeRTOS/FreeRTOS-Kernel repo) -- only the fork carries the RP2350_ARM_NTZ dual-core
# SMP port this project relies on. See lib/README.md.
include(FreeRTOS_Kernel_import.cmake)

# pico-vfs provides the POSIX file API (fopen/opendir/stat/...) that
# src/compat/LittleFS_compat.* is built on top of.
add_subdirectory(lib/pico-vfs)

add_subdirectory(src)
