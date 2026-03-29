# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(davids_pico_gamepad C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(davids_pico_gamepad
    davids_pico_gamepad.cpp
    8bitdo_n64_mk.cpp
    usb_descriptors.c
)

pico_set_program_name(davids_pico_gamepad "davids_pico_gamepad")
if(NOT DEFINED PROJECT_VERSION)
    set(PROJECT_VERSION "dev")
endif()
pico_set_program_version(davids_pico_gamepad "${PROJECT_VERSION}")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(davids_pico_gamepad 1)
pico_enable_stdio_usb(davids_pico_gamepad 0)

target_include_directories(davids_pico_gamepad PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(davids_pico_gamepad
        pico_stdlib
        pico_cyw43_arch_none
        pico_btstack_classic
        pico_btstack_cyw43
        pico_btstack_run_loop_async_context
        tinyusb_device
        tinyusb_board
        pico_multicore
)

# Configuration for TinyUSB
target_compile_definitions(davids_pico_gamepad PRIVATE
    CFG_TUSB_MCU=OPT_MCU_RP2040
    CFG_TUSB_OS=OPT_OS_PICO
    CFG_TUD_ENABLED=1
)

pico_add_extra_outputs(davids_pico_gamepad)
