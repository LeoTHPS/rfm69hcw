cmake_minimum_required(VERSION 3.24)

set(PICO_BOARD         pico)

set(CMAKE_C_STANDARD   17)
set(CMAKE_CXX_STANDARD 20)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rfm69hcw)
pico_sdk_init()

add_library(rfm69hcw rfm69hcw.cpp)
target_link_libraries(rfm69hcw hardware_spi hardware_gpio hardware_timer)
target_include_directories(rfm69hcw PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_executable(demo demo.cpp)
target_link_libraries(demo rfm69hcw pico_stdlib pico_status_led hardware_gpio)
pico_add_extra_outputs(demo)
pico_enable_stdio_usb(demo 1)
