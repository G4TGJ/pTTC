cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(pTTC C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pTTC
    io.c
)
pico_enable_stdio_usb(pTTC 1)
pico_enable_stdio_uart(pTTC 0)
pico_add_extra_outputs(pTTC)

add_subdirectory(TTC)
add_subdirectory(TRL)
add_subdirectory(pTRL)

target_link_libraries(pTTC
    pico_stdlib
    TTC
    TRL
    pTRL
    TRL_lcd_i2c
)
