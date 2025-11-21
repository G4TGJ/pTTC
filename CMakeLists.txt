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
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(pTTC C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options( -O1 )
add_compile_definitions( PSDR )

add_subdirectory(TTC)
add_subdirectory(TRL)
add_subdirectory(pTRL)

set(SRCS
    io.c
)

set(LIBS
    pico_stdlib
    pico_multicore
    hardware_dma
    hardware_adc
    hardware_pwm
    TTC
    TRL
    pTRL
    TRL_lcd_i2c
)

function(configTarget project)
    add_executable(${project} ${SRCS} )
    target_link_libraries(${project} ${LIBS})

    pico_enable_stdio_usb(${project} 1)
    pico_enable_stdio_uart(${project} 0)
    pico_add_extra_outputs(${project})
    pico_set_binary_type(${project} copy_to_ram)

endfunction()

configTarget(PSDR)

configTarget(PSDR2)
target_compile_definitions(PSDR2 PRIVATE PSDR2)
target_compile_definitions(PSDR2 PRIVATE SYS_CLK_MHZ=200)
