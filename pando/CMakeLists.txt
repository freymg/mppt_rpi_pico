cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "/Users/fredmg/pico/pico-sdk")
include(pico_sdk_import.cmake)

project(pando)
pico_sdk_init()

add_executable(${PROJECT_NAME} main.c)
pico_add_extra_outputs(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pwm hardware_timer hardware_i2c)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
