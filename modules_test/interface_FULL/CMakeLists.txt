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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(interface_FULL C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(interface_FULL interface_FULL.c
            lib/oled_screen/ssd1306.c
            lib/oled_screen/screen.c
            lib/keyboard/mat.c 
            lib/I2C_driver/i2c_driver.c
                lib/PID_lib/PID_lib.c
                lib/servo_lib/servo_lib.c
                lib/mpu6050/mpu6050.c
                lib/kalman_filter/kalman_filter.c
                lib/esc_lib/esc_lib.c
                
                lib/ADC/adc.c)

pico_set_program_name(interface_FULL "interface_FULL")
pico_set_program_version(interface_FULL "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(interface_FULL 0)
pico_enable_stdio_usb(interface_FULL 1)

# Add the standard library to the build
target_link_libraries(interface_FULL
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(interface_FULL PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(interface_FULL 
        pico_stdlib
        hardware_i2c
        hardware_timer
        hardware_pwm
        hardware_clocks
        hardware_adc)
        
pico_add_extra_outputs(interface_FULL)

