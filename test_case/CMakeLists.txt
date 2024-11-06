cmake_minimum_required(VERSION 3.13)

# Set the project name
project(test_case)

# Initialize the Pico SDK (ensure PICO_SDK_PATH environment variable is set)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

# Define the executable with your main file
add_executable(test_case test_case.c)

# Include directories for kissfft library
target_include_directories(test_case PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    lib/kissfft
)

# Include kissfft source files
file(GLOB KISSFFT_SOURCES "lib/kissfft/*.c")
target_sources(test_case PRIVATE ${KISSFFT_SOURCES})

# Link necessary libraries, including Pico's and kiss_fft
target_link_libraries(test_case pico_stdlib hardware_pwm hardware_adc hardware_uart hardware_gpio hardware_clocks)

# Enable USB and UART for standard I/O
pico_enable_stdio_usb(test_case 1)
pico_enable_stdio_uart(test_case 1)

# Create map/bin/hex/uf2 files for the Pico
pico_add_extra_outputs(test_case)

# Add URL if this is part of an example suite (optional, safe to remove if unused)
example_auto_set_url(test_case)