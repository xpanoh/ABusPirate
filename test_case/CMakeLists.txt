cmake_minimum_required(VERSION 3.13)

# Set the project name
project(test_case)

# Add the executable that includes both hello_pwm and analysis_and_monitoring
add_executable(test_case
test_case.c
)

# Pull in common dependencies and additional hardware support
target_link_libraries(test_case pico_stdlib hardware_pwm hardware_uart hardware_gpio hardware_clocks)

# Create map/bin/hex file etc.
pico_add_extra_outputs(test_case)

# Enable USB and UART for I/O
pico_enable_stdio_usb(test_case 1)
pico_enable_stdio_uart(test_case 1)

# Add URL via pico_set_program_url
example_auto_set_url(test_case)