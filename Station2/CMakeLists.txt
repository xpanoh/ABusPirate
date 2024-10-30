cmake_minimum_required(VERSION 3.13)

# Set the project name
project(Station2)

# Add the executable that includes both hello_pwm and analysis_and_monitoring
add_executable(Station2
Station2.c
)

# Pull in common dependencies and additional hardware support
target_link_libraries(Station2 pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Create map/bin/hex file etc.
pico_add_extra_outputs(Station2)

# Enable USB and UART for I/O
pico_enable_stdio_usb(Station2 1)
pico_enable_stdio_uart(Station2 1)

# Add URL via pico_set_program_url
example_auto_set_url(Station2)