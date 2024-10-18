cmake_minimum_required(VERSION 3.13)

# Set the project name
project(ABusPirate)

# Add the executable that includes both hello_pwm and analysis_and_monitoring
add_executable(ABusPirate
ABusPirate.c
)

# Pull in common dependencies and additional hardware support
target_link_libraries(ABusPirate pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Create map/bin/hex file etc.
pico_add_extra_outputs(ABusPirate)

# Enable USB and UART for I/O
pico_enable_stdio_usb(ABusPirate 1)
pico_enable_stdio_uart(ABusPirate 1)

# Add URL via pico_set_program_url
example_auto_set_url(ABusPirate)
