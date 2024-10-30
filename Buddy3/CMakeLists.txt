cmake_minimum_required(VERSION 3.13)

# Set the project name
project(AbusPirate3)

# Add the executable that includes both hello_pwm and analysis_and_monitoring
add_executable(AbusPirate3
AbusPirate3.c
)

# Pull in common dependencies and additional hardware support
target_link_libraries(AbusPirate3 pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Create map/bin/hex file etc.
pico_add_extra_outputs(AbusPirate3)

# Enable USB and UART for I/O
pico_enable_stdio_usb(AbusPirate3 1)
pico_enable_stdio_uart(AbusPirate3 1)

# Add URL via pico_set_program_url
example_auto_set_url(AbusPirate3)
