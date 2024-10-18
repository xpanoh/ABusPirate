cmake_minimum_required(VERSION 3.13)

# Set the project name
project(pwm_analysis_project)

# Add the executable that includes both hello_pwm and analysis_and_monitoring
add_executable(pwm_analysis_project
    hello_pwm.c
    analysis_and_monitoring.c
)

# Pull in common dependencies and additional hardware support
target_link_libraries(pwm_analysis_project pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Create map/bin/hex file etc.
pico_add_extra_outputs(pwm_analysis_project)

# Enable USB and UART for I/O
pico_enable_stdio_usb(pwm_analysis_project 1)
pico_enable_stdio_uart(pwm_analysis_project 1)

# Add URL via pico_set_program_url
example_auto_set_url(pwm_analysis_project)
