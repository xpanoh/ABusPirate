cmake_minimum_required(VERSION 3.13)

# Set the project name
project(signal_analysis)

# Add the executable that includes both hello_pwm and analysis_and_monitoring
add_executable(signal_analysis
signal_analysis.c
)

# Pull in common dependencies and additional hardware support
target_link_libraries(signal_analysis pico_stdlib hardware_pwm hardware_adc hardware_timer)

file(GLOB KISSFFT_SOURCES "lib/kissfft/*.c")
target_sources(signal_analysis PRIVATE ${KISSFFT_SOURCES})
target_include_directories(signal_analysis PRIVATE lib/kissfft)

# Create map/bin/hex file etc.
pico_add_extra_outputs(signal_analysis)

# Enable USB and UART for I/O
pico_enable_stdio_usb(signal_analysis 1)
pico_enable_stdio_uart(signal_analysis 1)

# Add URL via pico_set_program_url
example_auto_set_url(signal_analysis)