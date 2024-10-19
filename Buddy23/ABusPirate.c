#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Pin definitions
#define PWM_PIN 0           // Pin for PWM outputs (from hello_pwm.c)
#define PWM_INPUT_PIN 27    // Pin to read the PWM signal (from analysis_and_monitoring.c)
#define ADC_PIN 26          // Pin for ADC input (from analysis_and_monitoring.c)

// Variables for PWM analysis
absolute_time_t rising_edge_time;
absolute_time_t falling_edge_time;
absolute_time_t previous_rising_edge_time;
uint64_t pulse_width_us = 0;  // Stores the duration of the high pulse in microseconds
uint64_t period_us = 0;       // Stores the period of the PWM signal in microseconds
float duty_cycle = 0.0f;      // Percentage of time the signal is high
float frequency_hz = 0.0f;    // Frequency of the PWM signal in Hz

// GPIO Interrupt Handler to measure the pulse width and period (from analysis_and_monitoring.c)
// This function is called whenever a rising or falling edge is detected on the PWM input pin
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) { // Handle rising edge
        rising_edge_time = get_absolute_time(); // Record the time of the rising edge
        if (absolute_time_diff_us(previous_rising_edge_time, rising_edge_time) > 0) {
            // Calculate the period and update frequency and duty cycle
            period_us = absolute_time_diff_us(previous_rising_edge_time, rising_edge_time);
            if (period_us > 0) {
                frequency_hz = 1e6 / period_us; // Convert period to frequency in Hz
                duty_cycle = (pulse_width_us * 100.0) / period_us; // Calculate duty cycle percentage
            }
            printf("Duty Cycle: %.2f%%, Frequency: %.2f Hz\n", duty_cycle, frequency_hz);
        }
        previous_rising_edge_time = rising_edge_time; // Update the previous rising edge time
    }

    if (events & GPIO_IRQ_EDGE_FALL) { // Handle falling edge
        falling_edge_time = get_absolute_time(); // Record the time of the falling edge
        pulse_width_us = absolute_time_diff_us(rising_edge_time, falling_edge_time); // Calculate pulse width
    }
}

// Function to set up PWM (from hello_pwm.c)
// Initializes PWM on the specified pin with the given frequency and duty cycle
void setup_pwm(uint pwm_pin, uint frequency, float duty_cycle) {
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM); // Configure pin for PWM
    uint slice_num = pwm_gpio_to_slice_num(pwm_pin); // Determine PWM slice number for the pin
    uint32_t clock_freq = 125000000; // Clock frequency of the microcontroller (125 MHz)
    uint32_t divider16 = clock_freq / (frequency * 4096); // Calculate clock divider for desired frequency
    pwm_set_clkdiv(slice_num, divider16 / 16.0f); // Set PWM clock divider
    pwm_set_wrap(slice_num, 4095); // Set the counter's maximum value (wrap)
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pwm_pin), duty_cycle * 4095); // Set duty cycle level
    pwm_set_enabled(slice_num, true); // Enable the PWM slice
    printf("Initial PWM Setup -> Frequency: %d Hz, Duty Cycle: %.2f%%\n", frequency, duty_cycle * 100.0);
}

// Function to configure GPIO to measure pulse width and period (from analysis_and_monitoring.c)
// Sets up the GPIO pin to detect rising and falling edges for PWM signal analysis
void configure_gpio_for_pwm() {
    gpio_init(PWM_INPUT_PIN); // Initialize GPIO for input
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN); // Set GPIO direction to input
    gpio_pull_down(PWM_INPUT_PIN); // Enable pull-down resistor
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback); // Enable edge interrupts
}

// Function to configure the ADC (from analysis_and_monitoring.c)
// Sets up the ADC to read analog values from a specified pin
void configure_adc() {
    adc_init(); // Initialize the ADC hardware
    adc_gpio_init(ADC_PIN); // Configure the pin for ADC input
    adc_select_input(0); // Select ADC channel (corresponding to GPIO26)
}

int main() {
    stdio_init_all(); // Initialize standard I/O for communication

    // Set up the initial PWM signal (from hello_pwm.c)
    uint frequency = 1000; // Desired PWM frequency in Hz
    float duty_cycle = 0.5; // Desired initial duty cycle (50%)
    setup_pwm(PWM_PIN, frequency, duty_cycle); // Initialize PWM

    // Configure GPIO for pulse width and frequency measurement (from analysis_and_monitoring.c)
    configure_gpio_for_pwm();

    // Configure ADC (from analysis_and_monitoring.c)
    configure_adc();

    while (1) {
        // Monitor and print out the frequency and duty cycle detected
        printf("Duty Cycle: %.2f%%, Frequency: %.2f Hz\n", duty_cycle, frequency_hz);

        // Read and print ADC values (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read(); // Read raw ADC value
        float voltage = raw_adc * 3.3f / (1 << 12); // Convert raw ADC value to voltage
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage); // Print the ADC reading

        // Sleep for some time before the next measurement
        sleep_ms(500);
    }
}