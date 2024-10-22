#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdint.h>

// Pin definitions

#define ADC_PIN 26           // Pin for ADC input (from analysis_and_monitoring.c)
#define PWM_PIN0 0            // Pin for PWM output
#define PWM_INPUT_PIN 15     // Pin for receiving PWM signal

volatile uint32_t rising_edge_time = 0;  // Time of the rising edge
volatile uint32_t falling_edge_time = 0; // Time of the falling edge
volatile uint32_t pulse_width = 0;       // Pulse width in microseconds
volatile uint32_t period = 0;            // Period of the PWM signal in microseconds
volatile bool pwm_ready = false;         // Flag to indicate new PWM data is ready
// Modified function to generate PWM based on pulse width (in microseconds)
void generate_pwm_pulsewidth(uint gpio, float freq, float pulse_width_us) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the clock divider and set the PWM frequency
    float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
    float divider = clock_freq / (freq * 65536.0f);  // Compute divider for given frequency

    // Ensure the divider is within valid range (1.0 to 255.0)
    if (divider < 1.0f) {
        divider = 1.0f;
    } else if (divider > 255.0f) {
        divider = 255.0f;
    }

    // Set the PWM clock divider
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value for 16-bit resolution)
    pwm_set_wrap(slice_num, 65535);

    // Calculate the duty cycle based on the desired pulse width and frequency
    float period_us = 1000000.0f / freq;  // Period in microseconds
    float duty_cycle = pulse_width_us / period_us;  // Duty cycle as a fraction

    // Ensure the duty cycle does not exceed 100%
    if (duty_cycle > 1.0f) {
        duty_cycle = 1.0f;
    }

    // Set the duty cycle (scaled to 16-bit resolution)
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535.0f));

    // Enable the PWM output
    pwm_set_enabled(slice_num, true);
}


// Function to capture rising and falling edges of the PWM signal
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();  // Get the current time in microseconds

    if (events & GPIO_IRQ_EDGE_RISE) {
        // On rising edge, calculate the period (time between rising edges)
        period = current_time - rising_edge_time;
        rising_edge_time = current_time;   // Store the rising edge timestamp
    }

    if (events & GPIO_IRQ_EDGE_FALL) {
        falling_edge_time = current_time;  // Store the falling edge timestamp
        pulse_width = falling_edge_time - rising_edge_time;  // Calculate the pulse width (high time)
        pwm_ready = true;  // Set flag to indicate new PWM data is ready
    }
}
// Function to configure the PWM input pin (GP15)
void configure_pwm_input() {
    gpio_init(PWM_INPUT_PIN);  // Initialize GPIO
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);  // Set as input
    gpio_pull_down(PWM_INPUT_PIN);  // Pull down the pin by default

    // Set interrupts on rising and falling edges
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Function to set up the PWM
void generate_pwm(uint gpio, float freq, float duty_cycle) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the clock divider and set the PWM frequency
    float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
    float divider = clock_freq / (freq * 65536.0f);  // Compute divider for given frequency

    // Ensure the divider is within valid range (1.0 to 255.0)
    if (divider < 1.0f) {
        divider = 1.0f;
    } else if (divider > 255.0f) {
        divider = 255.0f;
    }

    // Set the PWM clock divider
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value for 16-bit resolution)
    pwm_set_wrap(slice_num, 65535);

    // Set the duty cycle (scaled to 16-bit resolution)
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535.0f));

    // Enable the PWM output
    pwm_set_enabled(slice_num, true);
}

// Function to configure the ADC (from analysis_and_monitoring.c)
void configure_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

int main() {
   
    stdio_init_all();

    // Configure PWM
    // Set up PWM on GPIO0
    //generate_pwm(PWM_PIN0, 100.0f, 0.1f);  // 100 Hz frequency, 50% duty cycle
generate_pwm_pulsewidth(PWM_PIN0, 100.0f, 1500.0f);  // 100 Hz, 1.5 ms pulse width

    // Configure ADC (from analysis_and_monitoring.c)
    configure_adc();
  // Configure GPIO 15 to receive PWM input
    configure_pwm_input();

    // Arrays of pulse widths (in microseconds) and periods (in microseconds)
    float pulse_widths_us[] = {1000.0f, 1200.0f, 1500.0f, 1700.0f, 2000.0f};
    float periods_us[] = {2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f};  // Periods of 10000 microsecond

    int num_pulse_widths = sizeof(pulse_widths_us) / sizeof(pulse_widths_us[0]);
    int num_periods = sizeof(periods_us) / sizeof(periods_us[0]);

    int current_pulse_index = 0;  // Index for pulse width
    int current_period_index = 0;  // Index for period
// Set PWM frequency

    while (1) {
        
     // Get the current pulse width and period
        float pulse_width_us = pulse_widths_us[current_pulse_index];
        float period_us = periods_us[current_period_index];

        // Generate PWM signal with the current pulse width and period
        generate_pwm_pulsewidth(PWM_PIN0, period_us, pulse_width_us);

        // Calculate the frequency based on the period
        float frequency = 1000000.0f / period_us;  // Frequency in Hz

        // Calculate the duty cycle as a percentage
        float duty_cycle = (pulse_width_us / period_us) * 100.0f;

        // Print the current pulse width, period, duty cycle, and frequency to the serial monitor
        printf("PWM Signal: Period = %.2f us, Frequency = %.2f Hz, Pulse Width = %.2f us, Duty Cycle = %.2f%%\n",
               period_us, frequency, pulse_width_us, duty_cycle);

        // Move to the next pulse width in the array
        current_pulse_index = (current_pulse_index + 1) % num_pulse_widths;

        // Move to the next period after all pulse widths are used
        if (current_pulse_index == 0) {
            current_period_index = (current_period_index + 1) % num_periods;
        }

        // Delay for 500 ms
        sleep_ms(500);

        // Read ADC (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);
        sleep_ms(500);
    }
}
