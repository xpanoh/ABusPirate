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
// Modified function to generate PWM based on high and low times (in microseconds)
// Modified function to generate PWM based on arrays of high and low times (in microseconds)
void generate_pwm_pulsewidth(uint gpio, float *high_times_us, float *low_times_us, int num_pulses) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Set up each pulse using the high and low times arrays
    for (int i = 0; i < num_pulses; i++) {
        // Calculate period and frequency for each pulse
        float period_us = high_times_us[i] + low_times_us[i];
        float freq = 1000000.0f / period_us;
        float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
        float divider = clock_freq / (freq * 65536.0f);

        // Ensure the divider is within valid range
        if (divider < 1.0f) divider = 1.0f;
        else if (divider > 255.0f) divider = 255.0f;

        // Set PWM clock divider
        pwm_set_clkdiv(slice_num, divider);

        // Set PWM wrap value
        pwm_set_wrap(slice_num, 65535);

        // Calculate and set duty cycle
        float duty_cycle = high_times_us[i] / period_us;
        pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535.0f));

        // Enable PWM output
        pwm_set_enabled(slice_num, true);

        // Print PWM parameters
        printf("PWM Signal: High Time = %.2f us, Low Time = %.2f us, Period = %.2f us, Frequency = %.2f Hz, Duty Cycle = %.2f%%\n",
               high_times_us[i], low_times_us[i], period_us, freq, duty_cycle * 100.0f);
 // Wait before switching to the next pulse
        sleep_ms(5000);
        
    }

    // Disable PWM output after completing the pulses
    pwm_set_enabled(slice_num, false);
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
    // Configure ADC (from analysis_and_monitoring.c)
    configure_adc();
  // Configure GPIO 15 to receive PWM input
    configure_pwm_input();

    // Arrays of pulse widths (in microseconds) and periods (in microseconds)
    float high_times_us[] = {1000.0f, 1200.0f, 1500.0f, 1700.0f, 2000.0f, 1800.0f, 1600.0f, 1400.0f, 1100.0f, 1300.0f};
    float low_times_us[] =  {1000.0f,  800.0f,  500.0f,  300.0f,  500.0f,  700.0f,  900.0f,  600.0f,  800.0f,  700.0f};
    int num_pulses = sizeof(high_times_us) / sizeof(high_times_us[0]);  // Number of pulses to generate
while (1) {
// Generate PWM signals with arrays of high and low times
    generate_pwm_pulsewidth(PWM_PIN0, high_times_us, low_times_us, num_pulses);

        // Read ADC (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);
}

  
    return 0;
}