#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdint.h>

#define LED_PIN 0               // Pin for LED output (connected to PWM)
#define BUTTON_PIN3 22          // Button 3 for generating the pulse

// Function to generate a single PWM pulse with a given pulse width (in microseconds)
void generate_pwm_pulse(uint gpio, float freq, float pulse_width_us) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the clock divider and set the PWM frequency
    float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
    float divider = clock_freq / (freq * 65536.0f);  // Compute divider for given frequency

    // Ensure the divider is within the valid range (1.0 to 255.0)
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

// Function to configure the button input pin
void configure_button() {
    // Initialize the button and set a pull-up resistor
    gpio_init(BUTTON_PIN3);
    gpio_set_dir(BUTTON_PIN3, GPIO_IN);
    gpio_pull_up(BUTTON_PIN3);
}

int main() {
    stdio_init_all();

    // Configure the button for input
    configure_button();

    // Define the pulse width for button 3 (in microseconds)
    float pulse_width3_us = 1500.0f;  // 1.5 ms pulse width
    float frequency = 100.0f;         // Set the PWM frequency to 100 Hz

    while (1) {
        // Check if button 3 (connected to BUTTON_PIN3) is being held down
        if (!gpio_get(BUTTON_PIN3)) {  // Button is active-low, so check if it's pressed
            printf("Button 3 is being held! Generating pulse for 1.5 seconds.\n");
            generate_pwm_pulse(LED_PIN, frequency, pulse_width3_us);
            sleep_ms(1500); // Keep the LED on for 1.5 seconds
        } else {
            // Turn off the PWM when the button is not pressed
            pwm_set_enabled(pwm_gpio_to_slice_num(LED_PIN), false);
        }

        sleep_ms(100); // Small delay to avoid CPU overload
    }
}
