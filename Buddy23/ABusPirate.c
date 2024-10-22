#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdint.h>

#define LED_PIN 0               // Pin for LED output (connected to PWM)
#define BUTTON_PIN1 20          // Button 1 for 1st pulse width
#define BUTTON_PIN2 21          // Button 2 for 2nd pulse width
#define BUTTON_PIN3 22          // Button 3 for 3rd pulse width

volatile bool button1_pressed = false; // Flag for button 1 press
volatile bool button2_pressed = false; // Flag for button 2 press
volatile bool button3_pressed = false; // Flag for button 3 press

// Function to generate a single PWM pulse based on pulse width (in microseconds)
void generate_single_pwm_pulse(uint gpio, float freq, float pulse_width_us) {
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

    // Delay to allow the pulse to be emitted
    sleep_us(pulse_width_us);

    // Disable the PWM output after the pulse
    pwm_set_enabled(slice_num, false);
}

// Button press interrupt handler
void button_callback(uint gpio, uint32_t events) {
    if (gpio == BUTTON_PIN1 && (events & GPIO_IRQ_EDGE_FALL)) {
        button1_pressed = true; // Set flag for button 1
    } else if (gpio == BUTTON_PIN2 && (events & GPIO_IRQ_EDGE_FALL)) {
        button2_pressed = true; // Set flag for button 2
    } else if (gpio == BUTTON_PIN3 && (events & GPIO_IRQ_EDGE_FALL)) {
        button3_pressed = true; // Set flag for button 3
    }
}

// Function to configure the button input pins
void configure_buttons() {
    // Initialize buttons and set pull-up resistors
    gpio_init(BUTTON_PIN1);
    gpio_set_dir(BUTTON_PIN1, GPIO_IN);
    gpio_pull_up(BUTTON_PIN1);

    gpio_init(BUTTON_PIN2);
    gpio_set_dir(BUTTON_PIN2, GPIO_IN);
    gpio_pull_up(BUTTON_PIN2);

    gpio_init(BUTTON_PIN3);
    gpio_set_dir(BUTTON_PIN3, GPIO_IN);
    gpio_pull_up(BUTTON_PIN3);

    // Set interrupts on falling edges for each button
    gpio_set_irq_enabled_with_callback(BUTTON_PIN1, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN2, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN3, GPIO_IRQ_EDGE_FALL, true, &button_callback);
}

int main() {
    stdio_init_all();

    // Configure the buttons for input
    configure_buttons();

    // Define pulse widths for each button
    float pulse_width1_us = 500.0f;   // 0.5 ms for button 1
    float pulse_width2_us = 1000.0f;  // 1 ms for button 2
    float pulse_width3_us = 1500.0f;  // 1.5 ms for button 3

    float frequency = 100.0f;  // Set the PWM frequency to 100 Hz

    while (1) {
        // Check if button 1 was pressed
        if (button1_pressed) {
            printf("Button 1 pressed! Generating pulse: %.2f us\n", pulse_width1_us);
            generate_single_pwm_pulse(LED_PIN, frequency, pulse_width1_us);
            button1_pressed = false; // Reset the flag
        }

        // Check if button 2 was pressed
        if (button2_pressed) {
            printf("Button 2 pressed! Generating pulse: %.2f us\n", pulse_width2_us);
            generate_single_pwm_pulse(LED_PIN, frequency, pulse_width2_us);
            button2_pressed = false; // Reset the flag
        }

        // Check if button 3 was pressed
        if (button3_pressed) {
            printf("Button 3 pressed! Generating pulse: %.2f us\n", pulse_width3_us);
            generate_single_pwm_pulse(LED_PIN, frequency, pulse_width3_us);
            button3_pressed = false; // Reset the flag
        }

        sleep_ms(100); // Small delay to avoid CPU overload
    }
}