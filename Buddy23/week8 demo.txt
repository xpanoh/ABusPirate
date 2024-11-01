#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

const uint32_t high_timings[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000};
const uint32_t low_timings[] = {10000, 20000, 30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000};

const int pulse_count = sizeof(high_timings) / sizeof(high_timings[0]);
const int repeat_count = 2; // Set desired number of repetitions for each button press

volatile bool trigger_pulse = false; // Flag to trigger the pulse sequence on button press

// Function to generate pulses
void generate_custom_pulse(uint gpio, const uint32_t *high_timings, const uint32_t *low_timings, int count, int repeat_count) {
    for (int repeat = 0; repeat < repeat_count; repeat++) {
        for (int i = 0; i < count; i++) {
            // Debug print to show the current timing values being sent
            printf("Generating pulse: High %u us, Low %u us\n", high_timings[i], low_timings[i]);

            // Set GPIO high and wait for the specified high duration
            gpio_put(gpio, 1);
            sleep_us(high_timings[i]);

            // Set GPIO low and wait for the specified low duration
            gpio_put(gpio, 0);
            sleep_us(low_timings[i]);
        }
    }
}

// Interrupt handler for button press on GPIO 22
void button_callback(uint gpio, uint32_t events) {
    trigger_pulse = true; // Set trigger flag to start generating pulses
}

int main() {
    stdio_init_all();

    const uint gpio_output = 15;
    const uint gpio_button = 22;

    // Initialize GPIO 15 as output for pulse generation
    gpio_init(gpio_output);
    gpio_set_dir(gpio_output, GPIO_OUT);

    // Initialize GPIO 22 as input with a pull-up resistor for the button
    gpio_init(gpio_button);
    gpio_set_dir(gpio_button, GPIO_IN);
    gpio_pull_up(gpio_button);

    // Set up interrupt for button press on GPIO 22 (falling edge, when button is pressed)
    gpio_set_irq_enabled_with_callback(gpio_button, GPIO_IRQ_EDGE_FALL, true, &button_callback);

    while (true) {
        // Check if the trigger flag is set by the button press
        if (trigger_pulse) {
            // Generate the custom pulse sequence
            generate_custom_pulse(gpio_output, high_timings, low_timings, pulse_count, repeat_count);

            // Reset the trigger flag
            trigger_pulse = false;
        }

        // Optional delay to prevent the loop from running too fast
        sleep_ms(100);
    }

    return 0;
}
