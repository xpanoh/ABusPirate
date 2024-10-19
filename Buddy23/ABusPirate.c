#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Pin definitions
#define PWM_PIN 0           // Pin for PWM outputs (from hello_pwm.c)
#define PWM_INPUT_PIN 15     // Pin to read the PWM signal (from analysis_and_monitoring.c)
#define ADC_PIN 26           // Pin for ADC input (from analysis_and_monitoring.c)

// Variables for PWM analysis
absolute_time_t rising_edge_time;
absolute_time_t falling_edge_time;
absolute_time_t previous_rising_edge_time;
uint64_t pulse_width_us = 0;
uint64_t period_us = 0;
float duty_cycle = 0.0f;
float frequency_hz = 0.0f;

// GPIO Interrupt Handler to measure the pulse width and period (from analysis_and_monitoring.c)
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        rising_edge_time = get_absolute_time();
        if (absolute_time_diff_us(previous_rising_edge_time, rising_edge_time) > 0) {
            period_us = absolute_time_diff_us(previous_rising_edge_time, rising_edge_time);
            if (period_us > 0) {
                frequency_hz = 1e6 / period_us;
                duty_cycle = (pulse_width_us * 100.0) / period_us;
            }
            printf("Duty Cycle: %.2f%%, Frequency: %.2f Hz\n", duty_cycle, frequency_hz);
        }
        previous_rising_edge_time = rising_edge_time;
    }

    if (events & GPIO_IRQ_EDGE_FALL) {
        falling_edge_time = get_absolute_time();
        pulse_width_us = absolute_time_diff_us(rising_edge_time, falling_edge_time);
    }
}

// Function to set up PWM (from hello_pwm.c)
void setup_pwm(uint pwm_pin, uint frequency, float duty_cycle) {
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
    uint32_t clock_freq = 125000000;
    uint32_t divider16 = clock_freq / (frequency * 4096);
    pwm_set_clkdiv(slice_num, divider16 / 16.0f);
    pwm_set_wrap(slice_num, 4095);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pwm_pin), duty_cycle * 4095);
    pwm_set_enabled(slice_num, true);
    printf("Initial PWM Setup -> Frequency: %d Hz, Duty Cycle: %.2f%%\n", frequency, duty_cycle * 100.0);
}

// Function to configure GPIO to measure pulse width and period (from analysis_and_monitoring.c)
void configure_gpio_for_pwm() {
    gpio_init(PWM_INPUT_PIN);
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);
    gpio_pull_down(PWM_INPUT_PIN);
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Function to configure the ADC (from analysis_and_monitoring.c)
void configure_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

int main() {
    stdio_init_all();

    // Set up the initial PWM signal (from hello_pwm.c)
    uint frequency = 1000;
    float duty_cycle = 0.5;
    setup_pwm(PWM_PIN, frequency, duty_cycle);

    // Configure GPIO for pulse width and frequency measurement (from analysis_and_monitoring.c)
    configure_gpio_for_pwm();

    // Configure ADC (from analysis_and_monitoring.c)
    configure_adc();

    while (1) {
        // Vary duty cycle and frequency to demonstrate PWM control
        for (float duty_cycle = 0.1; duty_cycle <= 1.0; duty_cycle += 0.1) {
            setup_pwm(PWM_PIN, 1000, duty_cycle);  // Vary duty cycle from 10% to 100% at 1000 Hz
            sleep_ms(1000);  // Pause to observe each change
        }

        for (uint frequency = 500; frequency <= 2000; frequency += 500) {
            setup_pwm(PWM_PIN, frequency, 0.5);  // Change frequency from 500 Hz to 2000 Hz at 50% duty cycle
            sleep_ms(1000);  // Pause to observe each change
        }

        // Read ADC (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);
        sleep_ms(500);
    }
}
