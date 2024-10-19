#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdint.h>

// Pin definitions

#define ADC_PIN 26           // Pin for ADC input (from analysis_and_monitoring.c)
#define PWM_PIN0 0            // Pin for PWM output


// Function to set up the PWM
void generate_pwm(uint gpio, float freq, float duty_cycle) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the PWM frequency and set the PWM wrap value
    float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
    uint32_t divider = clock_freq / (freq * 65536);  // Compute divider for given frequency
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num, 65535);  // 16-bit counter (0 - 65535)

    // Set the duty cycle
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));

    // Enable the PWM
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
    generate_pwm(PWM_PIN0, 100.0f, 0.5f);  // 100 Hz frequency, 50% duty cycle

    // Configure ADC (from analysis_and_monitoring.c)
    configure_adc();

    while (1) {
       

        // Read ADC (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);
        sleep_ms(500);
    }
}
