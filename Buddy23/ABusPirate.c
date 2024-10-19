#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Pin definitions
#define PWM_PIN 0           // Pin for PWM outputs (from hello_pwm.c)
#define PWM_INPUT_PIN 15     // Pin to read the PWM signal (from analysis_and_monitoring.c)
#define ADC_PIN 26           // Pin for ADC input (from analysis_and_monitoring.c)





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

    while (1) {
       

        // Read ADC (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);
        sleep_ms(500);
    }
}
