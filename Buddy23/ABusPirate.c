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
#define UART_TX_PIN 5    // UART1 TX pin (GP5)
#define UART_RX_PIN 4     // UART1 RX pin (GP4)




volatile uint32_t rising_edge_time = 0;  // Time of the rising edge
volatile uint32_t falling_edge_time = 0; // Time of the falling edge
volatile uint32_t pulse_width = 0;       // Pulse width in microseconds
volatile uint32_t period = 0;            // Period of the PWM signal in microseconds







void setup_uart() {
    // Initialize UART1
    uart_init(uart1, 9600); // Baud rate 115200
    // Set UART1 TX pin (GP3)
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // Set UART1 RX pin (GP4)
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}
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

        // Introduce a delay for each pulse to complete
        sleep_us((int)period_us);

        // Disable PWM output after each pulse (optional if generating multiple pulses)
        pwm_set_enabled(slice_num, false);
    }
}



// Callback function to capture rising and falling edges on PWM_INPUT_PIN
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE) {
        period = current_time - rising_edge_time; // Calculate period
        rising_edge_time = current_time;
    }

    if (events & GPIO_IRQ_EDGE_FALL) {
        falling_edge_time = current_time;
        pulse_width = falling_edge_time - rising_edge_time; // Calculate pulse width
    }
}

// Function to configure PWM input pin for measurement
void configure_pwm_input() {
    gpio_init(PWM_INPUT_PIN);
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);
    gpio_pull_down(PWM_INPUT_PIN);
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
   // Set up UART1
    setup_uart();
    printf("UART initialized on pins %d (TX) and %d (RX)\n", UART_TX_PIN, UART_RX_PIN);
    // Arrays of pulse widths (in microseconds) and periods (in microseconds)
    float high_times_us[] = {1000.0f, 1200.0f, 1500.0f, 1700.0f, 2000.0f, 1800.0f, 1600.0f, 1400.0f, 1100.0f, 1300.0f};
    float low_times_us[] =  {1000.0f,  800.0f,  500.0f,  300.0f,  500.0f,  700.0f,  900.0f,  600.0f,  800.0f,  700.0f};
    int num_pulses = sizeof(high_times_us) / sizeof(high_times_us[0]);  // Number of pulses to generate

       // generate_pwm(PWM_PIN0, 100.0f, 0.1f);  // 100 Hz frequency, 50% duty cycle

        // Generate custom PWM signals with arrays of high and low times
        generate_pwm_pulsewidth(PWM_PIN0, high_times_us, low_times_us, num_pulses);

while (1) {
          // Check if new PWM data is ready
       
            // Ensure period is not zero to avoid division by zero errors
            if (period > 0) {
            float frequency = 1000000.0f / period;  // Calculate frequency in Hz
            float duty_cycle = ((float)pulse_width / period) * 100.0f;
            if (duty_cycle > 100.0f) duty_cycle = 100.0f; // Clamp duty cycle

            // Print results to the serial monitor
            printf("Pulse Width: %u us, Period: %u us, Frequency: %.2f Hz, Duty Cycle: %.2f%%\n",
                   pulse_width, period, frequency, duty_cycle);

            sleep_ms(500); // Delay for readability; adjust as needed
        }

        

uart_putc(uart1, 'A');
    sleep_ms(10);  // Delay to ensure stability

    if (uart_is_readable(uart1)) {
        char received = uart_getc(uart1);
        printf("Received: %c\n", received);
    } else {
        printf("No data available on UART RX\n");
    }
    sleep_ms(1000);  // Adjust as needed
}
        // Read ADC (from analysis_and_monitoring.c)
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);
          // Send a message over UART at the detected or specified baud rate
      
}

  
