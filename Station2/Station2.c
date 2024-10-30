#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdint.h>

// Pin definitions
#define ADC_PIN 26          // Pin for ADC input
#define PWM_PIN0 0          // Pin for PWM output (example pin for testing PWM generation)
#define PWM_INPUT_PIN 7     // Pin for PWM signal input to analyze
#define UART_TX_PIN 5       // UART1 TX pin (GP5)
#define UART_RX_PIN 4       // UART1 RX pin (GP4)

// Global Variables
volatile uint32_t rising_edge_time = 0;  // Time of the rising edge for PWM input
volatile uint32_t falling_edge_time = 0; // Time of the falling edge for PWM input
volatile uint32_t pulse_width = 0;       // Pulse width in microseconds for PWM input
volatile uint32_t period = 0;            // Period of the PWM signal in microseconds
volatile bool pwm_ready = false;         // Flag for new PWM measurement ready

// Function to initialize UART
void setup_uart() {
    uart_init(uart1, 9600); // Baud rate of 9600
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Function to capture the PWM signal's rising and falling edges on PWM_INPUT_PIN
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();
    if (gpio == PWM_INPUT_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            period = current_time - rising_edge_time; // Calculate period
            rising_edge_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            falling_edge_time = current_time;
            pulse_width = falling_edge_time - rising_edge_time; // Calculate pulse width
            pwm_ready = true; // Set flag for main loop to process
        }
    }
}

// Function to configure PWM input pin for measurement
void configure_pwm_input() {
    gpio_init(PWM_INPUT_PIN);
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);
    gpio_pull_down(PWM_INPUT_PIN);
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Function to set up ADC on ADC_PIN (Analog input)
void configure_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

// Function to calculate UART baud rate by measuring time between bits on RX pin
void measure_uart_baud_rate() {
    uint32_t start_time = 0, stop_time = 0;
    bool start_bit_detected = false;

    // Loop to detect the start and stop bits on the UART RX line
    while (true) {
        if (gpio_get(UART_RX_PIN) == 0 && !start_bit_detected) {
            // Start bit detected (line goes low)
            start_time = time_us_32();
            start_bit_detected = true;
        }
        if (gpio_get(UART_RX_PIN) == 1 && start_bit_detected) {
            // Stop bit detected (line goes high after data bits)
            stop_time = time_us_32();
            break;
        }
    }

    // Calculate baud rate
    uint32_t bit_duration = stop_time - start_time;
    uint32_t baud_rate = 1000000 / bit_duration;
    printf("UART Baud Rate: %u bps\n", baud_rate);
}

int main() {
    stdio_init_all();
    
    // Configure ADC
    configure_adc();

    // Configure PWM input on GP7 and UART
    configure_pwm_input();
    setup_uart();

    printf("System Initialized.\n");

    while (1) {
        // Check PWM data readiness
        if (pwm_ready) {
            // Ensure period is not zero to avoid division errors
            if (period > 0) {
                float pwm_frequency = 1000000.0f / period;  // Frequency in Hz
                float duty_cycle = ((float)pulse_width / period) * 100.0f;
                if (duty_cycle > 100.0f) duty_cycle = 100.0f; // Clamp duty cycle

                // Display PWM frequency and duty cycle
                printf("PWM Frequency: %.2f Hz, Duty Cycle: %.2f%%\n", pwm_frequency, duty_cycle);
            }
            pwm_ready = false;  // Reset flag
        }

        // Read ADC value and calculate frequency of the analog signal
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12); // Convert raw ADC to voltage
        printf("ADC Voltage: %.2f V\n", voltage);

        // Placeholder for analog frequency analysis (for demonstration)
        // Example: if a zero-crossing detection method is used to determine analog frequency
        // Additional code required here for actual frequency calculation

        // Measure UART baud rate periodically
        measure_uart_baud_rate();

        sleep_ms(1000); // Wait for readability and stability
    }
}