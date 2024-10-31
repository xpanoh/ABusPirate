#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "kiss_fft.h"
#include <math.h>
#include <stdint.h>

// Pin definitions
#define ADC_PIN 26              // Pin for ADC input
#define PWM_INPUT_PIN 27         // Pin for PWM signal input to analyze
#define UART_RX_PIN 4           // UART1 RX pin (GP4)

// FFT settings
#define NFFT 256                // Number of FFT points, must be a power of 2
#define SAMPLING_FREQUENCY 1000 // Sampling frequency in Hz

// Global Variables for PWM capture
volatile uint32_t rising_edge_time = 0;  // Time of the rising edge for PWM input
volatile uint32_t falling_edge_time = 0; // Time of the falling edge for PWM input
volatile uint32_t pulse_width = 0;       // Pulse width in microseconds for PWM input
volatile uint32_t period = 0;            // Period of the PWM signal in microseconds
volatile bool pwm_ready = false;         // Flag for new PWM measurement ready

// Variables for UART baud rate measurement
volatile bool uart_start_bit_detected = false; // Flag for detecting start bit
uint32_t uart_start_time = 0;
uint32_t uart_baud_rate = 0;

// KissFFT arrays for input and output
kiss_fft_cpx fft_in[NFFT], fft_out[NFFT];

// Function to configure the ADC on ADC_PIN (Analog input)
void configure_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0); // Assuming channel 0 is connected to ADC_PIN
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

// Function to configure the PWM input pin for measurement
void configure_pwm_input() {
    gpio_init(PWM_INPUT_PIN);
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);
    gpio_pull_down(PWM_INPUT_PIN);
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Function to measure UART baud rate non-blocking
void check_uart_baud_rate() {
    uint32_t current_time = time_us_32();

    if (!uart_start_bit_detected) {
        // Look for the start bit (line going low)
        if (gpio_get(UART_RX_PIN) == 0) {
            uart_start_bit_detected = true;
            uart_start_time = current_time;
        }
    } else {
        // Look for the stop bit (line going high after start bit detected)
        if (gpio_get(UART_RX_PIN) == 1) {
            uint32_t bit_duration = current_time - uart_start_time;
            uart_baud_rate = 1000000 / bit_duration; // Calculate baud rate in bps
            printf("UART Baud Rate: %u bps\n", uart_baud_rate);
            uart_start_bit_detected = false; // Reset flag for next measurement
        }
    }
}

// Function to perform FFT analysis on ADC data
void perform_fft_analysis() {
    kiss_fft_cfg cfg = kiss_fft_alloc(NFFT, 0, NULL, NULL);
    if (!cfg) {
        printf("Not enough memory for FFT.\n");
        return;
    }

    // Collect ADC samples
    for (int i = 0; i < NFFT; i++) {
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12); // Convert ADC value to voltage
        fft_in[i].r = voltage;   // Real part of the input
        fft_in[i].i = 0;         // Imaginary part of the input (0 for real signals)
        sleep_us(1000000 / SAMPLING_FREQUENCY); // Sampling delay
    }

    // Execute FFT
    kiss_fft(cfg, fft_in, fft_out);

    // Analyze FFT results: calculate the magnitude and print the dominant frequency
    float max_magnitude = 0;
    int dominant_frequency_bin = 0;
    for (int i = 0; i < NFFT / 2; i++) {
        float magnitude = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            dominant_frequency_bin = i;
        }
    }

    // Calculate and print the dominant frequency
    float frequency_resolution = (float)SAMPLING_FREQUENCY / NFFT;
    float dominant_frequency = dominant_frequency_bin * frequency_resolution;
    printf("Dominant Frequency: %.2f Hz, Magnitude: %.2f\n", dominant_frequency, max_magnitude);

    free(cfg);
}

int main() {
    stdio_init_all();

    // Configure ADC, PWM input, and UART RX for baud rate measurement
    configure_adc();
    configure_pwm_input();
    gpio_init(UART_RX_PIN);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);

    printf("System Initialized.\n");

    while (1) {
        // Check PWM data readiness
        if (pwm_ready) {
            if (period > 0) {
                float pwm_frequency = 1000000.0f / period;  // Frequency in Hz
                float duty_cycle = ((float)pulse_width / period) * 100.0f;
                if (duty_cycle > 100.0f) duty_cycle = 100.0f; // Clamp duty cycle

                // Display PWM frequency and duty cycle
                printf("PWM Frequency: %.2f Hz, Duty Cycle: %.2f%%\n", pwm_frequency, duty_cycle);
            }
            pwm_ready = false;  // Reset flag
        }

        // Perform FFT analysis on the collected ADC data from the IR sensor
        perform_fft_analysis();

        // Non-blocking check for UART baud rate
        check_uart_baud_rate();

        sleep_ms(50); // Delay for readability and stability in serial output
    }
}