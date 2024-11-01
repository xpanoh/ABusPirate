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
#define PWM_INPUT_PIN 7         // Pin for PWM signal input to analyze
#define UART_RX_PIN 4           // UART1 receiving RX pin (GP4)
#define BUTTON_PIN 20           // Pin for button to initiate baud rate detection
#define TOLERANCE_PERCENT 5      // Tolerance as a percentage of the expected duration

// Fast Fourier Transform (FFT) settings
#define NFFT 256                // Number of FFT points, must be a power of 2
#define SAMPLING_FREQUENCY 1000 // Sampling frequency in Hz

// Define standard baud rates and their expected bit durations
typedef struct {
    uint32_t baud_rate;
    uint32_t expected_duration; // Expected bit duration in microseconds
} BaudRateInfo;

BaudRateInfo baud_rates[] = {
    {4800, 208}, {9600, 104}, {19200, 52},
    {38400, 26}, {57600, 17}, {115200, 9},
    {230400, 4}, {460800, 2}, {921600, 1}
};

// Global Variables for PWM capture
volatile uint32_t rising_edge_time = 0;
volatile uint32_t falling_edge_time = 0;
volatile uint32_t pulse_width = 0;
volatile uint32_t period = 0;
volatile bool pwm_ready = false;

// Global variables for UART baud rate measurement
volatile bool uart_start_bit_detected = false;
volatile bool start_baud_detection = false;
uint32_t uart_start_time = 0;
uint32_t uart_baud_rate = 0;

// KissFFT arrays for input and output
kiss_fft_cpx fft_in[NFFT], fft_out[NFFT];

// Function to configure the ADC on ADC_PIN
void configure_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0); // Assuming channel 0 is connected to ADC_PIN
}

// Function to capture the PWM signal's rising and falling edges on PWM_INPUT_PIN
void gpio_callback_pwm(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();
    if (gpio == PWM_INPUT_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            period = current_time - rising_edge_time;
            rising_edge_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            falling_edge_time = current_time;
            pulse_width = falling_edge_time - rising_edge_time;
            pwm_ready = true;
        }
    }
}

// Function to handle button press on BUTTON_PIN to initiate baud rate detection
void gpio_callback_button(uint gpio, uint32_t events) {
    if (gpio == BUTTON_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        start_baud_detection = true;
    }
}

// Function to configure the PWM input pin for measurement
void configure_pwm_input() {
    gpio_init(PWM_INPUT_PIN);
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);
    gpio_pull_down(PWM_INPUT_PIN);
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_pwm);
}

// Function to configure the button on GPIO 20
void configure_button() {
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_button);
}

// Function to detect and identify baud rate on GP4 RX pin
void check_uart_baud_rate() {
    uint32_t current_time = time_us_32();

    if (!uart_start_bit_detected) {
        if (gpio_get(UART_RX_PIN) == 0) {
            uart_start_bit_detected = true;
            uart_start_time = current_time;
        }
    } else {
        if (gpio_get(UART_RX_PIN) == 1) {
            uint32_t bit_duration = current_time - uart_start_time;
            uart_baud_rate = 0;

            for (int i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++) {
                uint32_t expected_duration = baud_rates[i].expected_duration;
                uint32_t lower_bound = expected_duration * (100 - TOLERANCE_PERCENT) / 100;
                uint32_t upper_bound = expected_duration * (100 + TOLERANCE_PERCENT) / 100;

                if (bit_duration >= lower_bound && bit_duration <= upper_bound) {
                    uart_baud_rate = baud_rates[i].baud_rate;
                    break;
                }
            }

            if (uart_baud_rate != 0) {
                printf("Detected UART Baud Rate: %u bps\n", uart_baud_rate);
            } else {
                printf("Baud Rate detection failed.\n");
            }

            uart_start_bit_detected = false;
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

    for (int i = 0; i < NFFT; i++) {
        uint16_t raw_adc = adc_read();
        float voltage = raw_adc * 3.3f / (1 << 12);
        fft_in[i].r = voltage;
        fft_in[i].i = 0;
        sleep_us(1000000 / SAMPLING_FREQUENCY);
    }

    kiss_fft(cfg, fft_in, fft_out);

    float max_magnitude = 0;
    int dominant_frequency_bin = 0;
    for (int i = 0; i < NFFT / 2; i++) {
        float magnitude = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            dominant_frequency_bin = i;
        }
    }

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
    configure_button(); // Configure button to initiate baud rate detection
    gpio_init(UART_RX_PIN);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);

    printf("System Initialized.\n");
    
    while (1) {
        if (pwm_ready) {
            if (period > 0) {
                float pwm_frequency = 1000000.0f / period;
                float duty_cycle = ((float)pulse_width / period) * 100.0f;
                if (duty_cycle > 100.0f) duty_cycle = 100.0f;

                printf("PWM Frequency: %.2f Hz, Duty Cycle: %.2f%%\n", pwm_frequency, duty_cycle);
            }
            pwm_ready = false;
        }

        perform_fft_analysis();

        if (start_baud_detection) {
            printf("Initiating UART Baud Rate Detection...\n");
            check_uart_baud_rate();
            start_baud_detection = false;
        }

        sleep_ms(50);
    }
}