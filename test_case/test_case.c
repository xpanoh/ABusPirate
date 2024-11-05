#include <stdio.h>          // Include stdio.h for printf
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define PWM_PIN 6         // GPIO6 for PWM output
#define UART_ID uart0     // UART0 for communication
#define TX_PIN 12          // GPIO0 as TX for UART

// Array of commonly used baud rates
const uint32_t baud_rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
const int baud_rate_count = sizeof(baud_rates) / sizeof(baud_rates[0]);
const char *test_message = "UART Baud Rate Test\n";
const uint32_t test_duration_ms = 5000;  // Duration to test each baud rate (5 seconds)

// Function to set the UART baud rate and send test data
void send_uart_data(uint32_t baud_rate) {
    // Initialize UART with the current baud rate
    uart_init(UART_ID, baud_rate);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    printf("Setting UART baud rate to %u bps\n", baud_rate);

    // Transmit data at the set baud rate for the specified duration
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start_time < test_duration_ms) {
        uart_puts(UART_ID, test_message);   // Send the test message
        sleep_ms(1000);                      // Wait for 1 second between transmissions
    }
}

int main() {
    stdio_init_all();  // Initialize standard I/O for printf (optional)

    // Initialize GPIO pin for PWM
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    
    // Find PWM slice number for the chosen GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    
    // Set PWM frequency to 1 kHz
    uint32_t pwm_frequency = 1000;
    uint32_t clock_frequency = 125000000;  // Pico's default 125 MHz clock
    uint16_t wrap_value = clock_frequency / pwm_frequency;

    pwm_set_wrap(slice_num, wrap_value);
    
    // Set duty cycle to 50%
    pwm_set_gpio_level(PWM_PIN, wrap_value / 2);
    
    // Enable the PWM slice
    pwm_set_enabled(slice_num, true);

    // Cycle through each baud rate in the array
    while (1) {
        for (int i = 0; i < baud_rate_count; i++) {
            send_uart_data(baud_rates[i]);
        }
    }

    return 0;
}
