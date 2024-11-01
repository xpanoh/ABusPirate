#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>

#define UART_ID uart1
#define UART_TX_PIN 5  // Transmit Pin for testing (GPIO 5)
#define UART_RX_PIN 4  // Receive Pin for detecting (GPIO 4)
#define TEST_PATTERN "U" // Known test pattern to detect

// List of possible baud rates to test
const int baud_rates[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
const int num_baud_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

void setup_uart(int baud_rate) {
    // Initialize UART with the specified baud rate
    uart_init(UART_ID, baud_rate);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void transmit_test_signal(int baud_rate) {
    setup_uart(baud_rate);
    uart_puts(UART_ID, TEST_PATTERN);  // Transmit known test pattern
    sleep_ms(100000);  // Small delay between transmissions
}

bool detect_baud_rate() {
    // Loop through each baud rate and attempt to read data
    for (int i = 0; i < num_baud_rates; i++) {
        int baud = baud_rates[i];
        setup_uart(baud);

        // Wait briefly for UART to stabilize
        sleep_ms(50);

        // Check if data is available and matches the test pattern
        if (uart_is_readable(UART_ID)) {
            char received = uart_getc(UART_ID);
            if (received == TEST_PATTERN[0]) {  // Check if received character matches pattern
                printf("Detected baud rate: %d\n", baud);
                return true;
            }
        }
    }
    return false;  // No matching baud rate detected
}

int main() {
    stdio_init_all();
    printf("UART Baud Rate Detection Test on GPIO 4 (RX) and GPIO 5 (TX)\n");

    // Transmit test signals at various baud rates on GPIO 5
    for (int i = 0; i < num_baud_rates; i++) {
        transmit_test_signal(baud_rates[i]);
    }

    // Attempt to detect the transmitted baud rate on GPIO 4
    if (!detect_baud_rate()) {
        printf("Failed to detect baud rate.\n");
    }

    return 0;
}
