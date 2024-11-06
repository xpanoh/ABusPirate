#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <math.h>
#include "kiss_fft.h"

#define ADC_PIN 26
#define FFT_SIZE 1024         // Size of FFT; power of 2
#define SAMPLE_RATE 10000     // Sample rate in Hz

kiss_fft_cpx in[FFT_SIZE], out[FFT_SIZE];  // Complex input/output for FFT

void adc_init_gpio() {
    adc_init();
    adc_gpio_init(ADC_PIN);   // Initialize GPIO26 for ADC
    adc_select_input(0);      // Select ADC0 (corresponding to GPIO26)
}

// Capture samples and remove DC offset
void capture_samples() {
    int sum = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        in[i].r = adc_read();  // Real part from ADC
        in[i].i = 0;           // Imaginary part is zero
        sum += in[i].r;
        sleep_us(1000000 / SAMPLE_RATE);  // Adjust for sample rate
    }

    // Remove DC offset
    int avg = sum / FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++) {
        in[i].r -= avg;
    }

    // Optional: Print sample values to verify
    for (int i = 0; i < 10; i++) { // Print first 10 samples for inspection
        printf("Sample %d: %d\n", i, in[i].r);
    }
}

// Analyze FFT output and find dominant frequency
float analyze_fft(kiss_fft_cpx *out) {
    float max_magnitude = 0;
    int max_index = 0;

    // Find the bin with the highest magnitude
    for (int i = 1; i < FFT_SIZE / 2; i++) {  // Start from 1 to skip the DC component
        float magnitude = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_index = i;
        }
    }

    // Optional: Print magnitude values for debugging
    printf("Max magnitude bin: %d, magnitude: %.2f\n", max_index, max_magnitude);

    // Calculate frequency of the dominant bin
    float frequency = (float)max_index * SAMPLE_RATE / FFT_SIZE;
    return frequency;
}

int main() {
    stdio_init_all();
    adc_init_gpio();

    // Initialize Kiss FFT
    kiss_fft_cfg cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);

    while (true) {
        capture_samples();  // Capture ADC data into FFT buffer

        // Perform FFT
        kiss_fft(cfg, in, out);

        // Analyze FFT result
        float frequency = analyze_fft(out);
        printf("Estimated Frequency: %.2f Hz\n", frequency);

        sleep_ms(1000);  // Output frequency once per second
    }

    // Free FFT config (though usually not reached in an embedded main loop)
    free(cfg);

    return 0;
}