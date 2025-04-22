#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"

// Configuration constants
#define LED_PIN        5               // GPIO pin connected to the LED ring's data input
#define LED_COUNT      24              // Number of LEDs in the ring
#define BRIGHTNESS     5             // Brightness level (0–255)
#define RMT_CHANNEL    RMT_CHANNEL_0   // RMT channel used for transmission
#define RMT_CLK_DIV    2               // Clock divider: 80MHz / 2 = 40MHz (25ns resolution)

// WS2812 timing definitions (in RMT ticks with 25ns resolution)
#define T0H 14  // '0' bit high time: 0.35us
#define T0L 34  // '0' bit low time: 0.85us
#define T1H 28  // '1' bit high time: 0.7us
#define T1L 20  // '1' bit low time: 0.6us

#define RESET_US 80  // Reset time in microseconds (minimum 50us)

// Initialize the RMT peripheral for WS2812 signal generation
void ws2812_init() {
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = LED_PIN,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }
    };
    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);
}

// Send a raw RGB buffer to the WS2812 LED ring
void ws2812_send(uint8_t *data, size_t length) {
  size_t num_bits = length * 8;             // Total number of bits (8 per byte)
  rmt_item32_t items[num_bits + 50];        // Array of RMT items (+ extra for reset pulse)
  size_t i = 0;                              // Current index in items[]

  // Loop through each byte (G, R, B for each LED)
  for (size_t byte_idx = 0; byte_idx < length; byte_idx++) {
      uint8_t byte = data[byte_idx];

      // For each bit (MSB first)
      for (int bit = 7; bit >= 0; bit--) {
          if (byte & (1 << bit)) {
              // If bit is '1' → longer high pulse
              items[i].level0 = 1;
              items[i].duration0 = T1H;     // High time
              items[i].level1 = 0;
              items[i].duration1 = T1L;     // Low time
          } else {
              // If bit is '0' → shorter high pulse
              items[i].level0 = 1;
              items[i].duration0 = T0H;
              items[i].level1 = 0;
              items[i].duration1 = T0L;
          }
          i++;
      }
  }

  // Add reset pulse (low for at least 50μs to latch data)
  items[i].level0 = 0;
  items[i].duration0 = (RESET_US * 40);     // 40 ticks/us → 50us = 2000 ticks
  items[i].level1 = 0;
  items[i].duration1 = 0;
  i++;

  // Start transmitting RMT data
  rmt_write_items(RMT_CHANNEL, items, i, true);         // Wait until done
  rmt_wait_tx_done(RMT_CHANNEL, pdMS_TO_TICKS(100));   // Timeout in case something fails
}

// Animate a rotating light with N active LEDs in the specified color
void animate_ring_color(const char *color_name, int active_leds) {
    uint8_t g = 0, r = 0, b = 0;

    if (strcmp(color_name, "red") == 0) {
        r = BRIGHTNESS;
    } else if (strcmp(color_name, "green") == 0) {
        g = BRIGHTNESS;
    } else if (strcmp(color_name, "yellow") == 0) {
        r = BRIGHTNESS;
        g = BRIGHTNESS;
    } else if (strcmp(color_name, "white") == 0) {
        r = BRIGHTNESS;
        g = BRIGHTNESS;
        b = BRIGHTNESS;
    }

    uint8_t leds[LED_COUNT * 3];

    for (int pos = 0; pos < LED_COUNT; pos++) {
        // Clear all LEDs
        memset(leds, 0, sizeof(leds));

        // Light up a trail of active LEDs starting at current position
        for (int i = 0; i < active_leds; i++) {
            int led_index = (pos + i) % LED_COUNT;
            leds[led_index * 3]     = g;
            leds[led_index * 3 + 1] = r;
            leds[led_index * 3 + 2] = b;
        }

        ws2812_send(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay between animation steps
    }
}

// Set the entire ring to a solid color
void set_ring_color(const char *color_name) {
    uint8_t g = 0, r = 0, b = 0;

    if (strcmp(color_name, "red") == 0) {
        r = BRIGHTNESS;
    } else if (strcmp(color_name, "green") == 0) {
        g = BRIGHTNESS;
    } else if (strcmp(color_name, "yellow") == 0) {
        r = BRIGHTNESS;
        g = BRIGHTNESS;
    } else if (strcmp(color_name, "white") == 0) {
        r = BRIGHTNESS;
        g = BRIGHTNESS;
        b = BRIGHTNESS;
    }

    uint8_t leds[LED_COUNT * 3];
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i * 3]     = g;
        leds[i * 3 + 1] = r;
        leds[i * 3 + 2] = b;
    }

    ws2812_send(leds, sizeof(leds));
}

// Main application loop
void app_main(void) {
    ws2812_init();

    while (1) {
        animate_ring_color("red", 5);
    }
}
