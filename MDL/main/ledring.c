#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"

// Configuration constants
#define LED_PIN        5               // GPIO pin connected to the LED ring's data input
#define LED_COUNT      24              // Number of LEDs in the ring
#define BRIGHTNESS     10               // Brightness level (0–255)
#define RMT_CHANNEL    RMT_CHANNEL_0   // RMT channel used for transmission
#define RMT_CLK_DIV    2               // Clock divider: 80MHz / 2 = 40MHz (25ns resolution)

// Duration of logical '0' high signal (0.35 µs at 25 ns resolution)
#define T0H 14
// Duration of logical '0' low signal (0.85 µs at 25 ns resolution)
#define T0L 34
// Duration of logical '1' high signal (0.7 µs at 25 ns resolution)
#define T1H 28
// Duration of logical '1' low signal (0.6 µs at 25 ns resolution)
#define T1L 20
// Reset time between data frames (80 µs > 50 µs minimum required)
#define RESET_US 80

// Send a raw RGB buffer to the WS2812 LED ring
void ws2812_send(uint8_t *data, size_t length) {
  size_t num_bits = length * 8;             // Total number of bits (8 per byte)
  rmt_item32_t items[num_bits + 50];        // Array of RMT items (+ extra for reset pulse)
  size_t i = 0;                              // Current index in items[]

  for (size_t byte_idx = 0; byte_idx < length; byte_idx++) {
      uint8_t byte = data[byte_idx];

      for (int bit = 7; bit >= 0; bit--) {
          if (byte & (1 << bit)) {
              items[i].level0 = 1;
              items[i].duration0 = T1H;
              items[i].level1 = 0;
              items[i].duration1 = T1L;
          } else {
              items[i].level0 = 1;
              items[i].duration0 = T0H;
              items[i].level1 = 0;
              items[i].duration1 = T0L;
          }
          i++;
      }
  }

  items[i].level0 = 0;
  items[i].duration0 = (RESET_US * 40);
  items[i].level1 = 0;
  items[i].duration1 = 0;
  i++;

  rmt_write_items(RMT_CHANNEL, items, i, true);
  rmt_wait_tx_done(RMT_CHANNEL, pdMS_TO_TICKS(100));
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
        memset(leds, 0, sizeof(leds));

        for (int i = 0; i < active_leds; i++) {
            int led_index = (pos + i) % LED_COUNT;
            leds[led_index * 3]     = g;
            leds[led_index * 3 + 1] = r;
            leds[led_index * 3 + 2] = b;
        }

        ws2812_send(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(100));
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
    else if (strcmp(color_name, "off") == 0) {
        r = g = b = 0;
    }
    

    uint8_t leds[LED_COUNT * 3];
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i * 3]     = g;
        leds[i * 3 + 1] = r;
        leds[i * 3 + 2] = b;
    }

    ws2812_send(leds, sizeof(leds));
}



void test_led_ring(void) {
    
    set_ring_color("white");
    vTaskDelay(pdMS_TO_TICKS(1000));

    animate_ring_color("red", 3);

    set_ring_color("green");
    vTaskDelay(pdMS_TO_TICKS(1000));

    animate_ring_color("yellow", 5);

    set_ring_color("off");
}
