#ifndef LED_RING_H
#define LED_RING_H

#include <stdint.h>
#include <stddef.h>

// Animates the LED ring with a rotating pattern
void animate_ring_color(const char *color_name, int active_leds);

// Sets the entire LED ring to a solid color
void set_ring_color(const char *color_name);

void test_led_ring(void);

#endif 
