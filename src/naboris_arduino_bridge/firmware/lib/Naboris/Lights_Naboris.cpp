#include "Lights_Naboris.h"

/* -------------------------- *
 * LED strip global variables *
 * -------------------------- */


 Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_SIGNAL_PIN, NEO_GRB + NEO_KHZ800);
 int signal_r, signal_g, signal_b = 0;


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbowCycle(uint8_t wait) {
    uint16_t i, j;

    for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
        for(i=0; i< strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        strip.show();
        delay(wait);
    }
}


void fadeColors(int r, int g, int b, uint16_t cycles, uint8_t wait, int increment) {
    fadeColors(signal_r, signal_g, signal_b, r, g, b, cycles, wait, increment);
}

void fadeColors(int r1, int g1, int b1, int r2, int g2, int b2, uint16_t cycles, uint8_t wait, int increment)
{
    // Serial.print(r1); Serial.print('\t');
    // Serial.print(g1); Serial.print('\t');
    // Serial.print(b1); Serial.print('\n');
    // Serial.print(r2); Serial.print('\t');
    // Serial.print(g2); Serial.print('\t');
    // Serial.print(b2); Serial.print('\n');

    if (cycles % 2 == 0) {
        signal_r = r1;
        signal_g = g1;
        signal_b = b1;
    }
    else {
        signal_r = r2;
        signal_g = g2;
        signal_b = b2;
    }
    int red_diff = abs(r2 - r1);
    int green_diff = abs(g2 - g1);
    int blue_diff = abs(b2 - b1);

    char max_channel = 'r';
    int max_diff = red_diff;

    if (green_diff > max_diff) {
        max_diff = green_diff;
        max_channel = 'g';
    }
    if (blue_diff > max_diff) {
        max_diff = blue_diff;
        max_channel = 'b';
    }
    // Serial.println(max_channel);

    float red_slope = 0.0;
    float green_slope = 0.0;
    float blue_slope = 0.0;

    int start = 0;
    int end = 0;

    bool condition = true;

    switch (max_channel) {
        case 'r':
            if (r2 < r1) {
                increment *= -1;
            }
            break;
        case 'g':
            if (g2 < g1) {
                increment *= -1;
            }
            break;
        case 'b':
            if (b2 < b1) {
                increment *= -1;
            }
            break;
    }

    // Serial.println(cycles);
    for (uint16_t cycle = 0; cycle < cycles; cycle++)
    {
        switch (max_channel) {
            case 'r':
                condition = r1 < r2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = r1;
                    end = r2;
                }
                else {
                    start = r2;
                    end = r1;
                }
                green_slope = (float)(g2 - g1) / (r2 - r1);
                blue_slope = (float)(b2 - b1) / (r2 - r1);

                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                value,
                                green_slope * (value - r1) + g1,
                                blue_slope * (value - r1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                else if (start > end) {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                value,
                                green_slope * (value - r1) + g1,
                                blue_slope * (value - r1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                break;

            case 'g':
                condition = g1 < g2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = g1;
                    end = g2;
                }
                else {
                    start = g2;
                    end = g1;
                }


                red_slope = (float)(r2 - r1) / (g2 - g1);
                blue_slope = (float)(b2 - b1) / (g2 - g1);
                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - g1) + r1,
                                value,
                                blue_slope * (value - g1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                else {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - g1) + r1,
                                value,
                                blue_slope * (value - g1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                break;
            case 'b':
                condition = b1 < b2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = b1;
                    end = b2;
                }
                else {
                    start = b2;
                    end = b1;
                }
                red_slope = (float)(r2 - r1) / (b2 - b1);
                green_slope = (float)(g2 - g1) / (b2 - b1);

                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - b1) + r1,
                                green_slope * (value - b1) + g1,
                                value
                            )
                        );
                        delay(wait);
                    }
                }
                else {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - b1) + r1,
                                green_slope * (value - b1) + g1,
                                value
                            )
                        );
                        delay(wait);
                    }
                }

                break;
        }
        increment *= -1;

    }
}

void setColor(uint32_t c)
{
    for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
    }
    strip.show();
}
