#ifndef __LIGHTS_NABORIS_H__
#define __LIGHTS_NABORIS_H__

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>


#define NUM_LEDS 24
#define LED_SIGNAL_PIN 6

#define SIGNAL_DELAY 1
#define SIGNAL_INCREMENT 1
#define SIGNAL_CYCLES 4
#define SIGNAL_COLOR 100

uint32_t Wheel(byte WheelPos);
void rainbowCycle(uint8_t wait);
void fadeColors(int r, int g, int b, uint16_t cycles, uint8_t wait, int increment);
void fadeColors(int r1, int g1, int b1, int r2, int g2, int b2, uint16_t cycles, uint8_t wait, int increment);
void setColor(uint32_t c);

#endif  // __LIGHTS_NABORIS_H__
