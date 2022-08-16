#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <FastLED.h>
#include <drive_system_settings.h>
#include <joint_control_global_def.h>


void init_leds();

void blink_leds();

void set_leds(uint8_t rh, uint8_t gs, uint8_t bv, bool rgb, int blink_period_ms = 0);

void set_error();

void set_standard_lights();


#endif // !LED_CONTROL