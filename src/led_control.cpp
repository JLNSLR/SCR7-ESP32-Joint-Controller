#include <led_control.h>

CRGB leds[N_LEDS];

int led_blink_period_ms = 750;

#define STANDARD_COLOUR CRGB::OrangeRed
#define STANDARD_BRIGHTNESS 100

int led_brightness = 255;

void init_leds() {

    FastLED.addLeds<NEOPIXEL, RGB_LED_PIN>(leds, N_LEDS);
    leds[0] = STANDARD_COLOUR;

    led_brightness = STANDARD_BRIGHTNESS;
    FastLED.setBrightness(led_brightness);
    FastLED.show();

}

void blink_leds() {
    static long last_time = 0;
    static bool toggle = false;

    if (led_blink_period_ms != 0) {
        long current_time = millis();

        if (current_time - last_time >= led_blink_period_ms) {

            last_time = current_time;
            toggle = !toggle;

            if (toggle) {
                FastLED.setBrightness(0);
                FastLED.show();
            }
            else {
                FastLED.setBrightness(led_brightness);
                FastLED.show();
            }
        }
    }
}

void set_leds(uint8_t rh, uint8_t gs, uint8_t bv, bool rgb, int blink_period_ms) {

    if (rgb) {
        leds->setRGB(rh, gs, bv);
        FastLED.show();
    }
    else {
        leds->setHSV(rh, gs, bv);
        FastLED.show();
    }

    led_blink_period_ms = led_blink_period_ms;
}

void set_error() {

    leds->setRGB(255, 0, 0);
    led_blink_period_ms = 250;
    FastLED.setBrightness(255);
    FastLED.show();

}

void set_standard_lights() {

    leds[N_LEDS] = STANDARD_COLOUR;
    led_blink_period_ms = 0;
    FastLED.setBrightness(STANDARD_BRIGHTNESS);
    FastLED.show();
}
