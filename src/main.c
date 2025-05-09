/*
* Copyright (c) 2017 Linaro Limited
* Copyright (c) 2018 Intel Corporation
* Copyright (c) 2024 TOKITA Hiroshi
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <errno.h>
#include <string.h>
#include <math.h>

#include <zephyr/logging/log.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/drivers/regulator.h>

#include "hsv2rgb.h"

// #define DEBUG

// Define Devicetree Aliases
#define STRIP_NODE		DT_ALIAS(led_strip)

#define STRIP_NUM_PIXELS	16



#define CONFIG_SAMPLE_LED_UPDATE_DELAY 50
#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define LED_ENABLE DT_ALIAS(led_enable)
const struct device *enable_switch = DEVICE_DT_GET(LED_ENABLE);

#define BUTTON0 DT_ALIAS(sw0)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON0, gpios, {0});


#define MAX_BRIGHTNESS 0x25
// Create RGB Colors
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb red = RGB(MAX_BRIGHTNESS, 0x00, 0x00);
static struct led_rgb bluetoothColor = RGB(0x00, 0x00, MAX_BRIGHTNESS);

static struct gpio_callback button_cb_data;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

enum STATE {
    OFF, FIXED, WAVE
};

volatile enum STATE currentState = FIXED;

static enum STATE nextState(enum STATE currentState) {
    switch(currentState){
        case OFF:
            return FIXED;
            break;
        case FIXED:
            return WAVE;
            break;
        case WAVE:
            return OFF;
            break;
    }
    return FIXED;
}

// button callback to switch states
void buttonPressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    currentState = nextState(currentState);
}

int main(void)
{
    int err = 0;
	if (!device_is_ready(enable_switch)) {
        printk("Regulator device not ready\n");
        return 0;
    }

	int ret = regulator_enable(enable_switch);
    if (ret < 0) {
        printk("Failed to enable regulator: %d\n", ret);
    } else {
        printk("Regulator enabled\n");
    }
	printf("enable on\n");

    // Setup user push button
    err = gpio_pin_configure_dt(&button, (GPIO_INPUT));
    if (err) {
        #ifdef DEBUG
        printk("Error %d: failed to configure button", err);
        #endif
        return 0;
    }

    err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_EDGE_TO_ACTIVE));
    if (err) {
        #ifdef DEBUG
        printf("Could not configure sw0 GPIO interrupt (%d)\n", err);
        #endif
        return 0;
    }

    gpio_init_callback(&button_cb_data, buttonPressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    // // Setup LED Strip
    if (device_is_ready(strip)) {
        #ifdef DEBUG
        printf("Found LED strip device %s", strip->name);
        #endif
    } 
    else {
        #ifdef DEBUG
        printf("LED strip device %s is not ready", strip->name);
        #endif
        return 0;
    }

    bool update = false;
    int idx = 0;
    while (1) {		
        switch(currentState) {
            case OFF:
                int ret = regulator_disable(enable_switch);
                if (ret < 0) {
                    printk("Failed to disable regulator: %d\n", ret);
                } else {
                    printk("Regulator disabled\n");
                }

                memset(&pixels, 0x00, sizeof(pixels));
                for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
                    memcpy(&pixels[cursor], &(struct led_rgb) RGB(0, 0, 0), sizeof(struct led_rgb));
                }
                err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
                if (err) {
                    #ifdef DEBUG
                    printf("couldn't update strip: %d", err);
                    #endif
                }
                err = gpio_pin_interrupt_configure_dt(&button, 	GPIO_INT_LEVEL_ACTIVE);
                if (err) {
                    #ifdef DEBUG
                    printf("Could not configure sw0 GPIO interrupt (%d)\n", err);
                    #endif
                    return 0;
                }

                sys_poweroff();
                break;

            case FIXED:


                memset(&pixels, 0x00, sizeof(pixels));
                for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
                    memcpy(&pixels[cursor], &red, sizeof(struct led_rgb));
                }
                update = true;
                
                break;
            case WAVE:                
                memset(&pixels, 0x00, sizeof(pixels));
                for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
                    bluetoothColor = hsv2rgb((cursor + idx) % 360, 100, (MAX_BRIGHTNESS)/255.0 * 100);
                    memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
                }
                update = true;
                // idx = (idx + 1) % (3 * STRIP_NUM_PIXELS);
                idx = (idx + 8) % 360;
                break;
        }
        
        if (update) {
            err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
            if (err) {
                #ifdef DEBUG
                printf("couldn't update strip: %d", err);
                #endif
            }
            update = false;
        }

        k_sleep(DELAY_TIME);
    }

    return 0;
}