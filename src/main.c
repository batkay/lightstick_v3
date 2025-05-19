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
#include <zephyr/drivers/sensor.h>


#include "hsv2rgb.h"
#include "orientation.h"

#define DEBUG

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
const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

#define MAX_BRIGHTNESS 0x25
// Create RGB Colors
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb red = RGB(MAX_BRIGHTNESS, 0x00, 0x00);
static struct led_rgb bluetoothColor = RGB(0x00, 0x00, MAX_BRIGHTNESS);

static struct gpio_callback button_cb_data;

static struct led_rgb pixels[STRIP_NUM_PIXELS];
static bool orientationInit;
static uint32_t prevTimeMs;
static bool sensorInit;

enum STATE {
    OFF, FIXED, WAVE, IMU
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
            return IMU;
            break;
        case IMU:
            return OFF;
            break;
    }
    return FIXED;
}

static int process_mpu6050(const struct device *dev) {
    uint32_t currTimeMs = k_uptime_get_32();

    struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}



	if (rc == 0) {
		// printf("[%s]:%g Cel\n"
		//        "  accel %f %f %f m/s/s\n"
		//        "  gyro  %f %f %f rad/s\n",
		//        now_str(),
		//        sensor_value_to_double(&accel[0]),
		//        sensor_value_to_double(&accel[1]),
		//        sensor_value_to_double(&accel[2]),
		//        sensor_value_to_double(&gyro[0]),
		//        sensor_value_to_double(&gyro[1]),
		//        sensor_value_to_double(&gyro[2]));

        double accYval = sensor_value_to_double(&accel[1]) * 3.1415 / 180.0;
        double accXval = sensor_value_to_double(&accel[0]) * 3.1415 / 180.0;
        double accZval = sensor_value_to_double(&accel[2]) * 3.1415 / 180.0;
    
        double pitch = atan2(accYval, accZval);
        double roll = atan2(accXval, accZval);
    
        if (!orientationInit) {
            orientation_initialize(pitch, roll, 0.0);
            orientationInit = true;
        }
        else {
            update_pitch(pitch, sensor_value_to_double(&gyro[0]), (currTimeMs - prevTimeMs)/1000.0);
            update_roll(roll, sensor_value_to_double(&gyro[1]), (currTimeMs - prevTimeMs)/1000.0);
            update_yaw(sensor_value_to_double(&gyro[2]), (currTimeMs - prevTimeMs)/1000.0);
        }
	} else {
		printf("sample fetch/get failed: %d\n", rc);
	}

    prevTimeMs = currTimeMs;


	return rc;
}

// button callback to switch states
void buttonPressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    currentState = nextState(currentState);
    printk("State %d\n", currentState);
}

int main(void)
{
    int err = 0;
    // Setup user push button
    err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_DISABLE));

    err = gpio_pin_configure_dt(&button, (GPIO_INPUT));
    if (err) {
        #ifdef DEBUG
        printk("Error %d: failed to configure button", err);
        #endif
        return 0;
    }
    err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_DISABLE));

	while (true) {
		int out = gpio_pin_get_dt(&button);
		if (out == 0) {
			break;
		}
		// printk("out: %i\n", out);

		k_sleep(K_MSEC(50));
	}

	if (!device_is_ready(enable_switch)) {
        printk("Regulator device not ready\n");
        return 0;
    }

	err = regulator_enable(enable_switch);
    if (err < 0) {
        printk("Failed to enable regulator: %d\n", err);
    } else {
        printk("Regulator enabled\n");
    }
	printf("enable on\n");



    err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_EDGE_TO_INACTIVE));
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
    sensorInit = false;
    err = device_init(mpu6050);
	if (err) {
		printk("Sensor failed init %i \n", err);
		// return 0;
	}
    else {
        sensorInit = true;
    }
	if (!sensorInit && !device_is_ready(mpu6050)) {
		printk("sensor: device not ready.\n");
		// return 0;
	}
    orientationInit = false;
    currentState = FIXED;
    bool update = false;
    int idx = 0;
    while (1) {		
        if (sensorInit) {
            process_mpu6050(mpu6050);
        }
        else {
            err = device_init(mpu6050);
            if (!err) {
                sensorInit = true;
                printk("initialized sensor");
            }
        }
        switch(currentState) {
            case OFF:
                err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_DISABLE));

                err = regulator_disable(enable_switch);
                if (err < 0) {
                    printk("Failed to disable regulator: %d\n", err);
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
                gpio_remove_callback(button.port, &button_cb_data);
                err = gpio_pin_interrupt_configure_dt(&button, 	GPIO_INT_LEVEL_ACTIVE);
                if (err) {
                    #ifdef DEBUG
                    printf("Could not remove callback (%d)\n", err);
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
            case IMU:
                double rotation_magnitude = sqrt(pow(get_delta_pitch(), 2) + pow(get_delta_roll(), 2));
                
                if (rotation_magnitude > 1) {
                    idx += (int) rotation_magnitude;
                    idx = idx % (360);
                }

                bluetoothColor = hsv2rgb(idx, 100, (MAX_BRIGHTNESS)/255.0 * 100);

                memset(&pixels, 0x00, sizeof(pixels));
                for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
                    memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
                }
                update = true;

                #ifdef DEBUG
                // printf("Pitch:%f,Yaw:%f,Roll:%f\n", get_delta_pitch(), get_delta_yaw(), get_delta_roll());
                #endif
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