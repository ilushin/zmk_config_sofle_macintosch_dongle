/*
 * TrackPoint indicator LED following keyboard backlight.
 * Stability-oriented version with a 50 ms observation interval.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/backlight.h>

#include "custom_led.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_custom_led),
             "TrackPoint LED enabled but no zmk,custom_led chosen node found");

static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_custom_led));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define LED_COUNT DT_NUM_CHILD(DT_CHOSEN(zmk_custom_led))

#define BRT_MIN 10
#define OFF_DELAY_MS 3000
#define POLL_INTERVAL_MS 50
#define FADE_STEP_MS 20
#define FADE_STEPS 20

static struct k_work_delayable auto_off_work;
static struct k_work_delayable poll_work;
static struct k_work_delayable fade_work;

static uint8_t last_brt = UINT8_MAX;
static uint8_t current_brt;
static uint8_t fade_start_brt;
static uint8_t target_brt;
static uint8_t fade_step;
static bool fade_active;
static uint8_t last_valid_brt = BRT_MIN;

static void apply_led(uint8_t brightness) {
    if (!device_is_ready(led_dev)) {
        return;
    }

    for (int i = 0; i < LED_COUNT; i++) {
        int ret = led_set_brightness(led_dev, i, brightness);
        if (ret != 0) {
            LOG_DBG("TrackPoint LED[%d] update failed: %d", i, ret);
        }
    }
    current_brt = brightness;
}

static void fade_handler(struct k_work *work) {
    ARG_UNUSED(work);
    if (!fade_active) {
        return;
    }

    fade_step++;
    int32_t difference = (int32_t)target_brt - (int32_t)fade_start_brt;
    int32_t level = (int32_t)fade_start_brt +
                    (difference * MIN(fade_step, FADE_STEPS) / FADE_STEPS);
    apply_led((uint8_t)CLAMP(level, 0, UINT8_MAX));

    if (fade_step >= FADE_STEPS) {
        fade_active = false;
        apply_led(target_brt);
        return;
    }

    (void)k_work_reschedule(&fade_work, K_MSEC(FADE_STEP_MS));
}

static void fade_to(uint8_t brightness) {
    target_brt = brightness;
    fade_start_brt = current_brt;
    fade_step = 0;
    fade_active = true;
    (void)k_work_reschedule(&fade_work, K_NO_WAIT);
}

static void auto_off_handler(struct k_work *work) {
    ARG_UNUSED(work);
    fade_to(0);
}

static void poll_handler(struct k_work *work) {
    ARG_UNUSED(work);

    uint8_t brt = zmk_backlight_get_brt();
    if (brt != last_brt) {
        last_brt = brt;
        uint8_t led_level = brt == 0 ? 0 : MAX(BRT_MIN, brt);

        if (led_level > 0) {
            last_valid_brt = led_level;
            if (current_brt == 0) {
                fade_to(led_level);
            } else {
                fade_active = false;
                (void)k_work_cancel_delayable(&fade_work);
                apply_led(led_level);
            }
            (void)k_work_reschedule(&auto_off_work, K_MSEC(OFF_DELAY_MS));
        } else {
            (void)k_work_cancel_delayable(&auto_off_work);
            fade_to(0);
        }
    }

    (void)k_work_reschedule(&poll_work, K_MSEC(POLL_INTERVAL_MS));
}

uint8_t custom_led_get_last_valid_brightness(void) { return last_valid_brt; }

static int init_led_follow(void) {
    if (!device_is_ready(led_dev)) {
        return -ENODEV;
    }

    k_work_init_delayable(&auto_off_work, auto_off_handler);
    k_work_init_delayable(&poll_work, poll_handler);
    k_work_init_delayable(&fade_work, fade_handler);

    current_brt = 0;
    fade_active = false;
    apply_led(0);
    (void)k_work_reschedule(&poll_work, K_NO_WAIT);

    LOG_DBG("TrackPoint LED follow driver initialized");
    return 0;
}

SYS_INIT(init_led_follow, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
