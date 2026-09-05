/*
 * A320 trackpad indicator LED.
 * Stability-oriented version: low-rate state polling, no 5 ms permanent loop.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/activity.h>
#include <zmk/backlight.h>
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>

#include "a320.h"
#include "trackpad_led.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define HID_INDICATORS_CAPS_LOCK (1U << 1)

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_trackpad_led),
             "Trackpad LED enabled but no zmk,trackpad_led chosen node found");

static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_trackpad_led));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define LED_COUNT DT_NUM_CHILD(DT_CHOSEN(zmk_trackpad_led))

#define BRT_MIN 10
#define BRT_MAX 100
#define BRT_LOW 20
#define BRT_STEP 5
#define CAPS_ANIMATION_INTERVAL_MS 40
#define ACTIVE_POLL_INTERVAL_MS 25
#define IDLE_POLL_INTERVAL_MS 100
#define AUTO_OFF_DELAY_MS 5000

static struct k_work_delayable poll_work;
static struct k_work_delayable animation_work;
static struct k_work_delayable auto_off_work;

static bool capslock_on;
static bool touch_active;
static bool animation_increasing = true;
static bool keyboard_active;
static uint8_t brightness = BRT_MIN;
static uint8_t last_valid_brt = BRT_MAX;
static uint8_t last_backlight_brt;

static void set_led_brightness(uint8_t level) {
    if (!device_is_ready(led_dev)) {
        return;
    }

    level = MIN(level, BRT_MAX);
    for (int i = 0; i < LED_COUNT; i++) {
        int ret = led_set_brightness(led_dev, i, level);
        if (ret != 0) {
            LOG_DBG("Trackpad LED[%d] update failed: %d", i, ret);
        }
    }
}

static void auto_off_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    if (!capslock_on && !tp_is_touched()) {
        touch_active = false;
        set_led_brightness(0);
    }
}

static void animation_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    if (!capslock_on) {
        return;
    }

    set_led_brightness(brightness);
    if (animation_increasing) {
        brightness = MIN((int)brightness + BRT_STEP, BRT_MAX);
        if (brightness >= BRT_MAX) {
            animation_increasing = false;
        }
    } else {
        brightness = MAX((int)brightness - BRT_STEP, BRT_LOW);
        if (brightness <= BRT_LOW) {
            animation_increasing = true;
        }
    }

    (void)k_work_reschedule(&animation_work, K_MSEC(CAPS_ANIMATION_INTERVAL_MS));
}

static void poll_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    bool current_touch = tp_is_touched();
    bool current_active = zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE;
    uint8_t current_brt = zmk_backlight_get_brt();

    if (current_active != keyboard_active) {
        keyboard_active = current_active;
        if (keyboard_active && current_brt > 0) {
            last_backlight_brt = current_brt;
            last_valid_brt = MAX(BRT_MIN, MIN(current_brt, BRT_MAX));
        }
    }

    if (!capslock_on && current_touch != touch_active) {
        touch_active = current_touch;
        if (touch_active) {
            if (keyboard_active && current_brt > 0) {
                last_valid_brt = MAX(BRT_MIN, MIN(current_brt, BRT_MAX));
            }
            set_led_brightness(last_valid_brt);
            (void)k_work_cancel_delayable(&auto_off_work);
        } else {
            (void)k_work_reschedule(&auto_off_work, K_MSEC(AUTO_OFF_DELAY_MS));
        }
    }

    if (!capslock_on && keyboard_active && current_brt != last_backlight_brt) {
        last_backlight_brt = current_brt;
        if (current_brt > 0) {
            last_valid_brt = MAX(BRT_MIN, MIN(current_brt, BRT_MAX));
            if (touch_active) {
                set_led_brightness(last_valid_brt);
            }
        }
    }

    int interval = (current_touch || keyboard_active) ? ACTIVE_POLL_INTERVAL_MS
                                                       : IDLE_POLL_INTERVAL_MS;
    (void)k_work_reschedule(&poll_work, K_MSEC(interval));
}

static int trackpad_led_hid_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    bool new_capslock = (ev->indicators & HID_INDICATORS_CAPS_LOCK) != 0;
    if (new_capslock == capslock_on) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    capslock_on = new_capslock;
    if (capslock_on) {
        brightness = BRT_MIN;
        animation_increasing = true;
        (void)k_work_cancel_delayable(&auto_off_work);
        (void)k_work_reschedule(&animation_work, K_NO_WAIT);
    } else {
        (void)k_work_cancel_delayable(&animation_work);
        touch_active = tp_is_touched();
        if (touch_active) {
            set_led_brightness(last_valid_brt);
        } else {
            set_led_brightness(0);
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

uint8_t indicator_tp_get_last_valid_brightness(void) { return last_valid_brt; }

static int indicator_tp_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("Trackpad LED device not ready");
        return -ENODEV;
    }

    capslock_on = false;
    touch_active = false;
    keyboard_active = false;
    last_backlight_brt = zmk_backlight_get_brt();
    if (last_backlight_brt > 0) {
        last_valid_brt = MAX(BRT_MIN, MIN(last_backlight_brt, BRT_MAX));
    }

    k_work_init_delayable(&poll_work, poll_work_handler);
    k_work_init_delayable(&animation_work, animation_work_handler);
    k_work_init_delayable(&auto_off_work, auto_off_work_handler);

    set_led_brightness(0);
    (void)k_work_reschedule(&poll_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(indicator_tp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(trackpad_led_listener, trackpad_led_hid_listener);
ZMK_SUBSCRIPTION(trackpad_led_listener, zmk_hid_indicators_changed);
