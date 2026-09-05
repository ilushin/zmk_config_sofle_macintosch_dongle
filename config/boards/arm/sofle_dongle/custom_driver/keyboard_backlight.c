/*
 * Peripheral keyboard backlight controller.
 * Event-driven stability version with no permanent WPM timer.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/backlight.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/rgb_underglow.h>

LOG_MODULE_REGISTER(keyboard_backlight, CONFIG_ZMK_LOG_LEVEL);

#define KEYBOARD_BACKLIGHT_NODE DT_NODELABEL(keyboard_backlight)
static const struct device *const backlight_dev = DEVICE_DT_GET(KEYBOARD_BACKLIGHT_NODE);

#define MAX_BRT 20
#define MIN_BRT 0
#define FADE_STEP 2
#define FADE_INTERVAL_MS 60
#define BOOT_FADE_DELAY_MS 3000
#define AUTO_OFF_MS 1000

enum bl_state {
    BL_OFF,
    BL_FADING_UP,
    BL_ON,
    BL_FADING_DOWN,
};

static enum bl_state bl_state = BL_OFF;
static int current_brt;
static struct k_work_delayable bl_work;
static struct k_work_delayable idle_off_work;

static void set_brightness(int brightness) {
    if (!device_is_ready(backlight_dev)) {
        return;
    }

    brightness = CLAMP(brightness, MIN_BRT, MAX_BRT);
    (void)led_set_brightness(backlight_dev, 0, brightness);
    current_brt = brightness;
}

static void force_off(void) {
    bl_state = BL_OFF;
    set_brightness(MIN_BRT);
}

static void bl_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    bool rgb_on = true;
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW)
    zmk_rgb_underglow_get_state(&rgb_on);
#endif

    if (!rgb_on) {
        force_off();
        return;
    }

    if (bl_state == BL_FADING_UP) {
        current_brt += FADE_STEP;
        if (current_brt >= MAX_BRT) {
            current_brt = MAX_BRT;
            bl_state = BL_ON;
        }
        set_brightness(current_brt);
        if (bl_state == BL_FADING_UP) {
            (void)k_work_reschedule(&bl_work, K_MSEC(FADE_INTERVAL_MS));
        }
        return;
    }

    if (bl_state == BL_FADING_DOWN) {
        current_brt -= FADE_STEP;
        if (current_brt <= MIN_BRT) {
            current_brt = MIN_BRT;
            bl_state = BL_OFF;
        }
        set_brightness(current_brt);
        if (bl_state == BL_FADING_DOWN) {
            (void)k_work_reschedule(&bl_work, K_MSEC(FADE_INTERVAL_MS));
        }
    }
}

static void trigger_activity(void) {
    if (!device_is_ready(backlight_dev)) {
        return;
    }

    if (bl_state == BL_OFF || bl_state == BL_FADING_DOWN) {
        bl_state = BL_FADING_UP;
        (void)k_work_reschedule(&bl_work, K_NO_WAIT);
    }

    (void)k_work_reschedule(&idle_off_work, K_MSEC(AUTO_OFF_MS));
}

static void idle_off_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (bl_state == BL_ON || bl_state == BL_FADING_UP) {
        bl_state = BL_FADING_DOWN;
        (void)k_work_reschedule(&bl_work, K_NO_WAIT);
    }
}

static int kb_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || ev->source != ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    trigger_activity();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(keyboard_backlight_listener, kb_listener_cb);
ZMK_SUBSCRIPTION(keyboard_backlight_listener, zmk_position_state_changed);

static int keyboard_backlight_init(void) {
    if (!device_is_ready(backlight_dev)) {
        LOG_ERR("Keyboard backlight device not ready");
        return -ENODEV;
    }

    k_work_init_delayable(&bl_work, bl_work_handler);
    k_work_init_delayable(&idle_off_work, idle_off_handler);

    current_brt = MIN_BRT;
    set_brightness(current_brt);

    bl_state = BL_FADING_UP;
    (void)k_work_reschedule(&bl_work, K_NO_WAIT);
    (void)k_work_reschedule(&idle_off_work, K_MSEC(BOOT_FADE_DELAY_MS));

    LOG_DBG("Keyboard backlight initialized (event-driven)");
    return 0;
}

SYS_INIT(keyboard_backlight_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
