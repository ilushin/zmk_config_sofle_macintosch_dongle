/*
 * A320 trackpad HID over I2C driver (Zephyr Input Subsystem)
 * Stability-hardened version for ZitaoTech Sofle.
 *
 * Key changes:
 * - balanced I2C mutex handling on every path
 * - initialized/propagated I2C errors
 * - bounded FIFO drain to prevent runaway work
 * - wider accumulators and bounded HID output
 * - no blocking HID waits
 * - no zero-motion reports
 * - rate-limited diagnostics
 * - touch state derived from recent motion instead of a short-lived flag
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT avago_a320

#include <errno.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/hid.h>

#include "a320.h"
#include "trackpad_led.h"

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

#define A320_WORKQ_STACK_SIZE 2048
#define A320_WORKQ_PRIORITY 8
#define A320_I2C_LOCK_TIMEOUT_MS 5
#define A320_MAX_DRAIN_PACKETS 16
#define A320_ERROR_LOG_INTERVAL_MS 5000U
#define A320_TOUCH_ACTIVE_MS 100U
#define A320_POINTER_LIMIT 127
#define A320_SCROLL_TICK_LIMIT 16
#define A320_RESIDUE_LIMIT 4096

#define SCROLL_X_DIR (-CONFIG_A320_SCROLL_X_DIR)
#define SCROLL_Y_DIR CONFIG_A320_SCROLL_Y_DIR
#define SCROLL_DEADZONE CONFIG_A320_SCROLL_DEADZONE
#define SCROLL_INPUT_MAX CONFIG_A320_SCROLL_INPUT_MAX
#define SCROLL_DIVISOR_SLOW CONFIG_A320_SCROLL_DIVISOR_SLOW
#define SCROLL_DIVISOR_FAST CONFIG_A320_SCROLL_DIVISOR_FAST

#define ARROW_DEADZONE CONFIG_A320_SCROLL_DEADZONE
#define ARROW_INPUT_MAX 128

#define DOMINANT_NUMERATOR CONFIG_A320_DOMINANT_NUMERATOR
#define DOMINANT_DENOMINATOR CONFIG_A320_DOMINANT_DENOMINATOR

#define MOUSE_BASE_SPEED (CONFIG_A320_MOUSE_BASE_SPEED_PERCENT / 100.0f)
#define MOUSE_SENS_BASE (CONFIG_A320_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_A320_MOUSE_SENS_STEP_PERCENT / 100.0f)
#define SLOW_KEY_MULTIPLIER 0.5f

#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 5
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

#define A320_PACKET_LEN 3

static struct k_mutex a320_i2c_mutex;
K_THREAD_STACK_DEFINE(a320_workq_stack, A320_WORKQ_STACK_SIZE);
static struct k_work_q a320_workq;

static bool scroll_key_pressed;
static bool arrow_key_pressed;
static bool slow_key_pressed;
static bool last_scroll_key_pressed;
static bool last_arrow_key_pressed;
static uint32_t last_motion_ms;
static uint32_t last_i2c_error_log_ms;

static zmk_hid_indicators_t current_indicators;
#define HID_INDICATORS_CAPS_LOCK (1U << 1)

struct a320_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct a320_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work;
    int32_t scroll_residue_x;
    int32_t scroll_residue_y;
    int32_t arrow_residue_x;
    int32_t arrow_residue_y;
};

static void reset_residue(struct a320_data *data) {
    data->scroll_residue_x = 0;
    data->scroll_residue_y = 0;
    data->arrow_residue_x = 0;
    data->arrow_residue_y = 0;
}

static void log_i2c_error_rate_limited(int err) {
    uint32_t now = k_uptime_get_32();

    if (last_i2c_error_log_ms == 0U ||
        (uint32_t)(now - last_i2c_error_log_ms) >= A320_ERROR_LOG_INTERVAL_MS) {
        LOG_WRN("A320 I2C read failed: %d", err);
        last_i2c_error_log_ms = now;
    }
}

static int a320_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct a320_config *cfg = dev->config;
    uint8_t buf[A320_PACKET_LEN] = {0};
    uint8_t reg = 0x82;

    int ret = k_mutex_lock(&a320_i2c_mutex, K_MSEC(A320_I2C_LOCK_TIMEOUT_MS));
    if (ret != 0) {
        return ret;
    }

    ret = i2c_write_dt(&cfg->i2c, &reg, sizeof(reg));
    if (ret == 0) {
        ret = i2c_burst_read_dt(&cfg->i2c, reg, buf, sizeof(buf));
    }

    if (ret == 0) {
        *dx = (int8_t)buf[1];
        *dy = -(int8_t)buf[2];
    }

    int unlock_ret = k_mutex_unlock(&a320_i2c_mutex);
    if (ret == 0 && unlock_ret != 0) {
        ret = unlock_ret;
    }

    return ret;
}

static inline int32_t clamp_residue(int32_t value) {
    return CLAMP(value, -A320_RESIDUE_LIMIT, A320_RESIDUE_LIMIT);
}

static inline int32_t nonlinear_divisor(int32_t abs_delta) {
    const int32_t input_max = MAX(1, SCROLL_INPUT_MAX);
    const int32_t slow = MAX(1, SCROLL_DIVISOR_SLOW);
    const int32_t fast = MAX(1, SCROLL_DIVISOR_FAST);
    const float t_linear = (float)MIN(abs_delta, input_max) / (float)input_max;
    const float t = t_linear * t_linear;
    int32_t divisor = (int32_t)((float)slow - ((float)(slow - fast) * t));
    return MAX(1, divisor);
}

static void process_scroll_axis(const struct device *dev, int32_t delta, int32_t *residue,
                                uint16_t input_code, int32_t dir_mult) {
    int32_t abs_delta = abs(delta);
    if (abs_delta <= SCROLL_DEADZONE) {
        return;
    }

    int32_t bounded_delta = CLAMP(delta, -MAX(1, SCROLL_INPUT_MAX), MAX(1, SCROLL_INPUT_MAX));
    int32_t divisor = nonlinear_divisor(abs_delta);

    *residue = clamp_residue(*residue + (bounded_delta * dir_mult));
    int32_t ticks = CLAMP(*residue / divisor, -A320_SCROLL_TICK_LIMIT,
                          A320_SCROLL_TICK_LIMIT);

    if (ticks != 0) {
        input_report_rel(dev, input_code, ticks, true, K_NO_WAIT);
        *residue -= ticks * divisor;
    }

    *residue = (*residue * 3) / 4;
}

static void process_arrow_axis(const struct device *dev, int32_t delta, int32_t *residue,
                               uint16_t key_neg, uint16_t key_pos) {
    int32_t abs_delta = abs(delta);
    if (abs_delta <= ARROW_DEADZONE) {
        return;
    }

    int32_t bounded_delta = CLAMP(delta, -ARROW_INPUT_MAX, ARROW_INPUT_MAX);
    int32_t divisor = nonlinear_divisor(abs_delta);
    *residue = clamp_residue(*residue + bounded_delta);

    int32_t ticks = *residue / divisor;
    if (ticks != 0) {
        uint16_t key = ticks > 0 ? key_pos : key_neg;
        input_report_key(dev, key, 1, false, K_NO_WAIT);
        input_report_key(dev, key, 0, true, K_NO_WAIT);
        *residue %= divisor;
    }

    *residue = (*residue * 3) / 4;
}

static void report_pointer(const struct device *dev, int32_t dx, int32_t dy) {
    uint8_t brightness = indicator_tp_get_last_valid_brightness();
    float sensitivity = MOUSE_SENS_BASE + (MOUSE_SENS_STEP * brightness);
    float pointer_scale = 0.5f * MOUSE_BASE_SPEED * sensitivity;
    float slow_scale = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;

    int32_t out_x = CLAMP((int32_t)((float)dx * pointer_scale * slow_scale),
                          -A320_POINTER_LIMIT, A320_POINTER_LIMIT);
    int32_t out_y = CLAMP((int32_t)((float)dy * pointer_scale * slow_scale),
                          -A320_POINTER_LIMIT, A320_POINTER_LIMIT);

    if (out_x == 0 && out_y == 0) {
        return;
    }

    if (out_x != 0) {
        input_report_rel(dev, INPUT_REL_X, out_x, out_y == 0, K_NO_WAIT);
    }
    if (out_y != 0) {
        input_report_rel(dev, INPUT_REL_Y, out_y, true, K_NO_WAIT);
    }
}

static void apply_dominant_axis(int32_t *dx, int32_t *dy) {
    int32_t abs_dx = abs(*dx);
    int32_t abs_dy = abs(*dy);

    if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
        *dx = 0;
    } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
        *dy = 0;
    } else {
        *dx = 0;
        *dy = 0;
    }
}

static void a320_work_cb(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, work);
    const struct device *dev = data->dev;

    int32_t total_dx = 0;
    int32_t total_dy = 0;
    bool got_data = false;

    for (int packet = 0; packet < A320_MAX_DRAIN_PACKETS; packet++) {
        int8_t dx = 0;
        int8_t dy = 0;
        int ret = a320_read_packet(dev, &dx, &dy);

        if (ret != 0) {
            if (!got_data) {
                log_i2c_error_rate_limited(ret);
            }
            break;
        }

        if (dx == 0 && dy == 0) {
            break;
        }

        total_dx += dx;
        total_dy += dy;
        got_data = true;
    }

    if (!got_data) {
        return;
    }

    uint32_t now = k_uptime_get_32();
    last_motion_ms = now;

    total_dx = CLAMP(total_dx, -512, 512);
    total_dy = CLAMP(total_dy, -512, 512);

    bool just_enter_scroll = scroll_key_pressed && !last_scroll_key_pressed;
    bool just_enter_arrow = arrow_key_pressed && !last_arrow_key_pressed;
    bool capslock = (current_indicators & HID_INDICATORS_CAPS_LOCK) != 0;

    if (arrow_key_pressed) {
        if (just_enter_arrow) {
            data->arrow_residue_x = total_dx;
            data->arrow_residue_y = total_dy;
        }

        apply_dominant_axis(&total_dx, &total_dy);
        process_arrow_axis(dev, total_dx, &data->arrow_residue_x, INPUT_BTN_1, INPUT_BTN_0);
        process_arrow_axis(dev, total_dy, &data->arrow_residue_y, INPUT_BTN_3, INPUT_BTN_2);
    } else if (scroll_key_pressed || capslock) {
        if (just_enter_scroll) {
            data->scroll_residue_x = total_dx * SCROLL_X_DIR;
            data->scroll_residue_y = total_dy * SCROLL_Y_DIR;
        }

        apply_dominant_axis(&total_dx, &total_dy);
        process_scroll_axis(dev, -total_dx, &data->scroll_residue_x, INPUT_REL_HWHEEL,
                            SCROLL_X_DIR);
        process_scroll_axis(dev, -total_dy, &data->scroll_residue_y, INPUT_REL_WHEEL,
                            SCROLL_Y_DIR);
    } else {
        report_pointer(dev, total_dx, total_dy);
    }

    last_scroll_key_pressed = scroll_key_pressed;
    last_arrow_key_pressed = arrow_key_pressed;
}

bool tp_is_touched(void) {
    if (last_motion_ms == 0U) {
        return false;
    }
    return (uint32_t)(k_uptime_get_32() - last_motion_ms) <= A320_TOUCH_ACTIVE_MS;
}

static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev != NULL) {
        current_indicators = ev->indicators;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(a320_hid_listener, hid_indicators_listener);
ZMK_SUBSCRIPTION(a320_hid_listener, zmk_hid_indicators_changed);

static int special_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->position == 35) {
        arrow_key_pressed = ev->state;
        LOG_DBG("arrow mode: %d", arrow_key_pressed);
    }
    if (ev->position == 61 || ev->position == 62) {
        scroll_key_pressed = ev->state;
        LOG_DBG("scroll mode: %d", scroll_key_pressed);
    }
    if (ev->position == 37) {
        slow_key_pressed = ev->state;
        LOG_DBG("slow mode: %d", slow_key_pressed);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(a320_special_key_listener, special_key_listener_cb);
ZMK_SUBSCRIPTION(a320_special_key_listener, zmk_position_state_changed);

static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb_data);
    (void)k_work_submit_to_queue(&a320_workq, &data->work);
}

static void a320_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, enable_irq_work);
    const struct a320_config *cfg = data->dev->config;

    int ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("A320 IRQ enable failed: %d", ret);
    } else {
        LOG_DBG("A320 IRQ enabled");
    }
}

static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c) || !gpio_is_ready_dt(&cfg->motion_gpio)) {
        return -ENODEV;
    }

    k_mutex_init(&a320_i2c_mutex);
    data->dev = dev;
    reset_residue(data);
    last_motion_ms = 0U;
    last_i2c_error_log_ms = 0U;

    k_work_init(&data->work, a320_work_cb);
    k_work_queue_start(&a320_workq, a320_workq_stack, K_THREAD_STACK_SIZEOF(a320_workq_stack),
                       A320_WORKQ_PRIORITY, NULL);

    int ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    if (ret != 0) {
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_DISABLE);
    if (ret != 0) {
        return ret;
    }

    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);
    if (ret != 0) {
        return ret;
    }

    k_work_init_delayable(&data->enable_irq_work, a320_enable_irq_work_cb);
    (void)k_work_schedule(&data->enable_irq_work, K_MSEC(200));

    LOG_INF("A320 driver initialized (stability-hardened)");
    return 0;
}

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static const struct a320_config a320_config_##inst = {                                         \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                   \
                        .pin = MOTION_GPIO_PIN,                                                      \
                        .dt_flags = MOTION_GPIO_FLAGS},                                             \
    };                                                                                              \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_config_##inst,           \
                          POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE);
