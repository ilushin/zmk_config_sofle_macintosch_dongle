/*
 * TrackPoint HID over I2C driver (Zephyr Input Subsystem)
 * Stability-hardened version for ZitaoTech Sofle.
 *
 * Key changes:
 * - validates mutex acquisition before I2C access
 * - no sleeps or infinite waits inside the input work queue
 * - skips zero-value HID reports
 * - bounds mouse/scroll output to prevent report bursts
 * - rate-limits transient I2C diagnostics
 * - coalesces GPIO interrupt work through the Zephyr work item
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <errno.h>
#include <math.h>
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
#include <zmk/events/position_state_changed.h>
#include <zmk/hid.h>

#include "custom_led.h"

LOG_MODULE_REGISTER(trackpoint, CONFIG_TRACKPOINT_LOG_LEVEL);

#define TP_WORKQ_STACK_SIZE 2048
#define TP_WORKQ_PRIORITY 8
#define TP_I2C_LOCK_TIMEOUT_MS 5
#define TP_ERROR_LOG_INTERVAL_MS 5000U
#define TP_POINTER_LIMIT 127
#define TP_SCROLL_LIMIT 16
#define TP_ERROR_RESET_THRESHOLD 3

#define SCROLL_X_DIR (-CONFIG_TRACKPOINT_SCROLL_X_DIR)
#define SCROLL_Y_DIR CONFIG_TRACKPOINT_SCROLL_Y_DIR
#define SCROLL_DEADZONE CONFIG_TRACKPOINT_SCROLL_DEADZONE
#define SCROLL_INPUT_MAX CONFIG_TRACKPOINT_SCROLL_INPUT_MAX
#define SCROLL_DIVISOR_SLOW CONFIG_TRACKPOINT_SCROLL_DIVISOR_SLOW
#define SCROLL_DIVISOR_FAST CONFIG_TRACKPOINT_SCROLL_DIVISOR_FAST

#define ARROW_DEADZONE CONFIG_TRACKPOINT_SCROLL_DEADZONE
#define ARROW_INPUT_MAX 256

#define DOMINANT_NUMERATOR CONFIG_TRACKPOINT_DOMINANT_NUMERATOR
#define DOMINANT_DENOMINATOR CONFIG_TRACKPOINT_DOMINANT_DENOMINATOR

#define MOUSE_BASE_SPEED (CONFIG_TRACKPOINT_MOUSE_BASE_SPEED_PERCENT / 100.0f)
#define MOUSE_SENS_BASE (CONFIG_TRACKPOINT_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_TRACKPOINT_MOUSE_SENS_STEP_PERCENT / 100.0f)
#define SLOW_KEY_MULTIPLIER 0.5f

#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

static struct k_mutex trackpoint_i2c_mutex;
K_THREAD_STACK_DEFINE(tp_workq_stack, TP_WORKQ_STACK_SIZE);
static struct k_work_q tp_workq;

static bool scroll_key_pressed;
static bool arrow_key_pressed;
static bool slow_key_pressed;
static bool last_scroll_key_pressed;
static bool last_arrow_key_pressed;
static uint32_t last_error_log_ms;

struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work;
    uint32_t last_packet_time;
    int32_t arrow_residue_x;
    int32_t arrow_residue_y;
    float scroll_fraction_x;
    float scroll_fraction_y;
    uint8_t consecutive_errors;
};

static void reset_motion_state(struct trackpoint_data *data) {
    data->arrow_residue_x = 0;
    data->arrow_residue_y = 0;
    data->scroll_fraction_x = 0.0f;
    data->scroll_fraction_y = 0.0f;
}

static void log_read_error_rate_limited(int err) {
    uint32_t now = k_uptime_get_32();
    if (last_error_log_ms == 0U ||
        (uint32_t)(now - last_error_log_ms) >= TP_ERROR_LOG_INTERVAL_MS) {
        LOG_WRN("TrackPoint I2C read failed: %d", err);
        last_error_log_ms = now;
    }
}

static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};

    int ret = k_mutex_lock(&trackpoint_i2c_mutex, K_MSEC(TP_I2C_LOCK_TIMEOUT_MS));
    if (ret != 0) {
        return ret;
    }

    ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));

    int unlock_ret = k_mutex_unlock(&trackpoint_i2c_mutex);
    if (ret == 0 && unlock_ret != 0) {
        ret = unlock_ret;
    }
    if (ret != 0) {
        return ret;
    }

    if (buf[0] != TRACKPOINT_MAGIC_BYTE0) {
        return -EIO;
    }

    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
#define TP_MAX_MULT 2.0f
static float trackpoint_exponential_factor(int8_t dx, int8_t dy, uint32_t delta_ms) {
    if (delta_ms == 0U) {
        delta_ms = 1U;
    }

    int distance = ABS((int)dx) + ABS((int)dy);
    if (distance < 1) {
        return 1.0f;
    }

    float speed = (float)distance / (float)delta_ms;
    float multiplier = expf(speed * 1.307357f);
    return MIN(multiplier, TP_MAX_MULT);
}
#endif

static int32_t nonlinear_divisor(int32_t abs_delta) {
    int32_t input_max = MAX(1, SCROLL_INPUT_MAX);
    int32_t slow = MAX(1, SCROLL_DIVISOR_SLOW);
    int32_t fast = MAX(1, SCROLL_DIVISOR_FAST);
    float linear = (float)MIN(abs_delta, input_max) / (float)input_max;
    float t = linear * linear;
    int32_t divisor = (int32_t)((float)slow - ((float)(slow - fast) * t));
    return MAX(1, divisor);
}

static void process_arrow_axis(const struct device *dev, int32_t delta, int32_t *residue,
                               uint16_t key_neg, uint16_t key_pos) {
    int32_t abs_delta = ABS(delta);
    if (abs_delta <= ARROW_DEADZONE) {
        return;
    }

    int32_t divisor = nonlinear_divisor(abs_delta);
    *residue = CLAMP(*residue + CLAMP(delta, -ARROW_INPUT_MAX, ARROW_INPUT_MAX), -4096, 4096);

    int32_t ticks = *residue / divisor;
    if (ticks != 0) {
        uint16_t key = ticks > 0 ? key_pos : key_neg;
        input_report_key(dev, key, 1, false, K_NO_WAIT);
        input_report_key(dev, key, 0, true, K_NO_WAIT);
        *residue %= divisor;
    }

    *residue = (*residue * 3) / 4;
}

static void apply_dominant_axis(int32_t *dx, int32_t *dy) {
    int32_t abs_dx = ABS(*dx);
    int32_t abs_dy = ABS(*dy);

    if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
        *dx = 0;
    } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
        *dy = 0;
    } else {
        *dx = 0;
        *dy = 0;
    }
}

static void report_scroll(struct trackpoint_data *data, const struct device *dev, int32_t dx,
                          int32_t dy) {
    float speed = sqrtf((float)(dx * dx + dy * dy));
    float scale = speed > 80.0f   ? 0.05f
                  : speed > 40.0f ? 0.04f
                  : speed > 20.0f ? 0.03f
                  : speed > 5.0f  ? 0.02f
                                  : 0.015f;

    data->scroll_fraction_x += (float)dx * scale * SCROLL_X_DIR;
    data->scroll_fraction_y += (float)dy * scale * SCROLL_Y_DIR;

    int32_t out_x = CLAMP((int32_t)data->scroll_fraction_x, -TP_SCROLL_LIMIT, TP_SCROLL_LIMIT);
    int32_t out_y = CLAMP((int32_t)data->scroll_fraction_y, -TP_SCROLL_LIMIT, TP_SCROLL_LIMIT);

    data->scroll_fraction_x -= (float)out_x;
    data->scroll_fraction_y -= (float)out_y;

    if (out_x == 0 && out_y == 0) {
        return;
    }

    if (out_x != 0) {
        input_report_rel(dev, INPUT_REL_HWHEEL, -out_x, out_y == 0, K_NO_WAIT);
    }
    if (out_y != 0) {
        input_report_rel(dev, INPUT_REL_WHEEL, out_y, true, K_NO_WAIT);
    }
}

static void report_pointer(struct trackpoint_data *data, const struct device *dev, int8_t dx,
                           int8_t dy, uint32_t now) {
    uint8_t brightness = custom_led_get_last_valid_brightness();
    float sensitivity = MOUSE_SENS_BASE + (MOUSE_SENS_STEP * brightness);

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
    uint32_t delta_ms = now - data->last_packet_time;
    float exponential = trackpoint_exponential_factor(dx, dy, delta_ms);
#else
    float exponential = 1.0f;
#endif

    float slow = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;
    int32_t out_x = CLAMP(-(int32_t)((float)dx * MOUSE_BASE_SPEED * sensitivity * exponential * slow),
                          -TP_POINTER_LIMIT, TP_POINTER_LIMIT);
    int32_t out_y = CLAMP(-(int32_t)((float)dy * MOUSE_BASE_SPEED * sensitivity * exponential * slow),
                          -TP_POINTER_LIMIT, TP_POINTER_LIMIT);

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

static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    uint32_t now = k_uptime_get_32();

    int8_t raw_dx = 0;
    int8_t raw_dy = 0;
    int ret = trackpoint_read_packet(dev, &raw_dx, &raw_dy);

    if (ret != 0) {
        data->consecutive_errors++;
        log_read_error_rate_limited(ret);
        if (data->consecutive_errors >= TP_ERROR_RESET_THRESHOLD) {
            reset_motion_state(data);
            data->consecutive_errors = 0;
        }
        return;
    }

    data->consecutive_errors = 0;
    if (raw_dx == 0 && raw_dy == 0) {
        return;
    }

    int32_t dx = raw_dx;
    int32_t dy = raw_dy;
    bool just_enter_scroll = scroll_key_pressed && !last_scroll_key_pressed;
    bool just_enter_arrow = arrow_key_pressed && !last_arrow_key_pressed;

    if (arrow_key_pressed) {
        if (just_enter_arrow) {
            data->arrow_residue_x = dx;
            data->arrow_residue_y = dy;
        }
        apply_dominant_axis(&dx, &dy);
        process_arrow_axis(dev, dx, &data->arrow_residue_x, INPUT_BTN_0, INPUT_BTN_1);
        process_arrow_axis(dev, dy, &data->arrow_residue_y, INPUT_BTN_2, INPUT_BTN_3);
    } else if (scroll_key_pressed) {
        if (just_enter_scroll) {
            data->scroll_fraction_x = 0.0f;
            data->scroll_fraction_y = 0.0f;
        }
        report_scroll(data, dev, dx, dy);
    } else {
        report_pointer(data, dev, raw_dx, raw_dy, now);
    }

    last_scroll_key_pressed = scroll_key_pressed;
    last_arrow_key_pressed = arrow_key_pressed;
    data->last_packet_time = now;
}

static int special_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->position == 35) {
        arrow_key_pressed = ev->state;
        LOG_DBG("arrow mode: %d", arrow_key_pressed);
    }
    if (ev->position == 62) {
        scroll_key_pressed = ev->state;
        LOG_DBG("scroll mode: %d", scroll_key_pressed);
    }
    if (ev->position == 37) {
        slow_key_pressed = ev->state;
        LOG_DBG("slow mode: %d", slow_key_pressed);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(trackpoint_special_key_listener, special_key_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_special_key_listener, zmk_position_state_changed);

static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct trackpoint_data *data = CONTAINER_OF(cb, struct trackpoint_data, motion_cb_data);
    (void)k_work_submit_to_queue(&tp_workq, &data->work);
}

static void trackpoint_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct trackpoint_data *data = CONTAINER_OF(dwork, struct trackpoint_data, enable_irq_work);
    const struct trackpoint_config *cfg = data->dev->config;

    int ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("TrackPoint IRQ enable failed: %d", ret);
    } else {
        LOG_DBG("TrackPoint IRQ enabled");
    }
}

static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c) || !gpio_is_ready_dt(&cfg->motion_gpio)) {
        return -ENODEV;
    }

    k_mutex_init(&trackpoint_i2c_mutex);
    data->dev = dev;
    data->last_packet_time = k_uptime_get_32();
    data->consecutive_errors = 0;
    reset_motion_state(data);
    last_error_log_ms = 0U;

    k_work_init(&data->work, trackpoint_work_cb);
    k_work_queue_start(&tp_workq, tp_workq_stack, K_THREAD_STACK_SIZEOF(tp_workq_stack),
                       TP_WORKQ_PRIORITY, NULL);

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

    k_work_init_delayable(&data->enable_irq_work, trackpoint_enable_irq_work_cb);
    (void)k_work_schedule(&data->enable_irq_work, K_MSEC(200));

    LOG_INF("TrackPoint driver initialized (stability-hardened)");
    return 0;
}

#define TRACKPOINT_DEFINE(inst)                                                                    \
    static struct trackpoint_data trackpoint_data_##inst;                                          \
    static const struct trackpoint_config trackpoint_config_##inst = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                   \
                        .pin = MOTION_GPIO_PIN,                                                      \
                        .dt_flags = MOTION_GPIO_FLAGS},                                             \
    };                                                                                              \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
