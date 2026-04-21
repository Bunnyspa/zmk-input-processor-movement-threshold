#define DT_DRV_COMPAT zmk_input_processor_movement_threshold

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(movement_threshold, CONFIG_ZMK_LOG_LEVEL);

struct movement_threshold_data {
    int accumulated;
    bool gated;
    bool skip_frame;
    int64_t last_event_ms;
};

static int movement_threshold_handle_event(const struct device *dev,
                                           struct input_event *event,
                                           uint32_t param1,
                                           uint32_t param2,
                                           struct zmk_input_processor_state *state) {
    struct movement_threshold_data *data = dev->data;
    int threshold = (int)param1;
    int idle_ms   = (int)param2;

    int64_t now = k_uptime_get();

    if (now - data->last_event_ms > idle_ms) {
        data->accumulated = 0;
        data->gated = true;
        data->skip_frame = false;
    }
    data->last_event_ms = now;

    if (event->sync) {
        bool drop = data->gated || data->skip_frame;
        data->skip_frame = false;
        return drop ? ZMK_INPUT_PROC_STOP : 0;
    }

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return data->gated ? ZMK_INPUT_PROC_STOP : 0;
    }

    if (!data->gated) {
        return 0;
    }

    data->accumulated += abs(event->value);

    if (data->accumulated >= threshold) {
        data->gated = false;
        data->skip_frame = true;
    }

    return ZMK_INPUT_PROC_STOP;
}

static const struct zmk_input_processor_driver_api movement_threshold_api = {
    .handle_event = movement_threshold_handle_event,
};

#define MOVEMENT_THRESHOLD_INST(n)                                          \
    static struct movement_threshold_data data_##n = {                     \
        .accumulated = 0,                                                   \
        .gated = true,                                                      \
        .skip_frame = false,                                                \
        .last_event_ms = 0,                                                 \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &data_##n, NULL,                  \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
                          &movement_threshold_api);

DT_INST_FOREACH_STATUS_OKAY(MOVEMENT_THRESHOLD_INST)
