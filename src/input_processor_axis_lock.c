/*
 * Axis lock input processor
 *
 * ジェスチャの「動き出し」で縦か横かを決め、そのジェスチャが終わるまで
 * その軸に固定する。もう一方の軸は完全にゼロにする。
 *
 * xy_clipper との違い:
 *   xy_clipper はイベントごとに優位軸を選び直す (しかも Y を 2倍
 *   優遇するので、|X| > 2|Y| でないと横が出ない)。こちらは最初に
 *   閾値を超えた時点で軸を確定し、release-ms 無入力になるまで
 *   その軸を保持する。斜めに流れても途中で軸が入れ替わらない。
 *
 * 出力量は xy_clipper と同じく accum / threshold。閾値未満の入力は
 * 蓄積されるだけで出力されない (ノイズ除去 + 分解能の粗調整を兼ねる)。
 *
 * 並行性: pointer_inertia と同じく、ZMK の標準経路では入力リスナと
 * 同じワークキュー上で直列化されるため排他は持たない。
 */

#define DT_DRV_COMPAT zmk_input_processor_axis_lock

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#include <drivers/input_processor.h>

LOG_MODULE_REGISTER(axis_lock, CONFIG_ZMK_LOG_LEVEL);

#define LOCK_NONE 0
#define LOCK_X    1
#define LOCK_Y    2

struct al_config {
    int32_t threshold;  /* この蓄積量で 1 単位を出力する */
    uint16_t release_ms; /* この時間 無入力ならジェスチャ終了 = 軸固定を解除 */
    uint16_t y_bias;    /* Y の優遇度 x1000。1000 = 優遇なし(動き出し勝負) */
};

struct al_data {
    int32_t accum_x;
    int32_t accum_y;
    uint8_t locked;
    int64_t last_ms;
};

static void al_reset(struct al_data *data) {
    data->accum_x = 0;
    data->accum_y = 0;
    data->locked = LOCK_NONE;
}

static int al_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    struct al_data *data = dev->data;
    const struct al_config *cfg = dev->config;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return 0;
    }

    const int64_t now = k_uptime_get();
    if (data->last_ms > 0 && (now - data->last_ms) > cfg->release_ms) {
        /* 前のジェスチャは終わっている。軸固定を解除して最初からやり直す */
        al_reset(data);
    }
    data->last_ms = now;

    if (event->code == INPUT_REL_X) {
        data->accum_x += event->value;
    } else {
        data->accum_y += event->value;
    }
    /* 以降で固定軸ぶんだけを載せ直すので、一旦ゼロにしておく */
    event->value = 0;

    if (data->locked == LOCK_NONE) {
        int32_t ax = abs(data->accum_x);
        int32_t ay = (int32_t)(((int64_t)abs(data->accum_y) * cfg->y_bias) / 1000);

        if (ax < cfg->threshold && ay < cfg->threshold) {
            /* まだどちらも閾値未満。蓄積だけしてこのイベントは無出力 */
            return 0;
        }

        data->locked = (ay >= ax) ? LOCK_Y : LOCK_X;
        LOG_DBG("axis_lock: locked to %s (ax=%d ay=%d)", data->locked == LOCK_Y ? "Y" : "X", ax,
                ay);
    }

    /* 固定軸から出力し、もう一方の蓄積は捨て続ける */
    int32_t *acc;
    if (data->locked == LOCK_Y) {
        acc = &data->accum_y;
        data->accum_x = 0;
        event->code = INPUT_REL_Y;
    } else {
        acc = &data->accum_x;
        data->accum_y = 0;
        event->code = INPUT_REL_X;
    }

    int32_t out = *acc / cfg->threshold;
    *acc -= out * cfg->threshold;
    event->value = out;

    return 0;
}

static int al_init(const struct device *dev) {
    struct al_data *data = dev->data;
    al_reset(data);
    data->last_ms = 0;
    return 0;
}

static const struct zmk_input_processor_driver_api al_api = {
    .handle_event = al_handle_event,
};

#define AL_INST(n)                                                                                 \
    static const struct al_config al_config_##n = {                                                \
        .threshold = DT_INST_PROP_OR(n, threshold, 20),                                            \
        .release_ms = DT_INST_PROP_OR(n, release_ms, 200),                                         \
        .y_bias = DT_INST_PROP_OR(n, y_bias, 1000),                                                \
    };                                                                                             \
    static struct al_data al_data_##n = {0};                                                       \
    DEVICE_DT_INST_DEFINE(n, al_init, NULL, &al_data_##n, &al_config_##n, POST_KERNEL,             \
                          CONFIG_ZMK_INPUT_PROCESSOR_AXIS_LOCK_INIT_PRIORITY, &al_api);

DT_INST_FOREACH_STATUS_OKAY(AL_INST)
