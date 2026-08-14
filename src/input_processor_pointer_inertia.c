/*
 * Pointer (cursor) inertia / coasting input processor
 *
 * ボールを速く弾いて止めた直後、カーソルを減衰させながら少しだけ
 * 滑らせる。ゆっくり動かしている時は発動しない (min-speed ゲート)。
 *
 * 設計:
 *   - 実イベントは一切書き換えずに素通しする (純粋なオブザーバ)。
 *   - 実イベントごとに軸別の速度を EMA で推定し、tick-ms 後に
 *     coast tick を予約する。次の実イベントが来たら予約はキャンセル。
 *   - tick が発火した = tick-ms の間 実イベントが来なかった、つまり
 *     ボールが止まった。ここで速度が min-speed を超えていれば惰性開始。
 *   - 惰性中は tick-ms ごとに decay を掛けた移動量を HID に直接送る。
 *     入力チェーンには戻さないので二重処理にならない。
 *   - duration-ms 経過、または速度が stop を下回ったら終了。
 *
 * 並行性:
 *   handle_event (input listener) と tick handler (システムワークキュー)
 *   は ZMK の標準経路では同一ワークキュー上で直列化されるため、
 *   pi_data へのアクセスは実質シングルスレッド。フィールドは int32_t
 *   中心で Cortex-M では単命令ロード/ストアなので、仮に別コンテキスト
 *   から入っても最悪 1 tick 分古い値を使うだけ。ロックは持たない。
 *   (mjmjm0101/zmk-input-processor-scroll-inertia と同じ方針)
 */

#define DT_DRV_COMPAT zmk_input_processor_pointer_inertia

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#include <drivers/input_processor.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(pointer_inertia, CONFIG_ZMK_LOG_LEVEL);

/* 速度は「1 tick あたりの移動カウント x 1000」の固定小数で持つ */
#define FP 1000

/* dt の異常値をクランプする範囲 (ms) */
#define DT_MIN 1
#define DT_MAX 100

struct pi_config {
    uint16_t tick_ms;     /* 惰性の出力周期 = 停止判定の猶予 */
    uint16_t duration_ms; /* 惰性の最大継続時間 */
    uint16_t decay;       /* tick ごとの減衰率 x1000 */
    uint16_t min_speed;   /* 発動に必要な速度 (counts/sec) */
    uint16_t stop;        /* この速度を下回ったら終了 (固定小数) */
    uint16_t smoothing;   /* EMA の新サンプル重み x1000 */
};

struct pi_data {
    const struct device *dev;
    struct k_work_delayable tick_work;

    int32_t vel_x, vel_y;     /* 固定小数: 1 tick あたりの移動カウント */
    int32_t accum_x, accum_y; /* 出力の端数 (固定小数) */
    int64_t last_ms_x, last_ms_y;
    int64_t coast_start_ms;
    bool coasting;
};

static inline uint16_t safe_tick(const struct pi_config *cfg) {
    return cfg->tick_ms ? cfg->tick_ms : 16;
}

static void pi_reset(struct pi_data *data) {
    data->coasting = false;
    data->vel_x = 0;
    data->vel_y = 0;
    data->accum_x = 0;
    data->accum_y = 0;
}

/*
 * 実イベント1件から、その軸の速度を更新する。
 * 瞬時速度[固定小数] = 移動カウント * tick_ms * FP / 経過ms
 * を EMA で均す。
 */
static void pi_update_velocity(const struct pi_config *cfg, int32_t *vel, int64_t *last_ms,
                               int32_t raw, int64_t now) {
    uint32_t dt = DT_MIN;
    if (*last_ms > 0 && now > *last_ms) {
        int64_t diff = now - *last_ms;
        if (diff > DT_MAX) {
            diff = DT_MAX;
        }
        dt = (uint32_t)diff;
    }
    *last_ms = now;

    int32_t inst = (int32_t)(((int64_t)raw * safe_tick(cfg) * FP) / dt);

    uint32_t w = cfg->smoothing;
    if (w > FP) {
        w = FP;
    }
    *vel = (int32_t)(((int64_t)(*vel) * (FP - w) + (int64_t)inst * w) / FP);
}

/* 固定小数の速度を、端数を繰り越しながら整数の移動量に変換する */
static int16_t pi_emit_delta(int32_t vel, int32_t *accum) {
    int32_t total = vel + *accum;
    int32_t out = total / FP;
    *accum = total - out * FP;
    if (out > INT16_MAX) {
        out = INT16_MAX;
    } else if (out < INT16_MIN) {
        out = INT16_MIN;
    }
    return (int16_t)out;
}

static void pi_tick_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pi_data *data = CONTAINER_OF(dwork, struct pi_data, tick_work);
    const struct pi_config *cfg = data->dev->config;
    const uint16_t tick = safe_tick(cfg);
    const int64_t now = k_uptime_get();

    if (!data->coasting) {
        /*
         * tick-ms の間 実イベントが来なかった = ボールが止まった。
         * 速度が min-speed 以上なら惰性を開始する。
         * 固定小数の速度 -> counts/sec は /tick_ms で得られる。
         */
        int32_t fastest = MAX(abs(data->vel_x), abs(data->vel_y));
        if ((uint32_t)(fastest / tick) < cfg->min_speed) {
            pi_reset(data);
            return;
        }
        data->coasting = true;
        data->coast_start_ms = now;
        data->accum_x = 0;
        data->accum_y = 0;
    }

    if (now - data->coast_start_ms >= cfg->duration_ms) {
        pi_reset(data);
        return;
    }

    /* 先に減衰させてから出す。最後の実移動より僅かに遅い所から始まるので繋ぎが自然 */
    data->vel_x = (int32_t)(((int64_t)data->vel_x * cfg->decay) / FP);
    data->vel_y = (int32_t)(((int64_t)data->vel_y * cfg->decay) / FP);

    if (abs(data->vel_x) < cfg->stop && abs(data->vel_y) < cfg->stop) {
        pi_reset(data);
        return;
    }

    int16_t dx = pi_emit_delta(data->vel_x, &data->accum_x);
    int16_t dy = pi_emit_delta(data->vel_y, &data->accum_y);

    if (dx != 0 || dy != 0) {
        zmk_hid_mouse_scroll_set(0, 0);
        zmk_hid_mouse_movement_set(dx, dy);
        /* ZMK は v0.3 以降 endpoints -> endpoint に改名した。同じリファクタで
         * 入った ZMK_ENDPOINT_NONE_COUNT の有無で分岐する。 */
#ifdef ZMK_ENDPOINT_NONE_COUNT
        zmk_endpoint_send_mouse_report();
#else
        zmk_endpoints_send_mouse_report();
#endif
        zmk_hid_mouse_movement_set(0, 0);
    }

    k_work_schedule(&data->tick_work, K_MSEC(tick));
}

static int pi_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    struct pi_data *data = dev->data;
    const struct pi_config *cfg = dev->config;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return 0;
    }

    /* 実入力が来たら惰性は即中断する (ボールを掴んで止める操作) */
    k_work_cancel_delayable(&data->tick_work);
    if (data->coasting) {
        pi_reset(data);
    }

    const int64_t now = k_uptime_get();
    if (event->code == INPUT_REL_X) {
        pi_update_velocity(cfg, &data->vel_x, &data->last_ms_x, event->value, now);
    } else {
        pi_update_velocity(cfg, &data->vel_y, &data->last_ms_y, event->value, now);
    }

    /* 次の tick までに実イベントが来なければ、そこで惰性判定に入る */
    k_work_schedule(&data->tick_work, K_MSEC(safe_tick(cfg)));

    /* イベント自体は素通し */
    return 0;
}

static int pi_init(const struct device *dev) {
    struct pi_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->tick_work, pi_tick_handler);
    pi_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api pi_api = {
    .handle_event = pi_handle_event,
};

#define PI_INST(n)                                                                                 \
    static const struct pi_config pi_config_##n = {                                                \
        .tick_ms = DT_INST_PROP_OR(n, tick_ms, 16),                                                \
        .duration_ms = DT_INST_PROP_OR(n, duration_ms, 200),                                       \
        .decay = DT_INST_PROP_OR(n, decay, 790),                                                   \
        .min_speed = DT_INST_PROP_OR(n, min_speed, 800),                                           \
        .stop = DT_INST_PROP_OR(n, stop, 300),                                                     \
        .smoothing = DT_INST_PROP_OR(n, smoothing, 600),                                           \
    };                                                                                             \
    static struct pi_data pi_data_##n = {0};                                                       \
    DEVICE_DT_INST_DEFINE(n, pi_init, NULL, &pi_data_##n, &pi_config_##n, POST_KERNEL,             \
                          CONFIG_ZMK_INPUT_PROCESSOR_POINTER_INERTIA_INIT_PRIORITY, &pi_api);

DT_INST_FOREACH_STATUS_OKAY(PI_INST)
