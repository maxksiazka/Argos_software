#include "baro_ctl.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include <math.h>

LOG_MODULE_REGISTER(baro_ctl, LOG_LEVEL_INF);

#define BARO_CALIB_SAMPLES 50U /* 1 s przy ODR = 50 Hz */

static const struct device *const baro_dev = DEVICE_DT_GET(DT_NODELABEL(bmp581));

SENSOR_DT_STREAM_IODEV(bmp581_stream_iodev, DT_NODELABEL(bmp581),
                       {SENSOR_TRIG_DATA_READY, SENSOR_STREAM_DATA_INCLUDE});

RTIO_DEFINE_WITH_MEMPOOL(bmp581_rtio, 4, 4, 8, 32, 4);

K_MSGQ_DEFINE(baro_msgq, sizeof(struct baro_sample), 8, 4);

K_THREAD_STACK_DEFINE(baro_rtio_stack, 1536);
static struct k_thread baro_rtio_thread;

static volatile bool reset_calib_req = true;

void baro_reset_ground_ref(void)
{
    reset_calib_req = true;
}

static void baro_rtio_worker(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    const struct sensor_decoder_api *decoder = NULL;
    if (sensor_get_decoder(baro_dev, &decoder) < 0 || decoder == NULL) {
        LOG_ERR("Failed to get BMP581 RTIO decoder");
        return;
    }

    rtio_sqe_handle_t stream_handle;
    int ret = sensor_stream(&bmp581_stream_iodev, &bmp581_rtio, NULL, &stream_handle);
    if (ret < 0) {
        LOG_ERR("Failed to start BMP581 stream: %d", ret);
        return;
    }

    float p0_hpa = 1013.25f;
    float p0_accum = 0.0f;
    uint32_t calib_count = 0;
    bool is_calibrated = false;

    const struct sensor_chan_spec press_spec = {
        .chan_type = SENSOR_CHAN_PRESS,
        .chan_idx = 0,
    };
    const struct sensor_chan_spec temp_spec = {
        .chan_type = SENSOR_CHAN_AMBIENT_TEMP,
        .chan_idx = 0,
    };

    while (1) {
        /* Blokujące oczekiwanie na kolejną ramkę z przerwania GP19 */
        struct rtio_cqe *cqe = rtio_cqe_consume_block(&bmp581_rtio);
        int cqe_res = cqe->result;

        uint8_t *buf = NULL;
        uint32_t buf_len = 0;
        int buf_res = rtio_cqe_get_mempool_buffer(&bmp581_rtio, cqe, &buf, &buf_len);

        rtio_cqe_release(&bmp581_rtio, cqe);

        if (cqe_res < 0) {
            /* Błąd w egzekutorze RTIO kończy zadanie multishot – wznawiamy strumień */
            if (buf != NULL) {
                rtio_release_buffer(&bmp581_rtio, buf, buf_len);
            }
            LOG_WRN_RATELIMIT("BMP581 stream error (%d), restarting...", cqe_res);
            sensor_stream(&bmp581_stream_iodev, &bmp581_rtio, NULL, &stream_handle);
            continue;
        }

        if (buf_res < 0 || buf == NULL) {
            LOG_WRN_RATELIMIT("BMP581 mempool buffer error: %d", buf_res);
            continue;
        }

        uint32_t press_fit = 0;
        uint32_t temp_fit = 0;
        struct sensor_q31_data press_data = {0};
        struct sensor_q31_data temp_data = {0};

        int n_press = decoder->decode(buf, press_spec, &press_fit, 1, &press_data);
        int n_temp = decoder->decode(buf, temp_spec, &temp_fit, 1, &temp_data);

        /* Zwalniamy blok z powrotem do puli RTIO od razu po zdekodowaniu */
        rtio_release_buffer(&bmp581_rtio, buf, buf_len);

        if (n_press > 0 && n_temp > 0) {
            float press_hpa = ldexpf((float)press_data.readings[0].value,
                                     press_data.shift - 31) * 10.0f;
            float temp_c = ldexpf((float)temp_data.readings[0].value,
                                  temp_data.shift - 31);
            float alt_m = 0.0f;

            if (reset_calib_req) {
                p0_accum = 0.0f;
                calib_count = 0;
                is_calibrated = false;
                reset_calib_req = false;
            }

            if (!is_calibrated) {
                p0_accum += press_hpa;
                if (++calib_count >= BARO_CALIB_SAMPLES) {
                    p0_hpa = p0_accum / (float)BARO_CALIB_SAMPLES;
                    is_calibrated = true;
                    LOG_INF("Baro P0 calibrated: %.2f hPa", (double)p0_hpa);
                }
            } else {
                alt_m = 44330.0f * (1.0f - powf(press_hpa / p0_hpa, 0.190295f));
            }

            struct baro_sample sample = {
                .cycles = k_cycle_get_32(),
                .pressure_hpa = press_hpa,
                .temperature_c = temp_c,
                .altitude_m = alt_m,
            };

            if (k_msgq_put(&baro_msgq, &sample, K_NO_WAIT) != 0) {
                LOG_WRN_RATELIMIT("Baro msgq full, dropping sample");
            }
        }
    }
}

int baro_init(void)
{
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("BMP581 device not ready");
        return -ENODEV;
    }

    baro_reset_ground_ref();

    k_thread_create(&baro_rtio_thread, baro_rtio_stack,
                    K_THREAD_STACK_SIZEOF(baro_rtio_stack),
                    baro_rtio_worker, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_NO_WAIT);
    k_thread_name_set(&baro_rtio_thread, "baro_rtio");

    LOG_INF("BMP581 RTIO stream initialized (50 Hz, GP19)");
    return 0;
}

int baro_get_sample(struct baro_sample *sample, k_timeout_t timeout)
{
    return k_msgq_get(&baro_msgq, sample, timeout);
}
