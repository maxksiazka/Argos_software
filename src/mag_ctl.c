#include "mag_ctl.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mag_ctl, LOG_LEVEL_INF);

static const struct device *const mag_dev = DEVICE_DT_GET(DT_NODELABEL(mmc5983ma));

K_MSGQ_DEFINE(mag_msgq, sizeof(struct mag_sample), 8, 4);

static float hard_iron_offset[3] = { 0.0f, 0.0f, 0.0f };

void mag_set_hard_iron_offset(float ox, float oy, float oz)
{
    hard_iron_offset[0] = ox;
    hard_iron_offset[1] = oy;
    hard_iron_offset[2] = oz;
}

static void mag_drdy_callback(const struct device *dev, const struct sensor_trigger *trigger)
{
    ARG_UNUSED(trigger);

    struct sensor_value mag[3];

    if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ) < 0) {
        return;
    }

    if (sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag) < 0) {
        return;
    }

    struct mag_sample sample = {
        .cycles = k_cycle_get_32(),
        .mag = {
            sensor_value_to_float(&mag[0]) - hard_iron_offset[0],
            sensor_value_to_float(&mag[1]) - hard_iron_offset[1],
            sensor_value_to_float(&mag[2]) - hard_iron_offset[2],
        },
    };

    if (k_msgq_put(&mag_msgq, &sample, K_NO_WAIT) != 0) {
        LOG_WRN_RATELIMIT("Mag msgq full, dropping sample");
    }
}

int mag_init(void)
{
    if (!device_is_ready(mag_dev)) {
        LOG_ERR("MMC5983MA device not ready");
        return -ENODEV;
    }

    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_MAGN_XYZ,
    };

    int ret = sensor_trigger_set(mag_dev, &trig, mag_drdy_callback);
    if (ret < 0) {
        LOG_ERR("Failed to set MMC5983MA trigger: %d", ret);
        return ret;
    }

    /* Odczyt czyszczący ewentualny wiszący stan DRDY na GP13 */
    (void)sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_MAGN_XYZ);

    LOG_INF("MMC5983MA initialized (50 Hz, trigger on GP13)");
    return 0;
}

int mag_get_sample(struct mag_sample *sample, k_timeout_t timeout)
{
    return k_msgq_get(&mag_msgq, sample, timeout);
}
