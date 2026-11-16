#include "imu_ctl.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_ctl, LOG_LEVEL_INF);

static const struct device *const imu_dev = DEVICE_DT_GET(DT_NODELABEL(bmi270));

/* Bufor na 16 próbek (ok. 80 ms zapasu przy 200 Hz) */
K_MSGQ_DEFINE(imu_msgq, sizeof(struct imu_sample), 16, 4);

static void imu_drdy_callback(const struct device *dev, const struct sensor_trigger *trigger)
{
    ARG_UNUSED(trigger);

    struct sensor_value accel[3], gyro[3];

    if (sensor_sample_fetch(dev) < 0) {
        LOG_ERR("Failed to request IMU sample");
        return;
    }
    LOG_DBG("Sample fetched");
    if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0 ||
        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro) < 0) {
        LOG_ERR("Failed to get IMU data");
        return;
    }
    LOG_DBG("Data ready");
    struct imu_sample sample = {
        .cycles = k_cycle_get_32(),
        .accel = {
            sensor_value_to_float(&accel[0]),
            sensor_value_to_float(&accel[1]),
            sensor_value_to_float(&accel[2]),
        },
        .gyro = {
            sensor_value_to_float(&gyro[0]),
            sensor_value_to_float(&gyro[1]),
            sensor_value_to_float(&gyro[2]),
        },
    };

    /* Nie blokujemy wątku triggera – jeśli konsument nie nadąża, odrzucamy najnowszą */
    if (k_msgq_put(&imu_msgq, &sample, K_NO_WAIT) != 0) {
        LOG_WRN_RATELIMIT("IMU msgq full, dropping sample");
    }
}

int imu_init(void)
{
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("BMI270 device not ready");
        return -ENODEV;
    }

    struct sensor_value odr = { .val1 = 200, .val2 = 0 }; /* 200 Hz */
    struct sensor_value accel_fs, gyro_fs;

    sensor_g_to_ms2(8, &accel_fs);          /* Zakres +-8g (odporny na rzut i wibracje) */
    sensor_degrees_to_rad(1000, &gyro_fs);  /* Zakres +-1000 deg/s */

    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &accel_fs);
    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &gyro_fs);

    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    LOG_DBG("Device configured, setting trigger");
    int ret = sensor_trigger_set(imu_dev, &trig, imu_drdy_callback);
    if (ret < 0) {
        LOG_ERR("Failed to set IMU trigger: %d", ret);
        return ret;
    }
    LOG_INF("BMI270 initialized (200 Hz, +-8g, +-1000 dps)");
    return 0;
}

int imu_get_sample(struct imu_sample *sample, k_timeout_t timeout)
{
    return k_msgq_get(&imu_msgq, sample, timeout);
}
