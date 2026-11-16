#ifndef IMU_CTL_H
#define IMU_CTL_H

#include <zephyr/kernel.h>
#include <stdint.h>

struct imu_sample {
    uint32_t cycles;
    float accel[3];        /* [m/s^2] X, Y, Z */
    float gyro[3];         /* [rad/s] X, Y, Z */
};

int imu_init(void);
int imu_get_sample(struct imu_sample *sample, k_timeout_t timeout);

#endif /* IMU_CTL_H */
