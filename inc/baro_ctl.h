#ifndef BARO_CTL_H_
#define BARO_CTL_H_

#include <zephyr/kernel.h>
#include <stdint.h>

struct baro_sample {
    uint32_t cycles;      /* Znacznik czasu z k_cycle_get_32() */
    float pressure_hpa;   /* [hPa] */
    float temperature_c;  /* [deg C] */
    float altitude_m;     /* [m] Wysokość względna względem punktu startu P0 */
};

int baro_init(void);
int baro_get_sample(struct baro_sample *sample, k_timeout_t timeout);
void baro_reset_ground_ref(void);

#endif /* BARO_CTL_H_ */
