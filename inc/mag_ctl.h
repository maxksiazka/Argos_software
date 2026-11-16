#ifndef MAG_CTL_H_
#define MAG_CTL_H_

#include <zephyr/kernel.h>
#include <stdint.h>

struct mag_sample {
    uint32_t cycles; /* Znacznik czasu z k_cycle_get_32() */
    float mag[3];    /* [Gauss] X, Y, Z (skalibrowane o offset hard-iron) */
};

int mag_init(void);
int mag_get_sample(struct mag_sample *sample, k_timeout_t timeout);
void mag_set_hard_iron_offset(float ox, float oy, float oz);

#endif /* MAG_CTL_H_ */
