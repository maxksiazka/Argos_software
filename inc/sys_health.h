#ifndef SYS_HEALTH_H_
#define SYS_HEALTH_H_

#include <zephyr/sys/atomic.h>
#include <stdbool.h>

enum sys_err_flag {
    SYS_ERR_IMU_INIT   = BIT(0), /* Błąd inicjalizacji BMI270 */
    SYS_ERR_BARO_INIT  = BIT(1), /* Błąd inicjalizacji BMP581 */
    SYS_ERR_MAG_INIT   = BIT(2), /* Błąd inicjalizacji MMC5983MA */
    SYS_ERR_SD_INIT    = BIT(3), /* Brak lub błąd karty microSD */
    SYS_ERR_SENSOR_WDG = BIT(4), /* Zatrzymanie próbek z I2C w locie */
    SYS_ERR_RC_LOST    = BIT(5), /* Brak ramek CRSF > 150 ms (failsafe) */
    SYS_ERR_LOW_BATT   = BIT(6), /* Niskie napięcie pakietu (GP27 ADC) */
};

void sys_err_set(enum sys_err_flag flag);
void sys_err_clear(enum sys_err_flag flag);
atomic_val_t sys_err_get_mask(void);

void sys_set_armed(bool armed);
bool sys_is_armed(void);

#endif /* SYS_HEALTH_H_ */
