#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "sys_health.h"
#include "imu_ctl.h"
#include "baro_ctl.h"
#include "mag_ctl.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct gpio_dt_spec led_err  = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); /* GP14 Red */
static const struct gpio_dt_spec led_stat = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); /* GP15 Green */

static atomic_t error_mask = ATOMIC_INIT(0);
static atomic_t armed_state = ATOMIC_INIT(0);

void sys_err_set(enum sys_err_flag flag)   { atomic_or(&error_mask, flag); }
void sys_err_clear(enum sys_err_flag flag) { atomic_and(&error_mask, ~flag); }
atomic_val_t sys_err_get_mask(void)        { return atomic_get(&error_mask); }

void sys_set_armed(bool armed)             { atomic_set(&armed_state, armed ? 1 : 0); }
bool sys_is_armed(void)                    { return atomic_get(&armed_state) != 0; }

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{

    gpio_pin_set_dt(&led_err, 1);
    gpio_pin_set_dt(&led_stat, 0);

    for (;;) {
        LOG_ERR("Fatal error %u, system halted", reason);
        k_busy_wait(500000); // half a second
    }
}
/* Aktualizacja diod wywoływana co 100 ms (10 Hz) */
static void update_status_leds(uint32_t tick)
{
    atomic_val_t err = sys_err_get_mask();
    bool red_on = false;
    bool green_on = false;

    if (err & (SYS_ERR_IMU_INIT | SYS_ERR_BARO_INIT | SYS_ERR_MAG_INIT)) {
        /* Krytyczny błąd czujnika przy starcie: szybkie miganie czerwonej (5 Hz) */
        red_on = (tick % 2) == 0;
    } else if (err & SYS_ERR_SENSOR_WDG) {
        /* Zawieszenie czujnika w trakcie pracy: podwójny błysk czerwonej */
        uint32_t phase = tick % 10;
        red_on = (phase == 0 || phase == 2);
    } else if (err & SYS_ERR_RC_LOST) {
        /* Failsafe ELRS: naprzemienne miganie czerwona / zielona */
        red_on = (tick % 4) < 2;
        green_on = !red_on;
    } else {
        /* Brak krytycznych błędów – zielona sygnalizuje stan uzbrojenia */
        if (sys_is_armed()) {
            green_on = true; /* Ciągłe zielone = UZBROJONY */
        } else {
            /* Podwójny impuls co 1 s = GOTOWY / ROZBROJONY */
            uint32_t phase = tick % 10;
            green_on = (phase == 0 || phase == 2);
        }

        /* Ostrzeżenie niekrytyczne (brak SD lub słaba bateria): krótki błysk czerwonej raz na sekundę */
        if (err & (SYS_ERR_SD_INIT | SYS_ERR_LOW_BATT)) {
            red_on = (tick % 10) == 5;
        }
    }

    gpio_pin_set_dt(&led_err, red_on ? 1 : 0);
    gpio_pin_set_dt(&led_stat, green_on ? 1 : 0);
}

int main(void)
{
    if (!gpio_is_ready_dt(&led_err) || !gpio_is_ready_dt(&led_stat)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led_err, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_stat, GPIO_OUTPUT_INACTIVE);

    /* Krótki test obu diod przy starcie (200 ms) */
    gpio_pin_set_dt(&led_err, 1);
    gpio_pin_set_dt(&led_stat, 1);
    k_sleep(K_MSEC(200));
    gpio_pin_set_dt(&led_err, 0);
    gpio_pin_set_dt(&led_stat, 0);

    /* Inicjalizacja podsystemów – błędy zapisujemy w masce zamiast wieszać program */
    if (imu_init() < 0) {
        sys_err_set(SYS_ERR_IMU_INIT);
    }
    if (baro_init() < 0) {
        sys_err_set(SYS_ERR_BARO_INIT);
    }
    if (mag_init() < 0) {
        sys_err_set(SYS_ERR_MAG_INIT);
    }

    /* Start pętli głównej – aktualizacja diod i logowanie zmian w masce błędów */
    uint32_t tick = 0;
    atomic_val_t prev_err = 0;

    while (1) {
        atomic_val_t curr_err = sys_err_get_mask();
        if (curr_err != prev_err) {
            LOG_WRN("System error mask changed: 0x%02lx -> 0x%02lx",
                    (unsigned long)prev_err, (unsigned long)curr_err);
            prev_err = curr_err;
        }

        update_status_leds(tick++);
        k_msleep(100);
    }

    return 0;
}
