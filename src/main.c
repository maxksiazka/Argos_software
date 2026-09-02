#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void) {
    int ret;
    struct net_if *iface = NULL;

    LOG_INF("Booting... Waiting for CYW43 subsystem");

    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED device not ready");
        return 0;
    }

    while ((iface = net_if_get_default()) == NULL) {
        k_msleep(50);
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED pin: %d", ret);
        return 0;
    }

    for (int i = 0; i < 10; i++) {
        gpio_pin_set_dt(&led, 0);
        k_msleep(100);
    }

    LOG_INF("Power-on LED locked HIGH");

    while (1) {
        k_msleep(1000);
    }

    return 0;
}
