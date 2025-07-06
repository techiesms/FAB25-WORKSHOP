#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app);

static const struct gpio_dt_spec red_led   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec blue_led  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

void set_color(int temp) {
    // Turn all LEDs off first
    gpio_pin_set_dt(&red_led, 1);
    gpio_pin_set_dt(&green_led, 1);
    gpio_pin_set_dt(&blue_led, 1);

    // Choose color based on temperature
    if (temp >= 30) {
        gpio_pin_set_dt(&blue_led, 0);   // Hot = Blue
    } else if (temp >= 20) {
        gpio_pin_set_dt(&green_led, 0);  // Normal = Green
    } else {
        gpio_pin_set_dt(&red_led, 0);    // Cold = Red
    }
}

int main(void) {
    const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme680);

    if (!device_is_ready(dev)) {
        LOG_ERR("BME680 sensor not ready");
        return 0;
    }

    if (!device_is_ready(red_led.port) ||
        !device_is_ready(green_led.port) ||
        !device_is_ready(blue_led.port)) {
        LOG_ERR("LEDs not ready");
        return 0;
    }

    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);

    while (1) {
        struct sensor_value temp;

        if (sensor_sample_fetch(dev) == 0 &&
            sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) == 0) {

            LOG_INF("Temperature: %d.%06d °C", temp.val1, temp.val2);
            set_color(temp.val1);
        } else {
            LOG_ERR("Sensor read error");
        }

        k_sleep(K_SECONDS(2));
    }

    return 0;
}
