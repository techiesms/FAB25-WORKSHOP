#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

// GPIO spec for RGB LEDs
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);


// Button spec
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

enum color_state {
	RED,
	GREEN,
	BLUE
};

void set_color(enum color_state color) {
	// Turn all LEDs OFF (active-low logic)
	gpio_pin_set_dt(&red_led, 1);
	gpio_pin_set_dt(&green_led, 1);
	gpio_pin_set_dt(&blue_led, 1);

	switch (color) {
	case RED:
		gpio_pin_set_dt(&red_led, 0);
		printf("Button pressed. New color: RED\n");
		break;
	case GREEN:
		gpio_pin_set_dt(&green_led, 0);
		printf("Button pressed. New color: GREEN\n");
		break;
	case BLUE:
		gpio_pin_set_dt(&blue_led, 0);
		printf("Button pressed. New color: BLUE\n");
		break;
	}
}

int main(void)
{
	if (!device_is_ready(red_led.port) ||
	    !device_is_ready(green_led.port) ||
	    !device_is_ready(blue_led.port) ||
	    !device_is_ready(button.port)) {
		printk("Error: Device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&button, GPIO_INPUT);

	enum color_state current_color = RED;
	set_color(current_color);

	while (1) {
		int val = gpio_pin_get_dt(&button);
		if (val == 0) {  // Button pressed (active low)
			k_msleep(50);  // debounce
            int val = gpio_pin_get_dt(&button);
		if (val == 0) {  // Button pressed (active low)
			// Manually cycle to next color
			if (current_color == RED) {
				current_color = GREEN;
			} else if (current_color == GREEN) {
				current_color = BLUE;
			} else {  // BLUE
				current_color = RED;
			}

			set_color(current_color);

			// Wait for button release
			while (gpio_pin_get_dt(&button) == 0) {
				k_msleep(50);
			}
		}
	}

		k_msleep(50);
	}
}