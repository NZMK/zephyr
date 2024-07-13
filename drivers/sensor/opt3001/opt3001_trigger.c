/*
 * Copyright (c) 2024 Chen Xingyu <hi@xingrz.me>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "opt3001.h"

LOG_MODULE_DECLARE(opt3001, CONFIG_SENSOR_LOG_LEVEL);

static void opt3001_int_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void opt3001_trigger_work_cb(struct k_work *work);

int opt3001_int_init(const struct device *dev)
{
	const struct opt3001_config *config = dev->config;
	struct opt3001_data *data = dev->data;

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("INT pin is not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT) != 0) {
		LOG_ERR("Failed to configure INT pin");
		return -EIO;
	}

	gpio_init_callback(&data->int_gpio_cb, opt3001_int_gpio_cb, BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb) != 0) {
		LOG_ERR("Failed to add INT pin callback");
		return -EIO;
	}

	if (opt3001_reg_update(dev, OPT3001_REG_CONFIG, OPT3001_LATCH_MASK, 0) != 0) {
		LOG_ERR("Failed to set latch mode");
		return -EIO;
	}

	k_work_init(&data->trigger_work, opt3001_trigger_work_cb);
	data->trigger_dev = dev;

	return 0;
}

static void opt3001_int_setup(const struct opt3001_config *config, bool enable)
{
	gpio_flags_t flags = enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;

	if (gpio_pin_interrupt_configure_dt(&config->int_gpio, flags) != 0) {
		LOG_ERR("Failed to configure INT pin interrupt");
	}
}

static void opt3001_int_gpio_cb(const struct device *gpio_dev, struct gpio_callback *cb,
				uint32_t pins)
{
	struct opt3001_data *data = CONTAINER_OF(cb, struct opt3001_data, int_gpio_cb);
	const struct device *dev = data->trigger_dev;

	opt3001_int_setup(dev->config, false);

	k_work_submit(&data->trigger_work);
}

static void opt3001_trigger_work_cb(struct k_work *work)
{
	struct opt3001_data *data = CONTAINER_OF(work, struct opt3001_data, trigger_work);
	const struct device *dev = data->trigger_dev;

	if (data->trigger_handler != NULL) {
		data->trigger_handler(dev, data->trigger);
	}

	opt3001_int_setup(dev->config, true);
}

int opt3001_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		     const struct sensor_value *val)
{
	uint16_t reg_val = 0;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	if (attr == SENSOR_ATTR_UPPER_THRESH) {
		if (opt3001_reg_val_encode(&reg_val, val) != 0) {
			return -EINVAL;
		}

		if (opt3001_reg_write(dev, OPT3001_REG_HIGH_LIMIT, reg_val) != 0) {
			return -EIO;
		}

		return 0;
	} else if (attr == SENSOR_ATTR_LOWER_THRESH) {
		if (opt3001_reg_val_encode(&reg_val, val) != 0) {
			return -EINVAL;
		}

		if (opt3001_reg_write(dev, OPT3001_REG_LOW_LIMIT, reg_val) != 0) {
			return -EIO;
		}

		return 0;
	}

	return -ENOTSUP;
}

int opt3001_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		     struct sensor_value *val)
{
	uint16_t reg_val;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	if (attr == SENSOR_ATTR_UPPER_THRESH) {
		if (opt3001_reg_read(dev, OPT3001_REG_HIGH_LIMIT, &reg_val) != 0) {
			return -EIO;
		}

		opt3001_reg_val_decode(reg_val, val);

		return 0;
	} else if (attr == SENSOR_ATTR_LOWER_THRESH) {
		if (opt3001_reg_read(dev, OPT3001_REG_LOW_LIMIT, &reg_val) != 0) {
			return -EIO;
		}

		opt3001_reg_val_decode(reg_val, val);

		return 0;
	}

	return -ENOTSUP;
}

int opt3001_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	const struct opt3001_config *config = dev->config;
	struct opt3001_data *data = dev->data;

	opt3001_int_setup(config, false);

	if (trig->type != SENSOR_TRIG_THRESHOLD || trig->chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	data->trigger = trig;
	data->trigger_handler = handler;

	opt3001_int_setup(config, true);
	if (gpio_pin_get_dt(&config->int_gpio) > 0) {
		k_work_submit(&data->trigger_work);
	}

	return 0;
}
