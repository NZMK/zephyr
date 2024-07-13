/*
 * Copyright (c) 2019 Actinius
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_OPT3001_H_
#define ZEPHYR_DRIVERS_SENSOR_OPT3001_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define OPT3001_REG_RESULT 0x00
#define OPT3001_REG_CONFIG 0x01
#define OPT3001_REG_LOW_LIMIT 0x02
#define OPT3001_REG_HIGH_LIMIT 0x03
#define OPT3001_REG_MANUFACTURER_ID 0x7E
#define OPT3001_REG_DEVICE_ID 0x7F

#define OPT3001_MANUFACTURER_ID_VALUE 0x5449
#define OPT3001_DEVICE_ID_VALUE 0x3001

#define OPT3001_LATCH_MASK BIT(1)
#define OPT3001_CONVERSION_MODE_MASK (BIT(10) | BIT(9))
#define OPT3001_CONVERSION_MODE_CONTINUOUS (BIT(10) | BIT(9))

#define OPT3001_SAMPLE_EXPONENT_SHIFT 12
#define OPT3001_MANTISSA_MASK BIT_MASK(12)

struct opt3001_data {
	uint16_t sample;
#ifdef CONFIG_OPT3001_TRIGGER
	struct gpio_callback int_gpio_cb;
	const struct sensor_trigger *trigger;
	sensor_trigger_handler_t trigger_handler;
	struct k_work trigger_work;
	const struct device *trigger_dev;
#endif /* CONFIG_OPT3001_TRIGGER */
};

struct opt3001_config {
	struct i2c_dt_spec i2c;
#ifdef CONFIG_OPT3001_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_OPT3001_TRIGGER */
};

static inline void opt3001_reg_val_decode(uint16_t reg_val, struct sensor_value *val)
{
	int32_t uval;

	/**
	 * sample consists of 4 bits of exponent and 12 bits of mantissa
	 * bits 15 to 12 are exponent bits
	 * bits 11 to 0 are the mantissa bits
	 *
	 * lux is the integer obtained using the following formula:
	 * (2^(exponent value)) * 0.01 * mantissa value
	 */
	uval = (1 << (reg_val >> OPT3001_SAMPLE_EXPONENT_SHIFT)) *
	       (reg_val & OPT3001_MANTISSA_MASK);
	val->val1 = uval / 100;
	val->val2 = (uval % 100) * 10000;
}

static inline int opt3001_reg_val_encode(uint16_t *reg_val, const struct sensor_value *val)
{
	int32_t uval;

	uval = val->val1 * 100 + val->val2 / 10000;

	if (uval < 0) {
		return -EINVAL;
	}

	*reg_val = 0;

	/* Find the exponent */
	while (uval > 2047) {
		(*reg_val)++;
		uval /= 2;
	}

	if (*reg_val > 0xF) {
		return -EINVAL;
	}

	*reg_val <<= OPT3001_SAMPLE_EXPONENT_SHIFT;

	/* Add the mantissa */
	*reg_val |= uval;

	return 0;
}

#ifdef CONFIG_OPT3001_TRIGGER
int opt3001_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);

int opt3001_reg_write(const struct device *dev, uint8_t reg, uint16_t val);

int opt3001_reg_update(const struct device *dev, uint8_t reg, uint16_t mask, uint16_t val);

int opt3001_int_init(const struct device *dev);

int opt3001_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		     const struct sensor_value *val);

int opt3001_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		     struct sensor_value *val);

int opt3001_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
#endif /* CONFIG_OPT3001_TRIGGER */

#endif /* _SENSOR_OPT3001_ */
