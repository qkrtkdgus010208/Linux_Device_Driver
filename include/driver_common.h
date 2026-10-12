/* SPDX-License-Identifier: GPL-2.0 */
#ifndef JETSON_DRIVER_COMMON_H
#define JETSON_DRIVER_COMMON_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/string.h>

struct jetson_chrdev {
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *device;
};

/* These modules target the two-argument class_create in 5.10 and 5.15. */
static inline int jetson_chrdev_create(struct jetson_chrdev *chr,
				      const char *name,
				      const struct file_operations *fops)
{
	int ret;

	ret = alloc_chrdev_region(&chr->devt, 0, 1, name);
	if (ret)
		return ret;
	cdev_init(&chr->cdev, fops);
	chr->cdev.owner = THIS_MODULE;
	ret = cdev_add(&chr->cdev, chr->devt, 1);
	if (ret)
		goto unregister;
	chr->class = class_create(THIS_MODULE, name);
	if (IS_ERR(chr->class)) {
		ret = PTR_ERR(chr->class);
		goto del;
	}
	chr->device = device_create(chr->class, NULL, chr->devt, NULL, "%s", name);
	if (IS_ERR(chr->device)) {
		ret = PTR_ERR(chr->device);
		goto destroy_class;
	}
	return 0;

destroy_class:
	class_destroy(chr->class);
del:
	cdev_del(&chr->cdev);
unregister:
	unregister_chrdev_region(chr->devt, 1);
	return ret;
}

static inline void jetson_chrdev_destroy(struct jetson_chrdev *chr)
{
	device_destroy(chr->class, chr->devt);
	class_destroy(chr->class);
	cdev_del(&chr->cdev);
	unregister_chrdev_region(chr->devt, 1);
}

static inline int jetson_gpio_match(struct gpio_chip *chip, void *label)
{
	return chip->label && !strcmp(chip->label, label);
}

/* Never treat a header pin number or chip-relative offset as a global GPIO. */
static inline int jetson_gpio_number(const char *label, unsigned int offset)
{
	struct gpio_chip *chip = gpiochip_find((void *)label, jetson_gpio_match);

	if (!chip)
		return -ENODEV;
	if (chip->base < 0 || offset >= chip->ngpio)
		return -EINVAL;
	return chip->base + offset;
}

#endif
