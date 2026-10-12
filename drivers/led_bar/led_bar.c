// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/compat.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include "common_ioctl.h"
#include "driver_common.h"

static char *gpiochip_label = "tegra234-gpio";
/* Header order: 16,18,22,36,37,38,40,24. */
static unsigned int bar_offsets[8] = { 126, 125, 123, 113, 124, 52, 51, 136 };
static unsigned int offset_count;
static bool active_low;
module_param(gpiochip_label, charp, 0444);
module_param_array(bar_offsets, uint, &offset_count, 0444);
module_param(active_low, bool, 0444);
MODULE_PARM_DESC(active_low, "Set true for sinking/active-low LED wiring");

static struct jetson_chrdev chr;
static int pins[8];
static DEFINE_MUTEX(led_lock);
static atomic_t opened = ATOMIC_INIT(0);
static u32 level;

static void set_level(u32 next)
{
	int i;

	for (i = 0; i < 8; i++)
		gpio_set_value_cansleep(pins[i], (i < next) ^ active_low);
	level = next;
}

static int led_open(struct inode *inode, struct file *file)
{
	if (atomic_cmpxchg(&opened, 0, 1))
		return -EBUSY;
	mutex_lock(&led_lock);
	set_level(0);
	mutex_unlock(&led_lock);
	return nonseekable_open(inode, file);
}

static int led_release(struct inode *inode, struct file *file)
{
	mutex_lock(&led_lock);
	set_level(0);
	mutex_unlock(&led_lock);
	atomic_set(&opened, 0);
	return 0;
}

static long led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 next;

	if (cmd == LED_BAR_IOC_SET_LEVEL) {
		if (copy_from_user(&next, (void __user *)arg, sizeof(next)))
			return -EFAULT;
		if (next > 8)
			return -EINVAL;
		mutex_lock(&led_lock);
		set_level(next);
		mutex_unlock(&led_lock);
		return 0;
	}
	if (cmd != LED_BAR_IOC_GET_LEVEL)
		return -ENOTTY;
	mutex_lock(&led_lock);
	next = level;
	mutex_unlock(&led_lock);
	return copy_to_user((void __user *)arg, &next, sizeof(next)) ? -EFAULT : 0;
}

#ifdef CONFIG_COMPAT
static long led_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return led_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations led_fops = {
	.owner = THIS_MODULE, .open = led_open, .release = led_release,
	.unlocked_ioctl = led_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = led_compat_ioctl,
#endif
	.llseek = no_llseek,
};

static int __init led_init(void)
{
	int i, ret;
	unsigned long flags = active_low ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;

	if (offset_count && offset_count != 8)
		return -EINVAL;
	for (i = 0; i < 8; i++) {
		pins[i] = jetson_gpio_number(gpiochip_label, bar_offsets[i]);
		if (pins[i] < 0) {
			ret = pins[i];
			goto free_pins;
		}
		ret = gpio_request_one(pins[i], flags, "led-bar");
		if (ret)
			goto free_pins;
	}
	ret = jetson_chrdev_create(&chr, "led_bar", &led_fops);
	if (ret)
		goto free_pins;
	pr_info("led_bar: /dev/led_bar ready, active_low=%d\n", active_low);
	return 0;
free_pins:
	while (i-- > 0)
		gpio_free(pins[i]);
	pr_err("led_bar: initialization failed: %d\n", ret);
	return ret;
}

static void __exit led_exit(void)
{
	int i;

	jetson_chrdev_destroy(&chr);
	set_level(0);
	for (i = 0; i < 8; i++)
		gpio_free(pins[i]);
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jetson Driver Project");
MODULE_DESCRIPTION("Eight GPIO LED level meter character device");
