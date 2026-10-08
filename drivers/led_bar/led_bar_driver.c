/*
 * WCNLB8-SR12 8-Bar LED Array Character Device Driver
 * Platform: NVIDIA Jetson Orin Nano / JetPack 5/6 (Linux Kernel 5.10 / 5.15)
 * Hardware Connections (Jetson 40-Pin Header):
 *   Bar 1: Pin 16 (PY.04, offset 126)
 *   Bar 2: Pin 18 (PY.03, offset 125)
 *   Bar 3: Pin 22 (PY.01, offset 123)
 *   Bar 4: Pin 36 (PR.05, offset 113)
 *   Bar 5: Pin 37 (PY.02, offset 124)
 *   Bar 6: Pin 38 (PI.01, offset 52)
 *   Bar 7: Pin 40 (PI.00, offset 51)
 *   Bar 8: Pin 24 (PZ.06, offset 136)
 * Device Node: /dev/led_bar
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/mutex.h>
#include <linux/version.h>

#include "../../include/common_ioctl.h"

#define DRIVER_NAME "led_bar"
#define CLASS_NAME  "led_bar_class"
#define DEVICE_NAME "led_bar"

#define NUM_BARS 8

/* Module Parameters */
static int gpio_base = -1;
module_param(gpio_base, int, 0644);
MODULE_PARM_DESC(gpio_base, "Base GPIO number for tegra234-gpio (-1 for auto-detect)");

static int bar_offsets[NUM_BARS] = {126, 125, 123, 113, 124, 52, 51, 136};
static int bar_count = NUM_BARS;
module_param_array(bar_offsets, int, &bar_count, 0644);
MODULE_PARM_DESC(bar_offsets, "Line offsets for Bar 1 to 8 (default: 126,125,123,113,124,52,51,136)");

static int active_low = 0;
module_param(active_low, int, 0644);
MODULE_PARM_DESC(active_low, "LED active level (0: active-high, 1: active-low)");

/* Global Variables */
static dev_t dev_number;
static struct cdev led_cdev;
static struct class *led_class = NULL;
static struct device *led_device = NULL;

static int real_gpios[NUM_BARS];
static uint8_t current_mask = 0;
static int current_level = 0;

static DEFINE_MUTEX(led_lock);

/* GPIO Match Helper for Tegra234 */
static int tegra_gpio_match(struct gpio_chip *gc, void *data)
{
    const char *name = (const char *)data;
    return (gc->label && strcmp(gc->label, name) == 0);
}

static int resolve_gpio(int offset)
{
    if (gpio_base < 0) {
        struct gpio_chip *gc = gpiochip_find("tegra234-gpio", tegra_gpio_match);
        if (gc) {
            gpio_base = gc->base;
            pr_info("led_bar: Auto-detected tegra234-gpio base: %d\n", gpio_base);
        } else {
            pr_warn("led_bar: tegra234-gpio not found, defaulting base to 348\n");
            gpio_base = 348;
        }
    }
    return gpio_base + offset;
}

/* Apply Bitmask to 8 GPIO lines */
static void apply_led_mask(uint8_t mask)
{
    int i;
    for (i = 0; i < NUM_BARS; i++) {
        int bit_val = (mask >> i) & 1;
        int out_val = active_low ? !bit_val : bit_val;
        gpio_set_value(real_gpios[i], out_val);
    }
    current_mask = mask;
}

/* Set Bargraph Level (0 to 8) */
static void set_led_level(int level)
{
    uint8_t mask = 0;
    int i;

    if (level < 0)
        level = 0;
    if (level > NUM_BARS)
        level = NUM_BARS;

    for (i = 0; i < level; i++) {
        mask |= (1 << i);
    }

    current_level = level;
    apply_led_mask(mask);
}

/* File Operations */
static int led_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int led_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t led_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[64];
    int len;

    mutex_lock(&led_lock);
    len = scnprintf(kbuf, sizeof(kbuf), "Level: %d/8 (Mask: 0x%02X)\n",
                    current_level, current_mask);
    mutex_unlock(&led_lock);

    return simple_read_from_buffer(buf, count, ppos, kbuf, len);
}

static ssize_t led_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[16];
    size_t copy_len;
    int val;

    if (count == 0)
        return 0;

    copy_len = min(count, sizeof(kbuf) - 1);
    if (copy_from_user(kbuf, buf, copy_len))
        return -EFAULT;
    kbuf[copy_len] = '\0';

    mutex_lock(&led_lock);

    if (sscanf(kbuf, "%d", &val) == 1) {
        if (val >= 0 && val <= NUM_BARS) {
            set_led_level(val);
        }
    } else if (kbuf[0] >= '0' && kbuf[0] <= '8') {
        val = kbuf[0] - '0';
        set_led_level(val);
    }

    mutex_unlock(&led_lock);
    return count;
}

static long led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int val;
    uint8_t raw_val;
    long ret = 0;

    mutex_lock(&led_lock);

    switch (cmd) {
    case LEDBAR_IOCTL_SET_LEVEL:
        if (copy_from_user(&val, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        if (val >= 0 && val <= NUM_BARS) {
            set_led_level(val);
        } else {
            ret = -EINVAL;
        }
        break;

    case LEDBAR_IOCTL_SET_RAW:
        if (copy_from_user(&raw_val, (uint8_t __user *)arg, sizeof(uint8_t))) {
            ret = -EFAULT;
            break;
        }
        apply_led_mask(raw_val);
        /* Approximate level from highest bit set */
        current_level = 0;
        {
            int i;
            for (i = 0; i < NUM_BARS; i++) {
                if (raw_val & (1 << i))
                    current_level = i + 1;
            }
        }
        break;

    case LEDBAR_IOCTL_GET_LEVEL:
        if (copy_to_user((int __user *)arg, &current_level, sizeof(int))) {
            ret = -EFAULT;
        }
        break;

    case LEDBAR_IOCTL_GET_RAW:
        if (copy_to_user((uint8_t __user *)arg, &current_mask, sizeof(uint8_t))) {
            ret = -EFAULT;
        }
        break;

    default:
        ret = -ENOTTY;
        break;
    }

    mutex_unlock(&led_lock);
    return ret;
}

static const struct file_operations led_fops = {
    .owner          = THIS_MODULE,
    .open           = led_open,
    .release        = led_release,
    .read           = led_read,
    .write          = led_write,
    .unlocked_ioctl = led_ioctl,
};

static int __init led_init(void)
{
    int ret, i;

    /* Allocate Character Device Number */
    ret = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("led_bar: Failed to allocate chrdev region\n");
        return ret;
    }

    cdev_init(&led_cdev, &led_fops);
    led_cdev.owner = THIS_MODULE;
    ret = cdev_add(&led_cdev, dev_number, 1);
    if (ret < 0) {
        pr_err("led_bar: Failed to add cdev\n");
        goto unregister_chrdev;
    }

    /* Create Device Class */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    led_class = class_create(CLASS_NAME);
#else
    led_class = class_create(THIS_MODULE, CLASS_NAME);
#endif
    if (IS_ERR(led_class)) {
        pr_err("led_bar: Failed to create class\n");
        ret = PTR_ERR(led_class);
        goto del_cdev;
    }

    led_device = device_create(led_class, NULL, dev_number, NULL, DEVICE_NAME);
    if (IS_ERR(led_device)) {
        pr_err("led_bar: Failed to create device /dev/%s\n", DEVICE_NAME);
        ret = PTR_ERR(led_device);
        goto destroy_class;
    }

    /* Request and Initialize 8 GPIO Pins */
    for (i = 0; i < NUM_BARS; i++) {
        char label[16];
        real_gpios[i] = resolve_gpio(bar_offsets[i]);
        snprintf(label, sizeof(label), "led_bar_%d", i + 1);

        ret = gpio_request(real_gpios[i], label);
        if (ret) {
            pr_err("led_bar: Failed to request GPIO %d (Bar %d, err: %d)\n",
                   real_gpios[i], i + 1, ret);
            goto free_gpios;
        }
        /* Default output LOW (OFF) */
        gpio_direction_output(real_gpios[i], active_low ? 1 : 0);
    }

    set_led_level(0);

    pr_info("led_bar: Driver loaded successfully (/dev/%s created, 8 bars initialized)\n",
            DEVICE_NAME);
    return 0;

free_gpios:
    while (--i >= 0) {
        gpio_set_value(real_gpios[i], active_low ? 1 : 0);
        gpio_free(real_gpios[i]);
    }
    device_destroy(led_class, dev_number);
destroy_class:
    class_destroy(led_class);
del_cdev:
    cdev_del(&led_cdev);
unregister_chrdev:
    unregister_chrdev_region(dev_number, 1);
    return ret;
}

static void __exit led_exit(void)
{
    int i;

    /* Turn off all LEDs and free GPIOs */
    for (i = 0; i < NUM_BARS; i++) {
        gpio_set_value(real_gpios[i], active_low ? 1 : 0);
        gpio_free(real_gpios[i]);
    }

    if (led_class) {
        device_destroy(led_class, dev_number);
        class_destroy(led_class);
    }
    cdev_del(&led_cdev);
    unregister_chrdev_region(dev_number, 1);

    pr_info("led_bar: Driver unloaded\n");
}

module_init(led_init);
module_exit(led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AIDL Embedded Driver Team");
MODULE_DESCRIPTION("WCNLB8-SR12 8-Bar LED Driver for Jetson Orin Nano");
MODULE_VERSION("1.0");
