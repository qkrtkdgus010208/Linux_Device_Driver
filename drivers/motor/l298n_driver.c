/*
 * L298N DC Motor Driver (HW-095)
 * Platform: NVIDIA Jetson Orin Nano / JetPack 5/6 (Linux Kernel 5.10 / 5.15)
 * Hardware Connections (Jetson 40-Pin Header):
 *   ENA (Speed PWM): Pin 32 (PG.06, 32e0000.pwm, pwmchip1)
 *   IN1 (Direction 1): Pin 29 (PQ.05, offset 105)
 *   IN2 (Direction 2): Pin 31 (PQ.06, offset 106)
 * Device Node: /dev/l298n_motor
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/pwm.h>
#include <linux/mutex.h>
#include <linux/version.h>

#include "../../include/common_ioctl.h"

#define DRIVER_NAME "l298n_motor"
#define CLASS_NAME  "l298n_class"
#define DEVICE_NAME "l298n_motor"

/* Module Parameters */
static int gpio_base = -1;
module_param(gpio_base, int, 0644);
MODULE_PARM_DESC(gpio_base, "Base GPIO number for tegra234-gpio (-1 for auto-detect)");

static int pin_in1 = 105; /* PQ.05 on Pin 29 */
module_param(pin_in1, int, 0644);
MODULE_PARM_DESC(pin_in1, "IN1 direction GPIO offset (default 105 for Pin 29)");

static int pin_in2 = 106; /* PQ.06 on Pin 31 */
module_param(pin_in2, int, 0644);
MODULE_PARM_DESC(pin_in2, "IN2 direction GPIO offset (default 106 for Pin 31)");

static int pwm_id = 3; /* Pin 32 is 32e0000.pwm (pwmchip3) */
module_param(pwm_id, int, 0644);
MODULE_PARM_DESC(pwm_id, "Linux PWM ID for ENA pin (default 3 for Pin 32)");

static int pwm_period_ns = 1000000; /* 1 kHz default (1,000,000 ns) */
module_param(pwm_period_ns, int, 0644);
MODULE_PARM_DESC(pwm_period_ns, "PWM period in nanoseconds (default 1000000 = 1kHz)");

/* Global Variables */
static dev_t dev_number;
static struct cdev motor_cdev;
static struct class *motor_class = NULL;
static struct device *motor_device = NULL;

static struct pwm_device *motor_pwm = NULL;
static int real_gpio_in1 = -1;
static int real_gpio_in2 = -1;

static DEFINE_MUTEX(motor_lock);

static struct motor_status current_status = {
    .dir = MOTOR_DIR_STOP,
    .speed_percent = 0,
    .speed_step = 0,
    .is_enabled = 1,
};

/* Convert 0~8 steps to duty percentage */
static const int step_to_percent_table[9] = {
    0,   /* Step 0: 0% */
    25,  /* Step 1: 25% */
    35,  /* Step 2: 35% */
    45,  /* Step 3: 45% */
    55,  /* Step 4: 55% */
    65,  /* Step 5: 65% */
    75,  /* Step 6: 75% */
    85,  /* Step 7: 85% */
    100  /* Step 8: 100% */
};

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
            pr_info("l298n: Auto-detected tegra234-gpio base: %d\n", gpio_base);
        } else {
            pr_warn("l298n: tegra234-gpio not found, defaulting base to 348\n");
            gpio_base = 348;
        }
    }
    return gpio_base + offset;
}

/* Hardware Hardware Control */
static void apply_direction(enum motor_dir dir)
{
    switch (dir) {
    case MOTOR_DIR_FORWARD:
        gpio_set_value(real_gpio_in1, 1);
        gpio_set_value(real_gpio_in2, 0);
        break;
    case MOTOR_DIR_BACKWARD:
        gpio_set_value(real_gpio_in1, 0);
        gpio_set_value(real_gpio_in2, 1);
        break;
    case MOTOR_DIR_BRAKE:
        gpio_set_value(real_gpio_in1, 1);
        gpio_set_value(real_gpio_in2, 1);
        break;
    case MOTOR_DIR_STOP:
    default:
        gpio_set_value(real_gpio_in1, 0);
        gpio_set_value(real_gpio_in2, 0);
        break;
    }
}

static void apply_speed(int percent)
{
    int duty_ns;

    if (!motor_pwm)
        return;

    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;

    if (!current_status.is_enabled || current_status.dir == MOTOR_DIR_STOP) {
        pwm_config(motor_pwm, 0, pwm_period_ns);
        pwm_disable(motor_pwm);
        return;
    }

    if (percent == 0) {
        pwm_config(motor_pwm, 0, pwm_period_ns);
        pwm_disable(motor_pwm);
    } else {
        duty_ns = (int)(((u64)pwm_period_ns * percent) / 100);
        pwm_config(motor_pwm, duty_ns, pwm_period_ns);
        pwm_enable(motor_pwm);
    }
}

static void motor_update_hardware(void)
{
    apply_direction(current_status.dir);
    apply_speed(current_status.speed_percent);
}

/* File Operations */
static int motor_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int motor_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t motor_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[128];
    int len;

    mutex_lock(&motor_lock);
    len = scnprintf(kbuf, sizeof(kbuf),
                    "Motor Status:\n"
                    "  Enabled: %s\n"
                    "  Direction: %s\n"
                    "  Speed: %d%%\n"
                    "  Step: %d/8\n",
                    current_status.is_enabled ? "ON" : "OFF",
                    current_status.dir == MOTOR_DIR_FORWARD ? "FORWARD" :
                    current_status.dir == MOTOR_DIR_BACKWARD ? "BACKWARD" :
                    current_status.dir == MOTOR_DIR_BRAKE ? "BRAKE" : "STOP",
                    current_status.speed_percent,
                    current_status.speed_step);
    mutex_unlock(&motor_lock);

    return simple_read_from_buffer(buf, count, ppos, kbuf, len);
}

static ssize_t motor_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[64];
    size_t copy_len;
    int val;

    copy_len = min(count, sizeof(kbuf) - 1);
    if (copy_from_user(kbuf, buf, copy_len))
        return -EFAULT;
    kbuf[copy_len] = '\0';

    mutex_lock(&motor_lock);

    if (strncmp(kbuf, "forward", 7) == 0) {
        current_status.dir = MOTOR_DIR_FORWARD;
    } else if (strncmp(kbuf, "backward", 8) == 0) {
        current_status.dir = MOTOR_DIR_BACKWARD;
    } else if (strncmp(kbuf, "stop", 4) == 0) {
        current_status.dir = MOTOR_DIR_STOP;
    } else if (strncmp(kbuf, "brake", 5) == 0) {
        current_status.dir = MOTOR_DIR_BRAKE;
    } else if (sscanf(kbuf, "step %d", &val) == 1) {
        if (val >= 0 && val <= 8) {
            current_status.speed_step = val;
            current_status.speed_percent = step_to_percent_table[val];
        }
    } else if (sscanf(kbuf, "%d", &val) == 1) {
        if (val >= 0 && val <= 100) {
            current_status.speed_percent = val;
            current_status.speed_step = (val * 8) / 100;
        }
    }

    motor_update_hardware();
    mutex_unlock(&motor_lock);

    return count;
}

static long motor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int val;
    long ret = 0;

    mutex_lock(&motor_lock);

    switch (cmd) {
    case MOTOR_IOCTL_SET_SPEED:
        if (copy_from_user(&val, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        if (val >= 0 && val <= 100) {
            current_status.speed_percent = val;
            current_status.speed_step = (val * 8 + 50) / 100;
            if (current_status.speed_step > 8)
                current_status.speed_step = 8;
            motor_update_hardware();
        } else {
            ret = -EINVAL;
        }
        break;

    case MOTOR_IOCTL_SET_DIR:
        if (copy_from_user(&val, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        if (val >= MOTOR_DIR_STOP && val <= MOTOR_DIR_BRAKE) {
            current_status.dir = (enum motor_dir)val;
            motor_update_hardware();
        } else {
            ret = -EINVAL;
        }
        break;

    case MOTOR_IOCTL_GET_STATUS:
        if (copy_to_user((struct motor_status __user *)arg,
                         &current_status, sizeof(struct motor_status))) {
            ret = -EFAULT;
        }
        break;

    case MOTOR_IOCTL_STOP:
        current_status.dir = MOTOR_DIR_STOP;
        motor_update_hardware();
        break;

    case MOTOR_IOCTL_BRAKE:
        current_status.dir = MOTOR_DIR_BRAKE;
        motor_update_hardware();
        break;

    case MOTOR_IOCTL_ENABLE:
        if (copy_from_user(&val, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        current_status.is_enabled = val ? 1 : 0;
        motor_update_hardware();
        break;

    default:
        ret = -ENOTTY;
        break;
    }

    mutex_unlock(&motor_lock);
    return ret;
}

static const struct file_operations motor_fops = {
    .owner          = THIS_MODULE,
    .open           = motor_open,
    .release        = motor_release,
    .read           = motor_read,
    .write          = motor_write,
    .unlocked_ioctl = motor_ioctl,
};

static int __init motor_init(void)
{
    int ret;

    /* Allocate Character Device Number */
    ret = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("l298n: Failed to allocate chrdev region\n");
        return ret;
    }

    cdev_init(&motor_cdev, &motor_fops);
    motor_cdev.owner = THIS_MODULE;
    ret = cdev_add(&motor_cdev, dev_number, 1);
    if (ret < 0) {
        pr_err("l298n: Failed to add cdev\n");
        goto unregister_chrdev;
    }

    /* Create Device Class */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    motor_class = class_create(CLASS_NAME);
#else
    motor_class = class_create(THIS_MODULE, CLASS_NAME);
#endif
    if (IS_ERR(motor_class)) {
        pr_err("l298n: Failed to create class\n");
        ret = PTR_ERR(motor_class);
        goto del_cdev;
    }

    motor_device = device_create(motor_class, NULL, dev_number, NULL, DEVICE_NAME);
    if (IS_ERR(motor_device)) {
        pr_err("l298n: Failed to create device /dev/%s\n", DEVICE_NAME);
        ret = PTR_ERR(motor_device);
        goto destroy_class;
    }

    /* Resolve and Request Direction GPIOs */
    real_gpio_in1 = resolve_gpio(pin_in1);
    real_gpio_in2 = resolve_gpio(pin_in2);

    ret = gpio_request(real_gpio_in1, "l298n_in1");
    if (ret) {
        pr_err("l298n: Failed to request IN1 GPIO %d (err: %d)\n", real_gpio_in1, ret);
        goto destroy_dev;
    }
    gpio_direction_output(real_gpio_in1, 0);

    ret = gpio_request(real_gpio_in2, "l298n_in2");
    if (ret) {
        pr_err("l298n: Failed to request IN2 GPIO %d (err: %d)\n", real_gpio_in2, ret);
        goto free_in1;
    }
    gpio_direction_output(real_gpio_in2, 0);

    /* Request PWM for ENA Speed Control */
    motor_pwm = pwm_request(pwm_id, "l298n_ena");
    if (IS_ERR(motor_pwm)) {
        pr_warn("l298n: Failed to request PWM id %d (err: %ld), trying fallback pwm_id 0...\n",
                pwm_id, PTR_ERR(motor_pwm));
        motor_pwm = pwm_request(0, "l298n_ena");
    }

    if (IS_ERR(motor_pwm)) {
        pr_err("l298n: PWM request failed completely (err: %ld). Check DT or pinmux.\n",
               PTR_ERR(motor_pwm));
        /* Allow module to load with warning so direction can still be tested */
        motor_pwm = NULL;
    } else {
        /* Initialize PWM: period = pwm_period_ns, duty = 0, disabled */
        pwm_config(motor_pwm, 0, pwm_period_ns);
        pwm_disable(motor_pwm);
        pr_info("l298n: PWM requested successfully (period: %d ns)\n", pwm_period_ns);
    }

    pr_info("l298n: Driver loaded successfully (/dev/%s created, IN1:%d, IN2:%d)\n",
            DEVICE_NAME, real_gpio_in1, real_gpio_in2);
    return 0;

free_in1:
    gpio_free(real_gpio_in1);
destroy_dev:
    device_destroy(motor_class, dev_number);
destroy_class:
    class_destroy(motor_class);
del_cdev:
    cdev_del(&motor_cdev);
unregister_chrdev:
    unregister_chrdev_region(dev_number, 1);
    return ret;
}

static void __exit motor_exit(void)
{
    /* Safe shutdown: stop motor and disable PWM */
    if (real_gpio_in1 >= 0) {
        gpio_set_value(real_gpio_in1, 0);
        gpio_free(real_gpio_in1);
    }
    if (real_gpio_in2 >= 0) {
        gpio_set_value(real_gpio_in2, 0);
        gpio_free(real_gpio_in2);
    }

    if (motor_pwm) {
        pwm_config(motor_pwm, 0, pwm_period_ns);
        pwm_disable(motor_pwm);
        pwm_free(motor_pwm);
    }

    if (motor_class) {
        device_destroy(motor_class, dev_number);
        class_destroy(motor_class);
    }
    cdev_del(&motor_cdev);
    unregister_chrdev_region(dev_number, 1);

    pr_info("l298n: Driver unloaded\n");
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AIDL Embedded Driver Team");
MODULE_DESCRIPTION("L298N DC Motor Driver for Jetson Orin Nano");
MODULE_VERSION("1.0");
