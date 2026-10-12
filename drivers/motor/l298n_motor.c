// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/pwm.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include "common_ioctl.h"
#include "driver_common.h"

static char *gpiochip_label = "tegra234-gpio";
static unsigned int in1_offset = 105; /* Header 29, PQ.05. */
static unsigned int in2_offset = 106; /* Header 31, PQ.06. */
static int pwm_id = -1;
static char *pwm_controller = "32e0000.pwm";
static unsigned int period_ns = 1000000;
static unsigned int reverse_deadtime_ms = 100;
static unsigned int watchdog_ms = 1000;
module_param(gpiochip_label, charp, 0444);
module_param(in1_offset, uint, 0444);
module_param(in2_offset, uint, 0444);
module_param(pwm_id, int, 0444);
module_param(pwm_controller, charp, 0444);
module_param(period_ns, uint, 0444);
module_param(reverse_deadtime_ms, uint, 0444);
module_param(watchdog_ms, uint, 0444);
MODULE_PARM_DESC(pwm_id, "Required legacy global PWM ID for 32e0000.pwm channel 0");
MODULE_PARM_DESC(period_ns, "PWM period in ns; default 1 ms (1 kHz)");
MODULE_PARM_DESC(watchdog_ms, "Heartbeat timeout in ms (200 to 60000)");

static struct jetson_chrdev chr;
static struct pwm_device *motor_pwm;
static int in1_gpio, in2_gpio;
static DEFINE_MUTEX(motor_lock);
static atomic_t opened = ATOMIC_INIT(0);
static struct motor_state state;
static struct delayed_work watchdog_work;
static unsigned long last_command;
static bool watchdog_tripped;
static bool watchdog_armed;

/* Caller holds motor_lock, except before the device is published. */
static void motor_stop(void)
{
	int ret;

	/* A zero duty command helps leave ENA low before disabling the PWM. */
	ret = pwm_config(motor_pwm, 0, period_ns);
	pwm_disable(motor_pwm);
	gpio_set_value_cansleep(in1_gpio, 0);
	gpio_set_value_cansleep(in2_gpio, 0);
	state.enabled = 0;
	state.duty_permille = 0;
	if (ret)
		pr_err_ratelimited("l298n_motor: zero duty failed: %d\n", ret);
}

static int motor_apply(const struct motor_state *next)
{
	int ret;
	unsigned int duty_ns;
	bool running = state.enabled && state.duty_permille;

	if (!next->enabled || !next->duty_permille) {
		motor_stop();
		state = *next;
		return 0;
	}
	if (running && next->direction != state.direction) {
		motor_stop();
		msleep(reverse_deadtime_ms);
	}
	/* Direction GPIOs change only with ENA inactive when reversing. */
	gpio_set_value_cansleep(in1_gpio, next->direction == MOTOR_FORWARD);
	gpio_set_value_cansleep(in2_gpio, next->direction == MOTOR_REVERSE);
	duty_ns = (u64)period_ns * next->duty_permille / 1000;
	ret = pwm_config(motor_pwm, duty_ns, period_ns);
	if (ret)
		goto stop;
	ret = pwm_enable(motor_pwm);
	if (ret)
		goto stop;
	state = *next;
	return 0;
stop:
	motor_stop();
	return ret;
}

static void motor_watchdog(struct work_struct *work)
{
	mutex_lock(&motor_lock);
	if (watchdog_armed) {
		if (!watchdog_tripped && time_after_eq(jiffies,
		    last_command + msecs_to_jiffies(watchdog_ms))) {
			motor_stop();
			watchdog_tripped = true;
			pr_warn("l298n_motor: heartbeat expired; motor stopped\n");
		}
		schedule_delayed_work(&watchdog_work,
			msecs_to_jiffies(watchdog_ms / 2));
	}
	mutex_unlock(&motor_lock);
}

static int motor_open(struct inode *inode, struct file *file)
{
	if (atomic_cmpxchg(&opened, 0, 1))
		return -EBUSY;
	mutex_lock(&motor_lock);
	motor_stop();
	watchdog_tripped = false;
	watchdog_armed = true;
	last_command = jiffies;
	mutex_unlock(&motor_lock);
	schedule_delayed_work(&watchdog_work, msecs_to_jiffies(watchdog_ms / 2));
	return nonseekable_open(inode, file);
}

static int motor_release(struct inode *inode, struct file *file)
{
	/* Keep exclusive ownership until work cancellation and stopping finish. */
	mutex_lock(&motor_lock);
	watchdog_armed = false;
	mutex_unlock(&motor_lock);
	cancel_delayed_work_sync(&watchdog_work);
	mutex_lock(&motor_lock);
	motor_stop();
	mutex_unlock(&motor_lock);
	atomic_set(&opened, 0);
	return 0;
}

static long motor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct motor_state next;
	int ret = 0;

	if (cmd == MOTOR_IOC_SET_STATE) {
		if (copy_from_user(&next, (void __user *)arg, sizeof(next)))
			return -EFAULT;
		if (next.duty_permille > 1000 || next.direction > MOTOR_REVERSE ||
		    next.enabled > 1 || next.reserved)
			return -EINVAL;
	} else if (cmd != MOTOR_IOC_GET_STATE && cmd != MOTOR_IOC_KEEPALIVE) {
		return -ENOTTY;
	}
	mutex_lock(&motor_lock);
	if (cmd == MOTOR_IOC_GET_STATE) {
		next = state;
	} else if (watchdog_tripped) {
		ret = -ETIMEDOUT;
	} else if (cmd == MOTOR_IOC_SET_STATE) {
		ret = motor_apply(&next);
		if (!ret)
			last_command = jiffies;
	} else {
		last_command = jiffies;
	}
	mutex_unlock(&motor_lock);
	if (!ret && cmd == MOTOR_IOC_GET_STATE &&
	    copy_to_user((void __user *)arg, &next, sizeof(next)))
		return -EFAULT;
	return ret;
}

#ifdef CONFIG_COMPAT
static long motor_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return motor_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations motor_fops = {
	.owner = THIS_MODULE, .open = motor_open, .release = motor_release,
	.unlocked_ioctl = motor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = motor_compat_ioctl,
#endif
	.llseek = no_llseek,
};

static int __init motor_init(void)
{
	struct pwm_state initial;
	int ret;

	if (pwm_id < 0) {
		pr_err("l298n_motor: specify pwm_id for 32e0000.pwm channel 0\n");
		return -EINVAL;
	}
	if (period_ns < 100000 || period_ns > 1000000000 ||
	    !reverse_deadtime_ms || reverse_deadtime_ms > 5000 ||
	    watchdog_ms < 200 || watchdog_ms > 60000)
		return -EINVAL;
	in1_gpio = jetson_gpio_number(gpiochip_label, in1_offset);
	in2_gpio = jetson_gpio_number(gpiochip_label, in2_offset);
	if (in1_gpio < 0 || in2_gpio < 0)
		return in1_gpio < 0 ? in1_gpio : in2_gpio;
	ret = gpio_request_one(in1_gpio, GPIOF_OUT_INIT_LOW, "l298n-in1");
	if (ret)
		return ret;
	ret = gpio_request_one(in2_gpio, GPIOF_OUT_INIT_LOW, "l298n-in2");
	if (ret)
		goto free_in1;
	motor_pwm = pwm_request(pwm_id, "l298n-ena");
	if (IS_ERR(motor_pwm)) {
		ret = PTR_ERR(motor_pwm);
		goto free_in2;
	}
	if (strcmp(dev_name(motor_pwm->chip->dev), pwm_controller) || motor_pwm->hwpwm) {
		pr_err("l298n_motor: PWM %d is not %s channel 0\n", pwm_id, pwm_controller);
		ret = -EINVAL;
		goto free_pwm;
	}
	/* Normalize polarity before using the requested legacy PWM wrappers. */
	pwm_get_state(motor_pwm, &initial);
	initial.period = period_ns;
	initial.duty_cycle = 0;
	initial.polarity = PWM_POLARITY_NORMAL;
	initial.enabled = false;
	ret = pwm_apply_state(motor_pwm, &initial);
	if (ret)
		goto free_pwm;
	motor_stop();
	INIT_DELAYED_WORK(&watchdog_work, motor_watchdog);
	ret = jetson_chrdev_create(&chr, "l298n_motor", &motor_fops);
	if (ret)
		goto free_pwm;
	pr_info("l298n_motor: PWM %d, GPIOs %d,%d; /dev/l298n_motor ready\n",
		pwm_id, in1_gpio, in2_gpio);
	return 0;
free_pwm:
	pwm_free(motor_pwm);
free_in2:
	gpio_free(in2_gpio);
free_in1:
	gpio_free(in1_gpio);
	pr_err("l298n_motor: initialization failed: %d\n", ret);
	return ret;
}

static void __exit motor_exit(void)
{
	jetson_chrdev_destroy(&chr);
	cancel_delayed_work_sync(&watchdog_work);
	motor_stop();
	pwm_free(motor_pwm);
	gpio_free(in2_gpio);
	gpio_free(in1_gpio);
}

module_init(motor_init);
module_exit(motor_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jetson Driver Project");
MODULE_DESCRIPTION("L298N hardware PWM, direction GPIO and heartbeat character device");
