// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/compat.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include "common_ioctl.h"
#include "driver_common.h"

static char *gpiochip_label = "tegra234-gpio";
static unsigned int clk_offset = 112; /* Header 11, PR.04. */
static unsigned int dt_offset = 122;  /* Header 13, PY.00. */
static unsigned int sw_offset = 85;   /* Header 15, PN.01. */
static unsigned int debounce_us = 1500;
static unsigned int button_debounce_ms = 15;
module_param(gpiochip_label, charp, 0444);
module_param(clk_offset, uint, 0444);
module_param(dt_offset, uint, 0444);
module_param(sw_offset, uint, 0444);
module_param(debounce_us, uint, 0444);
module_param(button_debounce_ms, uint, 0444);
MODULE_PARM_DESC(debounce_us, "Minimum interval between accepted CLK falling edges");

static struct jetson_chrdev chr;
static int pins[3];
static int clk_irq, sw_irq;
static DECLARE_KFIFO(events, struct ec11_event, 128);
static DEFINE_SPINLOCK(event_lock);
static DEFINE_MUTEX(read_lock);
static DECLARE_WAIT_QUEUE_HEAD(event_wait);
static atomic_t opened = ATOMIC_INIT(0);
static atomic_t button_active = ATOMIC_INIT(0);
static struct delayed_work button_work;
static u64 last_rotation_ns, button_start_ns;
static bool button_verified;
static unsigned int release_samples;
static u32 sequence, dropped;

static void push_event(u32 type, s32 value)
{
	struct ec11_event ev = {
		.type = type, .value = value, .timestamp_ns = ktime_get_ns(),
	};
	unsigned long flags;

	spin_lock_irqsave(&event_lock, flags);
	ev.sequence = ++sequence;
	if (atomic_read(&opened) && !kfifo_put(&events, ev))
		dropped++;
	spin_unlock_irqrestore(&event_lock, flags);
	wake_up_interruptible(&event_wait);
}

static bool has_events(void)
{
	unsigned long flags;
	bool ready;

	spin_lock_irqsave(&event_lock, flags);
	ready = !kfifo_is_empty(&events);
	spin_unlock_irqrestore(&event_lock, flags);
	return ready;
}

static irqreturn_t rotation_irq(int irq, void *data)
{
	u64 now = ktime_get_ns();

	if (gpio_get_value(pins[0]))
		return IRQ_HANDLED;
	if (last_rotation_ns && now - last_rotation_ns < (u64)debounce_us * 1000)
		return IRQ_HANDLED;
	last_rotation_ns = now;
	/* Wiring and EC11 variant determine which physical turn is positive. */
	push_event(EC11_EVENT_ROTATE, gpio_get_value(pins[1]) ? 1 : -1);
	return IRQ_HANDLED;
}

static void button_sample(struct work_struct *work)
{
	bool pressed = !gpio_get_value(pins[2]);
	u64 held_ms;

	if (!button_verified) {
		if (!pressed) {
			atomic_set(&button_active, 0);
			return;
		}
		button_verified = true;
	}
	if (pressed) {
		release_samples = 0;
	} else if (++release_samples >= 2) {
		held_ms = (ktime_get_ns() - button_start_ns) / NSEC_PER_MSEC;
		push_event(EC11_EVENT_BUTTON, min_t(u64, held_ms, INT_MAX));
		atomic_set(&button_active, 0);
		return;
	}
	schedule_delayed_work(&button_work, msecs_to_jiffies(button_debounce_ms));
}

static irqreturn_t button_irq(int irq, void *data)
{
	if (atomic_cmpxchg(&button_active, 0, 1) == 0) {
		button_start_ns = ktime_get_ns();
		button_verified = false;
		release_samples = 0;
		schedule_delayed_work(&button_work, msecs_to_jiffies(button_debounce_ms));
	}
	return IRQ_HANDLED;
}

static int ec11_open(struct inode *inode, struct file *file)
{
	unsigned long flags;

	if (atomic_cmpxchg(&opened, 0, 1))
		return -EBUSY;
	spin_lock_irqsave(&event_lock, flags);
	kfifo_reset(&events);
	sequence = 0;
	dropped = 0;
	spin_unlock_irqrestore(&event_lock, flags);
	return nonseekable_open(inode, file);
}

static int ec11_release(struct inode *inode, struct file *file)
{
	atomic_set(&opened, 0);
	return 0;
}

static ssize_t ec11_read(struct file *file, char __user *buf, size_t count,
			 loff_t *pos)
{
	struct ec11_event ev;
	unsigned long flags;
	int ret;

	if (!count)
		return 0;
	if (count < sizeof(ev))
		return -EINVAL;
	for (;;) {
		ret = mutex_lock_interruptible(&read_lock);
		if (ret)
			return ret;
		spin_lock_irqsave(&event_lock, flags);
		ret = kfifo_peek(&events, &ev);
		spin_unlock_irqrestore(&event_lock, flags);
		if (ret)
			break;
		mutex_unlock(&read_lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(event_wait, has_events());
		if (ret)
			return ret;
	}
	if (copy_to_user(buf, &ev, sizeof(ev))) {
		mutex_unlock(&read_lock);
		return -EFAULT;
	}
	spin_lock_irqsave(&event_lock, flags);
	ret = kfifo_get(&events, &ev);
	spin_unlock_irqrestore(&event_lock, flags);
	mutex_unlock(&read_lock);
	return ret ? sizeof(ev) : -EIO;
}

static __poll_t ec11_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &event_wait, wait);
	return has_events() ? EPOLLIN | EPOLLRDNORM : 0;
}

static long ec11_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ec11_stats stats;
	unsigned long flags;

	if (cmd != EC11_IOC_GET_STATS)
		return -ENOTTY;
	spin_lock_irqsave(&event_lock, flags);
	stats.dropped = dropped;
	stats.sequence = sequence;
	spin_unlock_irqrestore(&event_lock, flags);
	return copy_to_user((void __user *)arg, &stats, sizeof(stats)) ? -EFAULT : 0;
}

#ifdef CONFIG_COMPAT
static long ec11_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return ec11_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations ec11_fops = {
	.owner = THIS_MODULE, .open = ec11_open, .release = ec11_release,
	.read = ec11_read, .poll = ec11_poll, .unlocked_ioctl = ec11_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ec11_compat_ioctl,
#endif
	.llseek = no_llseek,
};

static int __init ec11_init(void)
{
	unsigned int offsets[] = { clk_offset, dt_offset, sw_offset };
	const char *labels[] = { "ec11-clk", "ec11-dt", "ec11-sw" };
	int i, ret;

	if (!debounce_us || debounce_us > 100000 ||
	    !button_debounce_ms || button_debounce_ms > 100)
		return -EINVAL;
	INIT_DELAYED_WORK(&button_work, button_sample);
	for (i = 0; i < 3; i++) {
		pins[i] = jetson_gpio_number(gpiochip_label, offsets[i]);
		if (pins[i] < 0) {
			ret = pins[i];
			goto free_pins;
		}
		ret = gpio_request_one(pins[i], GPIOF_IN, labels[i]);
		if (ret)
			goto free_pins;
		/* Hard IRQ sampling requires a non-sleeping SoC GPIO controller. */
		if (gpio_cansleep(pins[i])) {
			ret = -EOPNOTSUPP;
			i++;
			goto free_pins;
		}
	}
	clk_irq = gpio_to_irq(pins[0]);
	sw_irq = gpio_to_irq(pins[2]);
	if (clk_irq < 0 || sw_irq < 0) {
		ret = clk_irq < 0 ? clk_irq : sw_irq;
		goto free_pins;
	}
	ret = request_irq(clk_irq, rotation_irq, IRQF_TRIGGER_FALLING, "ec11-clk", &chr);
	if (ret)
		goto free_pins;
	ret = request_irq(sw_irq, button_irq, IRQF_TRIGGER_FALLING, "ec11-sw", &chr);
	if (ret)
		goto free_clk_irq;
	ret = jetson_chrdev_create(&chr, "ec11", &ec11_fops);
	if (ret)
		goto free_sw_irq;
	pr_info("ec11: GPIOs %d,%d,%d; /dev/ec11 ready\n", pins[0], pins[1], pins[2]);
	return 0;

free_sw_irq:
	free_irq(sw_irq, &chr);
	cancel_delayed_work_sync(&button_work);
free_clk_irq:
	free_irq(clk_irq, &chr);
free_pins:
	while (i-- > 0)
		gpio_free(pins[i]);
	pr_err("ec11: initialization failed: %d\n", ret);
	return ret;
}

static void __exit ec11_exit(void)
{
	int i;

	jetson_chrdev_destroy(&chr);
	free_irq(sw_irq, &chr);
	free_irq(clk_irq, &chr);
	cancel_delayed_work_sync(&button_work);
	for (i = 0; i < 3; i++)
		gpio_free(pins[i]);
}

module_init(ec11_init);
module_exit(ec11_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jetson Driver Project");
MODULE_DESCRIPTION("EC11 falling-edge encoder and debounced switch character device");
