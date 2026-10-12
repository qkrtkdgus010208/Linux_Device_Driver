/*
 * EC11 Rotary Encoder Character Device Driver
 * Platform: NVIDIA Jetson Orin Nano / JetPack 5/6 (Linux Kernel 5.10 / 5.15)
 * Hardware Connections (Jetson 40-Pin Header):
 *   S1  (Channel A / CLK): Pin 11 (PR.04, offset 112)
 *   S2  (Channel B / DT ): Pin 13 (PY.00, offset 122)
 *   KEY (Push Switch/ SW): Pin 15 (PN.01, offset 85)
 *   GND (Ground         ): Pin 6 (or any GND)
 *   5V  (Module Power   ): Pin 1 (3.3V recommended for Jetson logic level)
 * Device Node: /dev/ec11
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/version.h>

#include "../../include/common_ioctl.h"

#define DRIVER_NAME "ec11_encoder"
#define CLASS_NAME  "ec11_class"
#define DEVICE_NAME "ec11"

#define QUEUE_SIZE 64

/* Module Parameters */
static int gpio_base = -1;
module_param(gpio_base, int, 0644);
MODULE_PARM_DESC(gpio_base, "Base GPIO number for tegra234-gpio (-1 for auto-detect)");

static int pin_s1 = 112;
module_param(pin_s1, int, 0644);
MODULE_PARM_DESC(pin_s1, "S1 (Channel A / CLK) line offset (default 112 for Pin 11)");

static int pin_s2 = 122;
module_param(pin_s2, int, 0644);
MODULE_PARM_DESC(pin_s2, "S2 (Channel B / DT) line offset (default 122 for Pin 13)");

static int pin_key = 85;
module_param(pin_key, int, 0644);
MODULE_PARM_DESC(pin_key, "KEY (Push Switch / SW) line offset (default 85 for Pin 15)");

/* Backward compatibility parameter aliases */
static int pin_clk = -1;
module_param(pin_clk, int, 0644);
MODULE_PARM_DESC(pin_clk, "Alias for pin_s1");

static int pin_dt = -1;
module_param(pin_dt, int, 0644);
MODULE_PARM_DESC(pin_dt, "Alias for pin_s2");

static int pin_sw = -1;
module_param(pin_sw, int, 0644);
MODULE_PARM_DESC(pin_sw, "Alias for pin_key");

/*
 * Kept only so existing "insmod ... debounce_ms=N" command lines still load.
 * Rotation no longer uses a time filter: the quadrature state machine below
 * rejects contact bounce by itself, and a time filter would drop real edges.
 */
static int debounce_ms = 0;
module_param(debounce_ms, int, 0644);
MODULE_PARM_DESC(debounce_ms, "Deprecated, ignored (rotation is decoded by a quadrature state machine)");

static int steps_per_detent = 4;
module_param(steps_per_detent, int, 0644);
MODULE_PARM_DESC(steps_per_detent, "Quadrature transitions per detent: 4 = full cycle per click (default), 2 = half cycle per click");

static int rebound_ms = 0;
module_param(rebound_ms, int, 0644);
MODULE_PARM_DESC(rebound_ms, "Deprecated, ignored (recoil rejected naturally by Buxtronix FSM)");

static int key_debounce_ms = 150;
module_param(key_debounce_ms, int, 0644);
MODULE_PARM_DESC(key_debounce_ms, "Key switch debounce time in ms (default 150)");

static int invert_dir = 0;
module_param(invert_dir, int, 0644);
MODULE_PARM_DESC(invert_dir, "Invert rotation direction (0: normal, 1: invert)");

/* Global Variables */
static dev_t dev_number;
static struct cdev ec11_cdev;
static struct class *ec11_class = NULL;
static struct device *ec11_device = NULL;

static int real_gpio_s1 = -1;
static int real_gpio_s2 = -1;
static int real_gpio_key = -1;

static int irq_s1 = -1;
static int irq_s2 = -1;
static int irq_key = -1;

static int accumulated_pos = 0;
static ktime_t last_key_time;

/*
 * Standard Quadrature Lookup Table (16 states)
 * Row: old_state (0~3), Column: new_state (0~3)
 * State encoding: bit 1 = S1 (CLK), bit 0 = S2 (DT)
 * CW sequence:  11(3) -> 01(1) -> 00(0) -> 10(2) -> 11(3)  => +1 each
 * CCW sequence: 11(3) -> 10(2) -> 00(0) -> 01(1) -> 11(3)  => -1 each
 */
static const s8 quad_table[16] = {
     0, -1, +1,  0,   /* old=00: to 00(0), 01(-1), 10(+1), 11(0) */
    +1,  0,  0, -1,   /* old=01: to 00(+1), 01(0), 10(0), 11(-1) */
    -1,  0,  0, +1,   /* old=10: to 00(-1), 01(0), 10(0), 11(+1) */
     0, +1, -1,  0,   /* old=11: to 00(0), 01(+1), 10(-1), 11(0) */
};

static DEFINE_SPINLOCK(enc_lock);
static u8 enc_state = 3;
static int enc_acc = 0;
static bool step_fired = false;
static bool in_detent = true;

/* Circular Event Queue */
static struct ec11_event queue[QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;
static spinlock_t queue_lock;
static wait_queue_head_t ec11_waitq;

static inline bool is_queue_empty(void)
{
    return queue_head == queue_tail;
}

static inline bool is_queue_full(void)
{
    return ((queue_head + 1) % QUEUE_SIZE) == queue_tail;
}

static void enqueue_event(const struct ec11_event *ev)
{
    unsigned long flags;

    spin_lock_irqsave(&queue_lock, flags);
    if (!is_queue_full()) {
        queue[queue_head] = *ev;
        queue_head = (queue_head + 1) % QUEUE_SIZE;
    }
    spin_unlock_irqrestore(&queue_lock, flags);

    wake_up_interruptible(&ec11_waitq);
}

static int dequeue_event(struct ec11_event *ev)
{
    unsigned long flags;
    int ret = -1;

    spin_lock_irqsave(&queue_lock, flags);
    if (!is_queue_empty()) {
        *ev = queue[queue_tail];
        queue_tail = (queue_tail + 1) % QUEUE_SIZE;
        ret = 0;
    }
    spin_unlock_irqrestore(&queue_lock, flags);

    return ret;
}

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
            pr_info("ec11: Auto-detected tegra234-gpio base: %d\n", gpio_base);
        } else {
            pr_warn("ec11: tegra234-gpio not found, defaulting base to 348\n");
            gpio_base = 348;
        }
    }
    return gpio_base + offset;
}

/*
 * Shared handler for S1 and S2, both edges.
 * Accurately decodes rotation using detent exit-unlatch logic:
 * - When exiting detent: latch is unlocked for the new click.
 * - During movement: fires immediately at threshold 2 (immediate response on direction reversal).
 * - Once fired: strictly locked against double steps until entering AND exiting the next detent.
 * - When entering detent: accumulator is cleared, latch remains locked against settling chatter.
 */
static irqreturn_t ec11_rot_irq_handler(int irq, void *dev_id)
{
    unsigned long flags;
    u8 new_state;
    int delta;
    int step = 0;
    bool is_detent_state;
    struct ec11_event ev;

    spin_lock_irqsave(&enc_lock, flags);

    new_state = (gpio_get_value(real_gpio_s1) ? 2 : 0) |
                (gpio_get_value(real_gpio_s2) ? 1 : 0);

    if (new_state == enc_state) {
        spin_unlock_irqrestore(&enc_lock, flags);
        return IRQ_HANDLED;
    }

    delta = quad_table[(enc_state << 2) | new_state];
    enc_state = new_state;

    if (steps_per_detent == 2)
        is_detent_state = (new_state == 3 || new_state == 0);
    else
        is_detent_state = (new_state == 3);

    /*
     * Detent Transition Tracking:
     * When leaving detent (starting a new click), unlock the latch for this new click.
     */
    if (!is_detent_state) {
        if (in_detent) {
            in_detent = false;
            step_fired = false;
            enc_acc = 0;
        }
    } else {
        in_detent = true;
    }

    if (delta != 0) {
        enc_acc += delta;

        /*
         * Threshold 2: immediate response on the very 1st click during direction reversal.
         * step_fired latch strictly prevents double steps until the next physical click.
         */
        if (!step_fired) {
            if (enc_acc >= 2) {
                step = 1;
                step_fired = true;
            } else if (enc_acc <= -2) {
                step = -1;
                step_fired = true;
            }
        }
    }

    /* Clear accumulator when settled in detent to eliminate any residual bounce */
    if (is_detent_state) {
        enc_acc = 0;
    }

    if (step != 0) {
        if (invert_dir)
            step = -step;
        accumulated_pos += step;
        ev.delta = step;
        ev.sw_state = 0;
        ev.position = accumulated_pos;
    }

    spin_unlock_irqrestore(&enc_lock, flags);

    if (step != 0)
        enqueue_event(&ev);

    return IRQ_HANDLED;
}

/* Interrupt Handler for KEY (Push Switch) Falling Edge */
static irqreturn_t ec11_key_irq_handler(int irq, void *dev_id)
{
    ktime_t now = ktime_get();
    s64 delta_ms = ktime_to_ms(ktime_sub(now, last_key_time));
    int key_val;
    struct ec11_event ev;

    if (delta_ms < key_debounce_ms) {
        return IRQ_HANDLED;
    }

    /* Glitch & Vibration Validation: KEY switch must actually be pressed (LOW) */
    key_val = gpio_get_value(real_gpio_key);
    if (key_val != 0) {
        return IRQ_HANDLED;
    }

    last_key_time = now;

    ev.delta = 0;
    ev.sw_state = 2; /* 2 indicates click event */
    ev.position = accumulated_pos;

    enqueue_event(&ev);

    return IRQ_HANDLED;
}

/* File Operations */
static int ec11_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int ec11_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t ec11_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct ec11_event ev;
    int ret;

    if (count < sizeof(struct ec11_event)) {
        return -EINVAL;
    }

    if (file->f_flags & O_NONBLOCK) {
        if (dequeue_event(&ev) < 0) {
            return -EAGAIN;
        }
    } else {
        ret = wait_event_interruptible(ec11_waitq, dequeue_event(&ev) == 0);
        if (ret != 0) {
            return ret; /* -ERESTARTSYS */
        }
    }

    if (copy_to_user(buf, &ev, sizeof(struct ec11_event))) {
        return -EFAULT;
    }

    return sizeof(struct ec11_event);
}

static __poll_t ec11_poll(struct file *file, struct poll_table_struct *wait)
{
    __poll_t mask = 0;
    unsigned long flags;

    poll_wait(file, &ec11_waitq, wait);

    spin_lock_irqsave(&queue_lock, flags);
    if (!is_queue_empty()) {
        mask |= (EPOLLIN | EPOLLRDNORM);
    }
    spin_unlock_irqrestore(&queue_lock, flags);

    return mask;
}

static long ec11_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int val;
    unsigned long flags;

    switch (cmd) {
    case EC11_IOCTL_GET_POS:
        spin_lock_irqsave(&enc_lock, flags);
        val = accumulated_pos;
        spin_unlock_irqrestore(&enc_lock, flags);
        if (copy_to_user((int __user *)arg, &val, sizeof(int))) {
            return -EFAULT;
        }
        break;

    case EC11_IOCTL_RESET_POS:
        spin_lock_irqsave(&enc_lock, flags);
        accumulated_pos = 0;
        spin_unlock_irqrestore(&enc_lock, flags);
        break;

    case EC11_IOCTL_GET_SW:
        /* Key switch is active-low: return 1 if pressed, 0 if released */
        val = gpio_get_value(real_gpio_key) ? 0 : 1;
        if (copy_to_user((int __user *)arg, &val, sizeof(int))) {
            return -EFAULT;
        }
        break;

    default:
        return -ENOTTY;
    }

    return 0;
}

static const struct file_operations ec11_fops = {
    .owner          = THIS_MODULE,
    .open           = ec11_open,
    .release        = ec11_release,
    .read           = ec11_read,
    .poll           = ec11_poll,
    .unlocked_ioctl = ec11_ioctl,
};

static int __init ec11_init(void)
{
    int ret;

    /* Handle legacy alias parameters if set */
    if (pin_clk >= 0)
        pin_s1 = pin_clk;
    if (pin_dt >= 0)
        pin_s2 = pin_dt;
    if (pin_sw >= 0)
        pin_key = pin_sw;

    spin_lock_init(&queue_lock);
    init_waitqueue_head(&ec11_waitq);
    last_key_time = ktime_set(0, 0);

    /* Allocate Character Device Number */
    ret = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("ec11: Failed to allocate chrdev region\n");
        return ret;
    }

    cdev_init(&ec11_cdev, &ec11_fops);
    ec11_cdev.owner = THIS_MODULE;
    ret = cdev_add(&ec11_cdev, dev_number, 1);
    if (ret < 0) {
        pr_err("ec11: Failed to add cdev\n");
        goto unregister_chrdev;
    }

    /* Create Device Class (compat with Linux < 6.4 and >= 6.4) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    ec11_class = class_create(CLASS_NAME);
#else
    ec11_class = class_create(THIS_MODULE, CLASS_NAME);
#endif
    if (IS_ERR(ec11_class)) {
        pr_err("ec11: Failed to create class\n");
        ret = PTR_ERR(ec11_class);
        goto del_cdev;
    }

    ec11_device = device_create(ec11_class, NULL, dev_number, NULL, DEVICE_NAME);
    if (IS_ERR(ec11_device)) {
        pr_err("ec11: Failed to create device /dev/%s\n", DEVICE_NAME);
        ret = PTR_ERR(ec11_device);
        goto destroy_class;
    }

    /* Resolve and Request GPIOs */
    real_gpio_s1  = resolve_gpio(pin_s1);
    real_gpio_s2  = resolve_gpio(pin_s2);
    real_gpio_key = resolve_gpio(pin_key);

    pr_info("ec11: Initializing pins -> S1(CLK):%d, S2(DT):%d, KEY(SW):%d\n",
            real_gpio_s1, real_gpio_s2, real_gpio_key);

    ret = gpio_request(real_gpio_s1, "ec11_s1");
    if (ret) {
        pr_err("ec11: Failed to request S1 GPIO %d (err: %d)\n", real_gpio_s1, ret);
        goto destroy_dev;
    }
    gpio_direction_input(real_gpio_s1);

    ret = gpio_request(real_gpio_s2, "ec11_s2");
    if (ret) {
        pr_err("ec11: Failed to request S2 GPIO %d (err: %d)\n", real_gpio_s2, ret);
        goto free_s1;
    }
    gpio_direction_input(real_gpio_s2);

    ret = gpio_request(real_gpio_key, "ec11_key");
    if (ret) {
        pr_err("ec11: Failed to request KEY GPIO %d (err: %d)\n", real_gpio_key, ret);
        goto free_s2;
    }
    gpio_direction_input(real_gpio_key);

    /* Setup IRQ for S1 (Falling Edge) */
    irq_s1 = gpio_to_irq(real_gpio_s1);
    if (irq_s1 < 0) {
        pr_err("ec11: Failed to map S1 GPIO to IRQ (err: %d)\n", irq_s1);
        ret = irq_s1;
        goto free_key;
    }

    /* Initial encoder state */
    enc_state = (gpio_get_value(real_gpio_s1) ? 2 : 0) |
                (gpio_get_value(real_gpio_s2) ? 1 : 0);
    enc_acc = 0;
    step_fired = false;
    in_detent = true;

    if (steps_per_detent != 2 && steps_per_detent != 4) {
        pr_warn("ec11: steps_per_detent=%d invalid, using 4\n", steps_per_detent);
        steps_per_detent = 4;
    }

    ret = request_irq(irq_s1, ec11_rot_irq_handler,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "ec11_s1_irq", NULL);
    if (ret) {
        pr_err("ec11: Failed to request S1 IRQ %d (err: %d)\n", irq_s1, ret);
        goto free_key;
    }

    /* Setup IRQ for S2 (both edges) - needed for full quadrature decoding */
    irq_s2 = gpio_to_irq(real_gpio_s2);
    if (irq_s2 < 0) {
        pr_err("ec11: Failed to map S2 GPIO to IRQ (err: %d)\n", irq_s2);
        ret = irq_s2;
        goto free_irq_s1;
    }

    ret = request_irq(irq_s2, ec11_rot_irq_handler,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "ec11_s2_irq", NULL);
    if (ret) {
        pr_err("ec11: Failed to request S2 IRQ %d (err: %d)\n", irq_s2, ret);
        irq_s2 = -1;
        goto free_irq_s1;
    }

    /* Setup IRQ for KEY (Falling Edge) */
    irq_key = gpio_to_irq(real_gpio_key);
    if (irq_key < 0) {
        pr_err("ec11: Failed to map KEY GPIO to IRQ (err: %d)\n", irq_key);
        ret = irq_key;
        goto free_irq_s2;
    }

    ret = request_irq(irq_key, ec11_key_irq_handler,
                      IRQF_TRIGGER_FALLING, "ec11_key_irq", NULL);
    if (ret) {
        pr_err("ec11: Failed to request KEY IRQ %d (err: %d)\n", irq_key, ret);
        irq_key = -1;
        goto free_irq_s2;
    }

    pr_info("ec11: Driver loaded (/dev/%s, S1 IRQ:%d, S2 IRQ:%d, KEY IRQ:%d, %d steps/detent)\n",
            DEVICE_NAME, irq_s1, irq_s2, irq_key, steps_per_detent);
    return 0;

free_irq_s2:
    free_irq(irq_s2, NULL);
    irq_s2 = -1;
free_irq_s1:
    free_irq(irq_s1, NULL);
    irq_s1 = -1;
free_key:
    gpio_free(real_gpio_key);
free_s2:
    gpio_free(real_gpio_s2);
free_s1:
    gpio_free(real_gpio_s1);
destroy_dev:
    device_destroy(ec11_class, dev_number);
destroy_class:
    class_destroy(ec11_class);
del_cdev:
    cdev_del(&ec11_cdev);
unregister_chrdev:
    unregister_chrdev_region(dev_number, 1);
    return ret;
}

static void __exit ec11_exit(void)
{
    if (irq_key >= 0)
        free_irq(irq_key, NULL);
    if (irq_s2 >= 0)
        free_irq(irq_s2, NULL);
    if (irq_s1 >= 0)
        free_irq(irq_s1, NULL);

    if (real_gpio_key >= 0)
        gpio_free(real_gpio_key);
    if (real_gpio_s2 >= 0)
        gpio_free(real_gpio_s2);
    if (real_gpio_s1 >= 0)
        gpio_free(real_gpio_s1);

    if (ec11_class) {
        device_destroy(ec11_class, dev_number);
        class_destroy(ec11_class);
    }
    cdev_del(&ec11_cdev);
    unregister_chrdev_region(dev_number, 1);

    pr_info("ec11: Driver unloaded\n");
}

module_init(ec11_init);
module_exit(ec11_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AIDL Embedded Driver Team");
MODULE_DESCRIPTION("EC11 Rotary Encoder Driver (S1, S2, KEY) for Jetson Orin Nano");
MODULE_VERSION("1.2");