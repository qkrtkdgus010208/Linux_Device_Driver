/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef JETSON_COMMON_IOCTL_H
#define JETSON_COMMON_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define EC11_EVENT_ROTATE 1U
#define EC11_EVENT_BUTTON 2U

/* read() returns one complete record; button value is hold time in ms. */
struct ec11_event {
	__u32 type;
	__s32 value;
	__aligned_u64 timestamp_ns;
	__u32 sequence;
	__u32 reserved;
};

struct ec11_stats {
	__u32 dropped;
	__u32 sequence;
};

#define MOTOR_FORWARD 0U
#define MOTOR_REVERSE 1U

/* duty_permille preserves the exact 12.5 percent spacing of eight steps. */
struct motor_state {
	__u32 duty_permille;
	__u32 direction;
	__u32 enabled;
	__u32 reserved;
};

#define EC11_IOC_GET_STATS _IOR('E', 0x01, struct ec11_stats)
#define MOTOR_IOC_SET_STATE _IOW('M', 0x01, struct motor_state)
#define MOTOR_IOC_GET_STATE _IOR('M', 0x02, struct motor_state)
#define MOTOR_IOC_KEEPALIVE _IO('M', 0x03)
#define LED_BAR_IOC_SET_LEVEL _IOW('L', 0x01, __u32)
#define LED_BAR_IOC_GET_LEVEL _IOR('L', 0x02, __u32)

#endif
