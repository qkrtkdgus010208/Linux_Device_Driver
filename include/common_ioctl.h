#ifndef _COMMON_IOCTL_H_
#define _COMMON_IOCTL_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

/*
 * EC11 Rotary Encoder definitions
 * Hardware Pins: S1 (CLK), S2 (DT), KEY (SW)
 */
#define EC11_MAGIC 'E'

/* Event structure read from /dev/ec11 */
struct ec11_event {
    int32_t delta;       /* Direction change: +1 = CW, -1 = CCW, 0 = Key click only */
    int32_t sw_state;    /* Switch/Key state: 0 = Released, 1 = Pressed, 2 = Click */
    int32_t position;    /* Accumulated encoder step position */
};

#define EC11_IOCTL_GET_POS    _IOR(EC11_MAGIC, 1, int32_t)
#define EC11_IOCTL_RESET_POS  _IO(EC11_MAGIC, 2)
#define EC11_IOCTL_GET_SW     _IOR(EC11_MAGIC, 3, int32_t)
#define EC11_IOCTL_GET_KEY    EC11_IOCTL_GET_SW

/*
 * L298N Motor Driver definitions
 */
#define MOTOR_MAGIC 'M'

enum motor_dir {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_BACKWARD = 2,
    MOTOR_DIR_BRAKE = 3
};

struct motor_status {
    int32_t dir;            /* enum motor_dir */
    int32_t speed_percent;  /* 0 ~ 100 % */
    int32_t speed_step;     /* 0 ~ 8 step */
    int32_t is_enabled;     /* 0: disabled/off, 1: enabled/on */
};

#define MOTOR_IOCTL_SET_SPEED   _IOW(MOTOR_MAGIC, 1, int32_t)
#define MOTOR_IOCTL_SET_DIR     _IOW(MOTOR_MAGIC, 2, int32_t)
#define MOTOR_IOCTL_GET_STATUS  _IOR(MOTOR_MAGIC, 3, struct motor_status)
#define MOTOR_IOCTL_STOP        _IO(MOTOR_MAGIC, 4)
#define MOTOR_IOCTL_BRAKE       _IO(MOTOR_MAGIC, 5)
#define MOTOR_IOCTL_ENABLE      _IOW(MOTOR_MAGIC, 6, int32_t)

/*
 * WCNLB8-SR12 8-Bar LED Driver definitions
 */
#define LEDBAR_MAGIC 'L'

#define LEDBAR_MAX_LEVEL 8

#define LEDBAR_IOCTL_SET_LEVEL  _IOW(LEDBAR_MAGIC, 1, int32_t)
#define LEDBAR_IOCTL_SET_RAW    _IOW(LEDBAR_MAGIC, 2, uint8_t)
#define LEDBAR_IOCTL_GET_LEVEL  _IOR(LEDBAR_MAGIC, 3, int32_t)
#define LEDBAR_IOCTL_GET_RAW    _IOR(LEDBAR_MAGIC, 4, uint8_t)

#endif /* _COMMON_IOCTL_H_ */
