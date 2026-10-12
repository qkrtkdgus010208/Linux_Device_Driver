#ifndef FAN_H
#define FAN_H

#include <gpiod.h>
#include <stdint.h>

struct fan_config {
    uint64_t period_ns;
    unsigned int duty_percent[8];
};

struct fan {
    struct gpiod_line_bulk direction;
    struct fan_config config;
    char chip_path[4096];
    char pwm_path[4096];
    int lock_fd;
    int direction_requested;
    int exported_here;
    int pwm_ready;
};

int fan_parse_config(int argc, char **argv, struct fan_config *config);
int fan_init(struct fan *fan, struct gpiod_chip *chip,
             const struct fan_config *config);
int fan_set(struct fan *fan, int speed);
int fan_stop(struct fan *fan);
int fan_close(struct fan *fan);

#endif
