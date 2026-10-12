#ifndef LEDBAR_H
#define LEDBAR_H

#include <gpiod.h>

struct ledbar {
    struct gpiod_line_bulk lines;
    int requested;
};

int ledbar_init(struct ledbar *bar, struct gpiod_chip *chip);
int ledbar_set(struct ledbar *bar, int speed);
int ledbar_close(struct ledbar *bar);

#endif
