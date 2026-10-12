#include <errno.h>
#include "ledbar.h"

/* 0: HIGH turns LED on. 1: LOW turns LED on. */
#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0
#endif

int ledbar_init(struct ledbar *bar, struct gpiod_chip *chip)
{
    const unsigned int offsets[8] = {
        144, 112, 122, 126, 125, 123, 136, 137
    };
    int initial[8] = {0};
    int flags = LED_ACTIVE_LOW
              ? GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW : 0;

    bar->requested = 0;
    gpiod_line_bulk_init(&bar->lines);

    for (int i = 0; i < 8; i++) {
        struct gpiod_line *line =
            gpiod_chip_get_line(chip, offsets[i]);

        if (!line)
            return -1;

        gpiod_line_bulk_add(&bar->lines, line);
    }

    /* Request all eight outputs, initially OFF. */
    if (gpiod_line_request_bulk_output_flags(
            &bar->lines, "fan-ledbar", flags, initial) < 0)
        return -1;

    bar->requested = 1;
    return 0;
}

int ledbar_set(struct ledbar *bar, int speed)
{
    int values[8];

    if (!bar->requested || speed < 0 || speed > 8) {
        errno = EINVAL;
        return -1;
    }

    for (int i = 0; i < 8; i++)
        values[i] = (i < speed);

    return gpiod_line_set_value_bulk(&bar->lines, values);
}

int ledbar_close(struct ledbar *bar)
{
    int result;

    if (!bar->requested)
        return 0;

    result = ledbar_set(bar, 0);
    gpiod_line_release_bulk(&bar->lines);
    bar->requested = 0;

    return result;
}
