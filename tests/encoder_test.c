#define _POSIX_C_SOURCE 200809L

#include "ledbar.h"

#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

#define KEY_OFFSET 43
#define KEY_DEBOUNCE_NS 30000000LL

static volatile sig_atomic_t running = 1;

struct encoder {
    int previous;
    int sum;
    int synced;
};

struct key_state {
    int candidate;
    int stable;
    int armed;
    struct timespec changed_at;
};

static void stop_program(int signo)
{
    (void)signo;
    running = 0;
}

static int earlier(const struct timespec *a,
                   const struct timespec *b)
{
    if (a->tv_sec != b->tv_sec)
        return a->tv_sec < b->tv_sec;

    return a->tv_nsec < b->tv_nsec;
}

/* KEY is active-low. Require a stable release before each press. */
static int check_key(struct gpiod_line *key, struct key_state *state)
{
    struct timespec now;
    int value = gpiod_line_get_value(key);

    if (value < 0)
        return -1;

    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0)
        return -1;

    if (value != state->candidate) {
        state->candidate = value;
        state->changed_at = now;
        return 0;
    }

    long long elapsed =
        (long long)(now.tv_sec - state->changed_at.tv_sec)
        * 1000000000LL
        + now.tv_nsec - state->changed_at.tv_nsec;

    if (elapsed < KEY_DEBOUNCE_NS)
        return 0;

    if (value == 1) {
        state->stable = 1;
        state->armed = 1;
        return 0;
    }

    if (state->stable != 0) {
        state->stable = 0;

        if (state->armed) {
            state->armed = 0;
            return 1;
        }
    }

    return 0;
}

/* Return +1, -1, or 0 for a complete encoder cycle. */
static int decode(struct encoder *enc, int current, int enabled)
{
    /* Positive: 00 -> 10 -> 11 -> 01 -> 00 */
    static const int delta[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    int old = enc->previous;
    enc->previous = current;

    if (!enabled) {
        enc->sum = 0;
        enc->synced = (current == 0);
        return 0;
    }

    if (current == old)
        return 0;

    if ((old ^ current) == 3) {
        enc->sum = 0;
        enc->synced = 0;
        puts("Invalid AB transition; resynchronizing.");
        return 0;
    }

    if (!enc->synced) {
        if (current == 0) {
            enc->sum = 0;
            enc->synced = 1;
        }
        return 0;
    }

    enc->sum += delta[(old << 2) | current];

    if (current != 0)
        return 0;

    int step = 0;

    if (enc->sum == 4)
        step = 1;
    else if (enc->sum == -4)
        step = -1;

    enc->sum = 0;
    return step;
}

static int apply_step(int speed, int step)
{
    int next = speed + step;

    if (next < 1)
        next = 1;
    else if (next > 8)
        next = 8;

    return next;
}

int main(void)
{
    const unsigned int offsets[2] = {105, 106};

    struct gpiod_chip *chip = NULL;
    struct gpiod_line *lines[2] = {NULL, NULL};
    struct gpiod_line *key = NULL;
    struct pollfd fds[2] = {0};
    struct gpiod_line_event pending[2];
    struct ledbar leds = {0};
    struct encoder enc = {0};
    struct key_state key_state = {0};

    int have[2] = {0, 0};
    int requested = 0;
    int key_requested = 0;
    int speed = 0;
    int result = 1;

    /*
     * Ignore rotation events queued before the latest power change.
     * GPIO event timestamps and this cutoff use CLOCK_MONOTONIC.
     */
    struct timespec rotation_cutoff = {0};

    struct sigaction action = {0};
    action.sa_handler = stop_program;
    sigemptyset(&action.sa_mask);

    if (sigaction(SIGINT, &action, NULL) < 0 ||
        sigaction(SIGTERM, &action, NULL) < 0) {
        perror("sigaction");
        return 1;
    }

    setvbuf(stdout, NULL, _IOLBF, 0);

    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        perror("open gpiochip0");
        goto cleanup;
    }

    for (int i = 0; i < 2; i++) {
        lines[i] = gpiod_chip_get_line(chip, offsets[i]);
        if (!lines[i]) {
            perror("get encoder line");
            goto cleanup;
        }

        if (gpiod_line_request_both_edges_events(
                lines[i], "encoder-test") < 0) {
            perror("request encoder events");
            goto cleanup;
        }
        requested++;

        fds[i].fd = gpiod_line_event_get_fd(lines[i]);
        fds[i].events = POLLIN;

        if (fds[i].fd < 0) {
            perror("get event fd");
            goto cleanup;
        }

        int flags = fcntl(fds[i].fd, F_GETFL);
        if (flags < 0 ||
            fcntl(fds[i].fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            perror("set nonblocking");
            goto cleanup;
        }
    }

    int a = gpiod_line_get_value(lines[0]);
    int b = gpiod_line_get_value(lines[1]);

    if (a < 0 || b < 0) {
        perror("read initial encoder state");
        goto cleanup;
    }

    enc.previous = (a << 1) | b;
    enc.synced = (enc.previous == 0);

    key = gpiod_chip_get_line(chip, KEY_OFFSET);
    if (!key) {
        perror("get KEY line");
        goto cleanup;
    }

    if (gpiod_line_request_input(key, "fan-key") < 0) {
        perror("request KEY input");
        goto cleanup;
    }
    key_requested = 1;

    int initial_key = gpiod_line_get_value(key);
    if (initial_key < 0) {
        perror("read initial KEY");
        goto cleanup;
    }

    key_state.candidate = initial_key;
    key_state.stable = initial_key;
    key_state.armed = 0;

    if (clock_gettime(CLOCK_MONOTONIC,
                      &key_state.changed_at) < 0) {
        perror("initialize KEY timer");
        goto cleanup;
    }

    if (ledbar_init(&leds, chip) < 0) {
        perror("initialize LEDs");
        goto cleanup;
    }

    puts("POWER: OFF");
    puts("KEY: ON/OFF | Rotate: speed 1..8 | Ctrl+C: exit");

    while (running) {
        int pressed = check_key(key, &key_state);

        if (pressed < 0) {
            perror("read KEY");
            goto cleanup;
        }

        if (pressed) {
            if (clock_gettime(CLOCK_MONOTONIC,
                              &rotation_cutoff) < 0) {
                perror("read power-change timestamp");
                goto cleanup;
            }

            speed = (speed == 0) ? 1 : 0;
            enc.sum = 0;
            enc.synced = (enc.previous == 0);

            if (ledbar_set(&leds, speed) < 0) {
                perror("update LEDs after KEY");
                goto cleanup;
            }

            if (speed == 0)
                puts("POWER: OFF");
            else
                puts("POWER: ON | SPEED: 1 / 8");
        }

        /* Merge the two event streams by timestamp. */
        for (int i = 0; i < 2; i++) {
            if (have[i])
                continue;

            if (gpiod_line_event_read_fd(
                    fds[i].fd, &pending[i]) == 0) {
                have[i] = 1;
            } else if (errno != EAGAIN &&
                       errno != EWOULDBLOCK &&
                       errno != EINTR) {
                perror("read encoder event");
                goto cleanup;
            }
        }

        if (!have[0] && !have[1]) {
            int ready = poll(fds, 2, 5);

            if (ready < 0) {
                if (errno == EINTR)
                    continue;

                perror("poll");
                goto cleanup;
            }

            for (int i = 0; i < 2; i++) {
                if (fds[i].revents &
                    (POLLERR | POLLHUP | POLLNVAL)) {
                    fprintf(stderr, "GPIO descriptor error\n");
                    goto cleanup;
                }
            }
            continue;
        }

        int i;
        if (have[0] && have[1])
            i = earlier(&pending[0].ts, &pending[1].ts) ? 0 : 1;
        else
            i = have[0] ? 0 : 1;

        int bit = (i == 0) ? 2 : 1;
        int current = enc.previous;

        if (pending[i].event_type == GPIOD_LINE_EVENT_RISING_EDGE)
            current |= bit;
        else
            current &= ~bit;

        int enabled =
            speed > 0 &&
            !earlier(&pending[i].ts, &rotation_cutoff);

        have[i] = 0;

        int step = decode(&enc, current, enabled);
        if (step == 0)
            continue;

        int next = apply_step(speed, step);

        if (next == speed) {
            printf("LIMIT: %d / 8\n", speed);
            continue;
        }

        if (ledbar_set(&leds, next) < 0) {
            perror("update LEDs after rotation");
            goto cleanup;
        }

        speed = next;
        printf("SPEED: %d / 8\n", speed);
    }

    result = 0;

cleanup:
    if (ledbar_close(&leds) < 0) {
        perror("turn LEDs off");
        result = 1;
    }

    if (key_requested)
        gpiod_line_release(key);

    for (int i = 0; i < requested; i++)
        gpiod_line_release(lines[i]);

    if (chip)
        gpiod_chip_close(chip);

    return result;
}
