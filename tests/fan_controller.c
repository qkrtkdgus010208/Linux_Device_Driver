#define _POSIX_C_SOURCE 200809L
#include "fan.h"
#include "ledbar.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

static volatile sig_atomic_t running = 1;

struct encoder { int previous, sum, synced; };
struct key_state {
    int candidate, stable, armed;
    struct timespec changed_at;
};

static void stop_program(int signo)
{
    (void)signo;
    running = 0;
}

static int earlier(const struct timespec *a, const struct timespec *b)
{
    return a->tv_sec != b->tv_sec ? a->tv_sec < b->tv_sec :
                                   a->tv_nsec < b->tv_nsec;
}

/* Active-low KEY: accept a state after 30 ms; rearm on release. */
static int check_key(struct gpiod_line *key, struct key_state *state)
{
    struct timespec now;
    int value = gpiod_line_get_value(key);
    if (value < 0 || clock_gettime(CLOCK_MONOTONIC, &now) < 0)
        return -1;
    if (value != state->candidate) {
        state->candidate = value;
        state->changed_at = now;
        return 0;
    }
    long long elapsed =
        (long long)(now.tv_sec - state->changed_at.tv_sec) * 1000000000LL
        + now.tv_nsec - state->changed_at.tv_nsec;
    if (elapsed < 30000000LL)
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

static int decode(struct encoder *enc, int current, int enabled)
{
    static const int delta[16] = {
         0, -1,  1,  0,  1,  0,  0, -1,
        -1,  0,  0,  1,  0,  1, -1,  0
    };
    int old = enc->previous;
    enc->previous = current;
    if (!enabled) {
        enc->sum = 0;
        enc->synced = current == 0;
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
    int step = enc->sum == 4 ? 1 : (enc->sum == -4 ? -1 : 0);
    enc->sum = 0;
    return step;
}

static int apply_step(int speed, int step)
{
    int next = speed + step;
    return next < 1 ? 1 : (next > 8 ? 8 : next);
}

/* Both outputs must succeed; main's cleanup stops everything on failure. */
static int set_outputs(struct fan *fan, struct ledbar *leds, int speed)
{
    if (fan_set(fan, speed) < 0) {
        perror("set motor");
        return -1;
    }
    if (ledbar_set(leds, speed) < 0) {
        perror("set LEDs");
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    struct fan_config config;
    if (fan_parse_config(argc, argv, &config) < 0) {
        fprintf(stderr,
            "Usage: %s PERIOD_NS D1 D2 D3 D4 D5 D6 D7 D8\n"
            "Duties: integer percent, 0 < D1 < ... < D8 <= 100.\n"
            "No hardware was requested. No default motor calibration.\n",
            argv[0]);
        return 1;
    }

    const unsigned int offsets[2] = {105, 106};
    struct gpiod_chip *chip = NULL;
    struct gpiod_line *lines[2] = {NULL, NULL}, *key = NULL;
    struct pollfd fds[2] = {0};
    struct gpiod_line_event pending[2];
    struct ledbar leds = {0};
    struct fan fan = {.lock_fd = -1};
    struct encoder enc = {0};
    struct key_state key_state = {0};
    struct timespec cutoff = {0};
    int have[2] = {0, 0};
    int requested = 0, key_requested = 0, speed = 0, result = 1;
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

    /* Acquire the motor in a stopped state before initializing inputs. */
    if (fan_init(&fan, chip, &config) < 0) {
        perror("initialize motor (check PWM ownership and pinmux)");
        goto cleanup;
    }
    if (!running)
        goto cleanup;
    if (ledbar_init(&leds, chip) < 0) {
        perror("initialize LEDs");
        goto cleanup;
    }
    for (int i = 0; i < 2; i++) {
        lines[i] = gpiod_chip_get_line(chip, offsets[i]);
        if (!lines[i]) {
            perror("get encoder line");
            goto cleanup;
        }
        if (gpiod_line_request_both_edges_events(lines[i], "fan-encoder") < 0) {
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
        if (flags < 0 || fcntl(fds[i].fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            perror("set nonblocking");
            goto cleanup;
        }
    }
    int a = gpiod_line_get_value(lines[0]);
    int b = gpiod_line_get_value(lines[1]);
    if (a < 0 || b < 0) {
        perror("read encoder");
        goto cleanup;
    }
    enc.previous = (a << 1) | b;
    enc.synced = enc.previous == 0;
    key = gpiod_chip_get_line(chip, 43);
    if (!key) {
        perror("get KEY");
        goto cleanup;
    }
    if (gpiod_line_request_input(key, "fan-key") < 0) {
        perror("request KEY");
        goto cleanup;
    }
    key_requested = 1;
    int initial_key = gpiod_line_get_value(key);
    if (initial_key < 0) {
        perror("read KEY");
        goto cleanup;
    }
    key_state.candidate = key_state.stable = initial_key;
    if (clock_gettime(CLOCK_MONOTONIC, &key_state.changed_at) < 0) {
        perror("initialize clock");
        goto cleanup;
    }
    puts("POWER: OFF | KEY: ON/OFF | Rotate: 1..8 | Ctrl+C: stop/exit");
    while (running) {
        int pressed = check_key(key, &key_state);
        if (pressed < 0) {
            perror("read KEY");
            goto cleanup;
        }
        if (pressed) {
            if (clock_gettime(CLOCK_MONOTONIC, &cutoff) < 0) {
                perror("read clock");
                goto cleanup;
            }
            int next = speed == 0 ? 1 : 0;
            enc.sum = 0;
            enc.synced = enc.previous == 0;
            if (!running)
                break;
            if (set_outputs(&fan, &leds, next) < 0)
                goto cleanup;
            speed = next;
            puts(speed ? "POWER: ON | SPEED: 1 / 8" : "POWER: OFF");
        }
        for (int i = 0; i < 2; i++) {
            if (have[i])
                continue;
            if (gpiod_line_event_read_fd(fds[i].fd, &pending[i]) == 0)
                have[i] = 1;
            else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                perror("read encoder event");
                goto cleanup;
            }
        }
        if (!have[0] && !have[1]) {
            if (poll(fds, 2, 5) < 0) {
                if (errno == EINTR)
                    continue;
                perror("poll");
                goto cleanup;
            }
            for (int i = 0; i < 2; i++) {
                if (fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                    fprintf(stderr, "GPIO event descriptor error\n");
                    goto cleanup;
                }
            }
            continue;
        }
        int i = have[0] && have[1]
              ? (earlier(&pending[0].ts, &pending[1].ts) ? 0 : 1)
              : (have[0] ? 0 : 1);
        int bit = i == 0 ? 2 : 1;
        int current = enc.previous;
        if (pending[i].event_type == GPIOD_LINE_EVENT_RISING_EDGE)
            current |= bit;
        else
            current &= ~bit;
        int enabled = speed > 0 && !earlier(&pending[i].ts, &cutoff);
        have[i] = 0;
        int step = decode(&enc, current, enabled);
        if (!step)
            continue;
        int next = apply_step(speed, step);
        if (next == speed) {
            printf("LIMIT: %d / 8\n", speed);
            continue;
        }
        if (!running)
            break;
        if (set_outputs(&fan, &leds, next) < 0)
            goto cleanup;
        speed = next;
        printf("SPEED: %d / 8\n", speed);
    }
    result = 0;

cleanup:
    /* Motor first. Attempt LED and GPIO cleanup even if motor stop fails. */
    if (fan_close(&fan) < 0) {
        perror("STOP FAILED: remove motor power");
        result = 1;
    }
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
