#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>

static volatile sig_atomic_t running = 1;

static void stop_program(int signo)
{
    (void)signo;
    running = 0;
}

/* Return true if a happened before b. */
static int earlier(const struct timespec *a, const struct timespec *b)
{
    if (a->tv_sec != b->tv_sec)
        return a->tv_sec < b->tv_sec;

    return a->tv_nsec < b->tv_nsec;
}

/* Decode transitions, counting a complete cycle at state 00. */
static void decode(int *previous, int current,
                   int *sum, int *synced, int *speed)
{
    /* Positive sequence: 00 -> 10 -> 11 -> 01 -> 00 */
    static const int delta[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    int old = *previous;
    *previous = current;

    if (current == old) {
        /* Same state: ignore without discarding this cycle. */
        puts("Duplicate event: ignored");
        return;
    }

    printf("AB: %d%d -> %d%d\n",
           (old >> 1) & 1, old & 1,
           (current >> 1) & 1, current & 1);

    if ((old ^ current) == 3) {
        *sum = 0;
        *synced = 0;
        puts("Invalid transition: waiting for state 00");
        return;
    }

    if (!*synced) {
        if (current == 0) {
            *synced = 1;
            *sum = 0;
            puts("Synchronized at 00");
        }
        return;
    }

    *sum += delta[(old << 2) | current];

    if (current == 0) {
        if (*sum == 4 || *sum == -4) {
            int step = (*sum > 0) ? 1 : -1;
                        int next = *speed + step;

            if (next < 0)
                next = 0;
            else if (next > 8)
                next = 8;

            if (next == *speed) {
                printf("LIMIT: speed remains %d / 8\n", *speed);
            } else {
                *speed = next;
                printf("SPEED: %d / 8\n", *speed);
            }
        }
        *sum = 0;
    }
}

int main(void)
{
    const unsigned int offsets[2] = {105, 106};
    struct gpiod_chip *chip = NULL;
    struct gpiod_line *lines[2] = {NULL, NULL};
    struct pollfd fds[2];
    struct gpiod_line_event pending[2];
    int have[2] = {0, 0};
    int requested = 0;
    int result = 1;
    int values[2];
    int previous, synced, sum = 0, speed = 0;

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
            perror("get GPIO line");
            goto cleanup;
        }

        if (gpiod_line_request_both_edges_events(
                lines[i], "encoder-test") < 0) {
            perror("request GPIO events");
            goto cleanup;
        }
        requested++;

        fds[i].fd = gpiod_line_event_get_fd(lines[i]);
        fds[i].events = POLLIN;
        fds[i].revents = 0;
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

    /* Keep the knob still while the program starts. */
    for (int i = 0; i < 2; i++) {
        values[i] = gpiod_line_get_value(lines[i]);
        if (values[i] < 0) {
            perror("read initial GPIO value");
            goto cleanup;
        }
    }

    previous = (values[0] << 1) | values[1];
    synced = (previous == 0);

    printf("Start AB=%d%d. Rotate slowly. Ctrl+C to exit.\n",
           values[0], values[1]);
    if (!synced)
        puts("Waiting for state 00 before counting.");

    while (running) {
        /* Keep one queued event per line, then pick the earlier one. */
        for (int i = 0; i < 2; i++) {
            if (have[i])
                continue;

            if (gpiod_line_event_read_fd(fds[i].fd, &pending[i]) == 0) {
                have[i] = 1;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK &&
                       errno != EINTR) {
                perror("read event");
                goto cleanup;
            }
        }

        if (!have[0] && !have[1]) {
            int ready = poll(fds, 2, 200);
            if (ready < 0) {
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

        int i;
        if (have[0] && have[1])
            i = earlier(&pending[0].ts, &pending[1].ts) ? 0 : 1;
        else
            i = have[0] ? 0 : 1;

        int bit = (i == 0) ? 2 : 1;
        int current = previous;

        if (pending[i].event_type == GPIOD_LINE_EVENT_RISING_EDGE)
            current |= bit;
        else
            current &= ~bit;

        have[i] = 0;
        decode(&previous, current, &sum, &synced, &speed);
    }

    result = 0;

cleanup:
    for (int i = 0; i < requested; i++)
        gpiod_line_release(lines[i]);
    if (chip)
        gpiod_chip_close(chip);

    return result;
}
