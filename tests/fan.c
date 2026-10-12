#define _XOPEN_SOURCE 700
#include "fan.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* Header pin 32: GP113_PWM7, controller 32e0000.pwm, channel 0. */
static int make_path(char *out, size_t size, const char *base,
                     const char *name)
{
    int n = snprintf(out, size, "%s/%s", base, name);
    if (n < 0 || (size_t)n >= size) {
        errno = ENAMETOOLONG;
        return -1;
    }
    return 0;
}

static int write_attr(const char *base, const char *name, const char *value)
{
    char path[4096];
    if (make_path(path, sizeof(path), base, name) < 0)
        return -1;
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0)
        return -1;

    size_t length = strlen(value);
    ssize_t n;
    do {
        n = write(fd, value, length);
    } while (n < 0 && errno == EINTR);
    int error = n < 0 ? errno : ((size_t)n != length ? EIO : 0);
    if (close(fd) < 0 && !error)
        error = errno;
    if (error) {
        fprintf(stderr, "PWM write %s=%s: %s\n", path, value, strerror(error));
        errno = error;
        return -1;
    }
    return 0;
}

static int write_number(const char *base, const char *name, uint64_t value)
{
    char text[32];
    snprintf(text, sizeof(text), "%" PRIu64, value);
    return write_attr(base, name, text);
}

static int read_attr(const char *base, const char *name,
                     char *value, size_t size)
{
    char path[4096];
    if (make_path(path, sizeof(path), base, name) < 0)
        return -1;
    FILE *file = fopen(path, "r");
    if (!file)
        return -1;
    if (!fgets(value, (int)size, file)) {
        int error = errno ? errno : EIO;
        fclose(file);
        errno = error;
        return -1;
    }
    if (fclose(file) != 0)
        return -1;
    value[strcspn(value, "\r\n")] = '\0';
    return 0;
}

static int find_pwm(struct fan *fan)
{
    DIR *dir = opendir("/sys/class/pwm");
    if (!dir)
        return -1;
    struct dirent *entry;
    int found = 0;
    while ((entry = readdir(dir))) {
        char path[4096], resolved[4096];
        if (strncmp(entry->d_name, "pwmchip", 7) != 0)
            continue;
        if (make_path(path, sizeof(path), "/sys/class/pwm", entry->d_name) < 0)
            continue;
        if (!realpath(path, resolved))
            continue;
        if (!strstr(resolved, "/32e0000.pwm/"))
            continue;
        if (++found > 1)
            break;
        snprintf(fan->chip_path, sizeof(fan->chip_path), "%s", path);
    }
    closedir(dir);
    if (found != 1) {
        errno = found ? EEXIST : ENODEV;
        return -1;
    }
    return make_path(fan->pwm_path, sizeof(fan->pwm_path),
                     fan->chip_path, "pwm0");
}

static int parse_number(const char *text, uint64_t *number)
{
    if (!*text) {
        errno = EINVAL;
        return -1;
    }
    for (const char *p = text; *p; p++) {
        if (*p < '0' || *p > '9') {
            errno = EINVAL;
            return -1;
        }
    }
    char *end;
    errno = 0;
    unsigned long long n = strtoull(text, &end, 10);
    if (errno || *end) {
        if (!errno)
            errno = EINVAL;
        return -1;
    }
    *number = n;
    return 0;
}

int fan_parse_config(int argc, char **argv, struct fan_config *config)
{
    if (argc != 10) {
        errno = EINVAL;
        return -1;
    }
    if (parse_number(argv[1], &config->period_ns) < 0)
        return -1;
    /* This application's bounds, not a claim about hardware support. */
    if (config->period_ns < 100 || config->period_ns > 1000000000ULL) {
        errno = ERANGE;
        return -1;
    }
    unsigned int previous = 0;
    for (int i = 0; i < 8; i++) {
        uint64_t duty;
        if (parse_number(argv[i + 2], &duty) < 0)
            return -1;
        if (duty <= previous || duty > 100) {
            errno = ERANGE;
            return -1;
        }
        config->duty_percent[i] = (unsigned int)duty;
        previous = (unsigned int)duty;
    }
    return 0;
}

int fan_init(struct fan *fan, struct gpiod_chip *chip,
             const struct fan_config *config)
{
    fan->config = *config;
    fan->lock_fd = open("/run/linux-fan-controller.lock",
                        O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fan->lock_fd < 0)
        return -1;
    if (flock(fan->lock_fd, LOCK_EX | LOCK_NB) < 0)
        return -1;
    if (find_pwm(fan) < 0)
        return -1;

    char value[64];
    if (read_attr(fan->chip_path, "npwm", value, sizeof(value)) < 0)
        return -1;
    if (strtoul(value, NULL, 10) < 1) {
        errno = ENODEV;
        return -1;
    }

    /* Do not take over a channel already generating PWM. */
    struct stat st;
    if (stat(fan->pwm_path, &st) == 0) {
        if (read_attr(fan->pwm_path, "enable", value, sizeof(value)) < 0)
            return -1;
        if (strcmp(value, "0") != 0) {
            errno = EBUSY;
            return -1;
        }
    } else if (errno != ENOENT) {
        return -1;
    }

    const unsigned int offsets[2] = {135, 134};
    int off[2] = {0, 0};
    gpiod_line_bulk_init(&fan->direction);
    for (int i = 0; i < 2; i++) {
        struct gpiod_line *line = gpiod_chip_get_line(chip, offsets[i]);
        if (!line)
            return -1;
        gpiod_line_bulk_add(&fan->direction, line);
    }
    if (gpiod_line_request_bulk_output(&fan->direction, "fan-direction", off) < 0)
        return -1;
    fan->direction_requested = 1;

    if (stat(fan->pwm_path, &st) < 0) {
        if (errno != ENOENT)
            return -1;
        if (write_attr(fan->chip_path, "export", "0") < 0)
            return -1;
        fan->exported_here = 1;
        int ready = 0;
        for (int i = 0; i < 100; i++) {
            if (stat(fan->pwm_path, &st) == 0) {
                ready = 1;
                break;
            }
            struct timespec delay = {.tv_sec = 0, .tv_nsec = 10000000};
            nanosleep(&delay, NULL);
        }
        if (!ready) {
            errno = ETIMEDOUT;
            return -1;
        }
    }
    /* Configure a zero initial period before duty/enable writes. */
    if (read_attr(fan->pwm_path, "period", value, sizeof(value)) < 0)
        return -1;
    uint64_t old_period;
    if (parse_number(value, &old_period) < 0)
        return -1;
    printf("PWM initial period=%" PRIu64 " ns\n", old_period);
    if (old_period == 0 &&
        write_number(fan->pwm_path, "period", config->period_ns) < 0)
        return -1;
    fan->pwm_ready = 1;
    if (write_attr(fan->pwm_path, "duty_cycle", "0") < 0 ||
        write_attr(fan->pwm_path, "enable", "0") < 0 ||
        write_number(fan->pwm_path, "period", config->period_ns) < 0)
        return -1;
    if (read_attr(fan->pwm_path, "polarity", value, sizeof(value)) < 0)
        return -1;
    if (strcmp(value, "normal") &&
        write_attr(fan->pwm_path, "polarity", "normal") < 0)
        return -1;

    printf("PWM: %s (32e0000.pwm, channel 0), period=%" PRIu64 " ns\n",
           fan->chip_path, config->period_ns);
    return 0;
}

int fan_stop(struct fan *fan)
{
    int error = 0;
    /* Attempt every stop operation even if one fails. */
    if (fan->pwm_ready) {
        if (write_attr(fan->pwm_path, "duty_cycle", "0") < 0)
            error = errno;
        if (write_attr(fan->pwm_path, "enable", "0") < 0 && !error)
            error = errno;
    }
    if (fan->direction_requested) {
        int off[2] = {0, 0};
        if (gpiod_line_set_value_bulk(&fan->direction, off) < 0 && !error)
            error = errno;
    }
    if (error) {
        errno = error;
        return -1;
    }
    return 0;
}

int fan_set(struct fan *fan, int speed)
{
    if (speed < 0 || speed > 8) {
        errno = EINVAL;
        return -1;
    }
    if (!speed)
        return fan_stop(fan);
    if (!fan->pwm_ready || !fan->direction_requested) {
        errno = ENODEV;
        return -1;
    }
    int forward[2] = {1, 0};
    uint64_t duty = fan->config.period_ns *
                    fan->config.duty_percent[speed - 1] / 100;
    if (gpiod_line_set_value_bulk(&fan->direction, forward) < 0 ||
        write_number(fan->pwm_path, "duty_cycle", duty) < 0 ||
        write_attr(fan->pwm_path, "enable", "1") < 0) {
        int error = errno;
        if (fan_stop(fan) < 0)
            perror("emergency fan stop");
        errno = error;
        return -1;
    }
    printf("MOTOR: level=%d duty=%u%%\n", speed,
           fan->config.duty_percent[speed - 1]);
    return 0;
}

int fan_close(struct fan *fan)
{
    int error = 0;
    if (fan_stop(fan) < 0)
        error = errno;
    if (fan->exported_here) {
        if (write_attr(fan->chip_path, "unexport", "0") < 0 && !error)
            error = errno;
        fan->exported_here = 0;
    }
    fan->pwm_ready = 0;
    if (fan->direction_requested) {
        gpiod_line_release_bulk(&fan->direction);
        fan->direction_requested = 0;
    }
    if (fan->lock_fd >= 0) {
        close(fan->lock_fd);
        fan->lock_fd = -1;
    }
    if (error) {
        errno = error;
        return -1;
    }
    return 0;
}
