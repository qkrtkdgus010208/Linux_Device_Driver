/*
 * User-space Control Daemon for Jetson Orin Nano
 * Coordinates EC11 Encoder, L298N Motor, and WCNLB8-SR12 LED Bar
 *
 * Devices:
 *   /dev/ec11        - Rotary encoder input
 *   /dev/l298n_motor - DC motor PWM & direction control
 *   /dev/led_bar     - 8-Bar LED array level meter
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <time.h>
#include <termios.h>

static struct termios orig_termios;
static int raw_mode_active = 0;

static void disable_raw_mode(void) {
    if (raw_mode_active) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        raw_mode_active = 0;
    }
}

static void enable_raw_mode(void) {
    if (isatty(STDIN_FILENO)) {
        if (tcgetattr(STDIN_FILENO, &orig_termios) == 0) {
            struct termios raw = orig_termios;
            raw.c_lflag &= ~(ECHO | ICANON);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
            raw_mode_active = 1;
        }
    }
}

#include "../include/common_ioctl.h"

#define DEV_EC11     "/dev/ec11"
#define DEV_MOTOR    "/dev/l298n_motor"
#define DEV_LEDBAR   "/dev/led_bar"

/* 0~8 step to motor duty cycle percentage table */
static const int speed_table[9] = {
    0,   /* Step 0: 0% (Stopped) */
    25,  /* Step 1: 25% (Minimum start torque) */
    35,  /* Step 2: 35% */
    45,  /* Step 3: 45% */
    55,  /* Step 4: 55% */
    65,  /* Step 5: 65% */
    75,  /* Step 6: 75% */
    85,  /* Step 7: 85% */
    100  /* Step 8: 100% (Full Speed) */
};

/* Global State */
static volatile sig_atomic_t g_running = 1;

static int g_fd_ec11   = -1;
static int g_fd_motor  = -1;
static int g_fd_ledbar = -1;

static int g_speed_step = 0;       /* 0 to 8 */
static int g_direction = MOTOR_DIR_FORWARD;
static int g_enabled = 1;          /* 1: running, 0: paused/disabled */
static int g_position = 0;

/* Signal Handler for graceful shutdown */
static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* Update Motor and LED Bar Hardware */
static void update_hardware(void)
{
    int speed_percent = 0;

    if (g_enabled && g_speed_step > 0) {
        speed_percent = speed_table[g_speed_step];
    } else {
        speed_percent = 0;
    }

    /* Update Motor Direction */
    if (g_fd_motor >= 0) {
        int dir = (speed_percent == 0) ? MOTOR_DIR_STOP : g_direction;
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_DIR, &dir);
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_SPEED, &speed_percent);
    }

    /* Update LED Bar */
    if (g_fd_ledbar >= 0) {
        int led_level = (g_enabled) ? g_speed_step : 0;
        ioctl(g_fd_ledbar, LEDBAR_IOCTL_SET_LEVEL, &led_level);
    }
}

static void render_dashboard(void);

static int64_t get_time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int64_t g_last_toggle_ms = 0;

/* Safely reverse motor direction: stop briefly to prevent back-EMF spike & drain induced noise */
static void change_direction_safe(void)
{
    /* 1. Toggle target direction */
    g_direction = (g_direction == MOTOR_DIR_FORWARD) ? MOTOR_DIR_BACKWARD : MOTOR_DIR_FORWARD;

    /* 2. Soft reversal: if motor is spinning, briefly pause to avoid inductive voltage surge */
    if (g_fd_motor >= 0 && g_speed_step > 0) {
        int zero = 0;
        int stop_dir = MOTOR_DIR_STOP;
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_SPEED, &zero);
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_DIR, &stop_dir);
    }

    /* 3. Flash LED bar briefly for visual feedback */
    if (g_fd_ledbar >= 0) {
        uint8_t flash_mask = 0xFF;
        ioctl(g_fd_ledbar, LEDBAR_IOCTL_SET_RAW, &flash_mask);
    }
    usleep(50000); /* 50ms pause: lets motor current drop to 0 and back-EMF dissipate */

    /* 4. Restore motor speed and new direction */
    update_hardware();
    render_dashboard();

    /* 5. Drain any false pulses induced on the EC11 lines by the motor reversal */
    if (g_fd_ec11 >= 0) {
        usleep(30000); /* 30ms buffer for contact/motor inrush ringing to settle */
        int fl = fcntl(g_fd_ec11, F_GETFL, 0);
        fcntl(g_fd_ec11, F_SETFL, fl | O_NONBLOCK);
        struct ec11_event dummy;
        while (read(g_fd_ec11, &dummy, sizeof(dummy)) > 0) {}
        fcntl(g_fd_ec11, F_SETFL, fl);
    }

    /* 6. Discard extra queued keys from terminal input buffer (e.g. held Spacebar) */
    tcflush(STDIN_FILENO, TCIFLUSH);
}

/* Debounced direction toggle with 350ms cooldown */
static void try_change_direction(void)
{
    int64_t now_ms = get_time_ms();
    if (now_ms - g_last_toggle_ms < 350)
        return;

    change_direction_safe();
    g_last_toggle_ms = get_time_ms();
}

/* Render Dashboard in Console */
static void render_dashboard(void)
{
    int speed_percent = (g_enabled && g_speed_step > 0) ? speed_table[g_speed_step] : 0;
    char bar_str[64];
    int i;

    /* Build visual bar meter string */
    bar_str[0] = '[';
    bar_str[1] = ' ';
    int idx = 2;
    for (i = 1; i <= 8; i++) {
        if (i <= g_speed_step && g_enabled) {
            bar_str[idx++] = '#';
        } else {
            bar_str[idx++] = '.';
        }
        bar_str[idx++] = ' ';
    }
    bar_str[idx++] = ']';
    bar_str[idx] = '\0';

    /* Clear screen or reposition cursor */
    printf("\033[H\033[J");
    printf("==================================================================\n");
    printf("          JETSON ORIN NANO EMBEDDED MOTOR CONTROLLER              \n");
    printf("==================================================================\n");
    printf(" System State : [%s]\n",
           !g_enabled ? "PAUSED / MUTED" :
           (g_speed_step == 0 ? "IDLE / STOPPED" : "RUNNING"));
    printf(" Direction    : [%s]\n",
           g_direction == MOTOR_DIR_FORWARD ? "FORWARD  (Clockwise)" :
           g_direction == MOTOR_DIR_BACKWARD ? "BACKWARD (Counter-Clockwise)" : "STOPPED");
    printf(" Speed Step   : [ %d / 8 ] (PWM Duty: %3d%%)\n", g_speed_step, speed_percent);
    printf(" LED Bar      : %s\n", bar_str);
    printf(" Encoder Pos  : %+d steps\n", g_position);
    printf("------------------------------------------------------------------\n");
    printf(" [Operation Instructions]\n");
    printf("  * Rotate Knob (S1/S2) or Press [W] / [S] : Speed Step (0 -> 8)\n");
    printf("  * Click Knob  (KEY)   or Press [SPACE]   : Toggle Direction (FWD <-> REV)\n");
    printf("  * Press [Q] or Ctrl + C                  : Safe Stop and Exit\n");
    printf("==================================================================\n");
    fflush(stdout);
}

/* Safe Hardware Cleanup */
static void cleanup_hardware(void)
{
    disable_raw_mode();
    printf("\nShutting down devices safely...\n");

    if (g_fd_motor >= 0) {
        int zero = 0;
        int stop_dir = MOTOR_DIR_STOP;
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_SPEED, &zero);
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_DIR, &stop_dir);
        ioctl(g_fd_motor, MOTOR_IOCTL_STOP, NULL);
        close(g_fd_motor);
        g_fd_motor = -1;
    }

    if (g_fd_ledbar >= 0) {
        int zero = 0;
        ioctl(g_fd_ledbar, LEDBAR_IOCTL_SET_LEVEL, &zero);
        close(g_fd_ledbar);
        g_fd_ledbar = -1;
    }

    if (g_fd_ec11 >= 0) {
        close(g_fd_ec11);
        g_fd_ec11 = -1;
    }

    printf("Cleanup complete. Daemon stopped.\n");
}

int main(int argc, char *argv[])
{
    struct sigaction sa;
    struct pollfd fds[2];

    (void)argc;
    (void)argv;

    /* Setup signal handler */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Open EC11 Device Node */
    g_fd_ec11 = open(DEV_EC11, O_RDONLY);
    if (g_fd_ec11 < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", DEV_EC11, strerror(errno));
        fprintf(stderr, "Hint: Did you insert the module? Try: sudo insmod drivers/encoder/ec11_driver.ko\n");
        return EXIT_FAILURE;
    }

    /* Open Motor Device Node */
    g_fd_motor = open(DEV_MOTOR, O_RDWR);
    if (g_fd_motor < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", DEV_MOTOR, strerror(errno));
        fprintf(stderr, "Hint: Did you insert the module? Try: sudo insmod drivers/motor/l298n_driver.ko\n");
        close(g_fd_ec11);
        return EXIT_FAILURE;
    }

    /* Open LED Bar Device Node */
    g_fd_ledbar = open(DEV_LEDBAR, O_RDWR);
    if (g_fd_ledbar < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", DEV_LEDBAR, strerror(errno));
        fprintf(stderr, "Hint: Did you insert the module? Try: sudo insmod drivers/led_bar/led_bar_driver.ko\n");
        close(g_fd_ec11);
        close(g_fd_motor);
        return EXIT_FAILURE;
    }

    /* Drain any stale events in queue from before daemon started */
    {
        int fl = fcntl(g_fd_ec11, F_GETFL, 0);
        fcntl(g_fd_ec11, F_SETFL, fl | O_NONBLOCK);
        struct ec11_event drain_ev;
        while (read(g_fd_ec11, &drain_ev, sizeof(drain_ev)) > 0) {}
        fcntl(g_fd_ec11, F_SETFL, fl);
        ioctl(g_fd_ec11, EC11_IOCTL_RESET_POS);
        g_position = 0;
    }

    /* Initialize hardware state */
    update_hardware();
    render_dashboard();

    /* Enable non-canonical raw terminal mode for instant keyboard control */
    enable_raw_mode();

    /* Prepare poll structures: fds[0] for EC11, fds[1] for Keyboard */
    fds[0].fd = g_fd_ec11;
    fds[0].events = POLLIN;
    fds[1].fd = STDIN_FILENO;
    fds[1].events = POLLIN;

    while (g_running) {
        int poll_ret = poll(fds, 2, 1000); /* 1000ms timeout for periodic refresh */

        if (poll_ret < 0) {
            if (errno == EINTR)
                break;
            perror("poll failed");
            break;
        }

        /* 1. Handle Keyboard Inputs */
        if (poll_ret > 0 && (fds[1].revents & POLLIN)) {
            char ch = 0;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == 'w' || ch == 'W' || ch == '+' || ch == 'k' || ch == 'K') {
                    if (g_speed_step < 8)
                        g_speed_step++;
                    update_hardware();
                    render_dashboard();
                } else if (ch == 's' || ch == 'S' || ch == '-' || ch == 'j' || ch == 'J') {
                    if (g_speed_step > 0)
                        g_speed_step--;
                    update_hardware();
                    render_dashboard();
                } else if (ch == ' ' || ch == 'd' || ch == 'D') {
                    try_change_direction();
                } else if (ch == 'q' || ch == 'Q' || ch == 27) {
                    g_running = 0;
                    break;
                }
            }
        }

        /* 2. Handle EC11 Hardware Encoder Events */
        if (poll_ret > 0 && (fds[0].revents & POLLIN)) {
            struct ec11_event ev;
            ssize_t bytes_read = read(g_fd_ec11, &ev, sizeof(ev));

            if (bytes_read == sizeof(ev)) {
                g_position = ev.position;

                /* Handle Rotation Event */
                if (ev.delta != 0) {
                    if (ev.delta > 0) {
                        /* Clockwise: Increase Speed */
                        if (g_speed_step < 8)
                            g_speed_step++;
                    } else {
                        /* Counter-Clockwise: Decrease Speed */
                        if (g_speed_step > 0)
                            g_speed_step--;
                    }
                    update_hardware();
                    render_dashboard();
                }

                /* Handle Push Switch Event */
                if (ev.sw_state == 2) {
                    try_change_direction();
                }
            }
        }
    }

    cleanup_hardware();
    return EXIT_SUCCESS;
}
