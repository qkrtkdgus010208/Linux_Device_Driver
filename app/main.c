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
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <time.h>

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

    if (g_speed_step > 0)
        speed_percent = speed_table[g_speed_step];
    else
        speed_percent = 0;

    /* Update Motor Direction and Speed */
    if (g_fd_motor >= 0) {
        int dir = (speed_percent == 0) ? MOTOR_DIR_STOP : g_direction;
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_DIR, &dir);
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_SPEED, &speed_percent);
    }

    /* Update LED Bar Level */
    if (g_fd_ledbar >= 0) {
        ioctl(g_fd_ledbar, LEDBAR_IOCTL_SET_LEVEL, &g_speed_step);
    }
}

/* Render Dashboard in Console */
static void render_dashboard(void)
{
    int speed_percent = (g_speed_step > 0) ? speed_table[g_speed_step] : 0;
    char bar_str[64];
    int i;
    int idx = 2;

    /* Build visual bar meter string */
    bar_str[0] = '[';
    bar_str[1] = ' ';
    for (i = 1; i <= 8; i++) {
        if (i <= g_speed_step)
            bar_str[idx++] = '#';
        else
            bar_str[idx++] = '.';
        bar_str[idx++] = ' ';
    }
    bar_str[idx++] = ']';
    bar_str[idx] = '\0';

    /* Clear screen and reposition cursor */
    printf("\033[H\033[J");
    printf("==================================================================\n");
    printf("          JETSON ORIN NANO EMBEDDED MOTOR CONTROLLER              \n");
    printf("==================================================================\n");
    printf(" System State : [%s]\n",
           (g_speed_step == 0 ? "IDLE / STOPPED" : "RUNNING"));
    printf(" Direction    : [%s]\n",
           g_direction == MOTOR_DIR_FORWARD ? "FORWARD  (Clockwise)" :
           g_direction == MOTOR_DIR_BACKWARD ? "BACKWARD (Counter-Clockwise)" : "STOPPED");
    printf(" Speed Step   : [ %d / 8 ] (PWM Duty: %3d%%)\n", g_speed_step, speed_percent);
    printf(" LED Bar      : %s\n", bar_str);
    printf(" Encoder Pos  : %+d steps\n", g_position);
    printf("------------------------------------------------------------------\n");
    printf(" [Operation Instructions]\n");
    printf("  * Rotate Knob (S1/S2) : Speed Step (0 -> 8)\n");
    printf("  * Click Knob  (KEY)   : Toggle Direction (FWD <-> REV)\n");
    printf("  * Press Ctrl + C      : Safe Stop and Exit\n");
    printf("==================================================================\n");
    fflush(stdout);
}

/* Safe Hardware Cleanup */
static void cleanup_hardware(void)
{
    printf("\nShutting down devices safely...\n");

    if (g_fd_motor >= 0) {
        int zero = 0;
        int stop_dir = MOTOR_DIR_STOP;
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_SPEED, &zero);
        ioctl(g_fd_motor, MOTOR_IOCTL_SET_DIR, &stop_dir);
        ioctl(g_fd_motor, MOTOR_IOCTL_STOP, 0);
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
    struct pollfd pfd;
    int64_t last_toggle_ms = 0;

    (void)argc;
    (void)argv;

    /* Setup signal handler */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Open EC11 Device Node (Non-blocking mode) */
    g_fd_ec11 = open(DEV_EC11, O_RDONLY | O_NONBLOCK);
    if (g_fd_ec11 < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", DEV_EC11, strerror(errno));
        fprintf(stderr, "Hint: Did you insert the module? Try: sudo make load\n");
        return EXIT_FAILURE;
    }

    /* Open Motor Device Node */
    g_fd_motor = open(DEV_MOTOR, O_RDWR);
    if (g_fd_motor < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", DEV_MOTOR, strerror(errno));
        fprintf(stderr, "Hint: Did you insert the module? Try: sudo make load\n");
        close(g_fd_ec11);
        return EXIT_FAILURE;
    }

    /* Open LED Bar Device Node */
    g_fd_ledbar = open(DEV_LEDBAR, O_RDWR);
    if (g_fd_ledbar < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", DEV_LEDBAR, strerror(errno));
        fprintf(stderr, "Hint: Did you insert the module? Try: sudo make load\n");
        close(g_fd_ec11);
        close(g_fd_motor);
        return EXIT_FAILURE;
    }

    /* Drain any stale events in queue from before daemon started */
    {
        struct ec11_event drain_ev;
        while (read(g_fd_ec11, &drain_ev, sizeof(drain_ev)) > 0) {}
        ioctl(g_fd_ec11, EC11_IOCTL_RESET_POS);
        g_position = 0;
    }

    /* Initialize hardware state */
    update_hardware();
    render_dashboard();

    /* Prepare poll structure for EC11 */
    pfd.fd = g_fd_ec11;
    pfd.events = POLLIN;

    while (g_running) {
        int poll_ret = poll(&pfd, 1, 1000); /* 1000ms timeout for periodic refresh */

        if (poll_ret < 0) {
            if (errno == EINTR)
                break;
            perror("poll failed");
            break;
        }

        /* Handle EC11 Hardware Encoder Events */
        if (poll_ret > 0 && (pfd.revents & POLLIN)) {
            struct ec11_event ev;
            int changed = 0;

            while (read(g_fd_ec11, &ev, sizeof(ev)) == sizeof(ev)) {
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
                    changed = 1;
                }

                /* Handle Push Switch Event (with 350ms anti-flutter cooldown) */
                if (ev.sw_state == 2) {
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    int64_t now_ms = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

                    if (now_ms - last_toggle_ms >= 350) {
                        last_toggle_ms = now_ms;

                        /* Toggle Direction between FORWARD and BACKWARD */
                        if (g_direction == MOTOR_DIR_FORWARD) {
                            g_direction = MOTOR_DIR_BACKWARD;
                        } else {
                            g_direction = MOTOR_DIR_FORWARD;
                        }

                        /* Flash LED bar briefly for direction change feedback */
                        if (g_fd_ledbar >= 0) {
                            uint8_t flash_mask = 0xFF;
                            ioctl(g_fd_ledbar, LEDBAR_IOCTL_SET_RAW, &flash_mask);
                            usleep(30000); /* 30ms flash */
                        }

                        changed = 1;
                    }
                }
            }

            if (changed) {
                update_hardware();
                render_dashboard();
            }
        }
    }

    cleanup_hardware();
    return EXIT_SUCCESS;
}
