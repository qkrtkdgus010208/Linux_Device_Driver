// SPDX-License-Identifier: GPL-2.0
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include "common_ioctl.h"

_Static_assert(sizeof(struct ec11_event) == 24, "EC11 event ABI size");
_Static_assert(sizeof(struct motor_state) == 16, "Motor ioctl ABI size");

enum button_mode { BUTTON_BOTH, BUTTON_TOGGLE, BUTTON_DIRECTION };
static volatile sig_atomic_t stopping;

static void stop_signal(int signo)
{
	(void)signo;
	stopping = 1;
}

static uint64_t monotonic_ms(void)
{
	struct timespec now;

	if (clock_gettime(CLOCK_MONOTONIC, &now)) {
		perror("clock_gettime");
		stopping = 1;
		return 0;
	}
	return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}

static int update_outputs(int motor_fd, int led_fd, unsigned int level,
			 bool enabled, unsigned int direction)
{
	struct motor_state state = {
		.duty_permille = enabled ? level * 125 : 0,
		.direction = direction, .enabled = enabled,
	};
	__u32 visible = enabled ? level : 0;

	if (ioctl(motor_fd, MOTOR_IOC_SET_STATE, &state)) {
		perror("MOTOR_IOC_SET_STATE");
		return -1;
	}
	if (ioctl(led_fd, LED_BAR_IOC_SET_LEVEL, &visible)) {
		perror("LED_BAR_IOC_SET_LEVEL");
		return -1;
	}
	printf("selected=%u/8 output=%u.%u%% enabled=%u direction=%s led=%u\n",
		level, state.duty_permille / 10, state.duty_permille % 10,
		enabled, direction == MOTOR_FORWARD ? "forward" : "reverse", visible);
	return 0;
}

static void usage(const char *program)
{
	printf("Usage: %s [options]\n"
	       "  --encoder PATH          Default /dev/ec11\n"
	       "  --motor PATH            Default /dev/l298n_motor\n"
	       "  --led PATH              Default /dev/led_bar\n"
	       "  --reverse-encoder       Invert rotation increments\n"
	       "  --button-mode MODE      both (default), toggle, direction\n"
	       "  --long-press-ms MS       Direction threshold, default 800\n"
	       "  --help\n", program);
}

int main(int argc, char **argv)
{
	static const struct option options[] = {
		{ "encoder", required_argument, NULL, 'e' },
		{ "motor", required_argument, NULL, 'm' },
		{ "led", required_argument, NULL, 'l' },
		{ "reverse-encoder", no_argument, NULL, 'r' },
		{ "button-mode", required_argument, NULL, 'b' },
		{ "long-press-ms", required_argument, NULL, 't' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	const char *encoder_path = "/dev/ec11";
	const char *motor_path = "/dev/l298n_motor";
	const char *led_path = "/dev/led_bar";
	enum button_mode mode = BUTTON_BOTH;
	int encoder_fd = -1, motor_fd = -1, led_fd = -1;
	int option, result = EXIT_FAILURE;
	unsigned int level = 0, direction = MOTOR_FORWARD, long_press_ms = 800;
	bool enabled = false, reverse_encoder = false;
	uint32_t expected_sequence = 1;
	uint64_t heartbeat;
	struct pollfd pfd;
	struct sigaction action = { .sa_handler = stop_signal };

	while ((option = getopt_long(argc, argv, "", options, NULL)) != -1) {
		switch (option) {
		case 'e': encoder_path = optarg; break;
		case 'm': motor_path = optarg; break;
		case 'l': led_path = optarg; break;
		case 'r': reverse_encoder = true; break;
		case 'b':
			if (!strcmp(optarg, "both")) mode = BUTTON_BOTH;
			else if (!strcmp(optarg, "toggle")) mode = BUTTON_TOGGLE;
			else if (!strcmp(optarg, "direction")) mode = BUTTON_DIRECTION;
			else { fprintf(stderr, "Invalid button mode\n"); return EXIT_FAILURE; }
			break;
		case 't': {
			char *end;
			long ms;
			errno = 0;
			ms = strtol(optarg, &end, 10);
			if (errno || end == optarg || *end || ms < 100 || ms > 10000) {
				fprintf(stderr, "long-press-ms must be 100..10000\n");
				return EXIT_FAILURE;
			}
			long_press_ms = (unsigned int)ms;
			break;
		}
		case 'h': usage(argv[0]); return EXIT_SUCCESS;
		default: usage(argv[0]); return EXIT_FAILURE;
		}
	}
	if (optind != argc) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	/* In direction-only mode, rotation supplies the initial enable command. */
	if (mode == BUTTON_DIRECTION)
		enabled = true;
	setvbuf(stdout, NULL, _IOLBF, 0);
	sigemptyset(&action.sa_mask);
	if (sigaction(SIGINT, &action, NULL) || sigaction(SIGTERM, &action, NULL)) {
		perror("sigaction");
		return EXIT_FAILURE;
	}
	encoder_fd = open(encoder_path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (encoder_fd < 0) { perror(encoder_path); goto cleanup; }
	motor_fd = open(motor_path, O_RDWR | O_CLOEXEC);
	if (motor_fd < 0) { perror(motor_path); goto cleanup; }
	led_fd = open(led_path, O_RDWR | O_CLOEXEC);
	if (led_fd < 0) { perror(led_path); goto cleanup; }
	if (update_outputs(motor_fd, led_fd, level, enabled, direction))
		goto cleanup;
	pfd.fd = encoder_fd;
	pfd.events = POLLIN;
	heartbeat = monotonic_ms();
	while (!stopping) {
		int ready = poll(&pfd, 1, 50);

		if (ready < 0) {
			if (errno == EINTR) continue;
			perror("poll");
			goto cleanup;
		}
		if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
			fprintf(stderr, "Encoder device disconnected\n");
			goto cleanup;
		}
		if (pfd.revents & POLLIN) {
			/* Bound draining so IRQ floods cannot starve the heartbeat. */
			for (unsigned int i = 0; i < 32 && !stopping; i++) {
				struct ec11_event ev;
				ssize_t size = read(encoder_fd, &ev, sizeof(ev));

				if (size < 0) {
					if (errno == EAGAIN) break;
					if (errno == EINTR) continue;
					perror("read ec11");
					goto cleanup;
				}
				if (size != (ssize_t)sizeof(ev) || ev.reserved ||
				    ev.sequence != expected_sequence++) {
					fprintf(stderr, "Invalid or lost encoder event; stopping\n");
					goto cleanup;
				}
				if (ev.type == EC11_EVENT_ROTATE && (ev.value == 1 || ev.value == -1)) {
					int next = (int)level + (reverse_encoder ? -ev.value : ev.value);
					level = next < 0 ? 0 : next > 8 ? 8 : (unsigned int)next;
				} else if (ev.type == EC11_EVENT_BUTTON && ev.value >= 0) {
					if (mode == BUTTON_DIRECTION ||
					    (mode == BUTTON_BOTH && (unsigned int)ev.value >= long_press_ms))
						direction ^= 1;
					else
						enabled = !enabled;
				} else {
					fprintf(stderr, "Unsupported encoder event; stopping\n");
					goto cleanup;
				}
				if (update_outputs(motor_fd, led_fd, level, enabled, direction))
					goto cleanup;
			}
		}
		if (monotonic_ms() - heartbeat >= 100) {
			struct ec11_stats stats;

			if (ioctl(encoder_fd, EC11_IOC_GET_STATS, &stats)) {
				perror("EC11_IOC_GET_STATS");
				goto cleanup;
			}
			if (stats.dropped) {
				fprintf(stderr, "Encoder FIFO overflow (%u); stopping\n", stats.dropped);
				goto cleanup;
			}
			if (ioctl(motor_fd, MOTOR_IOC_KEEPALIVE)) {
				perror("MOTOR_IOC_KEEPALIVE");
				goto cleanup;
			}
			heartbeat = monotonic_ms();
		}
	}
	result = EXIT_SUCCESS;

cleanup:
	if (motor_fd >= 0) {
		struct motor_state off = { .direction = direction };
		if (ioctl(motor_fd, MOTOR_IOC_SET_STATE, &off)) {
			perror("Motor shutdown ioctl (close also stops the motor)");
			result = EXIT_FAILURE;
		}
		close(motor_fd);
	}
	if (led_fd >= 0) {
		__u32 zero = 0;
		if (ioctl(led_fd, LED_BAR_IOC_SET_LEVEL, &zero)) {
			perror("LED shutdown ioctl");
			result = EXIT_FAILURE;
		}
		close(led_fd);
	}
	if (encoder_fd >= 0) close(encoder_fd);
	return result;
}
