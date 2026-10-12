// SPDX-License-Identifier: GPL-2.0
/* Test-only ioctl transport; no kernel modules or hardware are accessed. */
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include "common_ioctl.h"

int ioctl(int fd, unsigned long request, ...)
{
	va_list args;
	void *argument = NULL;
	const char *path = getenv("MOCK_LOG");
	const char *failure = getenv("MOCK_FAIL");
	FILE *log;
	int ret = 0;
	(void)fd;

	if (!path) { errno = EINVAL; return -1; }
	if (request != MOTOR_IOC_KEEPALIVE) {
		va_start(args, request);
		argument = va_arg(args, void *);
		va_end(args);
	}
	log = fopen(path, "a");
	if (!log) return -1;
	if (request == MOTOR_IOC_SET_STATE) {
		struct motor_state *state = argument;
		fprintf(log, "M %u %u %u\n", state->duty_permille, state->enabled, state->direction);
		if (failure && !strcmp(failure, "motor") && state->duty_permille) ret = -1;
	} else if (request == LED_BAR_IOC_SET_LEVEL) {
		__u32 value = *(__u32 *)argument;
		fprintf(log, "L %u\n", value);
		if (failure && !strcmp(failure, "led") && value) ret = -1;
	} else if (request == MOTOR_IOC_KEEPALIVE) {
		fprintf(log, "PING\n");
		if (failure && !strcmp(failure, "heartbeat")) ret = -1;
	} else if (request == EC11_IOC_GET_STATS) {
		struct ec11_stats *stats = argument;
		stats->sequence = 0;
		stats->dropped = failure && !strcmp(failure, "overflow") ? 1 : 0;
	} else {
		ret = -1;
	}
	fclose(log);
	if (ret) errno = EIO;
	return ret;
}
