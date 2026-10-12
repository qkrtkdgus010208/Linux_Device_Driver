CC := $(CROSS_COMPILE)gcc
CPPFLAGS += -I../include
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror
LDLIBS ?=

.PHONY: all clean
all: motor_daemon
motor_daemon: main.c ../include/common_ioctl.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ main.c $(LDLIBS)
clean:
	$(RM) motor_daemon
