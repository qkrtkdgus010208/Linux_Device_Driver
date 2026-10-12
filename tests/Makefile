CC := $(CROSS_COMPILE)gcc
CPPFLAGS += -I../include
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Werror

.PHONY: test clean
test: mock_ioctl.so
	python3 test_app.py
mock_ioctl.so: mock_ioctl.c ../include/common_ioctl.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -fPIC -shared -o $@ mock_ioctl.c
clean:
	$(RM) mock_ioctl.so
