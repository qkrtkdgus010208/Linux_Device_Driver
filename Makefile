# Top-level Makefile for Jetson Orin Nano Device Driver Project

SUBDIRS = drivers app

.PHONY: all clean drivers app load unload status help

all: drivers app
	@echo "=================================================="
	@echo " Build completed successfully!"
	@echo " Output Drivers:"
	@echo "   - drivers/encoder/ec11_driver.ko"
	@echo "   - drivers/motor/l298n_driver.ko"
	@echo "   - drivers/led_bar/led_bar_driver.ko"
	@echo " Output Daemon:"
	@echo "   - app/motor_daemon"
	@echo "=================================================="

drivers:
	$(MAKE) -C drivers

app:
	$(MAKE) -C app

clean:
	$(MAKE) -C drivers clean
	$(MAKE) -C app clean

load:
	@echo "Loading kernel modules..."
	sudo insmod drivers/encoder/ec11_driver.ko || true
	sudo insmod drivers/motor/l298n_driver.ko || true
	sudo insmod drivers/led_bar/led_bar_driver.ko || true
	@sleep 0.5
	@echo "Configuring device permissions..."
	sudo chmod 666 /dev/ec11 /dev/l298n_motor /dev/led_bar 2>/dev/null || true
	@echo "Modules loaded."

unload:
	@echo "Unloading kernel modules..."
	sudo rmmod led_bar_driver || true
	sudo rmmod l298n_driver || true
	sudo rmmod ec11_driver || true
	@echo "Modules unloaded."

status:
	@echo "=== Loaded Kernel Modules ==="
	@lsmod | grep -E "ec11_driver|l298n_driver|led_bar_driver" || echo "No project modules currently loaded."
	@echo "\n=== Device Nodes ==="
	@ls -la /dev/ec11 /dev/l298n_motor /dev/led_bar 2>/dev/null || echo "No project device nodes found in /dev."

help:
	@echo "Available make targets:"
	@echo "  make         : Build all kernel drivers and user-space daemon"
	@echo "  make clean   : Clean build artifacts in all directories"
	@echo "  make load    : Insert all kernel modules (requires sudo)"
	@echo "  make unload  : Remove all kernel modules (requires sudo)"
	@echo "  make status  : Check kernel module status and /dev nodes"
