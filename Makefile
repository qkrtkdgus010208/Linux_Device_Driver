KDIR ?= /lib/modules/$(shell uname -r)/build
export KDIR
MODULE_DIRS := drivers/encoder drivers/motor drivers/led_bar

.PHONY: all modules app clean test board-info $(MODULE_DIRS)
all: modules app

modules: $(MODULE_DIRS)

$(MODULE_DIRS):
	$(MAKE) -C $@ modules

app:
	$(MAKE) -C app

test: app
	$(MAKE) -C tests test

board-info:
	bash scripts/board-info.sh

clean:
	$(MAKE) -C drivers/encoder clean
	$(MAKE) -C drivers/motor clean
	$(MAKE) -C drivers/led_bar clean
	$(MAKE) -C app clean
	$(MAKE) -C tests clean
