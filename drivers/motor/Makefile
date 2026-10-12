ifneq ($(KERNELRELEASE),)
obj-m += l298n_motor.o
ccflags-y += -I$(src)/../../include
else
KDIR ?= /lib/modules/$(shell uname -r)/build
.PHONY: all modules clean
all: modules
modules:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
endif
