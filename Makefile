DEBUG_CFLAGS += -g -DDEBUG

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build/

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules
debug:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) EXTRA_CFLAGS="$(DEBUG_CFLAGS)" modules
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install
