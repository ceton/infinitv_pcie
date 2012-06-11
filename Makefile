KERNEL_VERSION := $(shell uname -r)
KERNEL_DIR	:= /lib/modules/$(KERNEL_VERSION)/build

PWD		:= $(shell pwd)

targets=

SOURCES := ctn91xx_driver.o ctn91xx_interrupt.o ctn91xx_ioctl.o \
	ctn91xx_util.o ctn91xx_event.o ctn91xx_mpeg.o \
	ctn91xx_net.o   ctn91xx_reset.o \
	   \
	ctn91xx_rpc.o

#assuming built-in if cross compiling
ifdef CROSS_COMPILE
targets=ctn91xx
obj-y := ctn91xx_builtin.o
EXTRA_CFLAGS := -DLINUX -DUSE_PCI=0 -DUSE_LEON=1 -DHAS_MPEG_DMA=1 -DUSE_INTERNAL=0
SOURCES += ctn91xx_leon.o 
else
targets=ctn91xx_module
obj-m := ctn91xx.o
EXTRA_CFLAGS := -DLINUX -DUSE_PCI=1 -DUSE_LEON=0 -DHAS_MPEG_DMA=1 -DUSE_INTERNAL=0
SOURCES += ctn91xx_pci.o ctn91xx_rtp.o
endif

ctn91xx_builtin-objs := $(SOURCES)

ctn91xx-objs := $(SOURCES)

all: $(targets)

ctn91xx:
	@echo "Building ctn91xx driver..."
	@(cd $(KERNEL_DIR) && make -j15 -C $(KERNEL_DIR) SUBDIRS=$(PWD))

ctn91xx_module:
	@(cd $(KERNEL_DIR) && make -j15 -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules)
	

install:
	@echo "Installing ctn91xx driver..."
	@(cd $(KERNEL_DIR) && make -j20 -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules_install)
	cp 98-ctn91xx.rules /etc/udev/rules.d/
	/sbin/depmod -a

clean:
	-rm -f *.o *.ko .*.cmd .*.flags *.mod.c Modules.symvers Module.symvers
	-rm -rf .tmp_versions *.ko.unsigned modules.order

uninstall:
	rm -f /lib/modules/$(KERNEL_VERSION)/extra/ctn91xx.ko
	@rm /etc/udev/rules.d/98-ctn91xx.rules
	/sbin/depmod -a
