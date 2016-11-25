DULE_NAME := axi_drv 
KERNEL_DIR :=/home/johnblack/work/Kernel/linux-xlnx/
SRC_DIR =/home/johnblack/zynq
c_src = $(shell ls $(SRC_DIR)*.c)
o_src = $(subst $(SRC_DIR),,$(c_src))

$(MODULE_NAME)-objs := $(patsubst %.c,%.o,$(o_src))
obj-m := kern_test.o
CROSS_COMPILE=arm-xilinx-linux-gnueabi-
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld

all:
	@echo $(SRC_DIR)
	@echo $(c_src)
	@echo $(o_src)
	@make -C $(KERNEL_DIR) SUBDIRS=$(SRC_DIR) ARCH=arm modules
clean:
	rm -rf *.cmd *.o *.mod.c *.ko .tmp_versions modules.* Module.*
.PHONY: clean

