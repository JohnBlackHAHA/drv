APP := axi_test
CROSS_COMPILE := arm-xilinx-linux-gnueabi-
CFLAGS := -ggdb3 
SYS_LIB	:= -lpthread -lrt
SRC := $(shell ls *.c)
OBJS := $(patsubst %.c,%.o,$(SRC))
$(APP) : $(OBJS) 
	$(CROSS_COMPILE)gcc -o  $@ $^  $(LIB_DIR) $(SYS_LIB) 
%.o : %.c 
	$(CROSS_COMPILE)gcc -c  $(CFLAGS)  $< -o $@
clean:
	rm -rf *.o 
	rm -rf $(APP)
