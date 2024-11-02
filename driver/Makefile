KDIR := /home/kdp-pi/src_kernel/linux-v6.6.y-Xenomai4-EVL
CURRENT_PATH := /home/kdp-pi/Space_AI/driver
obj-m := timer_space.o

#添加编译时需要Include的路径
ccflags-y := -I$(KDIR)/include \
 -I$(KDIR)/arch/arm64/include \
 -I$(KDIR)/arch/arm64/include/generated

build:kernel_modules

kernel_modules:
	$(MAKE) -C $(KDIR) M=$(CURRENT_PATH) modules
	mv $(CURRENT_PATH)/*.o $(CURRENT_PATH)/build
	mv $(CURRENT_PATH)/*.mod $(CURRENT_PATH)/build
	mv $(CURRENT_PATH)/*.mod.c $(CURRENT_PATH)/build
	mv $(CURRENT_PATH)/*.symvers $(CURRENT_PATH)/build
	mv $(CURRENT_PATH)/*.order $(CURRENvim_PATH)/build
	mv $(CURRENT_PATH)/.*.cmd $(CURRENT_PATH)/build
	
clean:
	$(MAKE) -C $(KDIR) M=$(CURRENT_PATH)/build clean 
	
debug:
	echo $(ccflags-y)