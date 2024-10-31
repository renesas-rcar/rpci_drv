obj-m += rcar-pci-host.o
obj-m += rcar-pci-epf.o

ccflags-y += -I$(KERNELSRC)/include

all:
	make -C $(KERNELSRC) M=$(shell pwd) modules

clean:
	make -C $(KERNELSRC) M=$(shell pwd) clean

install:
	$(CP) ./rcar_pci.h $(KERNELSRC)/include/

