obj-m:= range_sensor.o
KDIR := $(KDIR)
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

deploy: default
	./scripts/deploy

