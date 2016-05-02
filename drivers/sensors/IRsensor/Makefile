obj-y += IRsensor.o
obj-y += sysfs/
obj-y += hardware/
obj-y += report/
obj-y += property/
obj-y += i2c/
obj-y += gpio/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
