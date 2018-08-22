obj-y += ASH_Hardware.o
obj-y += IRsensor_Hardware/
#obj-y += HALLsensor_Hardware.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_I2C_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
