obj-y += gpio.o

INTEL = n
ifeq ($(INTEL),y)
	LOCAL_CFALG = -DIR_GPIO_INTEL
else
	LOCAL_CFALG = -DIR_GPIO_QCOM
endif

ccflags-y += $(LOCAL_CFALG)


