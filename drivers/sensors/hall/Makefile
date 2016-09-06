 #+++ SZ_BSP gauss_li "for porting"
ifeq ($(CONFIG_ASUS_ZC550KL_PROJECT),y)
	obj-y += hall_sensor_zc550kl.o
else
	obj-y += hall_sensor.o
endif
 #--- SZ_BSP gauss_li "for porting"

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DHALL_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
