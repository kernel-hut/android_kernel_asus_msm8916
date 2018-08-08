ifeq ($(CONFIG_ASUS_PROJECT),y)
obj-y +=  zc550kl_sensors/
else
obj-$(CONFIG_SENSORS)		+= sensors_class.o
obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
endif
