#obj-$(CONFIG_ASHTEST) 	+=ASH_testDriver/
#obj-$(CONFIG_ASH)	+=ASH_ATTR/ 

obj-y 	+=ASH_testDriver/
obj-y	+=ASH_ATTR/ 
obj-y	+=ASH_Report/
obj-y	+=ASH_sysfs/
obj-y	+=ASH_GPIO/

ifeq ($(ASHDEBUG),y)
	DEBFLAGS = -O -g -DASH_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

