#obj-$(CONFIG_SENSORS)		+= sensors_class.o
#obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
obj-y	+= ASH/

#+++ SZ_BSP gauss_li "hall_sensor"
ifeq ($(CONFIG_ASUS_ZC550KL_PROJECT),y)
obj-y	+=hall/
endif
#--- SZ_BSP gauss_li "hall_sensor"

#+++ ASUS_BSP Alian_Shen "Add for P/LSensor"
ifeq ($(CONFIG_ASUS_ZC550KL_PROJECT),y)
obj-$(CONFIG_AP3426_ZC550KL)	+=ap3426/
obj-y	+=ap3426/
endif
#--- ASUS_BSP Alian_Shen "Add for P/LSensor"
