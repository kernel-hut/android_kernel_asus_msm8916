#
# Makefile for the input misc drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT_UINPUT)		+= uinput.o
#obj-$(CONFIG_INPUT_WISTRON_BTNS)	+= wistron_btns.o
#obj-$(CONFIG_INPUT_WM831X_ON)		+= wm831x-on.o
#obj-$(CONFIG_INPUT_XEN_KBDDEV_FRONTEND)	+= xen-kbdfront.o
#obj-$(CONFIG_INPUT_YEALINK)		+= yealink.o
#obj-$(CONFIG_BMP18X)			+= bmp18x-core.o
#obj-$(CONFIG_BMP18X_I2C)		+= bmp18x-i2c.o
#obj-$(CONFIG_SENSORS_MMC3416X)		+= mmc3416x.o
#obj-$(CONFIG_SENSORS_MMC3X30)		+= mmc3x30.o
#obj-$(CONFIG_SENSORS_MMA8X5X)	  	+= mma8x5x.o
#obj-$(CONFIG_SENSORS_STK3X1X)		+= stk3x1x.o
#obj-$(CONFIG_SENSORS_CAPELLA_CM36283)	+= cm36283.o
#obj-$(CONFIG_SENSORS_MPU6050)		+= mpu6050.o
#obj-$(CONFIG_SENSORS_AKM8963)		+= akm8963.o
#obj-$(CONFIG_SENSORS_AKM09911)		+= akm09911.o
#obj-$(CONFIG_SENSORS_LIS3DH)		+= lis3dh_acc.o
#obj-$(CONFIG_SENSORS_AP3426)		+= ap3426.o
#obj-$(CONFIG_SENSORS_BMA2X2)	+= bstclass.o

#+++ ASUS_BSP alian_Shen "Add GSensor for ZC550KL"
ifeq ($(CONFIG_ASUS_ZC550KL_PROJECT),y)
#obj-$(CONFIG_KXTJ2_ZC550K) +=  kxtj9_zc550kl.o
obj-y +=  kxtj9_zc550kl.o
obj-y +=  bma2x2_zc550kl.o
endif
#--- ASUS_BSP alian_Shen "Add GSensor for ZC550KL"

obj-$(CONFIG_SENSORS_BMI160) += bmi160_driver.o bmi160.o
ifeq ($(CONFIG_BMI160_MAG_INTERFACE_SUPPORT),y)
	EXTRA_CFLAGS += -DBMI160_MAG_INTERFACE_SUPPORT
endif
ifeq ($(CONFIG_SENSORS_BMI160_ENABLE_INT1),y)
	EXTRA_CFLAGS += -DBMI160_ENABLE_INT1
endif

ifeq ($(CONFIG_SENSORS_BMI160_ENABLE_INT2),y)
	EXTRA_CFLAGS += -DBMI160_ENABLE_INT2
endif

obj-$(CONFIG_SENSORS_BMI160_I2C) += bmi160_i2c.o
ifeq ($(CONFIG_SENSORS_BMI160_I2C),y)
	EXTRA_CFLAGS += -DBMI_USE_BASIC_I2C_FUNC
endif
