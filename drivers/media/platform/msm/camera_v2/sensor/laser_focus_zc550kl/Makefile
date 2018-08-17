ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci

obj-$(CONFIG_MSMB_CAMERA) += vl6180/
obj-$(CONFIG_MSMB_CAMERA) += show_lib/
obj-$(CONFIG_MSMB_CAMERA) += laura/

KBUILD_CPPFLAGS +=-DI2C_DEVICE=1
