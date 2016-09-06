#
# Makefile for the input misc drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT_UINPUT)		+= uinput.o

#+++ ASUS_BSP alian_Shen "Add GSensor for ZC550KL"
ifeq ($(CONFIG_ASUS_ZC550KL_PROJECT),y)
#obj-$(CONFIG_KXTJ2_ZC550K) +=  kxtj9_zc550kl.o
obj-y +=  kxtj9_zc550kl.o
obj-y +=  bma2x2_zc550kl.o
endif
#--- ASUS_BSP alian_Shen "Add GSensor for ZC550KL"
