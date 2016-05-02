# Rules for making the exFAT driver.
#
# Copyright (c) 2009-2014 Anton Altaparmakov & Tuxera Inc. All rights reserved.
#
# You can set various options for the exFAT driver here:

CONFIG_EXFAT_NAME = texfat

CONFIG_EXFAT_FS = m

obj-$(CONFIG_EXFAT_FS) += $(CONFIG_EXFAT_NAME).o

$(CONFIG_EXFAT_NAME)-objs :=	aops.o \
				bitmap.o \
				chain.o \
				compat.o \
				debug.o \
				dir.o \
				fat.o \
				file.o \
				inode.o \
				lcnalloc.o \
				namei.o \
				super.o \
				sysctl.o \
				time.o \
				unistr.o
