# Rules for making the exFAT driver.
#
# Copyright (c) 2009-2015 Anton Altaparmakov & Tuxera Inc. All rights reserved.
#
# You can set various options for the exFAT driver here:

CONFIG_EXFAT_NAME = texfat

CONFIG_EXFAT_FS = m

ifeq ($(wildcard include/linux/compile.h include/generated/compile.h),)
EXTRA_CFLAGS += -DTUXERA_NO_COMPILE_H -DUTS_MACHINE=\"$(UTS_MACHINE)\"
ifdef CONFIG_SMP
SMP=SMP
endif
ifdef CONFIG_PREEMPT
PREEMPT=preempt
endif
EXTRA_CFLAGS += -DUTS_VERSION=\"$(SMP)\ $(PREEMPT)\ UNKNOWN\"
endif

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
				journal.o \
				lcnalloc.o \
				namei.o \
				super.o \
				sysctl.o \
				time.o \
				tuts.o \
				unistr.o
