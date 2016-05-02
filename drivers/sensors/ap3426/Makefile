obj-y += ap3426.o
obj-y += sensors_class.o


DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
