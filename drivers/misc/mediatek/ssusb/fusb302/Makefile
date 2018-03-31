
ifeq ($(CONFIG_USB_DEBUG),y)
	EXTRA_CFLAGS += -DDEBUG
endif

#obj-y := fusb302.o
obj-y := fusb302_typec.o
fusb302_typec-y := fusb302.o
fusb302_typec-y += usbpd.o
