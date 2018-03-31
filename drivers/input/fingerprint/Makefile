ifeq ($(ASUS_FTM), y)
	ccflags-y := -DASUS_FTM
endif

obj-$(CONFIG_FPC_FINGERPRINT_SPI) += fpc1020_main.o fpc1020_common.o fpc1020_input.o fpc1020_capture.o

obj-$(CONFIG_ELAN_FINGERPRINT_SPI) += efsa120s.o asus_fp_id.o
obj-$(CONFIG_SYNAPTICS_FINGERPRINT_SPI) += vfsSpiDrv.o asus_fp_id.o

