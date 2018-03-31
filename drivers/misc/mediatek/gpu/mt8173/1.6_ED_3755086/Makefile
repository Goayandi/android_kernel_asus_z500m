obj-$(CONFIG_POWERVR_ADF_FBDEV) += powervr_adf_fbdev.o
powervr_adf_fbdev-y += adf_common.o adf_fbdev.o

VER := $(if $(filter eng,$(TARGET_BUILD_VARIANT)),eng,user)
ccflags-y += \
	-include $(srctree)/drivers/misc/mediatek/gpu/mt8173/1.6_ED3755086/config_kernel_$(VER).h \
	-I$(srctree)/$(src) \
	-I$(srctree)/drivers/staging/android

obj-y += rogue/
obj-y += mt8173/
