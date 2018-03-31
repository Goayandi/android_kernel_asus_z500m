obj-y += mt8173.o
mt8173-y += mtk_module.o
mt8173-y += ion_support_mt8173.o
mt8173-y += mt8173_mfgsys.o
mt8173-y += mt8173_sysconfig.o

VER := $(if $(filter eng,$(TARGET_BUILD_VARIANT)),eng,user)
ccflags-y += -include $(srctree)/drivers/misc/mediatek/gpu/mt8173/1.6_ED3755086/config_kernel_$(VER).h \
 -I$(srctree)/$(src) \
 -I$(srctree)/drivers/misc/mediatek/gpu/mt8173/1.6_ED3755086 \
 -I$(srctree)/drivers/misc/mediatek/gpu/mt8173/1.6_ED3755086/rogue \
 -I$(srctree)/drivers/misc/mediatek/gpu/mt8173/1.6_ED3755086/rogue/hwdefs \
 -I$(srctree)/drivers/misc/mediatek/gpu/mt8173/1.6_ED3755086/rogue/hwdefs/km \
 -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
