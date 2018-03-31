MTK_PLATFORM:= $(subst ",,$(CONFIG_MTK_PLATFORM))

ccflags-y += -I$(srctree)/drivers/misc/mediatek/auxadc/$(MTK_PLATFORM)

obj-$(CONFIG_MTK_AUXADC) += mt_auxadc.o

obj-$(CONFIG_MTK_AUXADC) += asustek_battery_id.o

