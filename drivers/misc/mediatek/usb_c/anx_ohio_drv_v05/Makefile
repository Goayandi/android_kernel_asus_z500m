obj-$(CONFIG_ANALOGIX_OHIO)	+= anx_ohio_driver.o anx_ohio_private_interface.o anx_ohio_public_interface.o
obj-$(CONFIG_ANALOGIX_OHIO)	+= mt_pd_policy.o
obj-$(CONFIG_ANALOGIX_OHIO)	+= anx_update_ocm.o

subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/ssusb/mu3d
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/power/mt81xx
