#
# Makefile for Invensense inv-mpu-iio device.
#

obj-$(CONFIG_INV_MPU_IIO) += inv-mpu-iio.o

inv-mpu-iio-objs := inv_mpu_core.o
inv-mpu-iio-objs += inv_mpu_ring.o
inv-mpu-iio-objs += inv_mpu_trigger.o
inv-mpu-iio-objs += inv_mpu_misc.o
inv-mpu-iio-objs += inv_mpu3050_iio.o
inv-mpu-iio-objs += dmpDefaultMPU6050.o
inv-mpu-iio-objs += inv_slave_compass.o
inv-mpu-iio-objs += inv_slave_pressure.o

#$(info "KERNELVERSION=3.10")
#--yd ifeq ($(CONFIG_INV_KERNEL_3_10), y)
CFLAGS_inv_mpu_core.o      += -Idrivers/iio    -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_inv_mpu_ring.o      += -Idrivers/iio    -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_inv_mpu_trigger.o   += -Idrivers/iio    -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_inv_mpu_misc.o      += -Idrivers/iio    -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_inv_mpu3050_iio.o   += -Idrivers/iio    -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_dmpDefaultMPU6050.o += -Idrivers/iio    -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_inv_slave_compass.o   += -Idrivers/iio  -Iinclude/linux/iio  -DINV_KERNEL_3_10
CFLAGS_inv_slave_pressure.o   += -Idrivers/iio -Iinclude/linux/iio  -DINV_KERNEL_3_10

# the Bosch BMA250 driver is added to the inv-mpu device driver because it
# must be connected to an MPU3050 device on the secondary slave bus.
ifeq ($(CONFIG_INV_IIO_MPU3050_ACCEL_SLAVE_BMA250), y)
inv-mpu-iio-objs += inv_slave_bma250.o
#--yd ifeq ($(CONFIG_INV_KERNEL_3_10), y)
CFLAGS_inv_slave_bma250.o   += -Idrivers/iio
endif

# compile Invensense MPU IIO driver as DTS
ifeq ($(CONFIG_DTS_INV_MPU_IIO), y)
inv-mpu-iio-objs += inv_mpu_dts.o
#--yd ifeq ($(CONFIG_INV_KERNEL_3_10), y)
CFLAGS_inv_mpu_dts.o   += -Idrivers/iio
endif

