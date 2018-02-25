#
# Makefile for the linux kernel.
#
#

# Common objects
obj-y 			:= ka2000-time.o irq.o io.o id.o  \
			     devices.o

obj-y	+= clock.o

obj-$(CONFIG_SERIAL_KA2000)  += ka2000-serial.o

# Board specific
obj-$(CONFIG_MACH_KA2000_EVM)  += board-evm.o


obj-$(CONFIG_SDIO_KA2000)	+= ka2000-sdio.o

obj-$(CONFIG_PWM_KA2000)  += ka2000-pwm.o pwm.o
obj-$(CONFIG_SPI_KA2000) += ka2000-spi.o

#ka2000-sdhc-objs			:= ka2000-sdswitch.o ka2000-sdmmc.o

