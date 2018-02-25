#Roger here
#   zreladdr-y	:= 0x00208000
#params_phys-y	:= 0x00200100
#initrd_phys-y	:= 0x00400000
   zreladdr-y = $(CONFIG_KA2000_KERNEL_ZRELADDR)
params_phys-y = $(CONFIG_KA2000_KERNEL_PARAMS_PHYS)
initrd_phys-y = $(CONFIG_KA2000_KERNEL_INITRD_PHYS)
