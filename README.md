# android_kernel_armada_pxa1088
Kernel Source of Samsung Galaxy Win Pro Duo

1. How to Build
	- get Toolchain
		From android git server , codesourcery and etc ..
		- arm-eabi-4.6

	- edit Makefile
		edit "CROSS_COMPILE" to right toolchain path(You downloaded).
		  EX)  export CROSS_COMPILE= $(android platform directory you download)/android/gcc/prebuilts/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
		  Ex)  export CROSS_COMPILE=/usr/local/toolchain/arm-eabi-4.6/bin/arm-eabi-          // check the location of toolchain

		$ make ARCH=arm pxa1088_wilcox_defconfig
		$ make

2. Output files
	- Kernel : arch/arm/boot/zImage
	- module : drivers/*/*.ko

3. How to Clean
		$ make clean

