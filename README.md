Sample ARM Application for SystemC-based HW/SW Co-Simulation
============================================================

This package contains a (cross-compiled) application example for interaction 
with a FPGA hardware device. It specifically interfaces with the debug device 
modeled in SystemC that is part of Xilinx's TLM-Cosimulation demo at:
* http://www.wiki.xilinx.com/Co-simulation
* https://github.com/Xilinx/systemctlm-cosim-demo

To compile, go into the respective subdirectory, make sure the cross-compiler
```
  % aarch64-linux-gnu-gcc --version
```
as well as the kernel sources are available (edit the Makefile as needed) and
simply compile with the
```
  % make
```
command. Then transfer the generated 'example' executable (and possibly the
'fpga_drv.ko' kernel module) into the QEMU-simulated system, and run there.
If using a kernel module, first transfer and load that:
```
  # scp <host>:<path>/application.irq/fpga_dr.ko .  
  # insmod fpga_drv.ko
```
Then transfer and run the application:
```
  # scp <host>:<path>/application[.irq]/example .
  # ./example [val]
```

Contents:
---------

 application/ - Memory-mapped I/O

   Application that uses memory-mapped I/O to interface with the FPGA HW
   device. No kernel module and no interrupt support.

 application.irq/ - Kernel module driver for I/O and interrupt handling

   Application that uses a kernel module as hardware driver with all control
   register I/O via ioctl() calls to the kernel and access to the local FPGA
   memory through the /dev/fpga device from the user application. Including 
   interrupt support via asynchronous signaling. 

 boot/ - Device tree for kernel module driver

   Device tree blob that registers an FPGA device with the kernel needed for
   the kernel module device driver from the application.irq example to work
   (see notes below).

 project-spec/ - Device tree entry for kernel

   Petalinux recipes to generate a device tree that includes an FPGA device 
   entry to the kernel's device tree for the kernel module from 
   application.irq (see notes below).    


Notes:

* By default, the kernel module that is part of application.irq uses the
  kernel's device tree information to get associated resource information
  (memory addresses and interrupt IDs). For this, an 'fpga' device needs to
  be added to the kernel's device tree. Either by copying the provided device
  tree to the disk image's /boot partition, or in PetaLinux, by adding
     `/include/ "zynq-fpga.dtsi"`
   to 
     `project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi`
   and 
     `SRC_URI:append = " <...> file://zynqmp-fpga.dtsi"`
   to
     `project-spec/meta-user/recipes-bsp/device-tree/device-tree.bbappend`
   and recompiling the device tree
   ```
     % petalinux-build -c device-tree
   ```

* Alternatively, the kernel module can be compiled to not use the device tree
  and instead hardcode the 'fpga' resource information. Eiter by setting
    `#define NO_DTS`
  in fpga_dr.c or by adding 
    `KFLAGS = -DNO_DTS` 
  in the Makefile. In this case, the kernel module needs to be loaded as
  ```
    # insmod fpga_drv.ko install=1
  ```
  This requires the kernel module to be recompiled every time the hardware 
  interface is changed. At the same time, no device tree modification and 
  rebooting is necessary on hardware changes.

* In reality, a combination of memory-mapped and driver-based I/O is possible
  and recommended. For example, since ioctl() kernel calls carry a lot of
  overhead, transfering large amounts of data to/from hardware is better
  achieved via memory-mapped interfaces (either from the user application or
  in the kernel module itself using the /dev device or application-specific 
  ioctl() calls that copy data from the device into user space, possibly using
  DMA, etc.)

-- 
Andreas Gerstlauer <gerstl@ece.utexas.edu>
