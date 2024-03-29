VxWorks® 7 IPI Driver for the Xilinx® Zynq® UltraScale+™ MPSoC ZCU102 reference platform.
---

# Overview
This driver is part of the [OpenAMP for VxWorks Remote Compute](https://github.com/Wind-River/openamp-for-vxworks-remote-compute) project. It implements basic IPI communications required by a **VxWorks OpenAMP** client.  It is used by [VxWorks OpenAMP Layer for Zcu102](https://github.com/Wind-River/vxworks7-openamp-layer-for-zcu102) as part of **VxWorks OpenAMP** integration to support master/remote communications.

# Limitations
This IPI driver was written to be compatible with Linux Remoteproc on the **Xilinx Zcu102** and has the following limitations.

1. It supports the generation of IPI interrupts only (the available **Zynqmp** message buffer system is not used).
2. Only RPU0 and PL0 are defined as follows:
   *  RPU0 is used by the **VxWorks OpenAMP** remote image.
   *  PL0 is used by the **VxWorks OpenAMP** or **Wind River® Linux** master.

The driver was tested with the following BSPs
* xlnx_zynqmp SR0620 - **VxWorks OpenAMP** master running on the APU (all A53 cores)
* xlnx_zynqmp_r5 SR0620 - **VxWorks OpenAMP** remote running on the RPU (core R0 of the R5 processor)

# Project License
The source code for this project is provided under the BSD-3 license. 

# Prerequisite(s)
* Installation of **Wind River VxWorks® 7 for ARM, SR0620**.

# Installation
It is assumed that you are running on a **Linux** host and the **$WIND_HOME** variable is set to the root of your **Wind River** installation.
```
wruser@Mothra:~/WindRiver-SR0620/github$ echo $WIND_HOME
/home/wruser/WindRiver-SR0620
```
Only new files are added to your installation.  The  **xlnx_zynq** top level Makefile must be updated manually.

1. Run the following commands from a shell. 
```
cd /tmp
git clone https://github.com/Wind-River/vxworks7-ipi-driver-for-zcu102.git
cp $WIND_HOME/vxworks-7/pkgs_v2/os/psl/xilinx/xlnx_zynq-2.0.2.0/drv/Makefile $WIND_HOME/vxworks-7/pkgs_v2/os/psl/xilinx/xlnx_zynq-2.0.2.0/drv/Makefile.bak
cp -a vxworks7-ipi-driver-for-zcu102/* $WIND_HOME/vxworks-7/pkgs_v2/os/psl/xilinx/xlnx_zynq-2.0.2.0/
cd -
```

2. Open **$WIND_HOME/vxworks-7/pkgs_v2/os/psl/xilinx/xlnx_zynq-2.0.2.0/drv/Makefile** in an editor.

3. Add the **vxbFdtZynqmpIpi.o**  in the **OBJ** list for both the **_WRS_CONFIG_ARM_ARMV8A** and **_WRS_CONFIG_ARM_ARMV7A** macro and save the file. 

```
ifdef _WRS_CONFIG_ARM_ARMV8A
OBJS           = vxbFdtZynqSio.o         \
                 vxbFdtZynqI2c.o         \
                 vxbFdtZynqTimer.o       \
                 vxbFdtZynqmpClk.o       \
                 vxbFdtZynqmpQspi.o      \
                 vxbFdtZynqmpZdma.o      \
                 vxbFdtXlnxZynqPinmux.o  \
                 vxbFdtZynqNwlPcie.o     \
                 vxbFdtZynqmpIpi.o       \
                 vxbFdtZynqGpio.o

endif

ifdef _WRS_CONFIG_ARM_ARMV7A
OBJS            = vxbFdtZynqSio.o         \
                  vxbFdtZynqI2c.o         \
                  vxbFdtZynqTimer.o       \
                  vxbFdtZynq7kDevCfg.o    \
                  vxbFdtZynq7kMiscCfg.o   \
                  vxbFdtZynq7kAxi2PciEx.o \
                  vxbFdtZynq7kQspi.o      \
                  vxbFdtZynqmpQspi.o      \
                  vxbFdtZynqmpZdma.o      \
                  vxbFdtZynqmpClk.o       \
                  vxbFdtXlnxZynqPinmux.o  \
                  vxbFdtZynqNwlPcie.o     \
                  vxbFdtZynqmpIpi.o       \
                  vxbFdtZynqGpio.o
endif
```

# Device Tree entries

## VxWorks 7 OpenAMP Remote image xlnx_zynqmp_r5.dsti
```/* OpenAMP will use RPU0 0 for IPI interrupts */
                  ipi: ipi@ff310000
                   {
                   compatible = "xlnx,zynqmp-ipi";
                   status = "disabled";
                   reg = <0xff310000 0x100>;
                   interrupts = <65 0 4>;
                   interrupt-parent = <&intc>;
                   };
```

## VxWorks 7 OpenAMP Master image xlnx_zynqmp.dsti
```/* OpenAMP will use PL 0 for IPI interrupts */
                   ipi: ipi@ff340000
                      {
                      compatible = "xlnx,zynqmp-ipi";
                      status = "disabled";
                      reg = <0 0xff340000 0 0x100>;
                      interrupts = <61 0 4>;
                      interrupt-parent = <&intc>;
                      };
```

# Project License

The source code for this project is provided under the BSD-3-Clause license.
Text for the open-amp and libmetal applicable license notices can be found in
the LICENSE_NOTICES.txt file in the project top level directory. Different
files may be under different licenses. Each source file should include a
license notice that designates the licensing terms for the respective file.


# Legal Notices

All product names, logos, and brands are property of their respective owners. All company,
product and service names used in this software are for identification purposes only.
Wind River and VxWorks are registered trademarks of Wind River Systems, Inc.  Xilinx,
UltraScale, Zynq, are trademarks of Xilinx in the United States and other countries.
Arm and Cortex are registered trademarks of Arm Limited (or its subsidiaries) in the US
and/or elsewhere.

Disclaimer of Warranty / No Support: Wind River does not provide support 
and maintenance services for this software, under Wind River's standard 
Software Support and Maintenance Agreement or otherwise. Unless required 
by applicable law, Wind River provides the software (and each contributor 
provides its contribution) on an "AS IS" BASIS, WITHOUT WARRANTIES OF ANY 
KIND, either express or implied, including, without limitation, any warranties 
of TITLE, NONINFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A PARTICULAR 
PURPOSE. You are solely responsible for determining the appropriateness of 
using or redistributing the software and assume any risks associated with 
your exercise of permissions under the license.
