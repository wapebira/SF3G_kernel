# Description: Makefile for the SOCWatch driver.

# ***********************************************************************************************

 # This file is provided under a dual BSD/GPLv2 license.  When using or 
 # redistributing this file, you may do so under either license.

  # GPL LICENSE SUMMARY

  # Copyright(c) 2013 Intel Corporation. All rights reserved.

  # This program is free software; you can redistribute it and/or modify 
  # it under the terms of version 2 of the GNU General Public License as
  # published by the Free Software Foundation.

  # This program is distributed in the hope that it will be useful, but 
  # WITHOUT ANY WARRANTY; without even the implied warranty of 
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  # General Public License for more details.

  # You should have received a copy of the GNU General Public License 
  # along with this program; if not, write to the Free Software 
  # Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  # The full GNU General Public License is included in this distribution 
  # in the file called LICENSE.GPL.

  # Contact Information:
  # SOCWatch Developer Team <socwatchdevelopers@intel.com>

  # BSD LICENSE 

  # Copyright(c) 2013 Intel Corporation. All rights reserved.
  # All rights reserved.

  # Redistribution and use in source and binary forms, with or without 
  # modification, are permitted provided that the following conditions 
  # are met:

  #   * Redistributions of source code must retain the above copyright 
  #     notice, this list of conditions and the following disclaimer.
  #   * Redistributions in binary form must reproduce the above copyright 
  #     notice, this list of conditions and the following disclaimer in 
  #     the documentation and/or other materials provided with the 
  #     distribution.
  #   * Neither the name of Intel Corporation nor the names of its 
  #     contributors may be used to endorse or promote products derived 
  #     from this software without specific prior written permission.

  # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
  # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
  # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
  # A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
  # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
  # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
  # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
  # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  # ***********************************************************************************************


# ***************************************************************************************
#      Copyright (C) 2011 Intel Corporation.  All Rights Reserved.

#      This file is part of the Power Development Kit, and is free software;
#      you can redistribute it
#      and/or modify it under the terms of the GNU General Public License
#      version 2 as published by the Free Software Foundation.

#      This file is distributed in the hope that it will be useful,
#      but WITHOUT ANY WARRANTY; without even the implied warranty of
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#      GNU General Public License for more details.

#      You should have received a copy of the GNU General Public License
#      along with the Power Development Kit; if not, write to the Free Software
#      Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#      As a special exception, you may use this file as part of a free software
#      library without restriction.  Specifically, if other files instantiate
#      templates or use macros or inline functions from this file, or you compile
#      this file and link it with other files to produce an executable, this
#      file does not by itself cause the resulting executable to be covered by
#      the GNU General Public License.  This exception does not however
#      invalidate any other reasons why the executable file might be covered by
#      the GNU General Public License.
# ***************************************************************************************

# -------------------- user configurable options ------------------------

# location to install driver
INSTALL = .

INCDIR=$(PWD)/../include
INC2=$(PWD)/inc

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
EXTRA_CFLAGS += -I$(INCDIR)
EXTRA_CFLAGS += -I$(INC2)
ifeq ($(DO_DEBUG_BUILD), 1)
    EXTRA_CFLAGS += -Werror=strict-prototypes
    EXTRA_CFLAGS += -Werror=pointer-to-int-cast
    EXTRA_CFLAGS += -Werror=int-to-pointer-cast
    EXTRA_CFLAGS += -Werror=format
    EXTRA_CFLAGS += -Werror=attributes
    EXTRA_CFLAGS += -Wno-unused
    EXTRA_CFLAGS += -Werror=undef
    EXTRA_CFLAGS += -Werror
endif

# The driver needs to know what version we have
KERNEL_VERSION=$(shell uname -r | awk 'BEGIN{FS="[^0-9]"} {print $$3}')

ifeq ($(APWR_RED_HAT),)
APWR_RED_HAT := "0"
endif
ifeq ($(WAKELOCK_SAMPLE),)
WAKELOCK_SAMPLE := "0"
endif
ifeq ($(DO_ANDROID),)
DO_ANDROID := "1"
endif
ifeq ($(DO_INTEL_INTERNAL),)
DO_INTEL_INTERNAL := "0"
endif

# base name of APWR driver
#ifeq ($(DO_INTEL_INTERNAL),1)
#DRIVER_BASE=socwatch
#else
#DRIVER_BASE=SOCWATCH
#endif
DRIVER_BASE=socwatch
DRIVER_MAJOR=0
DRIVER_MINOR=4
# basic name of driver
DRIVER_NAME=${DRIVER_BASE}${DRIVER_MAJOR}_${DRIVER_MINOR}


EXTRA_CFLAGS += -DAPWR_RED_HAT=$(APWR_RED_HAT)
EXTRA_CFLAGS += -DDO_WAKELOCK_SAMPLE=$(WAKELOCK_SAMPLE)
EXTRA_CFLAGS += -DDO_ANDROID=$(DO_ANDROID)
EXTRA_CFLAGS += -DDO_INTEL_INTERNAL=$(DO_INTEL_INTERNAL) 

# Workaround for Samsung compiler warnings
#MOD_SOCPERF = /home/gupadhya/Git_modules/working_dirs/current/tmp/matrix/socwatch_linux_INTERNAL/soc_perf_driver/src/Module.symvers
#KBUILD_EXTRA_SYMBOLS:=$(MOD_SOCPERF)

KBUILD_EXTRA_SYMBOLS:=$(MODULE_SYMVERS_FILE)

obj-m := $(DRIVER_NAME).o
$(DRIVER_NAME)-objs := src/sp_vmm_buffer.o \
                     src/sp.o \
                     src/sp_linux_buffer.o \
                     src/sp_linux_wakeup_handler.o \
                     src/sp_linux_irq_handler.o \
                     src/sp_linux_timer_handler.o

.PHONY: kernel_check

kernel_check:
ifeq "$(KERNEL_SRC_DIR)" ""
	@echo "Error: makefile MUST NOT be invoked directly! Use the \"build_driver\" script instead."
	@exit 255
endif

default: kernel_check
	@echo "************************************************************"
	@echo "KERNEL_SRC_DIR=$(KERNEL_SRC_DIR)"
	@echo "APWR_RED_HAT=$(APWR_RED_HAT)"
	@echo "DO_WAKELOCK_SAMPLE=$(WAKELOCK_SAMPLE)"
	@echo "DO_ANDROID=$(DO_ANDROID)"
	@echo "************************************************************"
	make -C $(KERNEL_SRC_DIR) M=$(PWD) modules PWD=$(PWD)

clean: kernel_check
	make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
