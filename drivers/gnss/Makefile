# Copyright (C) 2013 Intel Mobile Communications GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ------------------------------------------------------------------------


obj-$(CONFIG_GNSS_AG620) 		+= gnss_drv.o
gnss_drv-objs	:= imc_idi_gnss.o imc_idi_gnss_of.o imc_idi_gnss_pm.o

#ifeq ($(TARGET_BOARD_PLATFORM),scv6321)
#ccflags-y += -DINC_PLATDATA
#endif

#all:
#	@$(MAKE) -C $(KLIB_BUILD) M=$(CURDIR) ARCH=$(ARCH) \
#		CROSS_COMPILE=$(CROSS_COMPILE) modules

#clean:
#	@echo "cleaning GNSS driver"
#	@$(MAKE) -C $(KLIB_BUILD) M=$(CURDIR) ARCH=$(ARCH) \
#		CROSS_COMPILE=$(CROSS_COMPILE) clean
