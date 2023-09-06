#
# Copyright 2012 The Android Open Source Project
# Portions copyright 2012-2013, 2015, 2018-2021 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

ifneq ($(BOARD_HAVE_BLUETOOTH_NXP),)

LOCAL_PATH := $(call my-dir)

BOARD_UART_DOWNLOAD_FW := true
# v2 is for 8887-FP101, and v3 is for other chips.
BOARD_UART_FW_LOADER_VERSION = v3

# libbt-vendor.so
include $(CLEAR_VARS)
ifeq ($(shell expr $(PLATFORM_VERSION) "==" Tiramisu),1)
BDROID_DIR := $(TOP_DIR)packages/modules/Bluetooth/system
else
ifeq ($(shell expr $(PLATFORM_SDK_VERSION) "<=" 32),1)
BDROID_DIR := $(TOP_DIR)system/bt
else
BDROID_DIR := $(TOP_DIR)packages/modules/Bluetooth/system
endif
endif

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/conf/nxp/ \
    $(TOP_DIR)hardware/interfaces/bluetooth/1.0/default

LOCAL_SRC_FILES := \
    bt_vendor_nxp.c \
    fw_loader_io.c \
    hardware_nxp.c

# VHAL_LOG_LEVEL decides maximum log level supported at compile time between 0-5.
LOCAL_CFLAGS += -DVHAL_LOG_LEVEL=5
LOCAL_CFLAGS += -Wsign-compare
ifneq ($(BOARD_UART_DOWNLOAD_FW), false)
LOCAL_CFLAGS += -DUART_DOWNLOAD_FW
#LOCAL_CFLAGS += -DDEBUG_PRINT
ifneq ($(BOARD_UART_FW_LOADER_VERSION), v2)
LOCAL_SRC_FILES += \
    fw_loader_uart.c
else
LOCAL_CFLAGS += -DFW_LOADER_V2
LOCAL_SRC_FILES += \
    fw_loader_uart_v2.c
endif
endif

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    liblog \
    libprotobuf-cpp-lite \
    libbase \
    libchrome
LOCAL_MODULE := libbt-vendor
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := nxp
LOCAL_MULTILIB := both
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)
ifeq ($(TARGET_PRODUCT), evk_8mp)
    include $(LOCAL_PATH)/conf/nxp/evk_8mp/Android.mk
endif
ifeq ($(TARGET_PRODUCT), evk_8mq)
    include $(LOCAL_PATH)/conf/nxp/evk_8mq/Android.mk
endif
ifeq ($(TARGET_PRODUCT), evk_8mn)
    include $(LOCAL_PATH)/conf/nxp/evk_8mn/Android.mk
endif
ifeq ($(TARGET_PRODUCT), evk_8mm)
    include $(LOCAL_PATH)/conf/nxp/evk_8mm/Android.mk
endif
ifeq ($(TARGET_PRODUCT), evk_8ulp)
    include $(LOCAL_PATH)/conf/nxp/evk_8ulp/Android.mk
endif
ifeq ($(TARGET_PRODUCT), watch_8ulp)
    include $(LOCAL_PATH)/conf/nxp/watch_8ulp/Android.mk
endif
ifeq ($(TARGET_PRODUCT), evk_93)
    include $(LOCAL_PATH)/conf/nxp/evk_93/Android.mk
endif
ifeq ($(TARGET_PRODUCT),$(filter $(TARGET_PRODUCT),mek_8q mek_8q_car mek_8q_car2))
    include $(LOCAL_PATH)/conf/nxp/mek_8q/Android.mk
endif
endif # BOARD_HAVE_BLUETOOTH_NXP
