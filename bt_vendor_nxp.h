/******************************************************************************
 *
 *  Copyright 2018-2021 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#ifndef _BT_VENDOR_NXP_H
#define _BT_VENDOR_NXP_H

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "bt_vendor_lib.h"

/******************************************************************************
**  Local type definitions
******************************************************************************/
typedef unsigned long long uint64;
typedef unsigned int uint32;
typedef unsigned short uint16;
typedef unsigned char uint8;
typedef int int32;
typedef short int16;
typedef char int8;
typedef unsigned char BOOLEAN;

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#define BT_HAL_VERSION "008.014"

#define TIMEOUT_SEC 6
#define RW_SUCCESSFUL (1)
#define RW_FAILURE (~RW_SUCCESSFUL)
#define BIT(x) (0x1 << x)

#define TRUE 1
#define FALSE 0
#define WRITE_BD_ADDRESS_SIZE 8

#define DOWNLOAD_SUCCESS 0x0
#define OPEN_SERIAL_PORT_OR_FILE_ERROR 0x1
#define FEEK_SEEK_ERROR 0x2
#define FILESIZE_IS_ZERO 0x3
#define HEADER_SIGNATURE_TIMEOUT 0x4
#define READ_FILE_FAIL 0x5
#define CHANGE_BAUDRATE_FAIL 0x6
#define CHANGE_TIMEOUT_VALUE_FAIL 0x7
#define OPEN_FILE_FAIL 0x8
#define FILE_MODE_CANNOT_CHANGE 0X9
#define UNEXPECTED_BEHAVIOUR_IN_SETJMP 0xA
#define MALLOC_RETURNED_NULL 0xB
#define START_INDICATION_NOT_FOUND 0xC

#define BLE_SET_1M_POWER 0x01
#define BLE_SET_2M_POWER 0x02

/* Run-time configuration file */
#ifndef VENDOR_LIB_CONF_FILE
#define VENDOR_LIB_CONF_FILE "/vendor/etc/bluetooth/bt_vendor.conf"
#endif

#ifndef NXP_LOAD_BT_CALIBRATION_DATA
#define NXP_LOAD_BT_CALIBRATION_DATA FALSE
#endif

#ifndef NXP_SET_BLE_TX_POWER_LEVEL
#define NXP_SET_BLE_TX_POWER_LEVEL FALSE
#endif

#ifndef NXP_ENABLE_BT_TX_MAX_POWER
#define NXP_ENABLE_BT_TX_MAX_POWER FALSE
#endif

#ifndef NXP_ENABLE_INDEPENDENT_RESET_VSC
#define NXP_ENABLE_INDEPENDENT_RESET_VSC FALSE
#endif

#ifndef NXP_ENABLE_INDEPENDENT_RESET_CMD5
#define NXP_ENABLE_INDEPENDENT_RESET_CMD5 FALSE
#endif

#ifndef NXP_IR_IMX_GPIO_TOGGLE
#define NXP_IR_IMX_GPIO_TOGGLE FALSE
#endif

#ifndef NXP_ENABLE_RFKILL_SUPPORT
#define NXP_ENABLE_RFKILL_SUPPORT FALSE
#endif

#ifndef NXP_VND_DBG
#define NXP_VND_DBG FALSE
#endif

#if (NXP_VND_DBG == TRUE)
#define VNDDBG(fmt, ...) \
  ALOGD("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define VNDDBG(fmt, ...)
#endif

/***********************************************************
 *  Externs
 ***********************************************************
 */
extern unsigned char* bdaddr;
extern int write_bdaddrss;
extern uint8_t write_bd_address[WRITE_BD_ADDRESS_SIZE];
extern const bt_vendor_callbacks_t* vnd_cb;
extern char pFilename_cal_data[];
extern int8_t ble_1m_power;
extern int8_t ble_2m_power;
extern int8_t bt_max_power_sel;
extern uint8_t set_1m_2m_power;
extern uint8_t bt_set_max_power;
extern uint8_t independent_reset_mode;
extern uint8_t independent_reset_gpio_pin;
extern uint32_t target_soc;
/*****************************************************************************
**   Functions Prototype
*****************************************************************************/
void hw_config_start(void);
int32 init_uart(int8* dev, int32 dwBaudRate, uint8 ucFlowCtrl);
#endif
