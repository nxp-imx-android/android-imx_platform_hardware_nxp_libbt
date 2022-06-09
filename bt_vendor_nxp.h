/******************************************************************************
 *
 *  Copyright 2018-2022 NXP
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

#define BT_HAL_VERSION "009.005"

#define TIMEOUT_SEC 6
#define RW_SUCCESSFUL (1)
#define RW_FAILURE (~RW_SUCCESSFUL)
#define BIT(x) (0x1 << x)

#define TRUE 1
#define FALSE 0
#define WRITE_BD_ADDRESS_SIZE 8
#define MAX_PATH_LEN 512
#define MAX_FILE_LEN 128
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

#define NXP_WAKEUP_ADV_PATTERN_LENGTH 16  // company id + vendor information

#define PROP_BLUETOOTH_OPENED "bluetooth.nxp.uart_configured"
#define PROP_BLUETOOTH_FW_DOWNLOADED "bluetooth.nxp.fw_downloaded"
#define PROP_BLUETOOTH_DELAY "bluetooth.nxp.fw_downloaded_delay"
#define PROP_BLUETOOTH_INBAND_CONFIGURED "bluetooth.nxp.inband_ir_configured"
/* Run-time configuration file */
#ifndef VENDOR_LIB_CONF_FILE
#define VENDOR_LIB_CONF_FILE "/vendor/etc/bluetooth/bt_vendor.conf"
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

/******************************************************************************
**  Structure
******************************************************************************/
enum { IR_TRIGGER_NONE, IR_TRIGGER_RFKILL, IR_TRIGGER_GPIO };
enum { IR_MODE_NONE, IR_MODE_OOB_VSC, IR_MODE_INBAND_VSC };
enum { wakeup_power_key, wakeup_netflix_key, wakeup_key_num };

typedef struct {
  unsigned char gpio_pin;
  unsigned char high_duration;  // 100ms
  unsigned char low_duration;   // 100ms
} wakeup_gpio_config_t;

typedef struct {
  unsigned char length;
  unsigned char adv_pattern[NXP_WAKEUP_ADV_PATTERN_LENGTH];
} wakeup_adv_pattern_config_t;

typedef struct {
  unsigned char le_scan_type;
  unsigned short interval;
  unsigned short window;
  unsigned char own_addr_type;
  unsigned char scan_filter_policy;
} wakeup_scan_param_config_t;

typedef struct {
  unsigned char heartbeat_timer_value;  // 100ms
} wakeup_local_param_config_t;

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
extern bool enable_sco_config;
extern bool use_controller_addr;
extern char pFilename_fw_init_config_bin[];
extern bool enable_heartbeat_config;
extern wakeup_gpio_config_t wakeup_gpio_config[wakeup_key_num];
extern wakeup_adv_pattern_config_t wakeup_adv_config;
extern wakeup_scan_param_config_t wakeup_scan_param_config;
extern wakeup_local_param_config_t wakup_local_param_config;
void wakeup_kill_heartbeat_thread(void);
#ifdef UART_DOWNLOAD_FW
extern uint8_t enable_poke_controller;
#endif
/*****************************************************************************
**   Functions Prototype
*****************************************************************************/
void hw_config_start(void);
int32 init_uart(int8* dev, int32 dwBaudRate, uint8 ucFlowCtrl);
int get_prop_int32(char* name);
int set_prop_int32(char* name, int value);
#endif
