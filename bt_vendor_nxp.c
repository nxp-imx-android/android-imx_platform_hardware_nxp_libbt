/******************************************************************************
 *  Copyright 2012 The Android Open Source Project
 *  Portions copyright (C) 2009-2012 Broadcom Corporation
 *  Portions copyright 2012-2013, 2015, 2018-2023 NXP
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

/******************************************************************************
 *
 *  Filename:      bt_vendor_nxp.c
 *
 *  Description:   NXP vendor specific library exported functions with
 *                 configuration file processing function
 *
 ******************************************************************************/

#define LOG_TAG "bt-vnd-nxp"

/*============================== Include Files ===============================*/

#include <ctype.h>
#include <cutils/properties.h>
#include <errno.h>
#include <grp.h>
#include <pthread.h>
#include <pwd.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#ifdef FW_LOADER_V2
#include "fw_loader_uart_v2.h"
#else
#include "fw_loader_uart.h"
#endif

#include <linux/gpio.h>

#include "bt_vendor_log.h"
#include "bt_vendor_nxp.h"
#include "fw_loader_io.h"
/*================================== Macros ==================================*/
/*[NK] @NXP - Driver FIX
  ioctl command to release the read thread before driver close */

#define MBTCHAR_IOCTL_RELEASE _IO('M', 1)

/*
 * Defines for wait for Bluetooth firmware ready Specify durations
 * between polls and max wait time
 */
#define POLL_DRIVER_DURATION_US (100000)
#define POLL_DRIVER_MAX_TIME_MS (20000)
#define POLL_CONFIG_UART_MS (10)
#define POLL_RETRY_TIMEOUT_MS (1)
#define POLL_MAX_TIMEOUT_MS (1000)

#define CONF_COMMENT '#'
#define CONF_DELIMITERS " =\n\r\t"
#define CONF_VALUES_DELIMITERS "=\n\r\t"
#define CONF_MAX_LINE_LEN 1024
#define UNUSED(x) (void)(x)
#define BD_ADDR_LEN 6
#define BD_STR_ADDR_LEN 17

/*================================== Typedefs ================================*/

typedef int(conf_action_t)(char* p_conf_name, char* p_conf_value, int param);

typedef struct {
  const char* conf_entry;
  conf_action_t* p_action;
  int param;
} conf_entry_t;

/*============================ Function Prototypes ===========================*/
static int send_hci_reset(void);

/*================================ Variables =================================*/
int mchar_fd = 0;
int vhal_trace_level = BT_TRACE_LEVEL_INFO;
struct termios ti;
static uint8_t adapterState;
unsigned char* bdaddr = NULL;
const bt_vendor_callbacks_t* vnd_cb = NULL;
/* for NXP USB/SD interface */
static char mbt_port[MAX_PATH_LEN] = "/dev/mbtchar0";
/* for NXP Uart interface */
static char mchar_port[MAX_PATH_LEN] = "/dev/ttyUSB0";
static int is_uart_port = 0;
static int32_t baudrate_fw_init = 115200;
static int32_t baudrate_bt = 3000000;
int write_bdaddrss = 0;
int8_t ble_1m_power = 0;
int8_t ble_2m_power = 0;
uint8_t set_1m_2m_power = 0;
int8_t bt_max_power_sel = 0;
uint8_t bt_set_max_power = 0;
uint8_t independent_reset_gpio_pin = 0xFF;
bool enable_sco_config = TRUE;
bool enable_pdn_recovery = FALSE;
bool use_controller_addr = TRUE;
uint8_t ir_host_gpio_pin = 14;
static char chrdev_name[32] = "/dev/gpiochip5";
/* 0:disable IR(default); 1:OOB IR(FW Config); 2:Inband IR;*/
uint8_t independent_reset_mode = IR_MODE_NONE;
/* 0:disable OOB IR Trigger; 1:RFKILL Trigger; 2:GPIO Trigger;*/
uint8_t send_oob_ir_trigger = IR_TRIGGER_NONE;
bool enable_heartbeat_config = FALSE;
bool wakeup_enable_uart_low_config = FALSE;
char pFilename_fw_init_config_bin[MAX_PATH_LEN];
uint8_t write_bd_address[WRITE_BD_ADDRESS_SIZE] = {
    0xFE, /* Parameter ID */
    0x06, /* bd_addr length */
    0x00, /* 6th byte of bd_addr */
    0x00, /* 5th */
    0x00, /* 4th */
    0x00, /* 3rd */
    0x00, /* 2nd */
    0x00  /* 1st */
};
/*Low power mode LPM enables controller to go in sleep mode, this command
 * enables host to controller sleep(H2C)*/
static bool enable_lpm = FALSE;
bool lpm_configured = FALSE;
static int32_t lpm_timeout_ms = 1000;
#ifdef UART_DOWNLOAD_FW
static int enable_download_fw = 0;
static int uart_sleep_after_dl = 100;
static int download_helper = 0;
static bool auto_select_fw_name = TRUE;
static int32_t baudrate_dl_helper = 115200;
static int32_t baudrate_dl_image = 3000000;
static char pFileName_helper[MAX_PATH_LEN] =
    "/vendor/firmware/helper_uart_3000000.bin";
static char pFileName_image[MAX_PATH_LEN] =
    "/vendor/firmware/uart8997_bt_v4.bin";
static int32_t iSecondBaudrate = 0;
uint8_t enable_poke_controller = 0;
static bool send_boot_sleep_trigger = FALSE;
#endif
char pFilename_cal_data[MAX_PATH_LEN];
static int rfkill_id = -1;
static char* rfkill_state_path = NULL;
int last_baudrate = 0;
wakeup_gpio_config_t wakeup_gpio_config[wakeup_key_num] = {
    {.gpio_pin = 13, .high_duration = 2, .low_duration = 2},
    {.gpio_pin = 13, .high_duration = 4, .low_duration = 4}};
wakeup_adv_pattern_config_t wakeup_adv_config = {.length = 0};
wakeup_scan_param_config_t wakeup_scan_param_config = {.le_scan_type = 0,
                                                       .interval = 128,
                                                       .window = 96,
                                                       .own_addr_type = 0,
                                                       .scan_filter_policy = 0};
wakeup_local_param_config_t wakeup_local_param_config = {.heartbeat_timer_value =
                                                            8};

/*============================== Coded Procedures ============================*/

static int set_enable_sco_config(char* p_conf_name, char* p_conf_value,
                                 int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_sco_config = (atoi(p_conf_value) == 0) ? FALSE : TRUE;
  return 0;
}

static int set_use_controller_addr(char* p_conf_name, char* p_conf_value,
                                   int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  use_controller_addr = (atoi(p_conf_value) == 0) ? FALSE : TRUE;
  return 0;
}
#define MCHAR_PORT_PROP "ro.boot.bluetooth.mchar_port"
static int set_mchar_port(char* p_conf_name, char* p_conf_value, int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_PATH_LEN);
  if (len_p_conf_value < MAX_PATH_LEN) {
    property_get(MCHAR_PORT_PROP, mchar_port, p_conf_value);
    VND_LOGI("the mchar_port is %s", mchar_port);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_PATH_LEN - 1);
  }
  is_uart_port = 1;
  return 0;
}

static int set_mbt_port(char* p_conf_name, char* p_conf_value, int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_PATH_LEN);
  if (len_p_conf_value < MAX_PATH_LEN) {
    strlcpy(mbt_port, p_conf_value, len_p_conf_value + 1);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_PATH_LEN - 1);
  }
  return 0;
}

static int set_baudrate_bt(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  baudrate_bt = atoi(p_conf_value);
  return 0;
}

static int set_enable_lpm(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_lpm = atoi(p_conf_value) > 0 ? TRUE : FALSE;
  return 0;
}

static int set_lpm_timeout(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  lpm_timeout_ms = atoi(p_conf_value);
  return 0;
}

static int set_baudrate_fw_init(char* p_conf_name, char* p_conf_value,
                                int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  baudrate_fw_init = atoi(p_conf_value);
  return 0;
}
static int set_ble_1m_power(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  ble_1m_power = atoi(p_conf_value);
  set_1m_2m_power |= BLE_SET_1M_POWER;
  return 0;
}

static int set_ble_2m_power(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  ble_2m_power = atoi(p_conf_value);
  set_1m_2m_power |= BLE_SET_2M_POWER;
  return 0;
}
static int set_bt_tx_power(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  bt_max_power_sel = atoi(p_conf_value);
  bt_set_max_power = 1;
  return 0;
}

static int set_pFilename_fw_init_config_bin(char* p_conf_name,
                                            char* p_conf_value, int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_PATH_LEN);
  if (len_p_conf_value < MAX_PATH_LEN) {
    strlcpy(pFilename_fw_init_config_bin, p_conf_value, len_p_conf_value + 1);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_PATH_LEN - 1);
  }
  return 0;
}
static int set_independent_reset_mode(char* p_conf_name, char* p_conf_value,
                                      int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  independent_reset_mode = atoi(p_conf_value);
  return 0;
}

static int set_enable_pdn_recovery(char* p_conf_name, char* p_conf_value,
                                   int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_pdn_recovery = (atoi(p_conf_value) == 0) ? FALSE : TRUE;
  return 0;
}

static int set_send_oob_ir_trigger(char* p_conf_name, char* p_conf_value,
                                   int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  send_oob_ir_trigger = atoi(p_conf_value);
  return 0;
}

static int set_independent_reset_gpio_pin(char* p_conf_name, char* p_conf_value,
                                          int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  independent_reset_gpio_pin = atoi(p_conf_value);
  return 0;
}

static int set_oob_ir_host_gpio_pin(char* p_conf_name, char* p_conf_value,
                                    int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  ir_host_gpio_pin = atoi(p_conf_value);
  return 0;
}
static int set_charddev_name(char* p_conf_name, char* p_conf_value, int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_DEVICE_LEN);
  if (len_p_conf_value < MAX_DEVICE_LEN) {
    strlcpy(chrdev_name, p_conf_value, len_p_conf_value + 1);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_DEVICE_LEN - 1);
  }
  return 0;
}

static int set_bd_address_buf(char* p_conf_name, char* p_conf_value,
                              int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  int i = 0;
  int j = 7;
  int len = 0;
  if (p_conf_value == NULL) return 0;
  len = strnlen(p_conf_value, BD_STR_ADDR_LEN + 1);
  if (len != BD_STR_ADDR_LEN) {
    VND_LOGE("Invalid string length, unable to process");
    VND_LOGE("Source length: %d and expected length: %d", len, BD_STR_ADDR_LEN);
    return 0;
  }
  for (i = 0; i < BD_STR_ADDR_LEN; i++) {
    if (((i + 1) % 3) == 0 && p_conf_value[i] != ':') return 0;
    if (((i + 1) % 3) != 0 && !isxdigit(p_conf_value[i])) return 0;
    char tmp = p_conf_value[i];
    if (isupper(p_conf_value[i])) {
      p_conf_value[i] = p_conf_value[i] - 'A' + 10;
    } else if (islower(p_conf_value[i])) {
      p_conf_value[i] = p_conf_value[i] - 'a' + 10;
    } else if (isdigit(p_conf_value[i])) {
      p_conf_value[i] = p_conf_value[i] - '0';
    } else if (p_conf_value[i] == ':')
      p_conf_value[i] = tmp;
    else
      return 0;
  }
  for (i = 0; i < BD_STR_ADDR_LEN; i = i + 3) {
    write_bd_address[j--] = (p_conf_value[i] << 4) | p_conf_value[i + 1];
  }
  write_bdaddrss = 1;
  return 0;
}

#ifdef UART_DOWNLOAD_FW
static int set_enable_download_fw(char* p_conf_name, char* p_conf_value,
                                  int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_download_fw = atoi(p_conf_value);
  return 0;
}

static int set_pFileName_image(char* p_conf_name, char* p_conf_value,
                               int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_PATH_LEN);
  if (len_p_conf_value < MAX_PATH_LEN) {
    strlcpy(pFileName_image, p_conf_value, len_p_conf_value + 1);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_PATH_LEN - 1);
  }
  auto_select_fw_name = FALSE;
  return 0;
}

static int set_pFileName_helper(char* p_conf_name, char* p_conf_value,
                                int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_PATH_LEN);
  if (len_p_conf_value < MAX_PATH_LEN) {
    strlcpy(pFileName_helper, p_conf_value, len_p_conf_value + 1);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_PATH_LEN - 1);
  }
  download_helper = 1;
  return 0;
}

static int set_baudrate_dl_helper(char* p_conf_name, char* p_conf_value,
                                  int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  baudrate_dl_helper = atoi(p_conf_value);
  return 0;
}

static int set_baudrate_dl_image(char* p_conf_name, char* p_conf_value,
                                 int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  baudrate_dl_image = atoi(p_conf_value);
  return 0;
}

static int set_iSecondBaudrate(char* p_conf_name, char* p_conf_value,
                               int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  iSecondBaudrate = atoi(p_conf_value);
  return 0;
}

static int set_uart_sleep_after_dl(char* p_conf_name, char* p_conf_value,
                                   int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  uart_sleep_after_dl = atoi(p_conf_value);
  return 0;
}

static int set_enable_poke_controller(char* p_conf_name, char* p_conf_value,
                                      int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_poke_controller = atoi(p_conf_value);
  return 0;
}

static int set_send_boot_sleep_trigger(char* p_conf_name, char* p_conf_value,
                                       int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  send_boot_sleep_trigger = (atoi(p_conf_value) == 0) ? FALSE : TRUE;
  return 0;
}
#endif

static int set_Filename_cal_data(char* p_conf_name, char* p_conf_value,
                                 int param) {
  int len_p_conf_value = 0;
  UNUSED(p_conf_name);
  UNUSED(param);

  len_p_conf_value = strnlen(p_conf_value, MAX_PATH_LEN);
  if (len_p_conf_value < MAX_PATH_LEN) {
    strlcpy(pFilename_cal_data, p_conf_value, len_p_conf_value + 1);
  } else {
    VND_LOGE("String length too long, unable to process");
    VND_LOGE("Source length: %d and destination size: %d", len_p_conf_value,
             MAX_PATH_LEN - 1);
  }
  return 0;
}
static int set_vhal_trace_level(char* p_conf_name, char* p_conf_value,
                                int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  vhal_trace_level = atoi(p_conf_value);
  return 0;
}
static int set_enable_heartbeat_config(char* p_conf_name, char* p_conf_value,
                                       int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_heartbeat_config = (atoi(p_conf_value) == 0) ? FALSE : TRUE;
  return 0;
}

static int set_wakeup_enable_uart_low_config(char* p_conf_name,
                                             char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_enable_uart_low_config = (atoi(p_conf_value) == 0) ? FALSE : TRUE;
  return 0;
}

static int set_powerkey_gpio_pin(char* p_conf_name, char* p_conf_value,
                                 int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_gpio_config[wakeup_power_key].gpio_pin =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_powerkey_gpio_high_duration(char* p_conf_name,
                                           char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_gpio_config[wakeup_power_key].high_duration =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_powerkey_gpio_low_duration(char* p_conf_name, char* p_conf_value,
                                          int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_gpio_config[wakeup_power_key].low_duration =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_netflixkey_gpio_pin(char* p_conf_name, char* p_conf_value,
                                   int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_gpio_config[wakeup_netflix_key].gpio_pin =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_netflixkey_gpio_high_duration(char* p_conf_name,
                                             char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_gpio_config[wakeup_netflix_key].high_duration =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_netflixkey_gpio_low_duration(char* p_conf_name,
                                            char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_gpio_config[wakeup_netflix_key].low_duration =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_wakeup_adv_pattern(char* p_conf_name, char* p_conf_value,
                                  int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  if (wakeup_adv_config.length >= NXP_WAKEUP_ADV_PATTERN_LENGTH) {
    VND_LOGE("%s, invalid length:%d max limit %d", __func__,
             wakeup_adv_config.length, NXP_WAKEUP_ADV_PATTERN_LENGTH - 1);
    return -1;
  }
  wakeup_adv_config.adv_pattern[wakeup_adv_config.length] =
      (unsigned char)atoi(p_conf_value);
  wakeup_adv_config.length += 1;
  return 0;
}

static int set_wakeup_scan_type(char* p_conf_name, char* p_conf_value,
                                int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_scan_param_config.le_scan_type = (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_wakeup_scan_interval(char* p_conf_name, char* p_conf_value,
                                    int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_scan_param_config.interval = (unsigned short)atoi(p_conf_value);
  return 0;
}

static int set_wakeup_scan_window(char* p_conf_name, char* p_conf_value,
                                  int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_scan_param_config.window = (unsigned short)atoi(p_conf_value);
  return 0;
}

static int set_wakeup_scan_own_addr_type(char* p_conf_name, char* p_conf_value,
                                         int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_scan_param_config.own_addr_type = (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_wakeup_scan_filter_policy(char* p_conf_name, char* p_conf_value,
                                         int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_scan_param_config.scan_filter_policy =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

static int set_wakeup_local_heartbeat_timer_value(char* p_conf_name,
                                                  char* p_conf_value,
                                                  int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  wakeup_local_param_config.heartbeat_timer_value =
      (unsigned char)atoi(p_conf_value);
  return 0;
}

/*
 * Current supported entries and corresponding action functions
 */

static const conf_entry_t conf_table[] = {
    {"mchar_port", set_mchar_port, 0},
    {"mbt_port", set_mbt_port, 0},
    {"baudrate_bt", set_baudrate_bt, 0},
    {"baudrate_fw_init", set_baudrate_fw_init, 0},
    {"bd_address", set_bd_address_buf, 0},
    {"pFilename_fw_init_config_bin", set_pFilename_fw_init_config_bin, 0},
    {"ble_1m_power", set_ble_1m_power, 0},
    {"ble_2m_power", set_ble_2m_power, 0},
    {"bt_max_power_sel", set_bt_tx_power, 0},
    {"independent_reset_gpio_pin", set_independent_reset_gpio_pin, 0},
    {"oob_ir_host_gpio_pin", set_oob_ir_host_gpio_pin, 0},
    {"chardev_name", set_charddev_name, 0},
    {"independent_reset_mode", set_independent_reset_mode, 0},
    {"send_oob_ir_trigger", set_send_oob_ir_trigger, 0},
    {"enable_lpm", set_enable_lpm, 0},
    {"lpm_timeout", set_lpm_timeout, 0},
#ifdef UART_DOWNLOAD_FW
    {"enable_download_fw", set_enable_download_fw, 0},
    {"pFileName_image", set_pFileName_image, 0},
    {"pFileName_helper", set_pFileName_helper, 0},
    {"baudrate_dl_helper", set_baudrate_dl_helper, 0},
    {"baudrate_dl_image", set_baudrate_dl_image, 0},
    {"iSecondBaudrate", set_iSecondBaudrate, 0},
    {"uart_sleep_after_dl", set_uart_sleep_after_dl, 0},
    {"enable_poke_controller", set_enable_poke_controller, 0},
    {"send_boot_sleep_trigger", set_send_boot_sleep_trigger, 0},
#endif
    {"enable_pdn_recovery", set_enable_pdn_recovery, 0},
    {"pFilename_cal_data", set_Filename_cal_data, 0},
    {"vhal_trace_level", set_vhal_trace_level, 0},
    {"enable_sco_config", set_enable_sco_config, 0},
    {"use_controller_addr", set_use_controller_addr, 0},
    {"enable_heartbeat_config", set_enable_heartbeat_config, 0},
    {"wakeup_power_gpio_pin", set_powerkey_gpio_pin, 0},
    {"wakeup_enable_uart_low_config", set_wakeup_enable_uart_low_config, 0},
    {"wakeup_power_gpio_high_duration", set_powerkey_gpio_high_duration, 0},
    {"wakeup_power_gpio_low_duration", set_powerkey_gpio_low_duration, 0},
    {"wakeup_netflix_gpio_pin", set_netflixkey_gpio_pin, 0},
    {"wakeup_netflix_gpio_high_duration", set_netflixkey_gpio_high_duration, 0},
    {"wakeup_netflix_gpio_low_duration", set_netflixkey_gpio_low_duration, 0},
    {"wakeup_adv_pattern", set_wakeup_adv_pattern, 0},
    {"wakeup_scan_type", set_wakeup_scan_type, 0},
    {"wakeup_scan_interval", set_wakeup_scan_interval, 0},
    {"wakeup_scan_window", set_wakeup_scan_window, 0},
    {"wakeup_own_addr_type", set_wakeup_scan_own_addr_type, 0},
    {"wakeup_scan_filter_policy", set_wakeup_scan_filter_policy, 0},
    {"wakeup_local_heartbeat_timer_value",
     set_wakeup_local_heartbeat_timer_value, 0},
    {(const char*)NULL, NULL, 0}};

/*******************************************************************************
**
** Function        vnd_load_conf
**
** Description     Read conf file from mentioned p_path at run time and read
**                 one by one entry and call the corresponding config function
**
** Returns         None
**
*******************************************************************************/
static void vnd_load_conf(const char* p_path) {
  FILE* p_file;
  char* p_name;
  char* p_value;
  conf_entry_t* p_entry;
  char line[CONF_MAX_LINE_LEN + 1]; /* add 1 for \0 char */

  VND_LOGD("Attempt to load conf from %s", p_path);

  if ((p_file = fopen(p_path, "r")) != NULL) {
    /* read line by line */
    while (fgets(line, CONF_MAX_LINE_LEN + 1, p_file) != NULL) {
      if (line[0] == CONF_COMMENT) continue;

      p_name = strtok(line, CONF_DELIMITERS);

      if (NULL == p_name) {
        continue;
      }

      p_value = strtok(NULL, CONF_DELIMITERS);

      if (NULL == p_value) {
        ALOGW("vnd_load_conf: missing value for name: %s", p_name);
        continue;
      }

      p_entry = (conf_entry_t*)conf_table;

      while (p_entry->conf_entry != NULL) {
        if (strncmp(p_entry->conf_entry, (const char*)p_name,
                    MAX_CONF_PARA_LEN) == 0) {
          p_entry->p_action(p_name, p_value, p_entry->param);
          break;
        }

        p_entry++;
      }
    }

    fclose(p_file);
  } else {
    VND_LOGE("File open error at %s", p_path);
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
  }
}

/******************************************************************************
 **
 ** Function:        read_hci_event
 **
 ** Description:     Reads hci event.
 **
 ** Return Value:    0 is successful, -1 otherwise
 **
 *
 *****************************************************************************/

int read_hci_event(hci_event* evt_pkt, uint64_t retry_delay_ms,
                   uint32_t max_duration_ms) {
  int remain, r;
  int count;
  uint32_t total_duration = 0;

  /* The first byte identifies the packet type. For HCI event packets, it
   * should be 0x04, so we read until we get to the 0x04. */
  VND_LOGV("start read hci event 0x4");
  count = 0;
  while (fw_upload_GetBufferSize(mchar_fd) <
         HCI_EVENT_HEADER_SIZE + HCI_PACKET_TYPE_SIZE) {
    usleep(retry_delay_ms * 1000);
    total_duration += retry_delay_ms;
    if (total_duration >= max_duration_ms) {
      VND_LOGE("Read hci complete event failed timed out. Total_duration = %u",
               total_duration);
      return -1;
    }
  };
  r = read(mchar_fd, evt_pkt->raw_data,
           HCI_EVENT_HEADER_SIZE + HCI_PACKET_TYPE_SIZE);
  if (r <= 0) {
    VND_LOGV("read hci event 0x04 failed");
    VND_LOGV("Error %s (%d)", strerror(errno), errno);
  }
  if (evt_pkt->info.packet_type != HCI_PACKET_EVENT) {
    VND_LOGE("Invalid packet type(%02X) received", evt_pkt->info.packet_type);
    return -1;
  }
  /* Now we read the parameters. */
  VND_LOGV("start read hci event para");
  if (evt_pkt->info.para_len < HCI_EVENT_PAYLOAD_SIZE) {
    remain = evt_pkt->info.para_len;
  } else {
    remain = HCI_EVENT_PAYLOAD_SIZE;
    VND_LOGE("Payload size(%d) greater than capacity", evt_pkt->info.para_len);
  }
  while (fw_upload_GetBufferSize(mchar_fd) < (uint32_t)remain);
  while ((count) < remain) {
    r = read(mchar_fd, evt_pkt->info.payload + count, remain - (count));
    if (r <= 0) {
      VND_LOGE("read hci event para failed");
      VND_LOGE("Error: %s (%d)", strerror(errno), errno);
      return -1;
    }
    count += r;
  }
  return 0;
}

/******************************************************************************
 **
 ** Function:        check_hci_event_status
 **
 ** Description:     Parse evt_pkt for opcode.
 **
 ** Return Value:    0 is successful, -1 otherwise
 **
 *
 *****************************************************************************/
static int8_t check_hci_event_status(hci_event* evt_pkt, uint16_t opcode) {
  int ret = -1;
  uint8_t* ptr;
  uint16_t pkt_opcode;
  switch (evt_pkt->info.event_type) {
    case HCI_EVENT_COMMAND_COMPLETE:
      if (evt_pkt->info.para_len > HCI_EVT_PYLD_STATUS_IDX) {
        ptr = &evt_pkt->info.payload[HCI_EVT_PYLD_OPCODE_IDX];
        STREAM_TO_UINT16(pkt_opcode, ptr);
        VND_LOGD("Reply received for command 0x%04hX (%s) status 0x%02x",
                 pkt_opcode, hw_bt_cmd_to_str(pkt_opcode),
                 evt_pkt->info.payload[HCI_EVT_PYLD_STATUS_IDX]);
        if (evt_pkt->info.payload[HCI_EVT_PYLD_STATUS_IDX] != 0) {
          VND_LOGE(
              "Error status received for command 0x%04hX (%s) status 0x%02x",
              pkt_opcode, hw_bt_cmd_to_str(pkt_opcode),
              evt_pkt->info.payload[HCI_EVT_PYLD_STATUS_IDX]);
        }
        if (pkt_opcode == opcode) ret = 0;
      } else {
        VND_LOGE("Unexpected packet length received. Event type:%02x Len:%02x",
                 evt_pkt->info.event_type, evt_pkt->info.para_len);
      }
      break;
    case HCI_EVENT_HARDWARE_ERROR:
      VND_LOGE("Hardware error event(%02x) received ",
               evt_pkt->info.event_type);
      VND_LOGE("Payload length received %02x", evt_pkt->info.para_len);
      if (evt_pkt->info.para_len > 0) {
        VND_LOGE("Hardware error code: %02x", evt_pkt->info.payload[0]);
      }
      break;
    default:
      VND_LOGE("Invalid Event type %02x received ", evt_pkt->info.event_type);
  }
  VND_LOGV("Packet dump");
  for (int i = 0; i < evt_pkt->info.para_len + HCI_PACKET_TYPE_SIZE +
                          HCI_EVENT_HEADER_SIZE;
       i++) {
    VND_LOGV("Packet[%d]= %02x", i, evt_pkt->raw_data[i]);
  }
  if (evt_pkt->info.event_type == HCI_EVENT_HARDWARE_ERROR) {
    /*BLUETOOTH CORE SPECIFICATION Version 5.4 | Vol 4, Part A Point 4*/
    ret = send_hci_reset();
  }
  return ret;
}

/******************************************************************************
 **
 ** Function:        read_hci_event_status
 **
 ** Description:     Polls HCI_EVENT with opcode till max_duration_ms and checks
 *                   its status.
 **
 ** Return Value:    0 is successful, -1 otherwise
 **
 *
 *****************************************************************************/
static int8_t read_hci_event_status(int16_t opcode, uint64_t retry_delay_ms,
                                    uint64_t max_duration_ms) {
  int8_t ret = -1;
  hci_event evt_pkt;
  uint64_t start_ms = fw_upload_GetTime();
  uint64_t cost_ms;
  uint64_t remaining_time_ms = max_duration_ms;
  VND_LOGD("Reading %s event", hw_bt_cmd_to_str(opcode));
  while (((cost_ms = fw_upload_GetTime() - start_ms) < max_duration_ms) &&
         (read_hci_event(&evt_pkt, retry_delay_ms, remaining_time_ms) == 0)) {
    if ((ret = check_hci_event_status(&evt_pkt, opcode)) == 0) {
      break;
    }
    remaining_time_ms = max_duration_ms - (fw_upload_GetTime() - start_ms);
  }
  if (cost_ms >= max_duration_ms) {
    VND_LOGE("Read hci complete event failed, timed out at: %u",
             (uint32_t)cost_ms);
  }
  return ret;
}

/*******************************************************************************
**
** Function        send_hci_reset
**
** Description     Send HCI reset command over raw UART.
**
** Returns         0 on success, -1 on failure
**
*******************************************************************************/
static int send_hci_reset(void) {
  int ret = -1;
  if (hw_bt_send_hci_cmd_raw(HCI_CMD_NXP_RESET) != 0) {
    VND_LOGE("Failed to write reset command");
  } else if ((read_hci_event_status(HCI_CMD_NXP_RESET, POLL_CONFIG_UART_MS,
                                    POLL_MAX_TIMEOUT_MS) != 0)) {
    VND_LOGE("Failed to read HCI RESET CMD response!");
  } else {
    VND_LOGD("HCI reset completed successfully");
    ret = 0;
  }
  return ret;
}

void set_prop_int32(char* name, int value) {
  char init_value[PROPERTY_VALUE_MAX];
  int ret;

  sprintf(init_value, "%d", value);
  ret = property_set(name, init_value);
  if (ret < 0) {
    VND_LOGE("set_prop_int32 failed: %d", ret);
  } else {
    VND_LOGD("set_prop_int32: %s = %d", name, value);
  }
  return;
}

int get_prop_int32(char* name) {
  int ret;

  ret = property_get_int32(name, -1);
  VND_LOGD("get_prop_int32: %s = %d", name, ret);
  if (ret < 0) {
    return 0;
  }
  return ret;
}

/******************************************************************************
 **
 ** Function:        uart_speed
 **
 ** Description:     Return the baud rate corresponding to the frequency.
 **
 ** Return Value:    Baudrate
 *
 *****************************************************************************/

static int32 uart_speed(int32 s) {
  switch (s) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 3000000:
      return B3000000;
    default:
      return B0;
  }
}
/******************************************************************************
 **
 ** Function:        uart_speed_translate
 **
 ** Description:     Return the frequency to corresponding baud rate.
 **
 ** Return Value:    Baudrate
 *
 *****************************************************************************/

static int32 uart_speed_translate(int32 s) {
  switch (s) {
    case B9600:
      return 9600;
    case B19200:
      return 19200;
    case B38400:
      return 38400;
    case B57600:
      return 57600;
    case B115200:
      return 115200;
    case B230400:
      return 230400;
    case B460800:
      return 460800;
    case B500000:
      return 500000;
    case B576000:
      return 576000;
    case B921600:
      return 921600;
    case B1000000:
      return 1000000;
    case B1152000:
      return 1152000;
    case B1500000:
      return 1500000;
    case B3000000:
      return 3000000;
    default:
      return 0;
  }
}

/******************************************************************************
 *
 ** Function            uart_get_speed
 **
 ** Description         Get the last baud rate speed.

 ** Return Value:       Value of last baud rate
 **
 *****************************************************************************/

static int32 uart_get_speed(struct termios* ti) {
  int32 speed = 0;
  speed = (int32)cfgetospeed(ti);
  return (uart_speed_translate(speed));
}
/******************************************************************************
 *
 ** Function            uart_set_speed
 **
 ** Description         Set the baud rate speed.

 ** Return Value:       0 On success else -1

 **
 *****************************************************************************/

static int32 uart_set_speed(int32 fd, struct termios* ti, int32 speed) {
  if (cfsetospeed(ti, uart_speed(speed)) < 0) {
    VND_LOGE("Set O speed failed!");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    return -1;
  }

  if (cfsetispeed(ti, uart_speed(speed)) < 0) {
    VND_LOGE("Set I speed failed!");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    return -1;
  }

  if (tcsetattr(fd, TCSANOW, ti) < 0) {
    VND_LOGE("Set Attr speed failed!");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    return -1;
  }
  VND_LOGD("Host baudrate set to %d", speed);
  return 0;
}

/******************************************************************************
 **
 ** Name:               init_uart
 **
 ** Description         Initialize UART.
 **
 ** Return Value        Valid fd on success
 **
 *****************************************************************************/

int32 init_uart(int8* dev, int32 dwBaudRate, uint8 ucFlowCtrl) {
  int32 fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    VND_LOGE("Can't open serial port");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    return -1;
  }

  tcflush(fd, TCIOFLUSH);

  if (tcgetattr(fd, &ti) < 0) {
    VND_LOGE("Can't get port settings");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    close(fd);
    return -1;
  }

  cfmakeraw(&ti);
#ifdef FW_LOADER_V2
  ti.c_cflag |= CLOCAL | CREAD;
#else
  ti.c_cflag |= CLOCAL;
#endif

  /* Set 1 stop bit & no parity (8-bit data already handled by cfmakeraw) */
  ti.c_cflag &= ~(CSTOPB | PARENB);

  if (ucFlowCtrl) {
    ti.c_cflag |= CRTSCTS;
  } else {
    ti.c_cflag &= ~CRTSCTS;
  }

  /*FOR READS: set timeout time w/ no minimum characters needed
                         (since we read only 1 at at time...)          */
  ti.c_cc[VMIN] = 0;
  ti.c_cc[VTIME] = TIMEOUT_SEC * 10;

  if (tcsetattr(fd, TCSANOW, &ti) < 0) {
    VND_LOGE("Can't set port settings");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  if (independent_reset_mode == IR_MODE_INBAND_VSC) {
    last_baudrate = uart_get_speed(&ti);
    VND_LOGD("Last baud rate = %d", last_baudrate);
  }
  /* Set actual baudrate */
  if (uart_set_speed(fd, &ti, dwBaudRate) < 0) {
    VND_LOGE("Can't set baud rate");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    close(fd);
    return -1;
  }

  return fd;
}

/*******************************************************************************
**
** Function        uart_init_open
**
** Description     Open the serial port with the given configuration
**
** Returns         device fd
**
*******************************************************************************/

static int uart_init_open(int8* dev, int32 dwBaudRate, uint8 ucFlowCtrl) {
  int fd = 0, num = 0;
  do {
    fd = init_uart(dev, dwBaudRate, ucFlowCtrl);
    if (fd < 0) {
      num++;
      if (num >= 8) {
        VND_LOGE("exceed max retry count, return error");
        return -1;
      } else {
        VND_LOGW("open uart port %s failed fd: %d, retrying", dev, fd);
        VND_LOGW("Error: %s (%d)", strerror(errno), errno);
        usleep(50 * 1000);
        continue;
      }
    }
  } while (fd < 0);

  return fd;
}
#ifdef UART_DOWNLOAD_FW
/*******************************************************************************
**
** Function        detect_and_download_fw
**
** Description     Start firmware download process if fw is not already download
**
** Returns         0 : FW is ready
**                 1 : FW not ready
**
*******************************************************************************/

static int detect_and_download_fw() {
  int download_ret = 1;
#ifndef FW_LOADER_V2
  init_crc8();
#endif
/* force download only when header is received */
#ifdef FW_LOADER_V2
  if (bt_vnd_mrvl_check_fw_status_v2()) {
#else
  if (bt_vnd_mrvl_check_fw_status()) {
#endif
#ifdef UART_DOWNLOAD_FW
    if ((send_boot_sleep_trigger == TRUE) &&
        (!get_prop_int32(PROP_BLUETOOTH_BOOT_SLEEP_TRIGGER))) {
      VND_LOGD("setting PROP_BLUETOOTH_BOOT_SLEEP_TRIGGER to 1");
      set_prop_int32(PROP_BLUETOOTH_BOOT_SLEEP_TRIGGER, 1);
    }
#endif
    if (download_helper) {
#ifdef FW_LOADER_V2
      download_ret = bt_vnd_mrvl_download_fw_v2(mchar_port, baudrate_dl_helper,
                                                pFileName_helper);
#else
      download_ret = bt_vnd_mrvl_download_fw(mchar_port, baudrate_dl_helper,
                                             pFileName_helper, iSecondBaudrate);
#endif
      if (download_ret != 0) {
        VND_LOGE("helper download failed");
        goto done;
      }

      usleep(50000);
      /* flush additional A5 header if any */
      tcflush(mchar_fd, TCIFLUSH);

      /* close and open the port and set baud rate to baudrate_dl_image */
      close(mchar_fd);
      mchar_fd = uart_init_open(mchar_port, 3000000, 1);
      if (mchar_fd < 0) {
        download_ret = 1;
        goto done;
      }
      usleep(20000);
      tcflush(mchar_fd, TCIOFLUSH);
    }

    /* download fw image */
    if (auto_select_fw_name == TRUE) {
      fw_loader_get_default_fw_name(pFileName_image, sizeof(pFileName_image));
    }
#ifdef FW_LOADER_V2
    download_ret = bt_vnd_mrvl_download_fw_v2(mchar_port, baudrate_dl_image,
                                              pFileName_image);
#else
    download_ret = bt_vnd_mrvl_download_fw(mchar_port, baudrate_dl_image,
                                           pFileName_image, iSecondBaudrate);
#endif
    if (download_ret != 0) {
      VND_LOGE("fw download failed");
      goto done;
    }

    tcflush(mchar_fd, TCIFLUSH);
    if (uart_sleep_after_dl) usleep(uart_sleep_after_dl * 1000);
  }
done:
  return download_ret;
}
#endif

/*******************************************************************************
**
** Function        config_uart
**
** Description     Configure uart w.r.t different fw_init_baudrate
**                 and bt_baudrate and send relevant HCI command to confirm
                   uart configuration
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/

static int config_uart() {
  if (baudrate_fw_init != baudrate_bt) {
    /* set baud rate to baudrate_fw_init */
    if (uart_set_speed(mchar_fd, &ti, baudrate_fw_init) < 0) {
      VND_LOGE("Can't set baud rate");
      return -1;
    }
    if (send_hci_reset() != 0) {
      return -1;
    }
    /* Set bt chip Baud rate CMD */
    if ((baudrate_bt == 3000000) || (baudrate_bt == 115200)) {
      VND_LOGD("set fw baudrate as %d", baudrate_bt);
      if (hw_send_change_baudrate_raw(baudrate_bt)) {
        VND_LOGE("Failed to write set baud rate command");
        return -1;
      }
      VND_LOGV("start read hci event");
      if (read_hci_event_status(HCI_CMD_NXP_CHANGE_BAUDRATE,
                                POLL_CONFIG_UART_MS, POLL_MAX_TIMEOUT_MS) != 0) {
        VND_LOGE("Failed to read set baud rate command response! ");
        return -1;
      }
      VND_LOGD("Controller Baudrate changed successfully to %d", baudrate_bt);
    } else {
      VND_LOGD("Unsupported baudrate_bt %d", baudrate_bt);
    }

    usleep(60000); /* Sleep to allow baud rate setting to happen in FW */

    tcflush(mchar_fd, TCIOFLUSH);
    if (uart_set_speed(mchar_fd, &ti, baudrate_bt)) {
      VND_LOGE("Failed to  set baud rate ");
      return -1;
    }
    ti.c_cflag |= CRTSCTS;
    if (tcsetattr(mchar_fd, TCSANOW, &ti) < 0) {
      VND_LOGE("Set Flow Control failed!");
      VND_LOGE("Error: %s (%d)", strerror(errno), errno);
      return -1;
    }
    tcflush(mchar_fd, TCIOFLUSH);
  } else {
    /* set host uart speed according to baudrate_bt */
    VND_LOGD("Set host baud rate as %d", baudrate_bt);
    tcflush(mchar_fd, TCIOFLUSH);

    /* Close and open the port as setting baudrate to baudrate_bt */
    close(mchar_fd);
    mchar_fd = uart_init_open(mchar_port, baudrate_bt, 1);
    if (mchar_fd < 0) {
      return -1;
    }
    usleep(20000);
    tcflush(mchar_fd, TCIOFLUSH);
  }

  usleep(20 * 1000);
  return 0;
}
static bool bt_vnd_is_rfkill_disabled(void) {
  char value[PROPERTY_VALUE_MAX] = {'\0'};
  bool ret = FALSE;
  property_get("ro.rfkilldisabled", value, "0");
  VND_LOGD("ro.rfkilldisabled %c", value[0]);
  if ((int)value[0] == '1') {
    ret = TRUE;
  }
  return ret;
}

static int bt_vnd_init_rfkill() {
  char path[64];
  char buf[16];
  int fd, sz, id;
  sz = -1;  // initial
  if (bt_vnd_is_rfkill_disabled()) {
    return -1;
  }

  for (id = 0;; id++) {
    snprintf(path, sizeof(path), "/sys/class/rfkill/rfkill%d/type", id);
    fd = open(path, O_RDONLY);
    if (fd < 0) {
      VND_LOGE("open(%s) failed: %s (%d)", path, strerror(errno), errno);
      break;
    }
    sz = read(fd, &buf, sizeof(buf));
    if (sz < 0) {
      VND_LOGE("read failed: %s (%d)", strerror(errno), errno);
    }
    close(fd);
    if (sz >= 9 && memcmp(buf, "bluetooth", 9) == 0) {
      rfkill_id = id;
      break;
    }
  }

  if (rfkill_id == -1) {
    VND_LOGE("Bluetooth rfkill not found");
    return -1;
  } else {
    asprintf(&rfkill_state_path, "/sys/class/rfkill/rfkill%d/state", rfkill_id);
    VND_LOGD("rfkill_state_path set to %s ", rfkill_state_path);
    return 0;
  }
}

int bt_vnd_set_bluetooth_power(BOOLEAN bt_turn_on) {
  int sz;
  int fd = -1;
  int ret = -1;
  char buffer = bt_turn_on ? '1' : '0';
  /* check if we have rfkill interface */
  if (bt_vnd_is_rfkill_disabled()) {
    VND_LOGD("rfkill disabled, ignoring bluetooth power %s",
             bt_turn_on ? "ON" : "OFF");
    ret = 0;
    goto done;
  }

  if (rfkill_id == -1) {
    if (bt_vnd_init_rfkill()) {
      goto done;
    }
  }

  fd = open(rfkill_state_path, O_WRONLY);
  if (fd < 0) {
    VND_LOGE("open(%s) for write failed: %s (%d)", rfkill_state_path,
             strerror(errno), errno);
    goto done;
  }
  sz = write(fd, &buffer, 1);
  if (sz < 0) {
    VND_LOGE("write(%s) failed: %s (%d)", rfkill_state_path, strerror(errno),
             errno);
  }
  ret = 0;

done:
  if (fd >= 0) {
    close(fd);
  }
  return ret;
}

void bt_vnd_gpio_configuration(int value) {
  struct gpiohandle_request req;
  struct gpiohandle_data data;
  int fd = 0, ret = 0;

  /* Open device: gpiochip0 for GPIO bank A */
  fd = open(chrdev_name, 0);
  if (fd == -1) {
    VND_LOGW("Failed to open %s %s(%d)", chrdev_name, strerror(errno), errno);
    return;
  }
  /* Request GPIO Direction line as out */
  req.lineoffsets[0] = ir_host_gpio_pin;
  req.flags = GPIOHANDLE_REQUEST_OUTPUT;
  req.lines = 1;
  ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);

  if (ret == -1) {
    VND_LOGE("%s Failed to issue GET LINEHANDLE IOCTL(%d)", strerror(errno),
             errno);
    close(fd);
    return;
  }
  if (close(fd) == -1) {
    VND_LOGE("Failed to close GPIO character device file");
    return;
  }
  /* Get the value of line */
  memset(&data, 0, sizeof(data));
  ret = ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
  if (ret == -1) {
    close(req.fd);
    VND_LOGE("%s failed to issue GET LINEHANDLE(%d)", strerror(errno), errno);
    return;
  }
  VND_LOGD("Current Value of line=: %d", data.values[0]);

  /* Set the requested value to the line*/
  data.values[0] = value;
  ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
  if (ret == -1) {
    close(req.fd);
    VND_LOGE("%s failed to issue SET LINEHANDLE", strerror(errno));
    return;
  }
  ret = ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
  if (ret < 0) {
    VND_LOGE("ioctl error: %s (%d)", strerror(errno), errno);
  }
  VND_LOGD("Updated Value of Line:= %d", data.values[0]);

  /*  release line */
  ret = close(req.fd);
  if (ret == -1) {
    VND_LOGW("%s Failed to close GPIO LINEHANDLE device file", strerror(errno));
    return;
  }
}

/*******************************************************************************
**
** Function        bt_vnd_send_inband_ir
**
** Description     Send Inband Independent Reset to Controller.
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
static int bt_vnd_send_inband_ir(int32_t baudrate) {
  int32_t _last_baudrate = last_baudrate;

  if (get_prop_int32(PROP_BLUETOOTH_INBAND_CONFIGURED) == 1) {
    close(mchar_fd);
    mchar_fd = uart_init_open(mchar_port, _last_baudrate, 1);
    if (mchar_fd <= 0) {
      VND_LOGE("Can't set last baud rate %d", _last_baudrate);
      return -1;
    } else {
      VND_LOGD("Baud rate changed from %d to %d with flow control enabled",
               baudrate, _last_baudrate);
    }
    tcflush(mchar_fd, TCIOFLUSH);
    if (hw_bt_send_hci_cmd_raw(HCI_CMD_INBAND_RESET)) {
      VND_LOGE("Failed to write in-band reset command ");
      VND_LOGE("Error: %s (%d)", strerror(errno), errno);
      return -1;
    } else {
      VND_LOGV("start read hci event");
      if (read_hci_event_status(HCI_CMD_INBAND_RESET, POLL_RETRY_TIMEOUT_MS,
                                POLL_MAX_TIMEOUT_MS) != 0) {
        VND_LOGE("Failed to read Inband reset response");
        return -1;
      }
      VND_LOGD("=========Inband IR trigger sent successfully=======");
    }
    close(mchar_fd);
    mchar_fd = uart_init_open(mchar_port, baudrate, 0);
    if (mchar_fd <= 0) {
      VND_LOGE("Can't set last baud rate %d", _last_baudrate);
      return -1;
    } else {
      VND_LOGD("Baud rate changed from %d to %d with flow control disabled",
               _last_baudrate, baudrate);
    }
  }
  return 0;
}

/*******************************************************************************
**
** Function        send_exit_heartbeat_mode
**
** Description     Exit heartbeat mode during bluetooth disabling procedure
**
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
static void send_exit_heartbeat_mode(void) {
  hci_event evt_pkt;
  VND_LOGD("Start to send exit heartbeat cmd ...\n");
  memset(&evt_pkt, 0x00, sizeof(evt_pkt));

  if (hw_bt_send_wakeup_disable_raw()) {
    VND_LOGD("Failed to write exit heartbeat command \n");
    return;
  }
  if (read_hci_event(&evt_pkt, POLL_RETRY_TIMEOUT_MS, POLL_CONFIG_UART_MS) ==
          0 &&
      check_hci_event_status(&evt_pkt, HCI_CMD_NXP_BLE_WAKEUP) == 0 &&
      evt_pkt.info.para_len > HCI_EVT_PYLD_SUBCODE_IDX &&
      evt_pkt.info.payload[HCI_EVT_PYLD_SUBCODE_IDX] ==
          HCI_CMD_OTT_SUB_WAKEUP_EXIT_HEARTBEATS) {
    VND_LOGD("Exit heartbeat mode cmd sent successfully\n");
  } else {
    VND_LOGD("Failed to read exit heartbeat cmd response! \n");
  }
  return;
}

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int bt_vnd_init(const bt_vendor_callbacks_t* p_cb,
                       unsigned char* local_bdaddr) {
  vnd_cb = p_cb;
  if (vnd_cb == NULL) {
    VND_LOGE("vnd_cb is NULL");
  }
  ALOGI("bt_vnd_init --- BT Vendor HAL Ver: %s ---", BT_HAL_VERSION);
  vnd_load_conf(VENDOR_LIB_CONF_FILE);
  VND_LOGI("Max supported Log Level: %d", VHAL_LOG_LEVEL);
  VND_LOGI("Selected Log Level:%d", vhal_trace_level);
  ALOGI(
      "BT_VHAL Log Level:%d",
      (vhal_trace_level <= VHAL_LOG_LEVEL ? vhal_trace_level : VHAL_LOG_LEVEL));
  if (local_bdaddr) {
    bdaddr = (unsigned char*)malloc(BD_ADDR_LEN);
    if (bdaddr != NULL) {
      memcpy(bdaddr, local_bdaddr, 6);
      VND_LOGD(
          "BD address received from stack "
          "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
          bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
    }
  }
  return 0;
}

/** Requested operations */
static int bt_vnd_op(bt_vendor_opcode_t opcode, void* param) {
  int ret = 0;
  int local_st = 0;

  VND_LOGD("opcode = %d", opcode);
  switch (opcode) {
    case BT_VND_OP_POWER_CTRL: {
      int* state = (int*)param;

      if (*state == BT_VND_PWR_OFF) {
        VND_LOGD("power off --------------------------------------*");
        if (enable_heartbeat_config == TRUE) {
          wakeup_kill_heartbeat_thread();
        }
        if (adapterState == BT_VND_PWR_ON) {
          VND_LOGD("BT adapter switches from ON to OFF .. ");
          adapterState = BT_VND_PWR_OFF;
        }
      } else if (*state == BT_VND_PWR_ON) {
        VND_LOGD("power on --------------------------------------");
        if (independent_reset_mode == IR_MODE_INBAND_VSC) {
          VND_LOGD("Reset the download status for Inband IR ");
          set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
        }
        adapterState = BT_VND_PWR_ON;
        if (send_oob_ir_trigger == IR_TRIGGER_RFKILL) {
          bt_vnd_set_bluetooth_power(FALSE);
          usleep(5000);
          bt_vnd_set_bluetooth_power(TRUE);
          set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
        }
        if (send_oob_ir_trigger == IR_TRIGGER_GPIO) {
          set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);

          VND_LOGD("-------------- Setting GPIO LOW -----------------");
          bt_vnd_gpio_configuration(FALSE);
          usleep(1000);
          VND_LOGD("---------------- Setting GPIO HIGH ----------------");
          bt_vnd_gpio_configuration(TRUE);
        }
      }
    } break;
    case BT_VND_OP_FW_CFG:
      hw_config_start();
      break;

    case BT_VND_OP_SCO_CFG:
      if (vnd_cb) {
        vnd_cb->scocfg_cb(ret);
      }
      break;
    case BT_VND_OP_USERIAL_OPEN: {
      VND_LOGD("open serial port --------------------------------------");
      int(*fd_array)[] = (int(*)[])param;
      int idx;
      int bluetooth_opened;
      int num = 0;
      int32_t baudrate = 0;
      if (is_uart_port) {
        VND_LOGD("baudrate_bt %d", baudrate_bt);
        VND_LOGD("baudrate_fw_init %d", baudrate_fw_init);
#ifdef UART_DOWNLOAD_FW
        if (enable_download_fw) {
          VND_LOGD("download_helper %d", download_helper);
          VND_LOGD("baudrate_dl_helper %d", baudrate_dl_helper);
          VND_LOGD("baudrate_dl_image %d", baudrate_dl_image);
          VND_LOGD("pFileName_helper %s", pFileName_helper);
          VND_LOGD("pFileName_image %s", pFileName_image);
          VND_LOGD("iSecondBaudrate %d", iSecondBaudrate);
          VND_LOGD("enable_download_fw %d", enable_download_fw);
          VND_LOGD("uart_sleep_after_dl %d", uart_sleep_after_dl);
          VND_LOGD("independent_reset_mode %d", independent_reset_mode);
          VND_LOGD("send_oob_ir_trigger %d", send_oob_ir_trigger);
          VND_LOGD("independent_reset_gpio_pin %d", independent_reset_gpio_pin);
          VND_LOGD("ir_host_gpio_pin %d", ir_host_gpio_pin);
          VND_LOGD("chrdev_name %s", chrdev_name);
          VND_LOGD("send_boot_sleep_trigger %d", send_boot_sleep_trigger);
          VND_LOGD("enable_pdn_recovery %d", enable_pdn_recovery);
          VND_LOGD("enable_lpm %d", enable_lpm);
          VND_LOGD("use_controller_addr %d", use_controller_addr);
          VND_LOGD("bt_max_power_sel %d", bt_max_power_sel);
        }
#endif
      }

      if (is_uart_port) {
        /* ensure libbt can talk to the driver, only need open port once */
        if (get_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED))
          mchar_fd = uart_init_open(mchar_port, baudrate_bt, 1);
        else {
#ifdef UART_DOWNLOAD_FW
          if (enable_download_fw) {
            /* if define micro UART_DOWNLOAD_FW, then open uart must with
               baudrate 115200,
               since libbt can only communicate with bootloader with baudrate
               115200*/
            /* for 9098 helper is not need, so baudrate_dl_image is 115200, and
               iSecondBaudrate is true
               to set baudrate to 3000000 before download FW*/
            baudrate =
                (download_helper) ? baudrate_dl_helper : baudrate_dl_image;
          } else {
            baudrate = baudrate_fw_init;
          }
#else
          baudrate = baudrate_fw_init;
#endif
#ifdef UART_DOWNLOAD_FW
          if (send_boot_sleep_trigger &&
              !get_prop_int32(PROP_BLUETOOTH_BOOT_SLEEP_TRIGGER)) {
            VND_LOGD("boot sleep trigger is enabled and its first boot");
            mchar_fd = uart_init_open(mchar_port, baudrate, 1);
            close(mchar_fd);
          }
#endif
          mchar_fd = uart_init_open(mchar_port, baudrate, 0);
          if ((independent_reset_mode == IR_MODE_INBAND_VSC) &&
              (mchar_fd > 0)) {
            if (bt_vnd_send_inband_ir(baudrate) != 0) {
              return -1;
            }
          }
        }
        if (mchar_fd > 0) {
          VND_LOGI("open uart port successfully, fd=%d, mchar_port=%s",
                   mchar_fd, mchar_port);
        } else {
          VND_LOGE("open UART bt port %s failed fd: %d", mchar_port, mchar_fd);
          return -1;
        }
        bluetooth_opened = get_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED);
#ifdef UART_DOWNLOAD_FW
        if (enable_download_fw &&
            !get_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED)) {
          if (detect_and_download_fw()) {
            VND_LOGE("detect_and_download_fw failed");
            set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
            if (enable_pdn_recovery == TRUE) {
              int init_attempted =
                  get_prop_int32(PROP_BLUETOOTH_INIT_ATTEMPTED);
              init_attempted = (init_attempted == -1) ? 0 : init_attempted;
              if (++init_attempted >= PDN_RECOVERY_THRESHOLD) {
                VND_LOGE("%s: %s(%d) > %d, Triggering PDn recovery.\n",
                         __FUNCTION__, PROP_BLUETOOTH_INIT_ATTEMPTED,
                         init_attempted, PDN_RECOVERY_THRESHOLD);
                set_prop_int32(PROP_VENDOR_TRIGGER_PDN, 1);
                set_prop_int32(PROP_BLUETOOTH_INIT_ATTEMPTED, 0);
              } else {
                set_prop_int32(PROP_BLUETOOTH_INIT_ATTEMPTED, init_attempted);
                ALOGI("%s:%d\n", PROP_VENDOR_TRIGGER_PDN, init_attempted);
              }
            }
            return -1;
          }
        } else {
          ti.c_cflag |= CRTSCTS;
          if (tcsetattr(mchar_fd, TCSANOW, &ti) < 0) {
            VND_LOGE("Set Flow Control failed!");
            VND_LOGE("Error: %s (%d)", strerror(errno), errno);
            return -1;
          }
          tcflush(mchar_fd, TCIOFLUSH);
        }
#else
        ti.c_cflag |= CRTSCTS;
        if (tcsetattr(mchar_fd, TCSANOW, &ti) < 0) {
          VND_LOGE("Set Flow Control failed!");
          VND_LOGE("Error: %s (%d)", strerror(errno), errno);
          return -1;
        }
        tcflush(mchar_fd, TCIOFLUSH);
#endif
        if (!bluetooth_opened) {
#ifdef UART_DOWNLOAD_FW
          if (!enable_download_fw)
#endif
          {
            /*NXP Bluetooth use combo firmware which is loaded at wifi driver
            probe.
            This function will wait to make sure basic client netdev is created
            */
            int count =
                (POLL_DRIVER_MAX_TIME_MS * 1000) / POLL_DRIVER_DURATION_US;
            FILE* fd;

            while (count-- > 0) {
              if ((fd = fopen("/sys/class/net/wlan0", "r")) != NULL) {
                VND_LOGD("Error: %s (%d)", strerror(errno), errno);
                fclose(fd);
                break;
              }
              usleep(POLL_DRIVER_DURATION_US);
            }
          }

          if (config_uart()) {
            VND_LOGE("config_uart failed");
            set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
            return -1;
          }
        }
      } else {
        do {
          mchar_fd = open(mbt_port, O_RDWR | O_NOCTTY);
          if (mchar_fd < 0) {
            num++;
            if (num >= 8) {
              VND_LOGE("exceed max retry count, return error");
              return -1;
            } else {
              VND_LOGW("open USB/SD port %s failed fd: %d, retrying", mbt_port,
                       mchar_fd);
              VND_LOGW("Error: %s (%d)", strerror(errno), errno);
              sleep(1);
              continue;
            }
          } else {
            VND_LOGI("open USB or SD port successfully, fd=%d, mbt_port=%s",
                     mchar_fd, mbt_port);
          }
        } while (mchar_fd < 0);
      }

      for (idx = 0; idx < CH_MAX; idx++) {
        (*fd_array)[idx] = mchar_fd;
        ret = 1;
      }
      if (enable_pdn_recovery == TRUE) {
        // Reset PROP_BLUETOOTH_INIT_ATTEMPTED as init is successful
        set_prop_int32(PROP_BLUETOOTH_INIT_ATTEMPTED, 0);
      }
      VND_LOGD("open serial port over --------------------------------------");
    } break;
    case BT_VND_OP_USERIAL_CLOSE:

      if (enable_heartbeat_config == TRUE) {
        send_exit_heartbeat_mode();
      }
      send_hci_reset();
      /* mBtChar port is blocked on read. Release the port before we close it */
      if (is_uart_port) {
        if (mchar_fd) {
          tcflush(mchar_fd, TCIOFLUSH);
          close(mchar_fd);
          mchar_fd = 0;
        }
      } else {
        if (ioctl(mchar_fd, MBTCHAR_IOCTL_RELEASE, &local_st) < 0) {
          VND_LOGE("ioctl error: %s (%d)", strerror(errno), errno);
        }
        /* Give it sometime before we close the mbtchar */
        usleep(1000);
        if (mchar_fd) {
          if (close(mchar_fd) < 0) {
            VND_LOGE("close serial port failed!");
            VND_LOGE("Error: %s (%d)", strerror(errno), errno);
            ret = -1;
          }
        }
      }
      break;
    case BT_VND_OP_GET_LPM_IDLE_TIMEOUT: {
      uint32_t* timeout_ms = (uint32_t*)param;
      *timeout_ms = enable_lpm ? lpm_timeout_ms : 0;
      VND_LOGI("LPM timeout = %d", *timeout_ms);
    } break;
    case BT_VND_OP_LPM_SET_MODE:
      if (enable_lpm == TRUE) {
        uint8_t* lpm_mode = (uint8_t*)param;
        if (*lpm_mode == BT_VND_LPM_ENABLE) {
          VND_LOGI("Enable LPM mode");
          ret = hw_bt_configure_lpm(BT_SET_SLEEP_MODE);
        } else {
          VND_LOGI("Disable LPM mode");
          ret = hw_bt_configure_lpm(BT_SET_FULL_POWER_MODE);
        }
      }
      if (vnd_cb) {
        if (ret == 0) {
          vnd_cb->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
        } else {
          vnd_cb->lpm_cb(BT_VND_OP_RESULT_FAIL);
        }
      }
      break;
    case BT_VND_OP_LPM_WAKE_SET_STATE:
      if (lpm_configured == TRUE) {
        int status;
        uint8_t* wake_state = (uint8_t*)param;
        if (*wake_state == BT_VND_LPM_WAKE_ASSERT) {
          VND_LOGI("LPM: Wakeup BT Device");
          status = ioctl(mchar_fd, TIOCCBRK);
          VND_LOGI("Assert Status:%d\n", status);
        } else {
          VND_LOGI("LPM: Allow BT Device to sleep");
          status = ioctl(mchar_fd, TIOCSBRK);
          VND_LOGI("Deassert Status:%d\n", status);
        }
        if (status < 0) {
          VND_LOGE("LPM toggle Error: %s (%d)", strerror(errno), errno);
        }
      }
      break;
    default:
      ret = -1;
      break;
  }
  return ret;
}

/** Closes the interface */
static void bt_vnd_cleanup(void) {
  VND_LOGD("cleanup ...");
  vnd_cb = NULL;
  if (bdaddr) {
    free(bdaddr);
    bdaddr = NULL;
  }
}

/** Entry point of DLib */
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    bt_vnd_init,
    bt_vnd_op,
    bt_vnd_cleanup,
};
