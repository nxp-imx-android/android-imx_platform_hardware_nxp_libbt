/******************************************************************************
 *  Copyright 2012 The Android Open Source Project
 *  Portions copyright (C) 2009-2012 Broadcom Corporation
 *  Portions copyright 2012-2013, 2015, 2018-2021 NXP
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
 *  Description:   NXP vendor specific library implementation
 *
 ******************************************************************************/
#define LOG_TAG "bt-vnd-nxp"
#include <ctype.h>
#include <cutils/properties.h>
#include <errno.h>
#include <grp.h>
#include <log/log.h>
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
#include "bt_vendor_nxp.h"
/******************************************************************************
 **
 ** Constants and Macro's
 **
 ******************************************************************************/
/*[NK] @NXP - Driver FIX
  ioctl command to release the read thread before driver close */

#define MBTCHAR_IOCTL_RELEASE _IO('M', 1)

#define PROP_BLUETOOTH_OPENED "bluetooth.nxp.uart_configured"
#define PROP_BLUETOOTH_FW_DOWNLOADED "bluetooth.nxp.fw_downloaded"
#define PROP_BLUETOOTH_DELAY "bluetooth.nxp.fw_downloaded_delay"

/*
 * Defines for wait for Bluetooth firmware ready Specify durations
 * between polls and max wait time
 */
#define POLL_DRIVER_DURATION_US (100000)
#define POLL_DRIVER_MAX_TIME_MS (20000)

#define CONF_COMMENT '#'
#define CONF_DELIMITERS " =\n\r\t"
#define CONF_VALUES_DELIMITERS "=\n\r\t"
#define CONF_MAX_LINE_LEN 255
#define UNUSED(x) (void)(x)
#define BD_ADDR_LEN 6

/******************************************************************************
**  Variables
******************************************************************************/
int mchar_fd = 0;
struct termios ti;
static uint8_t adapterState;
unsigned char* bdaddr = NULL;
const bt_vendor_callbacks_t* vnd_cb = NULL;
/* for NXP USB/SD interface */
static char mbt_port[512] = "/dev/mbtchar0";
/* for NXP Uart interface */
static char mchar_port[512] = "/dev/ttyUSB0";
static int is_uart_port = 0;
static int uart_break_before_open = 0;
static int32_t baudrate_fw_init = 115200;
static int32_t baudrate_bt = 3000000;
int write_bdaddrss = 0;
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
int8_t ble_1m_power = 0;
int8_t ble_2m_power = 0;
uint8_t set_1m_2m_power = 0;
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
int8_t bt_max_power_sel = 0;
uint8_t bt_set_max_power = 0;
#endif
#if (NXP_ENABLE_INDEPENDENT_RESET_VSC == TRUE)
uint8_t independent_reset_gpio_pin = 0xFF;
#endif
#if (NXP_IR_IMX_GPIO_TOGGLE == TRUE)
uint8_t ir_host_gpio_pin = 14;
static char chrdev_name[32] = "/dev/gpiochip5";
#endif
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

#ifdef UART_DOWNLOAD_FW
int uart_break_before_change_baudrate = 0;
static int enable_download_fw = 0;
static int uart_break_after_dl_helper = 0;
static int uart_sleep_after_dl = 100;
static int download_helper = 0;
static int32_t baudrate_dl_helper = 115200;
static int32_t baudrate_dl_image = 3000000;
static char pFileName_helper[512] = "/vendor/firmware/helper_uart_3000000.bin";
static char pFileName_image[512] = "/vendor/firmware/uart8997_bt_v4.bin";
static int32_t iSecondBaudrate = 0;
#if (NXP_ENABLE_INDEPENDENT_RESET_CMD5 == TRUE)
int enable_ir_config = 0;
#endif
#endif
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
char pFilename_cal_data[512];
#endif
static pthread_mutex_t dev_file_lock = PTHREAD_MUTEX_INITIALIZER;
#if (NXP_ENABLE_RFKILL_SUPPORT == TRUE)
static int rfkill_id = -1;
static char* rfkill_state_path = NULL;
#endif
/*****************************************************************************
**
**   HELPER FUNCTIONS
**
*****************************************************************************/
typedef int(conf_action_t)(char* p_conf_name, char* p_conf_value, int param);

typedef struct {
  const char* conf_entry;
  conf_action_t* p_action;
  int param;
} conf_entry_t;

static int set_mchar_port(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  strcpy(mchar_port, p_conf_value);
  is_uart_port = 1;
  return 0;
}

static int set_mbt_port(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  strcpy(mbt_port, p_conf_value);
  is_uart_port = 0;
  return 0;
}

static int set_is_uart_port(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  is_uart_port = atoi(p_conf_value);
  return 0;
}

static int set_uart_break_before_open(char* p_conf_name, char* p_conf_value,
                                      int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  uart_break_before_open = atoi(p_conf_value);
  return 0;
}

static int set_baudrate_bt(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  baudrate_bt = atoi(p_conf_value);
  return 0;
}

static int set_baudrate_fw_init(char* p_conf_name, char* p_conf_value,
                                int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  baudrate_fw_init = atoi(p_conf_value);
  return 0;
}
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
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
#endif

#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
static int set_bt_tx_power(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  bt_max_power_sel = atoi(p_conf_value);
  bt_set_max_power = 1;
  return 0;
}
#endif

#if (NXP_ENABLE_INDEPENDENT_RESET_VSC == TRUE)
static int set_independent_reset_gpio_pin(char* p_conf_name, char* p_conf_value,
                                          int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  independent_reset_gpio_pin = atoi(p_conf_value);
  return 0;
}
#endif
#if (NXP_IR_IMX_GPIO_TOGGLE == TRUE)
static int set_oob_ir_host_gpio_pin(char* p_conf_name, char* p_conf_value,
                                    int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  ir_host_gpio_pin = atoi(p_conf_value);
  return 0;
}
static int set_charddev_name(char* p_conf_name, char* p_conf_value, int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  strcpy(chrdev_name, p_conf_value);
  return 0;
}
#endif
static int set_bd_address_buf(char* p_conf_name, char* p_conf_value,
                              int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  int i = 0;
  int j = 7;
  int len = 0;
  if (p_conf_value == NULL) return 0;
  len = strlen(p_conf_value);
  if (len != 17) return 0;
  for (i = 0; i < len; i++) {
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
  for (i = 0; i < 17; i++) {
    write_bd_address[j--] = (p_conf_value[i] << 4) | p_conf_value[i + 1];
    i = i + 2;
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

static int set_uart_break_before_change_baudrate(char* p_conf_name,
                                                 char* p_conf_value,
                                                 int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  uart_break_before_change_baudrate = atoi(p_conf_value);
  return 0;
}

static int set_uart_break_after_dl_helper(char* p_conf_name, char* p_conf_value,
                                          int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  uart_break_after_dl_helper = atoi(p_conf_value);
  return 0;
}

static int set_pFileName_image(char* p_conf_name, char* p_conf_value,
                               int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  strcpy(pFileName_image, p_conf_value);
  return 0;
}

static int set_pFileName_helper(char* p_conf_name, char* p_conf_value,
                                int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  strcpy(pFileName_helper, p_conf_value);
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

#if (NXP_ENABLE_INDEPENDENT_RESET_CMD5 == TRUE)
static int set_independent_reset_config(char* p_conf_name, char* p_conf_value,
                                        int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  enable_ir_config = atoi(p_conf_value);
  return 0;
}
#endif
#endif
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
static int set_Filename_cal_data(char* p_conf_name, char* p_conf_value,
                                 int param) {
  UNUSED(p_conf_name);
  UNUSED(param);
  strcpy(pFilename_cal_data, p_conf_value);
  return 0;
}
#endif
/*
 * Current supported entries and corresponding action functions
 */

static const conf_entry_t conf_table[] = {
    {"mchar_port", set_mchar_port, 0},
    {"mbt_port", set_mbt_port, 0},
    {"is_uart_port", set_is_uart_port, 0},
    {"uart_break_before_open", set_uart_break_before_open, 0},
    {"baudrate_bt", set_baudrate_bt, 0},
    {"baudrate_fw_init", set_baudrate_fw_init, 0},
    {"bd_address", set_bd_address_buf, 0},
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
    {"ble_1m_power", set_ble_1m_power, 0},
    {"ble_2m_power", set_ble_2m_power, 0},
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
    {"bt_max_power_sel", set_bt_tx_power, 0},
#endif
#if (NXP_ENABLE_INDEPENDENT_RESET_VSC == TRUE)
    {"independent_reset_gpio_pin", set_independent_reset_gpio_pin, 0},
#endif
#if (NXP_IR_IMX_GPIO_TOGGLE == TRUE)
    {"oob_ir_host_gpio_pin", set_oob_ir_host_gpio_pin, 0},
    {"chardev_name", set_charddev_name, 0},
#endif
#ifdef UART_DOWNLOAD_FW
    {"enable_download_fw", set_enable_download_fw, 0},
    {"uart_break_before_change_baudrate", set_uart_break_before_change_baudrate,
     0},
    {"uart_break_after_dl_helper", set_uart_break_after_dl_helper, 0},
    {"pFileName_image", set_pFileName_image, 0},
    {"pFileName_helper", set_pFileName_helper, 0},
    {"baudrate_dl_helper", set_baudrate_dl_helper, 0},
    {"baudrate_dl_image", set_baudrate_dl_image, 0},
    {"iSecondBaudrate", set_iSecondBaudrate, 0},
    {"uart_sleep_after_dl", set_uart_sleep_after_dl, 0},
#if (NXP_ENABLE_INDEPENDENT_RESET_CMD5 == TRUE)
    {"enable_ir_config", set_independent_reset_config, 0},
#endif
#endif
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
    {"pFilename_cal_data", set_Filename_cal_data, 0},
#endif
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

  ALOGI("Attempt to load conf from %s", p_path);

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
        if (strcmp(p_entry->conf_entry, (const char*)p_name) == 0) {
          p_entry->p_action(p_name, p_value, p_entry->param);
          break;
        }

        p_entry++;
      }
    }

    fclose(p_file);
  } else {
    ALOGI("vnd_load_conf file >%s< not found", p_path);
  }
}

static int set_speed(int fd, struct termios* ti, int speed) {
  if (cfsetospeed(ti, speed) < 0) {
    VNDDBG("Set O speed failed!\n");
    return -1;
  }

  if (cfsetispeed(ti, speed) < 0) {
    VNDDBG("Set I speed failed!\n");
    return -1;
  }

  if (tcsetattr(fd, TCSANOW, ti) < 0) {
    VNDDBG("Set Attr speed failed!\n");
    return -1;
  }

  return 0;
}

/******************************************************************************
 **
 ** Function:        read_hci_event
 **
 ** Description:     Reads the parameter of event received from controller.
 **
 ** Return Value:    offset of parameter
 **
 *
 *****************************************************************************/

static int read_hci_event(int fd, unsigned char* buf, int size) {
  int remain, r;
  int count = 0;
  int k = 0;

  if (size <= 0) return -1;

  /* The first byte identifies the packet type. For HCI event packets, it
   * should be 0x04, so we read until we get to the 0x04. */
  VNDDBG("start read hci event 0x4\n");
  while (k < 20) {
    r = read(fd, buf, 1);
    if (r <= 0) {
      VNDDBG("read hci event 0x04 failed, retry\n");
      k++;
      usleep(50 * 1000);
      continue;
    }
    if (buf[0] == 0x04) break;
  }
  if (k >= 20) {
    VNDDBG("read hci event 0x04 failed, return error. k = %d\n", k);
    return -1;
  }
  count++;

  /* The next two bytes are the event code and parameter total length. */
  VNDDBG("start read hci event code and len\n");
  while (count < 3) {
    r = read(fd, buf + count, 3 - count);
    if (r <= 0) {
      VNDDBG("read hci event code and len failed\n");
      return -1;
    }
    count += r;
  }

  /* Now we read the parameters. */
  VNDDBG("start read hci event para\n");
  if (buf[2] < (size - 3))
    remain = buf[2];
  else
    remain = size - 3;

  while ((count - 3) < remain) {
    r = read(fd, buf + count, remain - (count - 3));
    if (r <= 0) {
      VNDDBG("read hci event para failed\n");
      return -1;
    }
    count += r;
  }

  VNDDBG("over read count = %d\n", count);
  return count;
}

static int set_prop_int32(char* name, int value) {
  char init_value[PROPERTY_VALUE_MAX];
  int ret;

  sprintf(init_value, "%d", value);
  ret = property_set(name, init_value);
  if (ret < 0) {
    ALOGE("set_prop_int32 failed: %d", ret);
  }
  return ret;
}

static int get_prop_int32(char* name) {
  int ret;

  ret = property_get_int32(name, -1);
  VNDDBG("get_prop_int32: %d", ret);
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
 *
 ** Function            uart_set_speed
 **
 ** Description         Set the baud rate speed.

 ** Return Value:       0 On success else -1

 **
 *****************************************************************************/

static int32 uart_set_speed(int32 fd, struct termios* ti, int32 speed) {
  cfsetospeed(ti, uart_speed(speed));
  return tcsetattr(fd, TCSANOW, ti);
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
    VNDDBG("Can't open serial port");
    return -1;
  }

  tcflush(fd, TCIOFLUSH);

  if (tcgetattr(fd, &ti) < 0) {
    VNDDBG("Can't get port settings");
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
    VNDDBG("Can't set port settings");
    close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);

  /* Set actual baudrate */
  if (uart_set_speed(fd, &ti, dwBaudRate) < 0) {
    VNDDBG("Can't set baud rate");
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
        ALOGE("exceed max retry count, return error\n");
        return -1;
      } else {
        ALOGE("open uart port %s failed fd: %d, retrying\n", dev, fd);
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
  int download_ret = 0;
  int fw_downloaded = 0;

/* detect fw status */
#ifdef FW_LOADER_V2
  if (bt_vnd_mrvl_check_fw_status_v2()) {
#else
  if (bt_vnd_mrvl_check_fw_status()) {
#endif
    /* force download only when header is received */
    fw_downloaded = 0;
  } else {
    /* ignore download */
    fw_downloaded = 1;
    set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 1);
    goto done;
  }

  VNDDBG(" fw_downloaded %d", fw_downloaded);

  if (!fw_downloaded) {
#ifndef FW_LOADER_V2
    init_crc8();
#endif
    /* download helper */
    if (download_helper) {
#ifdef FW_LOADER_V2
      download_ret = bt_vnd_mrvl_download_fw_v2(mchar_port, baudrate_dl_helper,
                                                pFileName_helper);
#else
      download_ret = bt_vnd_mrvl_download_fw(mchar_port, baudrate_dl_helper,
                                             pFileName_helper, iSecondBaudrate);
#endif
      if (download_ret != 0) {
        VNDDBG("helper download failed");
        goto done;
      }

      usleep(50000);
      /* flush additional A5 header if any */
      tcflush(mchar_fd, TCIFLUSH);

      /* close and open the port and set baud rate to baudrate_dl_image */
      close(mchar_fd);
      mchar_fd = uart_init_open(mchar_port, 3000000, 1);
      usleep(20000);
      tcflush(mchar_fd, TCIOFLUSH);
    }

/* download fw image */
#ifdef FW_LOADER_V2
    download_ret = bt_vnd_mrvl_download_fw_v2(mchar_port, baudrate_dl_image,
                                              pFileName_image);
#else
    download_ret = bt_vnd_mrvl_download_fw(mchar_port, baudrate_dl_image,
                                           pFileName_image, iSecondBaudrate);
#endif
    if (download_ret != 0) {
      VNDDBG("fw download failed");
      goto done;
    }

    tcflush(mchar_fd, TCIFLUSH);
    if (uart_sleep_after_dl) usleep(uart_sleep_after_dl * 1000);

    set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 1);
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
  int clen;
  unsigned char set_speed_cmd_3m[8] = {0x01, 0x09, 0xFC, 0x04,
                                       0xC0, 0xC6, 0x2D, 0x00};
  unsigned char set_speed_cmd[8] = {0x01, 0x09, 0xFC, 0x04,
                                    0x00, 0xC2, 0x01, 0x00};
  unsigned char reset_cmd[4] = {0x01, 0x03, 0x0c, 0x00};
  int resp_size;
  unsigned char resp[10] = {0};
  unsigned char resp_cmp[7] = {0x4, 0xe, 0x4, 0x1, 0x9, 0xfc, 0x0};
  unsigned char resp_cmp_reset[7] = {0x4, 0xe, 0x4, 0x1, 0x3, 0xc, 0x0};

  if (baudrate_fw_init != baudrate_bt) {
    /* set baud rate to baudrate_fw_init */
    if (uart_set_speed(mchar_fd, &ti, baudrate_fw_init) < 0) {
      VNDDBG("Can't set baud rate");
      return -1;
    }

    /* Sending HCI reset CMD  */
    VNDDBG("start send bt hci reset\n");
    memset(resp, 0x00, 10);
    clen = sizeof(reset_cmd);
    VNDDBG("Write HCI Reset command\n");
    if (write(mchar_fd, reset_cmd, clen) != clen) {
      VNDDBG("Failed to write reset command \n");
      return -1;
    }

    if ((resp_size = read_hci_event(mchar_fd, resp, 10)) < 0 ||
        memcmp(resp, resp_cmp_reset, 7)) {
      VNDDBG("Failed to read HCI RESET CMD response! \n");
      return -1;
    }
    VNDDBG("over send bt hci reset\n");

    /* Set bt chip Baud rate CMD */
    VNDDBG("start set fw baud rate according to baudrate_bt\n");
    clen = sizeof(set_speed_cmd);
    if (baudrate_bt == 3000000) {
      VNDDBG("set fw baudrate as 3000000\n");
      if (write(mchar_fd, set_speed_cmd_3m, clen) != clen) {
        VNDDBG("Failed to write set baud rate command \n");
        return -1;
      }
    } else if (baudrate_bt == 115200) {
      VNDDBG("set fw baudrate as 115200");
      if (write(mchar_fd, set_speed_cmd, clen) != clen) {
        VNDDBG("Failed to write set baud rate command \n");
        return -1;
      }
    }
    usleep(60000); /* Sleep to allow baud rate setting to happen in FW */

    VNDDBG("start read hci event\n");
    memset(resp, 0x00, 10);
    if ((resp_size = read_hci_event(mchar_fd, resp, 100)) < 0 ||
        memcmp(resp, resp_cmp, 7)) {
      VNDDBG("Failed to read set baud rate command response! \n");
      return -1;
    }
    VNDDBG("over send bt chip baudrate\n");

    /* set host uart speed according to baudrate_bt */
    VNDDBG("start set host baud rate as baudrate_bt\n");
    tcflush(mchar_fd, TCIOFLUSH);
    if (set_speed(mchar_fd, &ti, uart_speed(baudrate_bt))) {
      VNDDBG("Failed to  set baud rate \n");
      return -1;
    }
    ti.c_cflag |= CRTSCTS;
    if (tcsetattr(mchar_fd, TCSANOW, &ti) < 0) {
      VNDDBG("Set Flow Control failed!\n");
      return -1;
    }
    tcflush(mchar_fd, TCIOFLUSH);
  } else {
    /* set host uart speed according to baudrate_bt */
    VNDDBG("start set host baud rate as baudrate_bt\n");
    tcflush(mchar_fd, TCIOFLUSH);

    /* Close and open the port as setting baudrate to baudrate_bt */
    close(mchar_fd);
    mchar_fd = uart_init_open(mchar_port, baudrate_bt, 1);
    usleep(20000);
    tcflush(mchar_fd, TCIOFLUSH);
  }

  usleep(20 * 1000);
  set_prop_int32(PROP_BLUETOOTH_OPENED, 1);
  return 0;
}
#if (NXP_ENABLE_RFKILL_SUPPORT == TRUE)

static bool bt_vnd_is_rfkill_disabled(void) {
  char value[PROPERTY_VALUE_MAX];

  property_get("ro.rfkilldisabled", value, "0");
  if (strcmp(value, "1") == 0) {
    return true;
  }
  return false;
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
      ALOGE("open(%s) failed: %s (%d)\n", path, strerror(errno), errno);
      break;
    }
    sz = read(fd, &buf, sizeof(buf));
    if (sz < 0) {
      ALOGE("read failed: %s (%d)\n", strerror(errno), errno);
    }
    close(fd);
    if (sz >= 9 && memcmp(buf, "bluetooth", 9) == 0) {
      rfkill_id = id;
      break;
    }
  }

  if (rfkill_id == -1) {
    ALOGE("bluetooth rfkill not found\n");
    return -1;
  } else {
    asprintf(&rfkill_state_path, "/sys/class/rfkill/rfkill%d/state", rfkill_id);
    VNDDBG("rfkill_state_path set to %s \n", rfkill_state_path);
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
    VNDDBG("rfkill disabled, ignoring bluetooth power %s\n",
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
    ALOGE("open(%s) for write failed: %s (%d)", rfkill_state_path,
          strerror(errno), errno);
    goto done;
  }
  sz = write(fd, &buffer, 1);
  if (sz < 0) {
    ALOGE("write(%s) failed: %s (%d)", rfkill_state_path, strerror(errno),
          errno);
  }
  ret = 0;

done:
  if (fd >= 0) {
    close(fd);
  }
  return ret;
}
#endif /*NXP_ENABLE_RFKILL_SUPPORT*/

#if (NXP_IR_IMX_GPIO_TOGGLE == TRUE)
void bt_vnd_gpio_configuration(int value) {
  struct gpiohandle_request req;
  struct gpiohandle_data data;
  int fd, ret;

  /* Open device: gpiochip0 for GPIO bank A */
  fd = open(chrdev_name, 0);
  if (fd == -1) {
    ALOGE("Failed to open %s %s\n", chrdev_name, strerror(errno));
    return;
  }
  /* Request GPIO Direction line as out */
  req.lineoffsets[0] = ir_host_gpio_pin;
  req.flags = GPIOHANDLE_REQUEST_OUTPUT;
  memcpy(req.default_values, &data, sizeof(req.default_values));
  req.lines = 1;
  ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);

  if (ret == -1) {
    ALOGE("%s Failed to issue GET LINEHANDLE IOCTL(%d)", strerror(errno), ret);
    close(fd);
    return;
  }
  if (close(fd) == -1) {
    ALOGE("Failed to close GPIO character device file");
    return;
  }
  /* Get the value of line */
  memset(&data, 0, sizeof(data));
  ret = ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
  if (ret == -1) {
    close(req.fd);
    ALOGE("%s failed to issue GET LINEHANDLE", strerror(errno));
    return;
  }
  VNDDBG("Current Value of line=: %d\n", data.values[0]);

  /* Set the requested value to the line*/
  data.values[0] = value;
  ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
  if (ret == -1) {
    close(req.fd);
    ALOGE("%s failed to issue SET LINEHANDLE", strerror(errno));
    return;
  }
  ret = ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
  VNDDBG("Updated Value of Line:= %d\n", data.values[0]);

  /*  release line */
  ret = close(req.fd);
  if (ret == -1) {
    ALOGE("%s Failed to close GPIO LINEHANDLE device file", strerror(errno));
    return;
  }
}
#endif /*NXP_IR_IMX_GPIO_TOGGLE*/

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int bt_vnd_init(const bt_vendor_callbacks_t* p_cb,
                       unsigned char* local_bdaddr) {
  vnd_cb = p_cb;
  if (vnd_cb == NULL) {
    VNDDBG("vnd_cb is NULL");
  }
  ALOGI("bt_vnd_init\n");
  VNDDBG("bt_vnd_init --- BT Vendor HAL Ver: %s ---\n", BT_HAL_VERSION);
  if (local_bdaddr) {
    bdaddr = (unsigned char*)malloc(BD_ADDR_LEN);
    if (bdaddr != NULL) {
      memcpy(bdaddr, local_bdaddr, 6);
      VNDDBG("bdaddr is %02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX\n", bdaddr[0],
             bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
    }
  }
  vnd_load_conf(VENDOR_LIB_CONF_FILE);
  return 0;
}

/** Requested operations */
static int bt_vnd_op(bt_vendor_opcode_t opcode, void* param) {
  int ret = 0;
  int local_st = 0;

  VNDDBG("opcode = %d\n", opcode);
  switch (opcode) {
    case BT_VND_OP_POWER_CTRL: {
      int* state = (int*)param;

      if (*state == BT_VND_PWR_OFF) {
        VNDDBG("power off --------------------------------------*\n");
        if (adapterState == BT_VND_PWR_ON) {
          VNDDBG("BT adapter switches from ON to OFF .. \n");
          adapterState = BT_VND_PWR_OFF;
        }
      } else if (*state == BT_VND_PWR_ON) {
        VNDDBG("power on --------------------------------------\n");
        adapterState = BT_VND_PWR_ON;
#if (NXP_ENABLE_RFKILL_SUPPORT == TRUE)
        bt_vnd_set_bluetooth_power(FALSE);
        usleep(5000);
        bt_vnd_set_bluetooth_power(TRUE);

        set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
        set_prop_int32(PROP_BLUETOOTH_OPENED, 0);
#endif
#if (NXP_IR_IMX_GPIO_TOGGLE == TRUE)
        set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
        set_prop_int32(PROP_BLUETOOTH_OPENED, 0);

        VNDDBG("-------------- Setting GPIO LOW -----------------\n");
        bt_vnd_gpio_configuration(FALSE);
        usleep(1000);
        VNDDBG("---------------- Setting GPIO HIGH ----------------\n");
        bt_vnd_gpio_configuration(TRUE);
#endif
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
      VNDDBG("open serial port --------------------------------------\n");
      int(*fd_array)[] = (int(*)[])param;
      int idx;
      int bluetooth_opened;
      int num = 0;
      int32_t baudrate = 0;

      if (is_uart_port) {
        VNDDBG("baudrate_bt %d\n", baudrate_bt);
        VNDDBG("baudrate_fw_init %d\n", baudrate_fw_init);
#ifdef UART_DOWNLOAD_FW
        if (enable_download_fw) {
          VNDDBG("download_helper %d\n", download_helper);
          VNDDBG("uart_break_before_change_baudrate %d\n",
                 uart_break_before_change_baudrate);
          VNDDBG("baudrate_dl_helper %d\n", baudrate_dl_helper);
          VNDDBG("baudrate_dl_image %d\n", baudrate_dl_image);
          VNDDBG("pFileName_helper %s\n", pFileName_helper);
          VNDDBG("pFileName_image %s\n", pFileName_image);
          VNDDBG("iSecondBaudrate %d\n", iSecondBaudrate);
          VNDDBG("uart_break_before_open %d\n", uart_break_before_open);
          VNDDBG("enable_download_fw %d\n", enable_download_fw);
          VNDDBG("uart_break_after_dl_helper %d\n", uart_break_after_dl_helper);
          VNDDBG("uart_sleep_after_dl %d\n", uart_sleep_after_dl);
        }
#endif
      }
      pthread_mutex_lock(&dev_file_lock);

      if (is_uart_port) {
        /* ensure libbt can talk to the driver, only need open port once */
        if (get_prop_int32(PROP_BLUETOOTH_OPENED))
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
          mchar_fd = uart_init_open(mchar_port, baudrate, 0);
        }
        if (mchar_fd > 0)
          VNDDBG("open uart port successfully, fd=%d, mchar_port=%s\n",
                 mchar_fd, mchar_port);
        else {
          ALOGE("open UART bt port %s failed fd: %d\n", mchar_port, mchar_fd);
          pthread_mutex_unlock(&dev_file_lock);
          return -1;
        }
      } else {
        do {
          mchar_fd = open(mbt_port, O_RDWR | O_NOCTTY);
          if (mchar_fd < 0) {
            num++;
            if (num >= 8) {
              ALOGE("exceed max retry count, return error");
              pthread_mutex_unlock(&dev_file_lock);
              return -1;
            } else {
              ALOGE("open USB/SD port %s failed fd: %d, retrying\n", mbt_port,
                    mchar_fd);
              sleep(1);
              continue;
            }
          } else {
            VNDDBG("open USB or SD port successfully, fd=%d, mbt_port=%s\n",
                   mchar_fd, mbt_port);
            is_uart_port = 0;
          }
        } while (mchar_fd < 0);
      }

      if (is_uart_port) {
#ifdef UART_DOWNLOAD_FW
        if (enable_download_fw) {
          if (detect_and_download_fw()) {
            ALOGE("detect_and_download_fw failed");
            set_prop_int32(PROP_BLUETOOTH_OPENED, 0);
            set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
            pthread_mutex_unlock(&dev_file_lock);
            return -1;
          }
        } else {
          ti.c_cflag |= CRTSCTS;
          if (tcsetattr(mchar_fd, TCSANOW, &ti) < 0) {
            ALOGE("Set Flow Control failed!\n");
            return -1;
          }
          tcflush(mchar_fd, TCIOFLUSH);
        }
#else
        ti.c_cflag |= CRTSCTS;
        if (tcsetattr(mchar_fd, TCSANOW, &ti) < 0) {
          ALOGE("Set Flow Control failed!\n");
          return -1;
        }
        tcflush(mchar_fd, TCIOFLUSH);
#endif
        bluetooth_opened = get_prop_int32(PROP_BLUETOOTH_OPENED);
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
                fclose(fd);
                break;
              }
              usleep(POLL_DRIVER_DURATION_US);
            }
          }

          if (config_uart()) {
            ALOGE("config_uart failed");
            set_prop_int32(PROP_BLUETOOTH_OPENED, 0);
            set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 0);
            pthread_mutex_unlock(&dev_file_lock);
            return -1;
          }
        }
      }

      for (idx = 0; idx < CH_MAX; idx++) {
        (*fd_array)[idx] = mchar_fd;
        ret = 1;
      }
      pthread_mutex_unlock(&dev_file_lock);
      VNDDBG("open serial port over --------------------------------------\n");
    } break;
    case BT_VND_OP_USERIAL_CLOSE:
      /* mBtChar port is blocked on read. Release the port before we close it */
      pthread_mutex_lock(&dev_file_lock);
      if (is_uart_port) {
        if (mchar_fd) {
          tcflush(mchar_fd, TCIFLUSH);
          close(mchar_fd);
          mchar_fd = 0;
        }
      } else {
        ioctl(mchar_fd, MBTCHAR_IOCTL_RELEASE, &local_st);
        /* Give it sometime before we close the mbtchar */
        usleep(1000);
        if (mchar_fd) {
          if (close(mchar_fd) < 0) {
            ALOGE("close serial port failed!\n");
            ret = -1;
          }
        }
      }
      pthread_mutex_unlock(&dev_file_lock);
      break;
    case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
      break;
    case BT_VND_OP_LPM_SET_MODE:
      if (vnd_cb) {
        vnd_cb->lpm_cb(ret);
      }
      break;
    case BT_VND_OP_LPM_WAKE_SET_STATE:
      break;
    default:
      ret = -1;
      break;
  }
  return ret;
}

/** Closes the interface */
static void bt_vnd_cleanup(void) {
  VNDDBG("cleanup ...");
  vnd_cb = NULL;
  if (bdaddr) {
    free(bdaddr);
    bdaddr = NULL;
  }
}

/** Entry point of DLib */
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t), bt_vnd_init, bt_vnd_op, bt_vnd_cleanup,
};
