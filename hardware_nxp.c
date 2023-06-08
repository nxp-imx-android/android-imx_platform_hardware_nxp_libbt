/******************************************************************************
 *
 *  Copyright 2012-2013, 2020-2023 NXP
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
 *  Filename:      hardware_nxp.c
 *
 *  Description:   Hardware controller configuration functions
 *
 ******************************************************************************/

#define LOG_TAG "hardware_nxp"

/*============================== Include Files ===============================*/

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bt_vendor_log.h"
#include "bt_vendor_nxp.h"
#include "fw_loader_io.h"

/*================================== Macros ==================================*/
/*****************************************************************************
 * ** copy from bt_hci_bdroid.h
 * ******************************************************************************/
#define MSG_STACK_TO_HC_HCI_CMD 0x2000 /* eq. BT_EVT_TO_LM_HCI_CMD */
#define BT_HC_HDR_SIZE (sizeof(HC_BT_HDR))

#define HCI_CMD_NXP_WRITE_VOICE_SETTINGS 0x0C26
#define HCI_CMD_NXP_WRITE_PCM_SETTINGS 0xFC07
#define HCI_CMD_NXP_WRITE_PCM_SYNC_SETTINGS 0xFC28
#define HCI_CMD_NXP_WRITE_PCM_LINK_SETTINGS 0xFC29
#define HCI_CMD_NXP_SET_SCO_DATA_PATH 0xFC1D

#define WRITE_VOICE_SETTINGS_SIZE 2
#define WRITE_PCM_SETTINGS_SIZE 1
#define WRITE_PCM_SYNC_SETTINGS_SIZE 3
#define WRITE_PCM_LINK_SETTINGS_SIZE 2
#define SET_SCO_DATA_PATH_SIZE 1

#define HCI_CMD_NXP_WRITE_BD_ADDRESS 0xFC22
#define HCI_BT_SET_EVENTMASK_OCF 0x0001
#define HCI_CMD_NXP_RESET 0x0C03
#define HCI_DISABLE_PAGE_SCAN_OCF 0x001a
#define HCI_READ_LOCAL_BDADDR 0x1009
#define HCI_COMMAND_COMPLETE_EVT 0x0E
#define HCI_PACKET_TYPE_EVENT 0x04

#define WRITE_BD_ADDRESS_SIZE 8
#define HCI_CMD_PREAMBLE_SIZE 3
#define HCI_EVT_CMD_CMPL_OPCODE 3

#define IS_SAME_MEM(src, val, size) \
  ((*src == val) && (memcmp(src, src + 1, size - 1) == 0))
/* Check if src has one of the following default BD_ADDR
  00:00:00:00:00:00, 41:41:41:41:41:41,88:88:88:88:88:88,99:99:99:99:99:99
  */
#define IS_DEFAULT_BDADDR(src)            \
  (IS_SAME_MEM(src, 0x00, BD_ADDR_LEN) || \
   IS_SAME_MEM(src, 0x41, BD_ADDR_LEN) || \
   IS_SAME_MEM(src, 0x88, BD_ADDR_LEN) || IS_SAME_MEM(src, 0x99, BD_ADDR_LEN))

/*1 byte for event code, 1 byte for parameter length (Volume 2, Part E, 5.4.4)
 */
#define HCI_EVENT_PREAMBLE_SIZE 2
/*2 bytes for opcode, 1 byte for parameter length (Volume 2, Part E, 5.4.1) */
#define HCI_COMMAND_PREAMBLE_SIZE 3
#define WRITE_BD_ADDRESS_SIZE 8
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY 6
#define BD_ADDR_LEN 6
#define HCI_CMD_NXP_LOAD_CONFIG_DATA 0xFC61
#define HCI_CMD_NXP_LOAD_CONFIG_DATA_SIZE 32
#define HCI_CMD_NXP_CUSTOM_OPCODE 0xFD60
#define HCI_CMD_NXP_SUB_ID_BLE_TX_POWER 0x01
#define HCI_CMD_NXP_BLE_TX_POWER_DATA_SIZE 0x03
#define HCI_CMD_NXP_BT_TX_POWER_DATA_SIZE 0x01
#define HCI_CMD_NXP_READ_FW_REVISION 0xFC0F
#define HCI_CMD_NXP_WRITE_BT_TX_POWER 0xFCEE
#define HCI_CMD_NXP_INDEPENDENT_RESET_SETTING 0xFC0D
#define HCI_CMD_NXP_INDEPENDENT_RESET_SETTING_SIZE 0x02
#define BT_CONFIG_DATA_SIZE 28
#define HCI_CMD_NXP_SET_BT_SLEEP_MODE 0xFC23
#define HCI_CMD_NXP_SET_BT_SLEEP_MODE_SIZE 0x03
#define STREAM_TO_UINT16(u16, p)                                \
  do {                                                          \
    u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); \
    (p) += 2;                                                   \
  } while (0)

#define UINT16_TO_STREAM(p, u16)    \
  do {                              \
    *(p)++ = (uint8_t)(u16);        \
    *(p)++ = (uint8_t)((u16) >> 8); \
  } while (0)

#define UINT8_TO_STREAM(p, u8) \
  { *(p)++ = (uint8_t)(u8); }

/*Move to next configuration in the sequence*/
#define hw_config_next() hw_config_seq(NULL)

#define HCI_CMD_NXP_BLE_WAKEUP 0xFD52
#define HCI_CMD_NXP_SUB_OCF_HEARTBEAT 0x00
#define HCI_CMD_NXP_SUB_OCF_GPIO_CONFIG 0x01
#define HCI_CMD_NXP_SUB_OCF_ADV_PATTERN_CONFIG 0x02
#define HCI_CMD_NXP_SUB_OCF_SCAN_PARAM_CONFIG 0x03
#define HCI_CMD_NXP_SUB_OCF_LOCAL_PARAM_CONFIG 0x04
#define HCI_CMD_NXP_SUB_OCF_SIZE 1
#define HCI_CMD_NXP_GPIO_CONFIG_SIZE 4
#define HCI_CMD_NXP_ADV_PATTERN_LENGTH_SIZE 1
#define HCI_CMD_NXP_SCAN_PARAM_CONFIG_SIZE 7
#define HCI_CMD_NXP_LOCAL_PARAM_CONFIG_SIZE 1
#define TIMER_UNIT_MS_TO_US 1000

/*================================== Typedefs=================================*/

struct bt_evt_param_t {
  uint16_t cmd;
  uint8_t cmd_ret_param;
};
typedef int8 (*hw_config_fun_ptr)(void);
typedef void (*hw_config_reply_handler)(void*);

/*============================ Function Prototypes ===========================*/

static int8 hw_bt_send_reset(void);
static int8 hw_config_set_bdaddr(void);
static int8 hw_bt_read_fw_revision(void);
static void hw_config_seq(void* packet);
static int8 hw_sco_config(void);
static int8 hw_bt_enable_independent_reset(void);
static int8 hw_config_read_bdaddr(void);
static int8 hw_bt_cal_data_load(void);
static int8 hw_ble_set_power_level(void);
static int8 hw_bt_enable_max_power_level_cmd(void);
static int8 set_wakeup_scan_parameter(void);
static int8 set_wakeup_adv_pattern(void);
static int8 set_wakeup_local_parameter(void);
static int8 set_wakeup_gpio_config(void);
static void* send_heartbeat_thread(void* data);
static void wakeup_event_handler(uint8_t sub_ocf);

/*================================ Global Vars================================*/

static struct {
  /*Pointer to Hardware Configuation Array*/
  hw_config_fun_ptr* seq_arr;
  /*Size of seq_arr array*/
  uint8_t size;
  /*Current index of HW configuration*/
  int8_t indx;
  /*Skip indx increment in next iteration*/
  uint8_t skip_seq_incr;
} hw_config;

static hw_config_fun_ptr hw_config_seq_arr[] = {
    hw_bt_send_reset,
    hw_bt_read_fw_revision,
    hw_bt_enable_independent_reset,
    hw_bt_cal_data_load,
    hw_ble_set_power_level,
    hw_bt_enable_max_power_level_cmd,
    hw_config_read_bdaddr, /*Note:hw_config_read_bdaddr shall precede
                              hw_config_set_bdaddr*/
    hw_config_set_bdaddr,
    set_wakeup_scan_parameter,
    set_wakeup_gpio_config,
    set_wakeup_adv_pattern,
    set_wakeup_local_parameter,
    hw_sco_config};

/*Write_Voice_Setting - Use Linear Input coding, uLaw Air coding, 16bit sample
 * size*/
static uint8_t write_voice_settings[WRITE_VOICE_SETTINGS_SIZE] = {0x61, 0x00};
/** #PCM settings PCM 0: PCM slave role, 2: PCM master */
static uint8_t write_pcm_settings[WRITE_PCM_SETTINGS_SIZE] = {0x00};
/** PCM SYNC settings, 16k short sync with CLK 2.048MHz */
static uint8_t write_pcm_sync_settings[WRITE_PCM_SYNC_SETTINGS_SIZE] = {
    0x03, 0x00, 0x04};
/**PCM LINK settings, SCO slot1*/
static uint8_t write_pcm_link_settings[WRITE_PCM_LINK_SETTINGS_SIZE] = {0x04,
                                                                        0x00};
/** PCM LINK settings, SCO slot1 */
static uint8_t set_sco_data_path[SET_SCO_DATA_PATH_SIZE] = {0x01};

static pthread_mutex_t mtx_wakeup = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond_wakeup = PTHREAD_COND_INITIALIZER;
static pthread_t p_headtbeat;
static bool heartbeat_event_received = false;
static unsigned char wakeup_gpio_config_state = wakeup_key_num;
static bool send_heartbeat = FALSE;
static uint8_t wake_gpio_config = 0;

/*============================== Coded Procedures ============================*/

#if (BT_TRACE_LEVEL_DEBUG <= VHAL_LOG_LEVEL)
static char* cmd_to_str(uint16_t cmd) {
  switch (cmd) {
    case HCI_CMD_NXP_WRITE_PCM_SETTINGS:
      return "write_pcm_settings";
    case HCI_CMD_NXP_WRITE_PCM_SYNC_SETTINGS:
      return "write_pcm_sync_settings";
    case HCI_CMD_NXP_WRITE_PCM_LINK_SETTINGS:
      return "write_pcm_link_settings";
    case HCI_CMD_NXP_SET_SCO_DATA_PATH:
      return "set_sco_data_path";
    case HCI_CMD_NXP_WRITE_VOICE_SETTINGS:
      return "write_voice_settings";
    case HCI_CMD_NXP_RESET:
      return "hw_bt_send_reset";
    case HCI_CMD_NXP_READ_FW_REVISION:
      return "hw_bt_read_fw_revision";
    case HCI_CMD_NXP_INDEPENDENT_RESET_SETTING:
      return "hw_bt_enable_independent_reset";
    case HCI_CMD_NXP_LOAD_CONFIG_DATA:
      return "hw_bt_cal_data_load";
    case HCI_CMD_NXP_CUSTOM_OPCODE:
      return "hw_ble_set_power_level";
    case HCI_CMD_NXP_WRITE_BT_TX_POWER:
      return "hw_bt_enable_max_power_level";
    case HCI_READ_LOCAL_BDADDR:
      return "hw_config_read_bdaddr";
    case HCI_CMD_NXP_WRITE_BD_ADDRESS:
      return "write_bd_address";
    case HCI_CMD_NXP_BLE_WAKEUP:
      return "ble_wake_config";
    case HCI_CMD_NXP_SET_BT_SLEEP_MODE:
      return "configure_lpm";
    default:
      break;
  }

  return "unknown command";
}
#endif
/******************************************************************************
 **
 ** Function:      hw_bt_send_packet
 **
 ** Description:   Sends packet to Controller
 **
 ** Return Value:  0 if success, -1 otherwise
 **
 *****************************************************************************/
int8 hw_bt_send_packet(HC_BT_HDR* packet, uint16_t opcode,
                       hw_config_reply_handler reply_handler) {
  int8 ret = -1;
  if (packet) {
    if (vnd_cb->xmit_cb(opcode, packet, reply_handler)) {
      VND_LOGD("Sending hci command 0x%04hX (%s)", opcode, cmd_to_str(opcode));
      ret = 0;
    } else {
      VND_LOGE("Error while sending packet %04x", opcode);
    }
  } else {
    VND_LOGE("%s Error:Sending Invalid Packet", __func__);
  }
  return ret;
}

static HC_BT_HDR* build_cmd_buf(uint16_t cmd, uint8_t pl_len,
                                uint8_t* payload) {
  HC_BT_HDR* p_buf;
  uint16_t cmd_len = HCI_CMD_PREAMBLE_SIZE + pl_len;
  uint8_t* p;

  assert(vnd_cb && payload);

  p_buf = (HC_BT_HDR*)vnd_cb->alloc(BT_HC_HDR_SIZE + cmd_len);

  if (!p_buf) return NULL;

  p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
  p_buf->offset = 0;
  p_buf->layer_specific = 0;
  p_buf->len = cmd_len;

  p = (uint8_t*)(p_buf + 1);

  /* opcode */
  UINT16_TO_STREAM(p, cmd);

  /* length of payload */
  *p = pl_len;
  ++p;

  /* payload */
  memcpy(p, payload, pl_len);

  return p_buf;
}

static void parse_evt_buf(HC_BT_HDR* p_evt_buf,
                          struct bt_evt_param_t* evt_params) {
  uint8_t* p = (uint8_t*)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;

  assert(p_evt_buf && evt_params);

  /* opcode */
  STREAM_TO_UINT16(evt_params->cmd, p);

  /* command return parameter */
  evt_params->cmd_ret_param = *p;
}

/*******************************************************************************
**
** Function         hw_sco_config_cb
**
** Description      Callback function for PCM SCO configuration request
**
** Returns          None
**
*******************************************************************************/

static void hw_sco_config_cb(void* p_mem) {
  HC_BT_HDR* p_evt_buf = (HC_BT_HDR*)p_mem;
  struct bt_evt_param_t evt_params = {0, 0};
  uint16_t cmd;
  HC_BT_HDR* p_buf;

  assert(vnd_cb && p_mem);

  parse_evt_buf(p_evt_buf, &evt_params);

  /* free the buffer */
  vnd_cb->dealloc(p_evt_buf);

  switch (evt_params.cmd) {
    case HCI_CMD_NXP_WRITE_PCM_SETTINGS:
      /* Send HCI_CMD_NXP_WRITE_PCM_SYNC_SETTINGS */
      cmd = HCI_CMD_NXP_WRITE_PCM_SYNC_SETTINGS;
      p_buf = build_cmd_buf(cmd, WRITE_PCM_SYNC_SETTINGS_SIZE,
                            write_pcm_sync_settings);
      break;

    case HCI_CMD_NXP_WRITE_PCM_SYNC_SETTINGS:
      /* Send HCI_CMD_NXP_WRITE_PCM_LINK_SETTINGS */
      cmd = HCI_CMD_NXP_WRITE_PCM_LINK_SETTINGS;
      p_buf = build_cmd_buf(cmd, WRITE_PCM_LINK_SETTINGS_SIZE,
                            write_pcm_link_settings);
      break;

    case HCI_CMD_NXP_WRITE_PCM_LINK_SETTINGS:
      /* Send HCI_CMD_NXP_SET_SCO_DATA_PATH */
      cmd = HCI_CMD_NXP_SET_SCO_DATA_PATH;
      p_buf = build_cmd_buf(cmd, SET_SCO_DATA_PATH_SIZE, set_sco_data_path);
      break;

    case HCI_CMD_NXP_SET_SCO_DATA_PATH:
      /* Send HCI_CMD_NXP_WRITE_VOICE_SETTINGS */
      cmd = HCI_CMD_NXP_WRITE_VOICE_SETTINGS;
      p_buf =
          build_cmd_buf(cmd, WRITE_VOICE_SETTINGS_SIZE, write_voice_settings);
      break;
    case HCI_CMD_NXP_WRITE_VOICE_SETTINGS:
      /* sco config succeeds */
      VND_LOGD("SCO PCM config succeeds!");
      vnd_cb->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
      hw_config_next();
      return;

    default:
      VND_LOGE("Received event for unexpected cmd (0x%04hX). Fail.",
               evt_params.cmd);
      p_buf = NULL;
      break;
  } /* switch (evt_params.cmd) */

  if (p_buf) {
    VND_LOGD("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
    if (vnd_cb->xmit_cb(cmd, p_buf, hw_sco_config_cb))
      return;
    else
      vnd_cb->dealloc(p_buf);
  }

  VND_LOGE("Vendor lib scocfg aborted");
  vnd_cb->scocfg_cb(BT_VND_OP_RESULT_FAIL);
  hw_config_next();
}

/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          0 if success, -1 otherwise
**
*******************************************************************************/
static int8 hw_sco_config(void) {
  HC_BT_HDR* p_buf;
  int8 ret = -1;
  uint16_t cmd;
  if (enable_sco_config) {
    ALOGV("Start SCO config ...");
    assert(vnd_cb);

    /* Start with HCI_CMD_NXP_WRITE_PCM_SETTINGS */
    cmd = HCI_CMD_NXP_WRITE_PCM_SETTINGS;
    p_buf = build_cmd_buf(cmd, WRITE_PCM_SETTINGS_SIZE, write_pcm_settings);

    if (p_buf) {
      VND_LOGD("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
      if (vnd_cb->xmit_cb(cmd, p_buf, hw_sco_config_cb))
        return 0;
      else
        vnd_cb->dealloc(p_buf);
    }
    VND_LOGE("Vendor lib scocfg aborted");
    vnd_cb->scocfg_cb(BT_VND_OP_RESULT_FAIL);
  }
  return ret;
}

/*******************************************************************************
**
** Function        make_command
**
** Description     Prepare packet using opcode and parameter size
**
** Returns         Pointer to base address of HC_BT_HDR structure
**
*******************************************************************************/

static HC_BT_HDR* make_command(uint16_t opcode, size_t parameter_size) {
  HC_BT_HDR* packet = (HC_BT_HDR*)malloc(
      sizeof(HC_BT_HDR) + HCI_COMMAND_PREAMBLE_SIZE + parameter_size);
  if (!packet) {
    VND_LOGE("%s Failed to allocate buffer", __func__);
    return NULL;
  }
  uint8_t* stream = packet->data;
  packet->event = 0;
  packet->offset = 0;
  packet->layer_specific = 0;
  packet->len = HCI_COMMAND_PREAMBLE_SIZE + parameter_size;
  UINT16_TO_STREAM(stream, opcode);
  UINT8_TO_STREAM(stream, parameter_size);
  return packet;
}

/*******************************************************************************
**
** Function        hw_config_process_packet
**
** Description     Processes configuration packet received from controller
**
** Returns         NA
**
*******************************************************************************/
static void hw_config_process_packet(void* packet) {
  uint8_t *stream, event, event_code, status, opcode_offset;
  uint16_t opcode, len;
  char* p_tmp;
  if (packet != NULL) {
    HC_BT_HDR* p_evt_buf = (HC_BT_HDR*)packet;
    stream = ((HC_BT_HDR*)packet)->data;
    event = ((HC_BT_HDR*)packet)->event;
    len = ((HC_BT_HDR*)packet)->len;
    opcode_offset = HCI_EVENT_PREAMBLE_SIZE + 1; /*Skip num packets.*/
    VND_LOGD("Packet length %d", len);
    /*Minimum length of command event should be 6 bytes*/
    if ((event == HCI_PACKET_TYPE_EVENT) && (len >= 6)) {
      event_code = stream[0];
      opcode = stream[opcode_offset] | (stream[opcode_offset + 1] << 8);
      if (event_code == HCI_COMMAND_COMPLETE_EVT) {
        status = stream[opcode_offset + 2];
        VND_LOGD("Reply received for command 0x%04hX (%s) status 0x%02x",
                 opcode, cmd_to_str(opcode), status);
        switch (opcode) {
          case HCI_CMD_NXP_RESET:
            if (status == 0) {
              set_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED, 1);
            }
            break;
          case HCI_CMD_NXP_CUSTOM_OPCODE:
            if ((stream[opcode_offset + 3] ==
                 HCI_CMD_NXP_SUB_ID_BLE_TX_POWER) &&
                set_1m_2m_power) {
              hw_config.skip_seq_incr = 1;
            }
            break;
          case HCI_CMD_NXP_READ_FW_REVISION: {
            VND_LOGD("%s Read FW version reply recieved", __func__);
            if ((status == 0) && (len >= 14)) {
              if (len >= 15) {
                VND_LOGI("FW version: %d.%d.%d.p%d.%d", stream[8], stream[7],
                         stream[6], stream[9], stream[14]);
              } else {
                VND_LOGI("FW version: %d.%d.%d.p%d", stream[8], stream[7],
                         stream[6], stream[9]);
              }
              VND_LOGI("ROM version: %02X %02X %02X %02X", stream[10],
                       stream[11], stream[12], stream[13]);
            } else {
              VND_LOGE("%s Error while reading FW version", __func__);
            }
          } break;
          case HCI_READ_LOCAL_BDADDR: {
            p_tmp =
                (char*)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY;
            VND_LOGI("Controller BD address:  %02X:%02X:%02X:%02X:%02X:%02X",
                     *(p_tmp + 5), *(p_tmp + 4), *(p_tmp + 3), *(p_tmp + 2),
                     *(p_tmp + 1), *p_tmp);
            if (use_controller_addr && !IS_DEFAULT_BDADDR(p_tmp)) {
              ++hw_config.indx; /*Skip writting bd address*/
            }
          } break;
          case HCI_CMD_NXP_WRITE_BD_ADDRESS: {
            if (status == 0) {
              VND_LOGI("Controller BD address:  %02X:%02X:%02X:%02X:%02X:%02X",
                       write_bd_address[7], write_bd_address[6],
                       write_bd_address[5], write_bd_address[4],
                       write_bd_address[3], write_bd_address[2]);
            }
          } break;
          case HCI_CMD_NXP_INDEPENDENT_RESET_SETTING: {
            if ((independent_reset_mode == IR_MODE_INBAND_VSC) &&
                (status == 0)) {
              set_prop_int32(PROP_BLUETOOTH_INBAND_CONFIGURED, 1);
            }
          } break;
          case HCI_CMD_NXP_BLE_WAKEUP: {
            if (status != 0) {
              enable_heartbeat_config = FALSE;
            }
            wakeup_event_handler(stream[opcode_offset + 3]);
          } break;
          case HCI_CMD_NXP_SET_BT_SLEEP_MODE: {
            if (status == 0) {
              lpm_configured = TRUE;
            }
          } break;
          default:
            break;
        }
      }
    } else {
      VND_LOGE("Unexpected packet received. Event type:%02x Len:%02x", event,
               len);
    }
  }
}

/*******************************************************************************
**
** Function        hw_config_seq
**
** Description     Handles HW configuration sequence and event callbacks.
**
** Returns         NA
**
*******************************************************************************/
static void hw_config_seq(void* packet) {
  hw_config_process_packet(packet);
  VND_LOGD("skip=%d ", hw_config.skip_seq_incr);
  if (hw_config.skip_seq_incr) {
    hw_config.skip_seq_incr = 0;
  } else {
    ++hw_config.indx;
  }
  VND_LOGD("seq_indx=%d/%d", hw_config.indx, hw_config.size);
  if (hw_config.indx < hw_config.size) {
    if (hw_config_seq_arr[hw_config.indx]() != 0) {
      hw_config_next();
    }
  } else if (hw_config.indx == hw_config.size) {
    VND_LOGI("FW config completed!");
    if (vnd_cb) {
      vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
      if (enable_heartbeat_config == TRUE) {
        int status;
        status =
            pthread_create(&p_headtbeat, NULL, send_heartbeat_thread, NULL);
        VND_LOGV("pthread_create status %d", status);
      }
    }
  } else {
    VND_LOGE("Invalid HW config sequence");
    if (vnd_cb) {
      vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
    }
  }
}

/*******************************************************************************
**
** Function         bt_bdaddress_set
**
** Description      Sets the bdaddress as per address received from
**                  stack/OTP/bt_vendor.conf
**
** Returns          0 if success, -1 otherwise
**
*******************************************************************************/
static int8 bt_bdaddress_set(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  int8 ret = -1;
  opcode = HCI_CMD_NXP_WRITE_BD_ADDRESS;
  packet = make_command(opcode, WRITE_BD_ADDRESS_SIZE);
  if (packet) {
    memcpy(&packet->data[3], write_bd_address, WRITE_BD_ADDRESS_SIZE);
    VND_LOGD(
        "Writting new BD Address %02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
        write_bd_address[7], write_bd_address[6], write_bd_address[5],
        write_bd_address[4], write_bd_address[3], write_bd_address[2]);
    ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
  }
  return ret;
}

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          0 if success, -1 otherwise
**
*******************************************************************************/

static int8 hw_config_set_bdaddr(void) {
  int8 ret = -1;
  if (write_bdaddrss == 1) {
    ret = bt_bdaddress_set();
    write_bdaddrss = 0;
  } else if (bdaddr != NULL) {
    for (int i = 0; i < 6; i++) {
      write_bd_address[7 - i] = bdaddr[i];
    }
    ret = bt_bdaddress_set();
  }
  return ret;
}

/*******************************************************************************
**
** Function         hw_config_read_bdaddr
**
** Description      Read controller's Bluetooth Device Address
**
** Returns          0 if success, -1 otherwise
**
*******************************************************************************/
static int8 hw_config_read_bdaddr(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  int8 ret = -1;
  if (write_bdaddrss == 0) {
    opcode = HCI_READ_LOCAL_BDADDR;
    packet = make_command(opcode, 0);
    ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
  }
  return ret;
}

/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Start controller configurations initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void) {
  hw_config.indx = -1;
  hw_config.skip_seq_incr = 0;
  hw_config.size = sizeof(hw_config_seq_arr) / sizeof(hw_config_seq_arr[0]);
  hw_config.seq_arr = hw_config_seq_arr;
  if (enable_heartbeat_config == TRUE) {
    wake_gpio_config = 0;
  }
  hw_config_seq(NULL);
}

/*******************************************************************************
**
** Function        hw_bt_send_reset
**
** Description     Send HCI reset command to controller
**
** Returns         0 if success, -1 otherwise
**
*******************************************************************************/
static int8 hw_bt_send_reset(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_RESET;
  packet = make_command(opcode, 0);
  return hw_bt_send_packet(packet, opcode, hw_config_seq);
}

/******************************************************************************
 **
 ** Function         hw_bt_load_cal_file
 **
 ** Description      Loads HEX data from cal_file_name file to cal_data
 **
 ** Return Value     Incase of success *cal_data_size shall contain size of data
 **                  in cal_data pointer
 **                  Returns 0 incase of operations are successful, -1
 **                  otherwise.
 **
 *****************************************************************************/
static int hw_bt_load_cal_file(const char* cal_file_name, uint8_t* cal_data,
                               uint32_t* cal_data_size) {
  FILE* fp = fopen(cal_file_name, "r");
  unsigned int data;
  int data_len = 0;
  int ret = -1;
  if (fp == NULL) {
    VND_LOGD("Can't open calibration file: %s ", cal_file_name);
    return ret;
  } else {
    VND_LOGD("Calibration file: %s opened successfully", cal_file_name);
  }
  while ((fscanf(fp, " %2x", &data) == 1)) {
    if (data_len >= (*cal_data_size)) {
      VND_LOGE("%s cal_file size too big", __func__);
      goto done;
    }
    cal_data[data_len++] = (unsigned char)data;
  }
  if (data_len == BT_CONFIG_DATA_SIZE) {
    *cal_data_size = data_len;
    ret = 0;
  }

done:
  fclose(fp);
  return ret;
}
/******************************************************************************
 **
 ** Function:      hw_bt_cal_data_load
 **
 ** Description:   Loads the Calibration data and sends
 **                HCI_CMD_NXP_LOAD_CONFIG_DATA command.
 **
 ** Return Value:  0 if success, -1 otherwise
 **
 *****************************************************************************/
static int8 hw_bt_cal_data_load(void) {
  uint8_t cal_data[BT_CONFIG_DATA_SIZE];
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  int i;
  int8 ret = -1;
  uint32_t cal_data_size = sizeof(cal_data);
  VND_LOGD("Loading calibration Data");
  memset(cal_data, 0, cal_data_size);
  if (hw_bt_load_cal_file(pFilename_cal_data, cal_data, &cal_data_size)) {
    VND_LOGD("%s Error while processing calibration file", __func__);
    goto done;
  }
  opcode = HCI_CMD_NXP_LOAD_CONFIG_DATA;
  packet = make_command(opcode, HCI_CMD_NXP_LOAD_CONFIG_DATA_SIZE);
  if (packet) {
    stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
    stream[0] = 0x00;
    stream[1] = 0x00;
    stream[2] = 0x00; /* Ignore checksum */
    stream[3] = 0x1C; /* Data Len */
    /* Convert cal data in Little Endian format for every 4 bytes */
    for (i = 4; i < HCI_CMD_NXP_LOAD_CONFIG_DATA_SIZE; i++) {
      stream[i] = *(cal_data + ((i / 4) * 8 - 1 - i));
    }
    ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
  }
done:
  return ret;
}
/******************************************************************************
 **
 ** Function:      hw_ble_send_power_level_cmd
 **
 ** Description:   Send BLE TX power level command.
 **
 ** Return Value: 0 if success, -1 otherwise
 **
 *****************************************************************************/
static int8 hw_ble_send_power_level_cmd(uint8_t phy_level, int8_t power_level) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  int8 ret = -1;
  opcode = HCI_CMD_NXP_CUSTOM_OPCODE;
  packet = make_command(opcode, HCI_CMD_NXP_BLE_TX_POWER_DATA_SIZE);
  if (packet) {
    stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
    stream[0] = HCI_CMD_NXP_SUB_ID_BLE_TX_POWER;
    stream[1] = phy_level;
    stream[2] = power_level;
    ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
  }
  return ret;
}

/******************************************************************************
 **
 ** Function:      hw_ble_set_power_level
 **
 ** Description:   Set BLE TX power level for PHY1 and PHY2.
 **
 ** Return Value: 0 if success, -1 otherwise
 **
 *****************************************************************************/
static int8 hw_ble_set_power_level(void) {
  int8 ret = -1;
  if (set_1m_2m_power & BLE_SET_1M_POWER) {
    VND_LOGD("Setting BLE 1M TX power level at %d dbm", ble_1m_power);
    set_1m_2m_power &= ~BLE_SET_1M_POWER;
    ret = hw_ble_send_power_level_cmd(1, ble_1m_power);
  } else if (set_1m_2m_power & BLE_SET_2M_POWER) {
    VND_LOGD("Setting BLE 2M TX power level at %d dbm", ble_2m_power);
    set_1m_2m_power &= ~BLE_SET_2M_POWER;
    ret = hw_ble_send_power_level_cmd(2, ble_2m_power);
  }
  return ret;
}
/******************************************************************************
 **
 ** Function:      hw_bt_enable_max_power_level_cmd
 **
 ** Description:   Send BT TX power level command.
 **
 ** Return Value: 0 if success, -1 otherwise
 **
 *****************************************************************************/
static int8 hw_bt_enable_max_power_level_cmd(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  int8 ret = -1;
  if (bt_set_max_power) {
    opcode = HCI_CMD_NXP_WRITE_BT_TX_POWER;
    packet = make_command(opcode, HCI_CMD_NXP_BT_TX_POWER_DATA_SIZE);
    if (packet) {
      stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
      stream[0] = bt_max_power_sel;
      ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
    }
  }
  return ret;
}

/******************************************************************************
 **
 ** Function:      hw_bt_enable_independent_reset
 **
 ** Description:   Sends command to enable Inband / OutofBand independent reset
 **
 ** Return Value:  0 if success, -1 otherwise
 **
 *****************************************************************************/
static int8 hw_bt_enable_independent_reset(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  int8 ret = -1;
  if ((independent_reset_mode == IR_MODE_OOB_VSC) ||
      (independent_reset_mode == IR_MODE_INBAND_VSC)) {
    opcode = HCI_CMD_NXP_INDEPENDENT_RESET_SETTING;
    packet = make_command(opcode, HCI_CMD_NXP_INDEPENDENT_RESET_SETTING_SIZE);
    if (packet) {
      stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
      stream[0] = independent_reset_mode;
      if (independent_reset_mode == IR_MODE_OOB_VSC) {
        stream[1] = independent_reset_gpio_pin;
      } else {
        stream[1] = 0xFF;
      }
      ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
    }
  }
  return ret;
}

/******************************************************************************
 **
 ** Function:      hw_bt_configure_lpm
 **
 ** Description:   Sends command to configure low power mode
 **
 ** Return Value:  0 if success, -1 otherwise
 **
 *****************************************************************************/
int8 hw_bt_configure_lpm(uint8 sleep_mode) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  int8 ret = -1;
  opcode = HCI_CMD_NXP_SET_BT_SLEEP_MODE;
  packet = make_command(opcode, HCI_CMD_NXP_SET_BT_SLEEP_MODE_SIZE);
  if (packet) {
    stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
    stream[0] = sleep_mode;
    stream[1] = 0;
    stream[2] = 0;
    ret = hw_bt_send_packet(packet, opcode, hw_config_process_packet);
  }
  return ret;
}

/*******************************************************************************
**
** Function         kill_thread_signal_handler
**
** Description      a signal handler to kill the heartbeat thread
**
** Returns          None
**
*******************************************************************************/
static void kill_thread_signal_handler(int sig) {
  (void)sig;
  VND_LOGD("send_heartbeat_thread killed");
  pthread_exit(0);
}

/*******************************************************************************
**
** Function         send_heartbeat_thread
**
** Description      thread function for periodically sending HCI cmd (heartbeat)
**
** Returns          None
**
*******************************************************************************/
static void* send_heartbeat_thread(void* data) {
  uint16_t opcode = HCI_CMD_NXP_BLE_WAKEUP;
  HC_BT_HDR* packet;
  (void)data;

  send_heartbeat = TRUE;
  VND_LOGD("Starting Heartbeat Thread");
  signal(SIGUSR1, kill_thread_signal_handler);
  while (send_heartbeat) {
    fw_upload_DelayInMs(wakup_local_param_config.heartbeat_timer_value * 100);
    pthread_mutex_lock(&mtx_wakeup);
    packet = make_command(opcode, HCI_CMD_NXP_SUB_OCF_SIZE);
    if (packet) {
      packet->data[3] = HCI_CMD_NXP_SUB_OCF_HEARTBEAT;
      hw_bt_send_packet(packet, opcode, hw_config_process_packet);
    }
    while (heartbeat_event_received == false) {
      pthread_cond_wait(&cond_wakeup, &mtx_wakeup);
    }
    heartbeat_event_received = false;
    pthread_mutex_unlock(&mtx_wakeup);
  }
  return NULL;
}

/*******************************************************************************
**
** Function         set_wakeup_gpio_config
**
** Description      set configurations related to GPIO
**
** Returns          None
**
*******************************************************************************/
static int8 set_wakeup_gpio_config(void) {
  uint16_t opcode;
  int8 ret = -1;
  HC_BT_HDR* packet;
  if ((enable_heartbeat_config == TRUE) &&
      (wake_gpio_config < wakeup_key_num)) {
    opcode = HCI_CMD_NXP_BLE_WAKEUP;
    packet = make_command(
        opcode, HCI_CMD_NXP_SUB_OCF_SIZE + HCI_CMD_NXP_GPIO_CONFIG_SIZE);
    if (packet != NULL) {
      packet->data[3] = HCI_CMD_NXP_SUB_OCF_GPIO_CONFIG;
      packet->data[4] = wake_gpio_config;
      packet->data[5] = wakeup_gpio_config[wake_gpio_config].gpio_pin;
      packet->data[6] = wakeup_gpio_config[wake_gpio_config].high_duration;
      packet->data[7] = wakeup_gpio_config[wake_gpio_config].low_duration;
      VND_LOGD("wakeup %s send out command successfully, %d, %d, %d, %d",
               __func__, wake_gpio_config,
               wakeup_gpio_config[wake_gpio_config].gpio_pin,
               wakeup_gpio_config[wake_gpio_config].high_duration,
               wakeup_gpio_config[wake_gpio_config].low_duration);
      wakeup_gpio_config_state = wake_gpio_config;
      wake_gpio_config++;
      ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
    }
  }
  return ret;
}

/*******************************************************************************
**
** Function         set_wakeup_adv_pattern
**
** Description      set specific advertising pattern for wakeup
**
** Returns          None
**
*******************************************************************************/
static int8 set_wakeup_adv_pattern(void) {
  uint16_t opcode;
  int8 ret = -1;
  HC_BT_HDR* packet;
  if (enable_heartbeat_config == TRUE) {
    opcode = HCI_CMD_NXP_BLE_WAKEUP;
    packet = make_command(opcode, HCI_CMD_NXP_SUB_OCF_SIZE +
                                      HCI_CMD_NXP_ADV_PATTERN_LENGTH_SIZE +
                                      wakeup_adv_config.length);
    if (packet) {
      packet->data[3] = HCI_CMD_NXP_SUB_OCF_ADV_PATTERN_CONFIG;
      packet->data[4] = wakeup_adv_config.length;
      memcpy(&packet->data[5], &wakeup_adv_config.adv_pattern[0],
             wakeup_adv_config.length);
      VND_LOGD(
          "wakeup %s send out command successfully local, %d, %d, %d, %d, %d, "
          "%d",
          __func__, wakeup_adv_config.length, wakeup_adv_config.adv_pattern[0],
          wakeup_adv_config.adv_pattern[1], wakeup_adv_config.adv_pattern[4],
          wakeup_adv_config.adv_pattern[7], wakeup_adv_config.adv_pattern[8]);
      VND_LOGD(
          "wakeup %s send out command successfully hci, %d, %d, %d, %d, %d, "
          "%d",
          __func__, packet->data[4], packet->data[5], packet->data[6],
          packet->data[10], packet->data[12], packet->data[13]);
      ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
    }
  }
  return ret;
}
/*******************************************************************************
**
** Function         set_wakeup_scan_parameter
**
** Description      set scan parameter for wakeup LE scan
**
** Returns          None
**
*******************************************************************************/
static int8 set_wakeup_scan_parameter(void) {
  uint16_t opcode;
  int8 ret = -1;
  HC_BT_HDR* packet;
  if (enable_heartbeat_config == TRUE) {
    opcode = HCI_CMD_NXP_BLE_WAKEUP;
    packet = make_command(
        opcode, HCI_CMD_NXP_SUB_OCF_SIZE + HCI_CMD_NXP_SCAN_PARAM_CONFIG_SIZE);
    if (packet) {
      packet->data[3] = HCI_CMD_NXP_SUB_OCF_SCAN_PARAM_CONFIG;
      packet->data[4] = wakeup_scan_param_config.le_scan_type;
      packet->data[5] = (unsigned char)wakeup_scan_param_config.interval;
      packet->data[6] = (unsigned char)(wakeup_scan_param_config.interval >> 8);
      packet->data[7] = (unsigned char)wakeup_scan_param_config.window;
      packet->data[8] = (unsigned char)(wakeup_scan_param_config.window >> 8);
      packet->data[9] = wakeup_scan_param_config.own_addr_type;
      packet->data[10] = wakeup_scan_param_config.scan_filter_policy;
      VND_LOGD("wakeup %s send out paramter, %d, %d, %d, %d, %d", __func__,
               wakeup_scan_param_config.le_scan_type,
               (packet->data[5] | (packet->data[6] << 8)),
               (packet->data[7] | (packet->data[8] << 8)),
               wakeup_scan_param_config.own_addr_type,
               wakeup_scan_param_config.scan_filter_policy);
      ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
    }
  }
  return ret;
}
/*******************************************************************************
**
** Function         set_wakeup_local_parameter
**
** Description      set parameter for local configuration
**
** Returns          None
**
*******************************************************************************/
static int8 set_wakeup_local_parameter(void) {
  uint16_t opcode;
  int ret = -1;
  if (enable_heartbeat_config == TRUE) {
    HC_BT_HDR* packet;
    opcode = HCI_CMD_NXP_BLE_WAKEUP;
    packet = make_command(
        opcode, HCI_CMD_NXP_SUB_OCF_SIZE + HCI_CMD_NXP_LOCAL_PARAM_CONFIG_SIZE);
    if (packet) {
      packet->data[3] = HCI_CMD_NXP_SUB_OCF_LOCAL_PARAM_CONFIG;
      packet->data[4] = wakup_local_param_config.heartbeat_timer_value;
      ret = hw_bt_send_packet(packet, opcode, hw_config_seq);
    }
  }
  return ret;
}
/*******************************************************************************
**
** Function         wakeup_event_handler
**
** Description      handle wakeup related hci event
**
** Returns          None
**
*******************************************************************************/
static void wakeup_event_handler(uint8_t sub_ocf) {
  switch (sub_ocf) {
    case HCI_CMD_NXP_SUB_OCF_HEARTBEAT:
      pthread_mutex_lock(&mtx_wakeup);
      heartbeat_event_received = true;
      pthread_cond_signal(&cond_wakeup);
      pthread_mutex_unlock(&mtx_wakeup);
      break;
    case HCI_CMD_NXP_SUB_OCF_GPIO_CONFIG:
      if (wake_gpio_config < wakeup_key_num) {
        hw_config.skip_seq_incr = 1;
      }
      break;
    default:
      break;
  }
}

/******************************************************************************
 **
 ** Function:      wakeup_kill_heartbeat_thread
 **
 ** Description:   launch a signal to kill heartbeat thread when BT Off
 **
 ** Return Value:  NA
 **
 *****************************************************************************/
void wakeup_kill_heartbeat_thread(void) {
  int status = -1;
  VND_LOGD("Killing heartbeat thread");
  if (send_heartbeat) {
    send_heartbeat = FALSE;
    status = pthread_kill(p_headtbeat, SIGUSR1);
    fw_upload_DelayInMs(10);
  }
  VND_LOGD("Killed heartbeat with status %d", status);
}

/******************************************************************************
 **
 ** Function:      hw_bt_read_fw_revision
 **
 ** Description:   Sends command to read revision
 **
 ** Return Value:  0 if success, -1 otherwise
 **
 *****************************************************************************/
static int8 hw_bt_read_fw_revision(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_READ_FW_REVISION;
  packet = make_command(opcode, 0);
  return hw_bt_send_packet(packet, opcode, hw_config_seq);
}
