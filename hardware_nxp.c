/******************************************************************************
 *
 *  Copyright 2012-2013, 2020-2021 NXP
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

#define LOG_TAG "hardware_nxp"

#include <assert.h>
#include <log/log.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bt_hci_bdroid.h"
#include "bt_vendor_nxp.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef NXP_INIT_SCO_CFG_INCLUDED
#define NXP_INIT_SCO_CFG_INCLUDED TRUE
#endif

#ifndef USE_CONTROLLER_BDADDR
#define USE_CONTROLLER_BDADDR TRUE
#endif

#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
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
#endif

#define HCI_CMD_NXP_WRITE_BD_ADDRESS 0xFC22
#define HCI_BT_SET_EVENTMASK_OCF 0x0001
#define HCI_CONTROLLER_CMD_OGF 0x03
#define HCI_RESET_OCF 0x03
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

#define OpCodePack(ogf, ocf) (uint16_t)((ocf & 0x03ff) | (ogf << 10))

#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
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
#endif

/******************************************************************************
**  Local type definitions
******************************************************************************/

struct bt_evt_param_t {
  uint16_t cmd;
  uint8_t cmd_ret_param;
};

/***********************************************************
 *  Prototype
 ***********************************************************
 */
#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
static void hw_sco_config(void);
#endif
static void hw_config_set_bdaddr(void);
static void hw_bt_read_fw_revision(void);
#if (NXP_ENABLE_INDEPENDENT_RESET_VSC == TRUE)
static void hw_bt_enable_independent_reset(void);
#endif
#if (USE_CONTROLLER_BDADDR == TRUE)
static void hw_config_read_bdaddr(void);
#endif
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
static int hw_bt_cal_data_load(void);
#endif
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
static void hw_ble_set_power_level(void);
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
static void hw_bt_enable_max_power_level_cmd(void);
#endif
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
static void set_wakeup_scan_parameter(void);
static void wakeup_event_handler(uint8_t sub_ocf, uint8_t status);
#endif
/***********************************************************
 *  Local variables
 ***********************************************************
 */

#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
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
#endif

#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
static pthread_mutex_t mtx_wakeup = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond_wakeup = PTHREAD_COND_INITIALIZER;
static pthread_t p_headtbeat;
static bool heartbeat_event_received = false;
static unsigned char wakeup_gpio_config_state = wakeup_key_num;
static bool send_heartbeat = FALSE;
#endif

/***********************************************************
**  HELPER FUNCTIONS
***********************************************************/
static char* cmd_to_str(uint16_t cmd) {
  switch (cmd) {
#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
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
#endif
    case HCI_CMD_NXP_WRITE_BD_ADDRESS:
      return "write_bd_address";
    default:
      break;
  }

  return "unknown command";
}

#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
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
      ALOGI("SCO PCM config succeeds!");
      vnd_cb->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
      /* fw config succeeds */
      ALOGI("FW config succeeds!");
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
      set_wakeup_scan_parameter();
#else
      vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
      return;

    default:
      ALOGE("Received event for unexpected cmd (0x%04hX). Fail.",
            evt_params.cmd);
      p_buf = NULL;
      break;
  } /* switch (evt_params.cmd) */

  if (p_buf) {
    ALOGI("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
    if (vnd_cb->xmit_cb(cmd, p_buf, hw_sco_config_cb))
      return;
    else
      vnd_cb->dealloc(p_buf);
  }

  ALOGE("Vendor lib scocfg aborted");
  vnd_cb->scocfg_cb(BT_VND_OP_RESULT_FAIL);
  /* fw config succeeds */
  ALOGI("FW config succeeds!");
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
  set_wakeup_scan_parameter();
#else
  vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
}

/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          None
**
*******************************************************************************/
static void hw_sco_config(void) {
  HC_BT_HDR* p_buf;
  uint16_t cmd;

  ALOGV("Start SCO config ...");
  assert(vnd_cb);

  /* Start with HCI_CMD_NXP_WRITE_PCM_SETTINGS */
  cmd = HCI_CMD_NXP_WRITE_PCM_SETTINGS;
  p_buf = build_cmd_buf(cmd, WRITE_PCM_SETTINGS_SIZE, write_pcm_settings);

  if (p_buf) {
    ALOGI("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
    if (vnd_cb->xmit_cb(cmd, p_buf, hw_sco_config_cb))
      return;
    else
      vnd_cb->dealloc(p_buf);
  }

  ALOGE("Vendor lib scocfg aborted");
  vnd_cb->scocfg_cb(BT_VND_OP_RESULT_FAIL);
  /* fw config succeeds */
  ALOGI("FW config succeeds!");
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
  set_wakeup_scan_parameter();
#else
  vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
}
#endif

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
    VNDDBG("%s Failed to allocate buffer\n", __func__);
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
inline static void bt_update_bdaddr(void) {
#if (USE_CONTROLLER_BDADDR == TRUE)
  if (write_bdaddrss == 0) {
    hw_config_read_bdaddr();
  } else {
    hw_config_set_bdaddr();
  }
#else
  hw_config_set_bdaddr();
#endif
}
/*******************************************************************************
**
** Function         hw_config_callback
**
** Description      Callback function for controller configuration
**
** Returns          None
**
*******************************************************************************/
static void hw_config_callback(void* packet) {
  uint8_t* stream, event, event_code, status, opcode_offset;
  uint16_t opcode, len;
#if (USE_CONTROLLER_BDADDR == TRUE)
  char* p_tmp;
  HC_BT_HDR* p_evt_buf = (HC_BT_HDR*)packet;
#endif
  stream = ((HC_BT_HDR*)packet)->data;
  event = ((HC_BT_HDR*)packet)->event;
  len = ((HC_BT_HDR*)packet)->len;
  opcode_offset = HCI_EVENT_PREAMBLE_SIZE + 1; /*Skip num packets.*/
  VNDDBG("Packet length %d", len);
  /*Minimum length of commad event should be 6 bytes*/
  if ((event == HCI_PACKET_TYPE_EVENT) && (len >= 6)) {
    event_code = stream[0];
    opcode = stream[opcode_offset] | (stream[opcode_offset + 1] << 8);
    if (event_code == HCI_COMMAND_COMPLETE_EVT) {
      status = stream[opcode_offset + 2];
      ALOGI("opcode 0x%04x status 0x%02x\n", opcode, status);
      switch (opcode) {
        case OpCodePack(HCI_CONTROLLER_CMD_OGF, HCI_RESET_OCF): {
          VNDDBG("Receive hci reset complete event");
#if (NXP_ENABLE_INDEPENDENT_RESET_VSC == TRUE)
          if ((independent_reset_mode == IR_MODE_OOB_VSC) ||
              (independent_reset_mode == IR_MODE_INBAND_VSC)) {
            hw_bt_enable_independent_reset();
          } else {
            VNDDBG("independent_reset_mode not enabled in bt_vendor.conf file");
            hw_bt_read_fw_revision();
          }
#else
          hw_bt_read_fw_revision();
#endif
          break;
        }
        case HCI_CMD_NXP_INDEPENDENT_RESET_SETTING: {
          VNDDBG("Independent reset command completed");
          hw_bt_read_fw_revision();
          break;
        }
        case HCI_CMD_NXP_READ_FW_REVISION: {
          VNDDBG("%s Read FW version reply recieved", __func__);
          if ((status == 0) && (len >= 14)) {
            if (len >= 15) {
              ALOGI("FW version: %d.%d.%d.p%d.%d", stream[8], stream[7],
                    stream[6], stream[9], stream[14]);
            } else {
              ALOGI("FW version: %d.%d.%d.p%d", stream[8], stream[7], stream[6],
                    stream[9]);
            }
            ALOGI("ROM version: %02X %02X %02X %02X", stream[10], stream[11],
                  stream[12], stream[13]);
          } else {
            ALOGI("%s Error while reading FW version", __func__);
          }
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
          if (hw_bt_cal_data_load()) {
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
            if (set_1m_2m_power) {
              hw_ble_set_power_level();
            } else
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
                if (bt_set_max_power) {
              hw_bt_enable_max_power_level_cmd();
            } else
#endif
            {
              bt_update_bdaddr();
            }
          }
#else
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
          if (set_1m_2m_power) {
            hw_ble_set_power_level();
          } else
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
              if (bt_set_max_power) {
            hw_bt_enable_max_power_level_cmd();
          } else
#endif
          {
            bt_update_bdaddr();
          }
#endif
          break;
        }
        case HCI_CMD_NXP_WRITE_BD_ADDRESS: {
          VNDDBG("Receive BD_ADDRESS write config event.\n");
#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
          hw_sco_config();
#else
          /* fw config succeeds */
          VNDDBG("FW config succeeds!");
          if (vnd_cb) {
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
            set_wakeup_scan_parameter();
#else
            vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
          }
#endif
          break;
        }
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
        case HCI_CMD_NXP_CUSTOM_OPCODE: {
          VNDDBG("Receive HCI_CMD_NXP_CUSTOM_OPCODE complete event.\n");
          /*Minimum length in case of HCI_CMD_NXP_CUSTOM_OPCODE should be 7
           * bytes*/
          if (len >= 7) {
            VNDDBG("Subid: %02x", stream[opcode_offset + 3]);
            switch (stream[opcode_offset + 3]) {
              case HCI_CMD_NXP_SUB_ID_BLE_TX_POWER: {
                if (set_1m_2m_power) {
                  hw_ble_set_power_level();
                } else
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
                    if (bt_set_max_power) {
                  hw_bt_enable_max_power_level_cmd();
                } else
#endif
                {
                  bt_update_bdaddr();
                }
              } break;
              default:
                ALOGE("Received event for unexpected subid");
                break;
            }
          } else {
            ALOGE("Subid not received");
          }
          break;
        }
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
        case HCI_CMD_NXP_WRITE_BT_TX_POWER: {
          VNDDBG("Receive HCI_CMD_NXP_WRITE_BT_TX_POWER complete event.\n");
          bt_update_bdaddr();
          break;
        }
#endif
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
        case HCI_CMD_NXP_LOAD_CONFIG_DATA: {
          VNDDBG("Receive HCI_CMD_NXP_LOAD_CONFIG_DATA complete event.\n");
#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
          if (set_1m_2m_power) {
            hw_ble_set_power_level();
          } else
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
              if (bt_set_max_power) {
            hw_bt_enable_max_power_level_cmd();
          } else
#endif
          {
            bt_update_bdaddr();
          }
          break;
        }
#endif
#if (USE_CONTROLLER_BDADDR == TRUE)
        case HCI_READ_LOCAL_BDADDR: {
          p_tmp = (char*)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY;
          if (IS_DEFAULT_BDADDR(p_tmp)) {
            hw_config_set_bdaddr();
          } else {
            VNDDBG("Controller OTP bdaddr %02X:%02X:%02X:%02X:%02X:%02X",
                   *(p_tmp + 5), *(p_tmp + 4), *(p_tmp + 3), *(p_tmp + 2),
                   *(p_tmp + 1), *p_tmp);
#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
            hw_sco_config();
#else
            /* fw config succeeds */
            VNDDBG("FW config succeeds!");
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
            set_wakeup_scan_parameter();
#else
            if (vnd_cb) {
              vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
            }
#endif
#endif
          }
          break;
        }
#endif
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
        case HCI_CMD_NXP_BLE_WAKEUP: {
          wakeup_event_handler(stream[opcode_offset + 3], status);
          break;
        }
#endif
      }
    }
  }
}

/*******************************************************************************
**
** Function         bt_bdaddress_set
**
** Description      Sets the bdaddress as per address received from stack/OTP/
                    bt_vendor.conf
**
** Returns          None
**
*******************************************************************************/
static void bt_bdaddress_set(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_WRITE_BD_ADDRESS;
  packet = make_command(opcode, WRITE_BD_ADDRESS_SIZE);
  if (packet) {
    memcpy(&packet->data[3], write_bd_address, WRITE_BD_ADDRESS_SIZE);
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      ALOGI("Sending hci command 0x%04hX (%s)", opcode, cmd_to_str(opcode));
      VNDDBG("bdaddr is %02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX\n",
             write_bd_address[7], write_bd_address[6], write_bd_address[5],
             write_bd_address[4], write_bd_address[3], write_bd_address[2]);
    }
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
}

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          None
**
*******************************************************************************/

static void hw_config_set_bdaddr(void) {
  /* bd_address is not set in bt_vendor.conf */
  VNDDBG("%s", __func__);
  if (write_bdaddrss == 0) {
    if (bdaddr == NULL) {
#if (NXP_INIT_SCO_CFG_INCLUDED == TRUE)
      hw_sco_config();
#else
      /* fw config succeeds */
      VNDDBG("FW config succeeds!");
#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
      set_wakeup_scan_parameter();
#else
      if (vnd_cb) vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
#endif
      return;
    } else if (bdaddr) {
      for (int i = 0; i < 6; i++) {
        write_bd_address[7 - i] = bdaddr[i];
      }
    }
  }
  bt_bdaddress_set();
  if (write_bdaddrss == 1) write_bdaddrss = 0;
}

#if (USE_CONTROLLER_BDADDR == TRUE)
/*******************************************************************************
**
** Function         hw_config_read_bdaddr
**
** Description      Read controller's Bluetooth Device Address
**
** Returns          None
**
*******************************************************************************/
static void hw_config_read_bdaddr(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_READ_LOCAL_BDADDR;
  packet = make_command(opcode, 0);
  if (packet) {
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback))
      VNDDBG("%s send out command successfully\n", __func__);
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
}
#endif

/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Start controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = OpCodePack(HCI_CONTROLLER_CMD_OGF, HCI_RESET_OCF);
  packet = make_command(opcode, 0);
  if (packet) {
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback))
      VNDDBG("hci reset command sent out successfully\n");
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
}

#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
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
    VNDDBG("Can't open calibration file: %s ", cal_file_name);
    return ret;
  }
  while ((fscanf(fp, " %2x", &data) == 1)) {
    if (data_len >= (*cal_data_size)) {
      VNDDBG("%s cal_file size too big \n", __func__);
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
 ** Return Value:  Incase case of success returns 0 else -1.
 **
 *****************************************************************************/
static int hw_bt_cal_data_load(void) {
  uint8_t cal_data[BT_CONFIG_DATA_SIZE];
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  int i;
  int ret = -1;
  uint32_t cal_data_size = sizeof(cal_data);
  ALOGI("Loading calibration Data");
  memset(cal_data, 0, cal_data_size);
  if (hw_bt_load_cal_file(pFilename_cal_data, cal_data, &cal_data_size)) {
    ALOGE("%s Error while processing calibration file \n", __func__);
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

    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      ret = 0;
      ALOGI("%s Sent Calibration Data(0x%04x) successfully\n", __func__,
            HCI_CMD_NXP_LOAD_CONFIG_DATA);
    }
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
done:
  return ret;
}
#endif

#if (NXP_SET_BLE_TX_POWER_LEVEL == TRUE)
/******************************************************************************
 **
 ** Function:      hw_ble_send_power_level_cmd
 **
 ** Description:   Send BLE TX power level command.
 **
 ** Return Value: NA
 **
 *****************************************************************************/
static void hw_ble_send_power_level_cmd(uint8_t phy_level, int8_t power_level) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  opcode = HCI_CMD_NXP_CUSTOM_OPCODE;
  packet = make_command(opcode, HCI_CMD_NXP_BLE_TX_POWER_DATA_SIZE);
  if (packet) {
    stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
    stream[0] = HCI_CMD_NXP_SUB_ID_BLE_TX_POWER;
    stream[1] = phy_level;
    stream[2] = power_level;

    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      ALOGI("%s Sent BLE power for %d PHY successfully\n", __func__, phy_level);
    }
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
  return;
}

/******************************************************************************
 **
 ** Function:      hw_ble_set_power_level
 **
 ** Description:   Set BLE TX power level for PHY1 and PHY2.
 **
 ** Return Value: NA
 **
 *****************************************************************************/
static void hw_ble_set_power_level(void) {
  if (set_1m_2m_power & BLE_SET_1M_POWER) {
    ALOGI("Setting BLE 1M TX power level at %d dbm", ble_1m_power);
    set_1m_2m_power &= ~BLE_SET_1M_POWER;
    hw_ble_send_power_level_cmd(1, ble_1m_power);
  } else if (set_1m_2m_power & BLE_SET_2M_POWER) {
    ALOGI("Setting BLE 2M TX power level at %d dbm", ble_2m_power);
    set_1m_2m_power &= ~BLE_SET_2M_POWER;
    hw_ble_send_power_level_cmd(2, ble_2m_power);
  }
}
#endif
#if (NXP_ENABLE_BT_TX_MAX_POWER == TRUE)
/******************************************************************************
 **
 ** Function:      hw_bt_enable_max_power_level_cmd
 **
 ** Description:   Send BT TX power level command.
 **
 ** Return Value: NA
 **
 *****************************************************************************/
static void hw_bt_enable_max_power_level_cmd(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
  opcode = HCI_CMD_NXP_WRITE_BT_TX_POWER;
  packet = make_command(opcode, HCI_CMD_NXP_BT_TX_POWER_DATA_SIZE);
  if (packet) {
    stream = &packet->data[HCI_COMMAND_PREAMBLE_SIZE];
    stream[0] = bt_max_power_sel;
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      ALOGI("%s Sent Max BT power successfully with %d parameter\n", __func__,
            bt_max_power_sel);
    }
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
  return;
}

#endif

#if (NXP_ENABLE_INDEPENDENT_RESET_VSC == TRUE)
/******************************************************************************
 **
 ** Function:      hw_bt_enable_independent_reset
 **
 ** Description:   Sends command to enable Inband / OutofBand independent reset
 **
 ** Return Value:  NA
 **
 *****************************************************************************/
static void hw_bt_enable_independent_reset(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  uint8_t* stream;
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
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      ALOGI("%s Enable Independent Reset %d command sent\n", __func__,
            independent_reset_mode);
    }
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
  return;
}
#endif

#if (NXP_HEARTBEAT_FEATURE_SUPPORT == TRUE)
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
  ALOGI("send_heartbeat_thread killed\n");
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
  (void)data;
  send_heartbeat = TRUE;

  usleep(100000);
  signal(SIGUSR1, kill_thread_signal_handler);
  while (send_heartbeat) {
    usleep(wakup_local_param_config.heartbeat_timer_value * 100 *
           TIMER_UNIT_MS_TO_US);

    pthread_mutex_lock(&mtx_wakeup);

    uint16_t opcode;
    HC_BT_HDR* packet;
    opcode = HCI_CMD_NXP_BLE_WAKEUP;
    packet = make_command(opcode, HCI_CMD_NXP_SUB_OCF_SIZE);
    if (packet) {
      packet->data[3] = HCI_CMD_NXP_SUB_OCF_HEARTBEAT;
      if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
        VNDDBG("wakeup %s send out command successfully \n", __func__);
      }
    } else {
      VNDDBG("wakeup %s no valid packet \n", __func__);
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
static void set_wakeup_gpio_config(unsigned char pattern_index) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_BLE_WAKEUP;
  packet = make_command(
      opcode, HCI_CMD_NXP_SUB_OCF_SIZE + HCI_CMD_NXP_GPIO_CONFIG_SIZE);
  if (packet) {
    packet->data[3] = HCI_CMD_NXP_SUB_OCF_GPIO_CONFIG;
    packet->data[4] = pattern_index;
    packet->data[5] = wakeup_gpio_config[pattern_index].gpio_pin;
    packet->data[6] = wakeup_gpio_config[pattern_index].high_duration;
    packet->data[7] = wakeup_gpio_config[pattern_index].low_duration;
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      VNDDBG("wakeup %s send out command successfully, %d, %d, %d, %d,\n",
             __func__, pattern_index,
             wakeup_gpio_config[pattern_index].gpio_pin,
             wakeup_gpio_config[pattern_index].high_duration,
             wakeup_gpio_config[pattern_index].low_duration);
      wakeup_gpio_config_state = pattern_index;
    }
  } else {
    VNDDBG("wakeup %s no valid packet \n", __func__);
  }
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
static void set_wakeup_adv_pattern(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_BLE_WAKEUP;
  packet = make_command(opcode, HCI_CMD_NXP_SUB_OCF_SIZE +
                                    HCI_CMD_NXP_ADV_PATTERN_LENGTH_SIZE +
                                    wakeup_adv_config.length);
  if (packet) {
    packet->data[3] = HCI_CMD_NXP_SUB_OCF_ADV_PATTERN_CONFIG;
    packet->data[4] = wakeup_adv_config.length;
    memcpy(&packet->data[5], &wakeup_adv_config.adv_pattern[0],
           wakeup_adv_config.length);
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      VNDDBG(
          "wakeup %s send out command successfully local, %d, %d, %d, %d, %d, "
          "%d\n",
          __func__, wakeup_adv_config.length, wakeup_adv_config.adv_pattern[0],
          wakeup_adv_config.adv_pattern[1], wakeup_adv_config.adv_pattern[4],
          wakeup_adv_config.adv_pattern[7], wakeup_adv_config.adv_pattern[8]);
      VNDDBG(
          "wakeup %s send out command successfully hci, %d, %d, %d, %d, %d, "
          "%d\n",
          __func__, packet->data[4], packet->data[5], packet->data[6],
          packet->data[10], packet->data[12], packet->data[13]);
    }
  } else {
    VNDDBG("wakeup %s no valid packet \n", __func__);
  }
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
static void set_wakeup_scan_parameter(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
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
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      VNDDBG("wakeup %s send out command successfully, %d, %d, %d, %d, %d\n",
             __func__, wakeup_scan_param_config.le_scan_type,
             (packet->data[5] | (packet->data[6] << 8)),
             (packet->data[7] | (packet->data[8] << 8)),
             wakeup_scan_param_config.own_addr_type,
             wakeup_scan_param_config.scan_filter_policy);
    }
  } else {
    VNDDBG("wakeup %s no valid packet \n", __func__);
  }
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
static void set_wakeup_local_parameter(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_BLE_WAKEUP;
  packet = make_command(
      opcode, HCI_CMD_NXP_SUB_OCF_SIZE + HCI_CMD_NXP_LOCAL_PARAM_CONFIG_SIZE);
  if (packet) {
    packet->data[3] = HCI_CMD_NXP_SUB_OCF_LOCAL_PARAM_CONFIG;
    packet->data[4] = wakup_local_param_config.heartbeat_timer_value;
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      VNDDBG("wakeup %s send out command successfully, %d\n", __func__,
             wakup_local_param_config.heartbeat_timer_value);
    }
  } else {
    VNDDBG("wakeup %s no valid packet \n", __func__);
  }
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
static void wakeup_event_handler(uint8_t sub_ocf, uint8_t status) {
  if (status == 0) {
    switch (sub_ocf) {
      case HCI_CMD_NXP_SUB_OCF_HEARTBEAT:
        pthread_mutex_lock(&mtx_wakeup);
        heartbeat_event_received = true;
        pthread_cond_signal(&cond_wakeup);
        pthread_mutex_unlock(&mtx_wakeup);
        break;
      case HCI_CMD_NXP_SUB_OCF_GPIO_CONFIG:
        switch (wakeup_gpio_config_state) {
          case wakeup_power_key:
            set_wakeup_gpio_config(wakeup_netflix_key);
            break;
          case wakeup_netflix_key:
            set_wakeup_adv_pattern();
            break;
          default:
            break;
        }
        break;
      case HCI_CMD_NXP_SUB_OCF_ADV_PATTERN_CONFIG:
        set_wakeup_local_parameter();
        break;
      case HCI_CMD_NXP_SUB_OCF_SCAN_PARAM_CONFIG:
        set_wakeup_gpio_config(wakeup_power_key);
        break;
      case HCI_CMD_NXP_SUB_OCF_LOCAL_PARAM_CONFIG:
        vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
        pthread_create(&p_headtbeat, NULL, send_heartbeat_thread, NULL);
        break;
      default:
        break;
    }
  } else {
    vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
  }
  ALOGI("wakup sub_ocf:0x%04x\n", sub_ocf);
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
  VNDDBG("Killing heartbeat thread");
  if (send_heartbeat) {
    status = pthread_kill(p_headtbeat, SIGUSR1);
    send_heartbeat = FALSE;
  }
  VNDDBG("Killed heartbeat with status %d", status);
}
#endif

/******************************************************************************
 **
 ** Function:      hw_bt_read_fw_revision
 **
 ** Description:   Sends command to read revision
 **
 ** Return Value:  NA
 **
 *****************************************************************************/
void hw_bt_read_fw_revision(void) {
  uint16_t opcode;
  HC_BT_HDR* packet;
  opcode = HCI_CMD_NXP_READ_FW_REVISION;
  packet = make_command(opcode, 0);
  if (packet) {
    if (vnd_cb->xmit_cb(opcode, packet, hw_config_callback)) {
      ALOGI("%s Read FW revision command sent\n", __func__);
    }
  } else {
    VNDDBG("%s no valid packet \n", __func__);
  }
  return;
}
