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

#include <log/log.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "bt_vendor_nxp.h"
#include "bt_hci_bdroid.h"

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
#if (USE_CONTROLLER_BDADDR == TRUE)
static void hw_config_read_bdaddr(void);
#endif
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
static int hw_bt_cal_data_load(void);
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
      vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
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
  vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
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
  vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
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
  uint16_t opcode;
#if (USE_CONTROLLER_BDADDR == TRUE)
  char* p_tmp;
  HC_BT_HDR* p_evt_buf = (HC_BT_HDR*)packet;
#endif
  stream = ((HC_BT_HDR*)packet)->data;
  event = ((HC_BT_HDR*)packet)->event;
  opcode_offset = HCI_EVENT_PREAMBLE_SIZE + 1;  // Skip num packets.

  if (event == HCI_PACKET_TYPE_EVENT) {
    event_code = stream[0];
    opcode = stream[opcode_offset] | (stream[opcode_offset + 1] << 8);
    if (event_code == HCI_COMMAND_COMPLETE_EVT) {
      status = stream[opcode_offset + 2];
      ALOGI("opcode 0x%04x status 0x%02x\n", opcode, status);
      switch (opcode) {
        case OpCodePack(HCI_CONTROLLER_CMD_OGF, HCI_RESET_OCF): {
          VNDDBG("Receive hci reset complete event");
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
          if (hw_bt_cal_data_load()) {
            bt_update_bdaddr();
          }
#else
          bt_update_bdaddr();
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
            vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
          }
#endif
          break;
        }
#if (NXP_LOAD_BT_CALIBRATION_DATA == TRUE)
        case HCI_CMD_NXP_LOAD_CONFIG_DATA: {
          VNDDBG("Receive HCI_CMD_NXP_LOAD_CONFIG_DATA complete event.\n");
          bt_update_bdaddr();
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
            if (vnd_cb) {
              vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
            }
#endif
          }
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
      if (vnd_cb) vnd_cb->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
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
