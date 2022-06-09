/******************************************************************************
 *
 *  Copyright 2009-2022 NXP
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

/*===================== Include Files ============================================*/
#include "fw_loader_uart.h"
#include "fw_loader_io.h"
#include <errno.h>
#include <memory.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>

#define LOG_TAG "fw_loader"
#define FW_DEFAULT_PATH "/vendor/firmware/"
#include <log/log.h>
#include <cutils/properties.h>
#define printf(fmt, ...) ALOGI("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)

/*--------------------------------fw_loader_io_linux.c-------------------------*/
#define TIMEOUT_FOR_READ        4000


/*===================== Macros ===================================================*/
#define VERSION "M322"
#define MAX_LENGTH 0xFFFF  // Maximum 2 byte value
#define END_SIG_TIMEOUT 2500
#define MAX_CTS_TIMEOUT 100  // 100ms
#define STRING_SIZE 6
#define HDR_LEN 16
#define CMD4 0x4
#define CMD6 0x6
#define CMD7 0x7
#define V1_HEADER_DATA_REQ 0xa5
#define V1_REQUEST_ACK 0x5a
#define V1_START_INDICATION 0xaa

#define V3_START_INDICATION 0xab
#define V3_HEADER_DATA_REQ 0xa7
#define V3_REQUEST_ACK 0x7a
#define V3_TIMEOUT_ACK 0x7b
#define V3_CRC_ERROR 0x7c

#define PRINT(...)         printf(__VA_ARGS__)
#define FW_INIT_CONFIG_LEN 64
#define REQ_HEADER_LEN 1
#define A6REQ_PAYLOAD_LEN 8
#define AbREQ_PAYLOAD_LEN 3

#define END_SIG 0x435000

#define GP 0x107 /* x^8 + x^2 + x + 1 */
#define DI 0x07

#define CRC_ERR_BIT 1 << 0
#define NAK_REC_BIT 1 << 1
#define TIMEOUT_REC_ACK_BIT 1 << 2
#define TIMEOUT_REC_HEAD_BIT 1 << 3
#define TIMEOUT_REC_DATA_BIT 1 << 4
#define INVALID_CMD_REC_BIT 1 << 5
#define WIFI_MIC_FAIL_BIT 1 << 6
#define BT_MIC_FAIL_BIT 1 << 7

#define SWAPL(x) \
  (((x >> 24) & 0xff) | ((x >> 8) & 0xff00) | ((x << 8) & 0xff0000L) | ((x << 24) & 0xff000000L))

#define POLYNOMIAL 0x04c11db7L

#define CLKDIVAddr 0x7f00008f
#define UARTDIVAddr 0x7f000090
#define UARTMCRAddr 0x7f000091
#define UARTREINITAddr 0x7f000092
#define UARTICRAddr 0x7f000093
#define UARTFCRAddr 0x7f000094

#define MCR 0x00000022
#define INIT 0x00000001
#define ICR 0x000000c7
#define FCR 0x000000c7 // TODO: why same as ICR

#define TIMEOUT_VAL_MILLISEC 510  // Timeout for getting 0xa5 or 0xab or 0xaa or 0xa7

static unsigned char crc8_table[256]; /* 8-bit table */
static int made_table = 0;

static unsigned long crc_table[256];
static BOOLEAN cmd7_Req = FALSE;
static BOOLEAN EntryPoint_Req = FALSE;
static uint32 change_baudrata_buffer_len = 0;
static uint32 cmd7_change_timeout_len = 0;
static uint32 cmd5_len = 0;
static BOOLEAN send_poke = TRUE;
static uint16 chip_id = 0;
static uint8 m_Buffer_Poke[2] = {0xdc,0xe9};
// CMD5 Header to change bootload baud rate
uint8 m_Buffer_CMD5_Header[16] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x2c, 0x00, 0x00, 0x00, 0x77, 0xdb, 0xfd, 0xe0};
uint8 m_Buffer_CMD7_ChangeTimeoutValue[16] = {0x07,0x00,0x00,0x00,0x70,0x00,0x00,0x00,
                                             0x00,0x00,0x00,0x00,0x5b,0x88,0xf8,0xba};
#if (UART_DOWNLOAD_FW == TRUE)
uint8 fw_init_config_bin[FW_INIT_CONFIG_LEN];
#endif

const UART_BAUDRATE UartCfgTbl[] = {
    {115200, 16, 0x0075F6FD}, {3000000, 1, 0x00C00000},
};

//#define DEBUG_PRINT
/*==================== Typedefs =================================================*/

/*===================== Global Vars ==============================================*/
// Maximum Length that could be asked by the Helper = 2 bytes
static uint8 ucByteBuffer[MAX_LENGTH];

// Size of the File to be downloaded
static long ulTotalFileSize = 0;

// Current size of the Download
static uint32 ulCurrFileSize = 0;
static uint32 ulLastOffsetToSend = 0xFFFF;
static BOOLEAN uiErrCase = FALSE;
// Received Header
static uint8 ucRcvdHeader = 0xFF;
static uint8 ucString[STRING_SIZE];
static BOOLEAN b16BytesData = FALSE;

static uint16 uiNewLen;
static uint32 ulNewOffset;
static uint16 uiNewError;
static uint8 uiNewCrc;

static uint8 uiProVer;
static BOOLEAN bVerChecked = FALSE;
static uint8 ucCalCrc[10];

typedef enum {
  Ver1,
  Ver2,
  Ver3,
} Version;

typedef struct {
  uint16_t soc_id;
  char default_fw_name[MAX_FILE_LEN];
} soc_fw_name_dict_t;

typedef enum {
  NXP_CHIPID_9098_A1 = 0x5C02,
  NXP_CHIPID_9098_A2 = 0x5C03,
  NXP_CHIPID_9177_A0 = 0x7600,
  NXP_CHIPID_9177_A1 = 0x7601
} NXP_CHIPID;

soc_fw_name_dict_t soc_fw_name_dict[] = {
    {NXP_CHIPID_9098_A1, "uart9098_bt_v1.bin"},
    {NXP_CHIPID_9098_A2, "uart9098_bt_v1.bin"},
    {NXP_CHIPID_9177_A0, "uartspi_n61x.bin"},
    {NXP_CHIPID_9177_A1, "uartspi_n61x_v1.bin"}
};

uint8 uiErrCnt[16] = {0};
static jmp_buf resync;  // Protocol restart buffer used in timeout cases.

// extern struct termios ti;
/*==================== Function Prototypes ======================================*/
// extern int32 uart_set_speed(int32 fd, struct termios* ti, int32 speed);

/*==================== Coded Procedures =========================================*/
#ifdef TEST_CODE

static uint32 ucTestCase = 0;
static uint32 ucSleepTimeMs = 0;
static uint8  ucTestDone = 0;
static uint16 uiCurrLenToSend = 0;
uint8 myCrcCorrByte, myChangeCrc = 0;
static BOOLEAN uiBaudRateDone = FALSE;

char* get_time()
{
  static char finalbuff[1000];

  static time_t rawtime;
  static struct tm * timeinfo;
  struct timeval tp;

  time_t t = time ( NULL);
  timeinfo = localtime ( &t );
  gettimeofday(&tp, 0);
  sprintf(finalbuff, "%02d-%02d-%d %02d:%02d:%02d:%06ld", timeinfo->tm_year + 1900, timeinfo->tm_mon,
            timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, tp.tv_usec);

  return finalbuff;
}

#endif
/******************************************************************************

 *
 * Name: gen_crc_table
 *
 * Description:
 *   Genrate crc table
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   None.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
void fw_upload_gen_crc_table() {
  int i, j;
  unsigned long crc_accum;

  for (i = 0; i < 256; i++) {
    crc_accum = ((unsigned long)i << 24);
    for (j = 0; j < 8; j++) {
      if (crc_accum & 0x80000000L) {
        crc_accum = (crc_accum << 1) ^ POLYNOMIAL;
      } else {
        crc_accum = (crc_accum << 1);
      }
    }
    crc_table[i] = crc_accum;
  }

  return;
}

/******************************************************************************

 *
 * Name: update_crc
 *
 * Description:
 *   update the CRC on the data block one byte at a time
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   ata_blk_ptr:   the buffer pointer for updating crc.
 *   data_blk_size: the size of buffer
 *
 * Return Value:
 *   CRC value.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
unsigned long fw_upload_update_crc(unsigned long crc_accum, uint8 *data_blk_ptr, int data_blk_size) {
  int i, j;

  for (j = 0; j < data_blk_size; j++) {
    i = ((int)(crc_accum >> 24) ^ *data_blk_ptr++) & 0xff;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

/******************************************************************************
 *
 * Name: init_crc8
 *
 * Description:
 *   This function init crc.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   None.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
void init_crc8() {
  int i, j;
  int crc;

  if (!made_table) {
    for (i = 0; i < 256; i++) {
      crc = i;
      for (j = 0; j < 8; j++) crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
      crc8_table[i] = crc & 0xFF;
      /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
    }
    made_table = 1;
  }
}
/******************************************************************************
 *
 * Name: crc8
 *
 * Description:
 *   This function calculate crc.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   array: array to be calculated.
 *   len :  len of array.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static unsigned char crc8(unsigned char *array, unsigned char len) {
  unsigned char CRC = 0xff;
  for (; len > 0; len--) {
    CRC = crc8_table[CRC ^ *array];
    array++;
  }
  return CRC;
}

/******************************************************************************
 *
 * Name: fw_upload_WaitForHeaderSignature(uint32 uiMs)
 *
 * Description:
 *   This function basically waits for reception
 *   of character 0xa5 on UART Rx. If no 0xa5 is
 *   received, it will kind of busy wait checking for
 *   0xa5.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   uiMs:   the expired time.
 *
 * Return Value:
 *   TRUE:   0xa5 or 0xab is received.
 *   FALSE:  0xa5 or 0xab is not received.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static BOOLEAN
fw_upload_WaitForHeaderSignature(uint32 uiMs)
{
  uint8 ucDone = 0;       // signature not Received Yet.
  uint64 startTime = 0;
  uint64 currTime = 0;
  BOOLEAN bResult = TRUE;
  ucRcvdHeader = 0xFF;
  startTime = fw_upload_GetTime();
  while (!ucDone) {
  ucRcvdHeader = fw_upload_ComReadChar(mchar_fd);
  if ((ucRcvdHeader == V1_HEADER_DATA_REQ) ||(ucRcvdHeader == V1_START_INDICATION) ||
      (ucRcvdHeader == V3_START_INDICATION) ||(ucRcvdHeader == V3_HEADER_DATA_REQ)) {
    ucDone = 1;
#ifdef DEBUG_PRINT
    PRINT("\nReceived 0x%x ", ucRcvdHeader);
#endif
    if (!bVerChecked) {
      if ((ucRcvdHeader == V1_HEADER_DATA_REQ) ||(ucRcvdHeader == V1_START_INDICATION)) {
        uiProVer = Ver1;
      } else {
          uiProVer = Ver3;
          if (V3_START_INDICATION) {
            while (fw_upload_GetBufferSize(mchar_fd) < 2) {
              usleep(1000);
            };
            chip_id = (fw_upload_ComReadChar(mchar_fd) |
                       (fw_upload_ComReadChar(mchar_fd) << 8));
            VNDDBG("Chip ID: 0x%x ", chip_id);
          }
        }
      bVerChecked = TRUE;
    }
  } else {
      if (uiMs) {
        currTime = fw_upload_GetTime();
        if ((currTime - startTime) > uiMs) {
          VNDDBG(
              "\n fw_upload_WaitForHeaderSignature Timeout, Header Received: "
              "0x%x, timeout %d, elapsed time %llu",
              ucRcvdHeader, uiMs, (currTime - startTime));
          bResult = FALSE;
          break;
        }
      }
      if (enable_poke_controller && send_poke &&
          (!get_prop_int32(PROP_BLUETOOTH_FW_DOWNLOADED))) {
        fw_upload_ComWriteChars(mchar_fd, m_Buffer_Poke, 2);
        VNDDBG("Poke Sent");
        send_poke = FALSE;
      }

      fw_upload_DelayInMs(1);
    }
  }
  send_poke = FALSE;
  return bResult;
}

/******************************************************************************
 *
 * Name: fw_upload_WaitFor_Len
 *
 * Description:
 *   This function waits to receive the 4 Byte length.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pFile: The handler of file
 *
 * Return Value:
 *   2 Byte Length to send back to the Helper.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint16 fw_upload_WaitFor_Len(FILE *pFile) {
  // Length Variables
  uint16 uiLen = 0x0;
  uint16 uiLenComp = 0x0;
  // uiLen and uiLenComp are 1's complement of each other.
  // In such cases, the XOR of uiLen and uiLenComp will be all 1's
  // i.e 0xffff.
  uint16 uiXorOfLen = 0xFFFF;

  while (fw_upload_GetBufferSize(mchar_fd) < 4){
    usleep(1000);
  };
  // Read the Lengths.
  fw_upload_ComReadChars(mchar_fd, (uint8 *)&uiLen, 2);
  fw_upload_ComReadChars(mchar_fd, (uint8 *)&uiLenComp, 2);

  // Check if the length is valid.
  if ((uiLen ^ uiLenComp) == uiXorOfLen)  // All 1's
  {
#ifdef DEBUG_PRINT
    PRINT("\n       bootloader asks for %d bytes \n ", uiLen);
#endif
    // Successful. Send back the ack.
    if ((ucRcvdHeader == V1_HEADER_DATA_REQ) || (ucRcvdHeader == V1_START_INDICATION)) {
      fw_upload_ComWriteChar(mchar_fd, V1_REQUEST_ACK);
#ifdef DEBUG_PRINT
      PRINT("\n  BOOT_HEADER_ACK 0x5a is sent\n");
#endif
      if (ucRcvdHeader == V1_START_INDICATION) {
        longjmp(resync, 1);
      }
    }
  } else {
#ifdef DEBUG_PRINT
    PRINT("\n    NAK case: bootloader LEN = %x bytes \n ", uiLen);
    PRINT("\n    NAK case: bootloader LENComp = %x bytes \n ", uiLenComp);
#endif
    // Failure due to mismatch.
    fw_upload_ComWriteChar(mchar_fd, (int8)0xbf);
    // Start all over again.
    if (pFile != NULL) {
      longjmp(resync, 1);
    } else {
      uiLen = 0;
    }
  }
  return uiLen;
}

/******************************************************************************
 *
 * Name: fw_upload_StoreBytes
 *
 * Description:
 *   This function stores mul-bytes variable to array.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   ulVal: variable to be stored.
 *   uiSize: size of bytes of this variable.
 *   uiStored: array to store variable.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static void fw_upload_StoreBytes(uint32 ulVal, uint8 uiSize, uint8 *uiStored) {
  uint8 i;
  for (i = 0; i < uiSize; i++) {
    uiStored[i] = (uint8)(ulVal >> (i * 8)) & 0xFF;
  }
}

/******************************************************************************
 *
 * Name: fw_upload_Send_Ack
 *
 * Description:
 *   This function sends ack to per req.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   uiAck: the ack type.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static void fw_upload_Send_Ack(uint8 uiAck) {
  uint8 uiAckCrc = 0;
  if ((uiAck == V3_REQUEST_ACK) || (uiAck == V3_CRC_ERROR)) {
#ifdef TEST_CODE
     if (ucRcvdHeader == V3_START_INDICATION)
     {

        // prepare crc for 0x7A or 0x7C
        ucCalCrc[0] = uiAck;
        uiAckCrc = crc8(ucCalCrc, 1);

        if(ucTestCase == 301 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Sleep %dms, NOT send V3_REQUEST_ACK for Header Signature %02X, NOT send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
        }
        else if (ucTestCase == 302 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Sleep %dms, NOT send V3_REQUEST_ACK for Header Signature %02X, send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
          ucTestDone = 1;
        }
        else if (ucTestCase == 303 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Sleep %dms, send V3_REQUEST_ACK for Header Signature %02X, NOT send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAck);
          ucTestDone = 1;
        }
        else if (ucTestCase == 304 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Sleep %dms, send V3_REQUEST_ACK for Header Signature %02X, send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAck);
          fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
          ucTestDone = 1;
        }
        else if (ucTestCase == 305 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  NOT send V3_REQUEST_ACK for Header Signature %02X, sleep %dms, send CRC byte", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
          ucTestDone = 1;
        }
        else if (ucTestCase == 306 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, sleep %dms, NOT send CRC byte", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAck);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
        }
        else if (ucTestCase == 307 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, sleep %dms, send CRC byte", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAck);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
          ucTestDone = 1;
        }
        else if (ucTestCase == 308 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  NOT send V3_REQUEST_ACK for Header Signature %02X, send CRC byte, sleep %dms", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
        }
        else if (ucTestCase == 309 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, NOT send CRC byte, sleep %dms", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAck);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
        }
        else if (ucTestCase == 310 && !ucTestDone)
        {
          PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, send CRC byte, sleep %dms", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
          fw_upload_ComWriteChar(mchar_fd, uiAck);
          fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
        }
        else
        {
         fw_upload_ComWriteChar(mchar_fd, uiAck);
         fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
        }

     }

     else if (ucRcvdHeader == V3_HEADER_DATA_REQ)
     {

        // prepare crc for 0x7A or 0x7C
        ucCalCrc[0] = uiAck;
        uiAckCrc = crc8(ucCalCrc, 1);

        if(ucTestCase == 311 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Sleep %dms, NOT send V3_REQUEST_ACK for Header Signature %02X, NOT send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
           fw_upload_DelayInMs(ucSleepTimeMs);
           ucTestDone = 1;
        }
        else if (ucTestCase == 312 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Sleep %dms, NOT send V3_REQUEST_ACK for Header Signature %02X, send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
           fw_upload_DelayInMs(ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
           ucTestDone = 1;
        }
        else if (ucTestCase == 313 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Sleep %dms, send V3_REQUEST_ACK for Header Signature %02X, NOT send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
           fw_upload_DelayInMs(ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           ucTestDone = 1;
        }
        else if (ucTestCase == 314 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Sleep %dms, send V3_REQUEST_ACK for Header Signature %02X, send CRC byte", ucTestCase, ucSleepTimeMs, ucRcvdHeader);
           fw_upload_DelayInMs(ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
           ucTestDone = 1;
        }
        else if (ucTestCase == 315 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  NOT send V3_REQUEST_ACK for Header Signature %02X, sleep %dms, send CRC byte", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
           fw_upload_DelayInMs(ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
           ucTestDone = 1;
        }
        else if (ucTestCase == 316 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, sleep %dms, NOT send CRC byte", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           fw_upload_DelayInMs(ucSleepTimeMs);
           ucTestDone = 1;
        }
        else if (ucTestCase == 317 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, sleep %dms, send CRC byte", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           fw_upload_DelayInMs(ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
           ucTestDone = 1;
        }
        else if (ucTestCase == 318 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  NOT send V3_REQUEST_ACK for Header Signature %02X, send CRC byte, sleep %dms", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
           fw_upload_DelayInMs(ucSleepTimeMs);
           ucTestDone = 1;
        }
        else if (ucTestCase == 319 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, NOT send CRC byte, sleep %dms", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           fw_upload_DelayInMs(ucSleepTimeMs);
           ucTestDone = 1;
        }
        else if (ucTestCase == 320 && !ucTestDone)
        {
           PRINT("\n         TC-%d:  Send V3_REQUEST_ACK for Header Signature %02X, send CRC byte, sleep %dms", ucTestCase, ucRcvdHeader, ucSleepTimeMs);
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
           fw_upload_DelayInMs(ucSleepTimeMs);
           ucTestDone = 1;
        }
        else
        {
           fw_upload_ComWriteChar(mchar_fd, uiAck);
           fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
        }
     }
#else
    fw_upload_ComWriteChar(mchar_fd, uiAck);

    // prepare crc for 0x7A or 0x7C
    ucCalCrc[0] = uiAck;
    uiAckCrc = crc8(ucCalCrc, 1);
    fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
#endif
  } else if (uiAck == V3_TIMEOUT_ACK) {
    fw_upload_ComWriteChar(mchar_fd, uiAck);

    // prepare crc for 0x7B
    ucCalCrc[0] = uiAck;
    fw_upload_StoreBytes(ulNewOffset, sizeof(ulNewOffset), &ucCalCrc[1]);
    fw_upload_ComWriteChars(mchar_fd, (uint8 *)&ulNewOffset, 4);
    uiAckCrc = crc8(ucCalCrc, 5);
    fw_upload_ComWriteChar(mchar_fd, uiAckCrc);
  } else {
    PRINT("\nNon-empty else statement\n");
  }
#ifdef DEBUG_PRINT
  printf("\n ===> ACK = %x, CRC = %x \n", uiAck, uiAckCrc);
#endif
}
/******************************************************************************
 *
 * Name: fw_upload_Check_ReqCrc
 *
 * Description:
 *   This function check the request crc.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   uiStr: array to put req header + payload.
 *   uiReq: the request type.
 *
 * Return Value:
 *   result of crc check.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
BOOLEAN fw_upload_Check_ReqCrc(uint8 *uiStr, uint8 uiReq) {
  uint8 uiCalCrc;

  if (uiReq == V3_HEADER_DATA_REQ) {
    uiCalCrc = crc8(uiStr, A6REQ_PAYLOAD_LEN + REQ_HEADER_LEN);
    if (uiCalCrc != uiStr[A6REQ_PAYLOAD_LEN + REQ_HEADER_LEN]) {
      return FALSE;
    }

  } else if (uiReq == V3_START_INDICATION) {
    uiCalCrc = crc8(uiStr, AbREQ_PAYLOAD_LEN + REQ_HEADER_LEN);
    if (uiCalCrc != uiStr[AbREQ_PAYLOAD_LEN + REQ_HEADER_LEN]) {
      return FALSE;
    }
  } else {
    PRINT("\nNon-empty else statement\n");
  }

  return TRUE;
}

/******************************************************************************
 *
 * Name: fw_upload_WaitFor_Req
 *
 * Description:
 *   This function waits for req from bootcode or helper.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   None.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static BOOLEAN fw_upload_WaitFor_Req(int32 iSecondBaudRate) {
  uint16 uiChipId = 0;
  uint8 uiVersion = 0, uiReqCrc = 0, uiTmp[20] = {0};
  BOOLEAN bCrcMatch = FALSE;
  BOOLEAN status = TRUE;

  if (ucRcvdHeader == V3_HEADER_DATA_REQ) {
    // 0xA7 <LEN><Offset><ERR><CRC8>
    fw_upload_ComReadChars(mchar_fd, (uint8 *)&uiNewLen, 2);
    fw_upload_ComReadChars(mchar_fd, (uint8 *)&ulNewOffset, 4);
    fw_upload_ComReadChars(mchar_fd, (uint8 *)&uiNewError, 2);
    fw_upload_ComReadChars(mchar_fd, (uint8 *)&uiNewCrc, 1);
#ifdef DEBUG_PRINT
    PRINT("\n <=== REQ = 0xA7, Len = %x,Off = %x,Err = %x,CRC = %x\n ", uiNewLen, ulNewOffset,
           uiNewError, uiNewCrc);
#endif
    // check crc
    uiTmp[0] = V3_HEADER_DATA_REQ;
    fw_upload_StoreBytes((uint32)uiNewLen, sizeof(uiNewLen), &uiTmp[1]);
    fw_upload_StoreBytes(ulNewOffset, sizeof(ulNewOffset), &uiTmp[3]);
    fw_upload_StoreBytes(uiNewError, sizeof(uiNewError), &uiTmp[7]);
    uiTmp[9] = uiNewCrc;
    bCrcMatch = fw_upload_Check_ReqCrc(uiTmp, V3_HEADER_DATA_REQ);

#ifdef TEST_CODE

    if (ucTestCase == 331 && !ucTestDone)
    {
       PRINT("\n         TC-%d:  Simulate Device CRC error on Header Signature 0x%X", ucTestCase, ucRcvdHeader);
       bCrcMatch = 0;
       ucTestDone = 1;
    }

#endif

    if (!bCrcMatch) {
#ifdef DEBUG_PRINT
      PRINT("\n === REQ = 0xA7, CRC Mismatched === ");
#endif
      fw_upload_Send_Ack(V3_CRC_ERROR);
      status = FALSE;
    }
  } else if (ucRcvdHeader == V3_START_INDICATION) {
    // 0xAB <CHIP ID> <SW loader REV 1 byte> <CRC8>
    fw_upload_ComReadChars(mchar_fd, (uint8 *)&uiChipId, 2);
    uiVersion = fw_upload_ComReadChar(mchar_fd);
    uiReqCrc = fw_upload_ComReadChar(mchar_fd);
    PRINT("\nChipID is : %x, Version is : %x\n", uiChipId, uiVersion);

    // check crc
    uiTmp[0] = V3_START_INDICATION;
    fw_upload_StoreBytes((uint32)uiChipId, sizeof(uiChipId), &uiTmp[1]);
    uiTmp[3] = uiVersion;
    uiTmp[4] = uiReqCrc;
    bCrcMatch = fw_upload_Check_ReqCrc(uiTmp, V3_START_INDICATION);

#ifdef TEST_CODE

    if (ucTestCase == 330 && !ucTestDone)
    {
       PRINT("\n         TC-%d:  Simulate Device CRC error on Header Signature 0x%X", ucTestCase, ucRcvdHeader);
       bCrcMatch = 0;
       ucTestDone = 1;
    }

#endif

    if (bCrcMatch) {
#ifdef DEBUG_PRINT
      PRINT("\n === REQ = 0xAB, CRC Matched === ");
#endif
      fw_upload_Send_Ack(V3_REQUEST_ACK);
      if(iSecondBaudRate == 0) {
        longjmp(resync, 1);
      }
    } else {
#ifdef DEBUG_PRINT
      PRINT("\n === REQ = 0xAB, CRC Mismatched === ");
#endif
      fw_upload_Send_Ack(V3_CRC_ERROR);
      status = FALSE;
      if(iSecondBaudRate == 0) {
        longjmp(resync, 1);
      }
    }
  } else {
    PRINT("\nNon-empty else statement\n");
  }
  return status;
}

/******************************************************************************
 *
 * Name: fw_upload_GetCmd
 *
 * Description:
 *   This function gets CMD value in the header.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   *buf: buffer that stores header and following data.
 *
 * Return Value:
 *   CMD value part in the buffer.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint32 fw_upload_GetCmd(uint8 *buf) {
  return (buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
}

/******************************************************************************
 *
 * Name: fw_upload_GetHeaderStartBytes
 *
 * Description:
 *   This function gets 0xa5 and it's following 4 bytes length.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   None.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *

*****************************************************************************/
static void fw_upload_GetHeaderStartBytes(uint8 *ucStr) {
  BOOLEAN ucDone = FALSE, ucStringCnt = 0, i;
  while (!ucDone) {
    ucRcvdHeader = fw_upload_ComReadChar(mchar_fd);

    if (ucRcvdHeader == V1_HEADER_DATA_REQ) {
      ucStr[ucStringCnt++] = ucRcvdHeader;
      ucDone = TRUE;
#ifdef DEBUG_PRINT
      PRINT("\nReceived 0x%x\n ", ucRcvdHeader);
#endif
    } else {
      fw_upload_DelayInMs(1);
    }
  }
  while (fw_upload_GetBufferSize(mchar_fd) < 4){
    usleep(1000);
  };
  for (i = 0; i < 4; i++) {
    ucRcvdHeader = fw_upload_ComReadChar(mchar_fd);
    ucStr[ucStringCnt++] = ucRcvdHeader;
  }
}
/******************************************************************************
 *
 * Name: fw_upload_GetLast5Bytes
 *
 * Description:
 *   This function gets last valid request.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   *buf: buffer that stores header and following data.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *

*****************************************************************************/
static void fw_upload_GetLast5Bytes(uint8 *buf) {
  uint8 a5cnt, i;
  uint8 ucTemp[STRING_SIZE];
  uint16 uiTempLen = 0;
  int32 fifosize;
  BOOLEAN alla5times = FALSE;

  // initialise
  memset(ucString, 0x00, STRING_SIZE);

  fifosize = fw_upload_GetBufferSize(mchar_fd);

  fw_upload_GetHeaderStartBytes(ucString);
  if(fw_upload_lenValid(&uiTempLen, ucString) == TRUE) {
  //Valid length recieved
#ifdef DEBUG_PRINT
      PRINT("\n Valid length = %d \n", uiTempLen);
#endif
  }

  if ((fifosize < 6) && ((uiTempLen == HDR_LEN) || (uiTempLen == fw_upload_GetDataLen(buf)))) {
#ifdef DEBUG_PRINT
    PRINT("=========>success case fifo size= %d\n", fifosize);
#endif
    uiErrCase = FALSE;
  } else  // start to get last valid 5 bytes
  {
#ifdef DEBUG_PRINT
    PRINT("=========>fail case fifo size= %d \n", fifosize);
#endif
    while (fw_upload_lenValid(&uiTempLen, ucString) == FALSE) {
      fw_upload_GetHeaderStartBytes(ucString);
      fifosize -= 5;
    }
#ifdef DEBUG_PRINT
    PRINT("Error cases 1, 2, 3, 4, 5...\n");
#endif
    if (fifosize > 5) {
      fifosize -= 5;
      do {
        do {
          a5cnt = 0;
          do {
            fw_upload_GetHeaderStartBytes(ucTemp);
            fifosize -= 5;
          } while ((fw_upload_lenValid(&uiTempLen, ucTemp) == TRUE) && (!alla5times) && (fifosize > 5));
          // if 5bytes are all 0xa5, continue to clear 0xa5
          for (i = 0; i < 5; i++) {
            if (ucTemp[i] == V1_HEADER_DATA_REQ) {
              a5cnt++;
            }
          }
          alla5times = TRUE;
        } while (a5cnt == 5);
#ifdef DEBUG_PRINT
        PRINT("a5 count in last 5 bytes: %d\n", a5cnt);
#endif
        if (fw_upload_lenValid(&uiTempLen, ucTemp) == FALSE) {
          for (i = 0; i < (5 - a5cnt); i++) {
            ucTemp[i + a5cnt] = fw_upload_ComReadChar(mchar_fd);
          }
          if(a5cnt > 0){
            memcpy(ucString, &ucTemp[a5cnt - 1], (5 - a5cnt)*sizeof(uint8));
          }
        } else {
          memcpy(ucString, ucTemp, 5*sizeof(uint8));
        }
      } while (fw_upload_lenValid(&uiTempLen, ucTemp) == FALSE);
    }
    uiErrCase = TRUE;
  }
}

/******************************************************************************
 *
 * Name: fw_upload_SendBuffer
 *
 * Description:
 *   This function sends buffer with header and following data.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *      uiLenToSend: len of header request.
 *            ucBuf: the buf to be sent.
 *   uiHighBaudrate: send the buffer for high baud rate change.
 * Return Value:
 *   Returns the len of next header request.
 *
 * Notes:
 *   None.
 *
*****************************************************************************/
static uint16 fw_upload_SendBuffer(uint16 uiLenToSend, uint8 *ucBuf, BOOLEAN uiHighBaudrate) {
  uint16 uiBytesToSend = HDR_LEN, uiFirstChunkSent = 0;
  uint16 uiDataLen = 0;
  uint8 ucSentDone = 0;
  BOOLEAN uiValidLen = FALSE;
  // Get data len
  uiDataLen = fw_upload_GetDataLen(ucBuf);
  // Send buffer
  while (!ucSentDone) {
    if (uiBytesToSend == uiLenToSend) {
      // All good
      if ((uiBytesToSend == HDR_LEN) && (!b16BytesData)) {
        if ((uiFirstChunkSent == 0) || ((uiFirstChunkSent == 1) && (uiErrCase == TRUE))) {
// Write first 16 bytes of buffer
#ifdef DEBUG_PRINT
          PRINT("\n====>  Sending first chunk...\n");
          PRINT("\n====>  Sending %d bytes...\n", uiBytesToSend);
#endif
          fw_upload_ComWriteChars(mchar_fd, (uint8 *)ucBuf, uiBytesToSend);
          if (cmd7_Req == TRUE || EntryPoint_Req == TRUE) {
            uiBytesToSend = HDR_LEN;
            uiFirstChunkSent = 1;
          } else {
            uiBytesToSend = uiDataLen;
            uiFirstChunkSent = 0;
            if (uiBytesToSend == HDR_LEN) {
              b16BytesData = TRUE;
            }
          }
        } else {
          // Done with buffer
          ucSentDone = 1;
          break;
        }
      } else {
// Write remaining bytes
#ifdef DEBUG_PRINT
        PRINT("\n====>  Sending %d bytes...\n", uiBytesToSend);
#endif
        if (uiBytesToSend != 0) {
          fw_upload_ComWriteChars(mchar_fd, (uint8 *)&ucBuf[HDR_LEN], uiBytesToSend);
          uiFirstChunkSent = 1;
          // We should expect 16, then next block will start
          uiBytesToSend = HDR_LEN;
          b16BytesData = FALSE;
          if (uiHighBaudrate) {
            return 0;
          }
        } else  // end of bin download
        {
#ifdef DEBUG_PRINT
          PRINT("\n ========== Download Complete =========\n\n");
#endif
          return 0;
        }
      }
    } else {
      // Something not good
      if ((uiLenToSend & 0x01) == 0x01) {
        // some kind of error
        if (uiLenToSend == (HDR_LEN + 1)) {
// Send first chunk again
#ifdef DEBUG_PRINT
          PRINT("\n1. Resending first chunk...\n");
#endif
          fw_upload_ComWriteChars(mchar_fd, (uint8 *)ucBuf, (uiLenToSend - 1));
          uiBytesToSend = uiDataLen;
          uiFirstChunkSent = 0;
        } else if (uiLenToSend == (uiDataLen + 1)) {
// Send second chunk again
#ifdef DEBUG_PRINT
          PRINT("\n2. Resending second chunk...\n");
#endif
          fw_upload_ComWriteChars(mchar_fd, (uint8 *)&ucBuf[HDR_LEN], (uiLenToSend - 1));
          uiBytesToSend = HDR_LEN;
          uiFirstChunkSent = 1;
        } else {
          PRINT("Non-empty terminating else statement uiLenToSend = %d ",
                uiLenToSend);
        }
      } else if (uiLenToSend == HDR_LEN) {
// Out of sync. Restart sending buffer
#ifdef DEBUG_PRINT
        PRINT("Restart sending the 1st chunk...");
#endif
        fw_upload_ComWriteChars(mchar_fd, (uint8*)ucBuf, uiLenToSend);
        uiBytesToSend = uiDataLen;
        uiFirstChunkSent = 0;
      } else if (uiLenToSend == uiDataLen) {
#ifdef DEBUG_PRINT
        PRINT("Restart sending 2nd chunk...");
#endif
        fw_upload_ComWriteChars(mchar_fd, (uint8*)&ucBuf[HDR_LEN], uiLenToSend);
        uiBytesToSend = HDR_LEN;
        uiFirstChunkSent = 1;
      } else {
        PRINT("Non-empty else statement uiLenToSend = %d", uiLenToSend);
      }
    }
    // Get last 5 bytes now
    fw_upload_GetLast5Bytes(ucBuf);
    // Get next length
    uiValidLen = FALSE;
    do {
      if (fw_upload_lenValid(&uiLenToSend, ucString) == TRUE) {
        // Valid length received
        uiValidLen = TRUE;
#ifdef DEBUG_PRINT
        PRINT("\n Valid length = %d \n", uiLenToSend);
#endif
        // ACK the bootloader
        fw_upload_ComWriteChar(mchar_fd, V1_REQUEST_ACK);
#ifdef DEBUG_PRINT
        PRINT("\n  BOOT_HEADER_ACK 0x5a sent \n");
#endif
      }
    } while (!uiValidLen);
  }
#ifdef DEBUG_PRINT
  PRINT("\n ========== Buffer is successfully sent =========\n\n");
#endif
  return uiLenToSend;
}

/******************************************************************************
 *
 * Name: fw_upload_V1SendLenBytes
 *
 * Description:
 *   This function sends Len bytes(header+data) to the boot code.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pFile: bin file being sent.
 *   uiLenTosend: the length will be sent.
 *
 * Return Value:
 *   the 'len' of next header request.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint16 fw_upload_V1SendLenBytes(uint8 * pFileBuffer, uint16 uiLenToSend) {
  uint16 ucDataLen, uiLen;
  uint32 ulCmd;
#ifdef DEBUG_PRINT
  uint16 i;
#endif
  memset(ucByteBuffer, 0, sizeof(ucByteBuffer));

  cmd7_Req = FALSE;
  EntryPoint_Req = FALSE;

  if(ulCurrFileSize + uiLenToSend > ulTotalFileSize)
    uiLenToSend = ulTotalFileSize - ulCurrFileSize;

  memcpy(&ucByteBuffer[uiLenToSend]-uiLenToSend,pFileBuffer+ulCurrFileSize,uiLenToSend);
  ulCurrFileSize += uiLenToSend;
  ulCmd = fw_upload_GetCmd(ucByteBuffer);
  if (ulCmd == CMD7) {
    cmd7_Req = TRUE;
    ucDataLen = 0;
  } else {
    ucDataLen = fw_upload_GetDataLen(ucByteBuffer);
    memcpy(&ucByteBuffer[uiLenToSend], pFileBuffer + ulCurrFileSize,ucDataLen);
    ulCurrFileSize += ucDataLen;
    if ((ulCurrFileSize < ulTotalFileSize) && (ulCmd == CMD6 || ulCmd == CMD4)) {
      EntryPoint_Req = TRUE;
    }
  }

#ifdef DEBUG_PRINT
  PRINT("The buffer is to be sent: %d", uiLenToSend + ucDataLen);
  for(i = 0; i < (uiLenToSend + ucDataLen); i ++)
  {
    if(i % 16 == 0)
    {
      PRINT("\n");
    }
      PRINT(" %02x ", ucByteBuffer[i]);
  }
#endif
  // start to send Temp buffer
  uiLen = fw_upload_SendBuffer(uiLenToSend, ucByteBuffer, FALSE);
#ifdef DEBUG_PRINT
  PRINT("File downloaded: %8u:%8ld\r", ulCurrFileSize, ulTotalFileSize);
#endif

  return uiLen;
}

/******************************************************************************
 *
 * Name: fw_upload_V3SendLenBytes
 *
 * Description:
 *   This function sends Len bytes to the Helper.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pFile: bin file being sent.
 *   uiLenTosend: the length will be sent.
 *   ulOffset: the offset of current sending.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static void fw_upload_V3SendLenBytes(uint8 * pFileBuffer, uint16 uiLenToSend, uint32 ulOffset) {
  // Retransmittion of previous block
  if (ulOffset == ulLastOffsetToSend)
  {
#ifdef DEBUG_PRINT
    PRINT("\nResend offset %d...\n", ulOffset);
#endif
    fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
  }
  else
  {
    // The length requested by the Helper is equal to the Block
    // sizes used while creating the FW.bin. The usual
    // block sizes are 128, 256, 512.
    // uiLenToSend % 16 == 0. This means the previous packet
    // was error free (CRC ok) or this is the first packet received.
    //  We can clear the ucByteBuffer and populate fresh data.
    memset (ucByteBuffer, 0, MAX_LENGTH* sizeof(uint8));
    memcpy(ucByteBuffer,pFileBuffer+ulOffset - change_baudrata_buffer_len - cmd7_change_timeout_len - cmd5_len,uiLenToSend);
   ulCurrFileSize = ulOffset - change_baudrata_buffer_len - cmd7_change_timeout_len - cmd5_len+ uiLenToSend;
#ifdef TEST_CODE

    if (uiLenToSend == HDR_LEN)
    {
       if (ucTestCase == 321 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Sleeping for %dms before sending %d bytes HEADER", ucTestCase, ucSleepTimeMs, uiLenToSend);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucTestDone = 1;
       }
       else if (ucTestCase == 322 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send only 8 bytes of 16-byte HEADER, then sleep for %dms", ucTestCase, ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, 8);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else if (ucTestCase == 323 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send 8 bytes of 16-byte HEADER, sleep for %dms, then send remaining 8 bytes HEADER", ucTestCase, ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, 8);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, &ucByteBuffer[8], 8);
          ucTestDone = 1;
       }
       else if (ucTestCase == 324 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send 8 bytes of 16-byte HEADER, sleep for %dms, then send full 16 bytes HEADER", ucTestCase, ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, 8);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucTestDone = 1;
       }
       else if (ucTestCase == 325 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Sleep for %dms, and NOT sending 16-bytes HEADER, but send DATA", ucTestCase, ucSleepTimeMs);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else if (ucTestCase == 326 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send 16-byte HEADER with last byte changed to 7C", ucTestCase);
          myCrcCorrByte = ucByteBuffer[uiLenToSend - 1];
          ucByteBuffer[uiLenToSend - 1] = 0x7c;
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucByteBuffer[uiLenToSend - 1] = myCrcCorrByte;
          ucTestDone = 1;
       }
       else if (ucTestCase == 327 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send 16-byte HEADER with last byte changed to 7C, then sleep for %dms", ucTestCase, ucSleepTimeMs);
          myCrcCorrByte = ucByteBuffer[uiLenToSend - 1];
          ucByteBuffer[uiLenToSend - 1] = 0x7c;
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucByteBuffer[uiLenToSend - 1] = myCrcCorrByte;
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else if (ucTestCase == 328 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Sleep for %dms, and NOT sending 16-bytes HEADER, and NOT sending DATA", ucTestCase, ucSleepTimeMs);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else
       {
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
       }
     }
    else
    {
       if (ucTestCase == 301 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Sleeping for %dms before sending %d bytes DATA", ucTestCase, ucSleepTimeMs, uiLenToSend);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucTestDone = 1;
       }
       else if (ucTestCase == 302 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send only first 8 bytes of %d bytes of DATA, then sleep for %dms", ucTestCase, uiLenToSend, ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, 8);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else if (ucTestCase == 303 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send first 8 bytes of %d bytes DATA, sleep for %dms, then send remaining %d DATA", ucTestCase, uiLenToSend, ucSleepTimeMs, uiLenToSend-8);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, 8);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, &ucByteBuffer[8], uiLenToSend-8);
          ucTestDone = 1;
       }
       else if (ucTestCase == 304 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send first 8 bytes of %d bytes DATA, sleep for %dms, then send full %d bytes DATA", ucTestCase, uiLenToSend, ucSleepTimeMs, uiLenToSend);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, 8);
          fw_upload_DelayInMs(ucSleepTimeMs);
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucTestDone = 1;
       }
       else if (ucTestCase == 305 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Sleep for %dms, and NOT sending %d bytes DATA", ucTestCase, ucSleepTimeMs, uiLenToSend);
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else if (ucTestCase == 306 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send %d bytes DATA with last byte changed to 7C", ucTestCase, uiLenToSend);
          myCrcCorrByte = ucByteBuffer[uiLenToSend - 1];
          ucByteBuffer[uiLenToSend - 1] = 0x7c;
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucByteBuffer[uiLenToSend - 1] = myCrcCorrByte;
          ucTestDone = 1;
       }
       else if (ucTestCase == 307 && !ucTestDone)
       {
          PRINT("\n         TC-%d:  Send %d bytes DATA with last byte changed to 7C, then sleep for %dms", ucTestCase, uiLenToSend, ucSleepTimeMs);
          myCrcCorrByte = ucByteBuffer[uiLenToSend - 1];
          ucByteBuffer[uiLenToSend - 1] = 0x7c;
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
          ucByteBuffer[uiLenToSend - 1] = myCrcCorrByte;
          fw_upload_DelayInMs(ucSleepTimeMs);
          ucTestDone = 1;
       }
       else
       {
          fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);
       }
    }

#else

     fw_upload_ComWriteChars(mchar_fd, ucByteBuffer, uiLenToSend);

#endif
     ulLastOffsetToSend = ulOffset;
   }
}

/******************************************************************************
 *
 * Name: fw_Change_Baudrate
 *
 * Description:
 *   This function changes the baud rate of bootrom.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pPortName:        Serial port value.
 *   iFirstBaudRate:   The default baud rate of boot rom.
 *   iSecondBaudRate:  The chaned baud rate.
 *
 * Return Value:
 *   TRUE:            Change baud rate successfully
 *   FALSE:           Change baud rate unsuccessfully
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static int32 fw_Change_Baudrate(int8 *pPortName, int32 iFirstBaudRate, int32 iSecondBaudRate, BOOLEAN bFirstWaitHeaderSignature) {
  uint8 uartConfig[60];
  uint8 ucBuffer[80];
  uint32 j;
  uint32 uartClk = 0x00C00000;
  uint32 uartDiv = 0x1;
  uint16 uiLenToSend = 0;
  uint32 uiCrc = 0;
  uint32 uiLen = 0;
  BOOLEAN bRetVal = FALSE;
  int32 ucResult = -1;
  uint8 ucLoadPayload = 0;
  uint32 waitHeaderSigTime = 0;
  BOOLEAN uiReUsedInitBaudrate = FALSE;
  uint32 headLen = 0;

  uint32 mcr = MCR;
  uint32 init = INIT;
  uint32 icr = ICR;
  uint32 fcr = FCR;
  uint32 brAddr = CLKDIVAddr;
  uint32 divAddr = UARTDIVAddr;
  uint32 mcrAddr = UARTMCRAddr;
  uint32 reInitAddr = UARTREINITAddr;
  uint32 icrAddr = UARTICRAddr;
  uint32 fcrAddr = UARTFCRAddr;

  PRINT("ComPort : %s\n", pPortName);

  for (j = 0; j < sizeof(UartCfgTbl) / sizeof(UART_BAUDRATE); j++) {
    if (iSecondBaudRate == (int32)UartCfgTbl[j].iBaudRate) {
      uartDiv = UartCfgTbl[j].iUartDivisor;
      uartClk = UartCfgTbl[j].iClkDivisor;
      ucResult = 0;
      break;
    }
  }

  if (ucResult != 0) {
    return ucResult;
  }

  // Generate CRC value for CMD5 payload
  memcpy(uartConfig + uiLen, &brAddr, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &uartClk, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &divAddr, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &uartDiv, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &mcrAddr, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &mcr, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &reInitAddr, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &init, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &icrAddr, 4);
  uiLen += 4;
  memcpy(uartConfig + uiLen, &icr, 4);
  uiLen += 4;
  memcpy(uartConfig+uiLen,&fcrAddr,4);
  uiLen += 4;
  memcpy(uartConfig+uiLen,&fcr,4);
  uiLen += 4;
  headLen = uiLen+4;

  fw_upload_gen_crc_table();
  memcpy(m_Buffer_CMD5_Header + 8, &headLen,4);

  uiCrc = fw_upload_update_crc(0, m_Buffer_CMD5_Header, 12);
  uiCrc = SWAPL(uiCrc);
  memcpy(m_Buffer_CMD5_Header+12,&uiCrc ,4);

  uiCrc = fw_upload_update_crc(0, uartConfig, uiLen); // TODO: why uint8 --> int8
  uiCrc = SWAPL(uiCrc);
  memcpy(uartConfig + uiLen, &uiCrc, 4);
  uiLen += 4;

  while (!bRetVal) {
    if (ucLoadPayload != 0 || uiReUsedInitBaudrate) {
      waitHeaderSigTime = TIMEOUT_VAL_MILLISEC;
    } else {
      waitHeaderSigTime = 2000;
    }
    // Wait to Receive 0xa5, 0xaa, 0xab, 0xa7
    // If the second baudrate is used, wait for 2s to check 0xa5
    if (bFirstWaitHeaderSignature && fw_upload_WaitForHeaderSignature(waitHeaderSigTime)) {
      if (ucLoadPayload) {
        if (uiProVer == Ver3) {
          change_baudrata_buffer_len =
          HDR_LEN + uiNewLen;
        }
        break;
      }
    } else {
      if (uiReUsedInitBaudrate) {
        ucResult = -2;
        return ucResult;
      }
      if (ucLoadPayload) {
        // If 0xa5 or 0xa7 is not received by using the second baudrate, change baud rate to the first
        // baudrate.
        close(mchar_fd);
        mchar_fd = init_uart(pPortName, iFirstBaudRate, 0);
        ucLoadPayload = 0;
        uiReUsedInitBaudrate = TRUE;
        continue;
      }
    }
    if (uiProVer == Ver1) {
      uiLenToSend = fw_upload_WaitFor_Len(NULL);
      if (uiLenToSend == 0) {
        continue;
      } else if (uiLenToSend == HDR_LEN) {
        // Download CMD5 header and Payload packet.
        tcflush(mchar_fd, TCIFLUSH);
        memcpy(ucBuffer, m_Buffer_CMD5_Header, HDR_LEN);
        memcpy(ucBuffer + HDR_LEN, uartConfig, uiLen);
        fw_upload_SendBuffer(uiLenToSend, ucBuffer, TRUE);
        close(mchar_fd);
        mchar_fd = init_uart(pPortName, iSecondBaudRate, 1); // TODO: there is no check if mchar_fd is valid
        ucLoadPayload = 1;
      } else {
        // Download CMD5 header and Payload packet
        fw_upload_ComWriteChars(mchar_fd, uartConfig, uiLen);
        close(mchar_fd);
        mchar_fd = init_uart(pPortName, iSecondBaudRate, 1);
        ucLoadPayload = 1;
      }
    } else if (uiProVer == Ver3) {
      if(!bFirstWaitHeaderSignature || fw_upload_WaitFor_Req(iSecondBaudRate)) {
        if (uiNewLen != 0 && ucRcvdHeader == V3_HEADER_DATA_REQ) {
          if (uiNewError == 0) {
            fw_upload_Send_Ack(V3_REQUEST_ACK);
            bFirstWaitHeaderSignature = TRUE;

            if (uiNewLen == HDR_LEN) {
              fw_upload_ComWriteChars(mchar_fd, m_Buffer_CMD5_Header, uiNewLen);
              ulLastOffsetToSend = ulNewOffset;
            } else {
              fw_upload_ComWriteChars(mchar_fd, uartConfig, uiNewLen);
              // Reopen Uart by using the second baudrate after downloading the payload.
              close(mchar_fd);
              mchar_fd = init_uart(pPortName, iSecondBaudRate, 1);
              ucLoadPayload = 1;
            }

          } else  // NAK,TIMEOUT,INVALID COMMAND...
          {
            tcflush(mchar_fd, TCIFLUSH);
            fw_upload_Send_Ack(V3_TIMEOUT_ACK);
          }
        }
      }
    } else {
      PRINT("%d Protocol Version not supported", uiProVer);
    }
  }
  return ucResult;
}

/******************************************************************************
 *
 * Name: fw_Change_Timeout
 *
 * Description:
 *   This function changes timeout value of boot loader
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pPortName:       Com port number.

 * Return Value:
 *   the status  of changing timeout value
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static int32 fw_Change_Timeout()
{

  int32 Status = -1;
  BOOLEAN bFirst = TRUE;
  BOOLEAN bRetVal = FALSE;
  uint8 reTryNumber  = 0;
  fw_upload_gen_crc_table();

  if (enable_poke_controller && (uiProVer == Ver3)) {
    send_poke = TRUE;
  }

  while (!bRetVal)
  {
    if(fw_upload_WaitForHeaderSignature(TIMEOUT_VAL_MILLISEC)) {
      // if(ucRcvdHeader != V3_START_INDICATION && ucRcvdHeader != V1_START_INDICATION) {
      //   return Status;
      // }
      if(uiProVer == Ver3) {
        if(fw_upload_WaitFor_Req(1)) {
          if(uiNewLen != 0) {
            if(uiNewError == 0) {
#ifdef DEBUG_PRINT
              PRINT("\n === Succ: REQ = 0xA7, Errcode = 0 ");
#endif
              if(bFirst || ulLastOffsetToSend == ulNewOffset) {
                fw_upload_Send_Ack(V3_REQUEST_ACK);
                fw_upload_ComWriteChars(mchar_fd, m_Buffer_CMD7_ChangeTimeoutValue, uiNewLen);
                ulLastOffsetToSend = ulNewOffset;
                bFirst = FALSE;
              } else {
                  bRetVal = TRUE;
                  Status = 0;
              }
            } else {
              if(reTryNumber < 6) {
                tcflush(mchar_fd, TCIFLUSH);
                fw_upload_Send_Ack(V3_TIMEOUT_ACK);
                reTryNumber++;
              } else {
                  bRetVal = TRUE;
              }
            }
          }
        }
      }
      if(uiProVer == Ver1) {
          Status = 1;
          break;
      }
    } else {
      PRINT("\n wait for head sig tiemout\n");
      break;
    }
  }
  return Status;
}
/******************************************************************************
 *
 * Function:      bt_send_cmd5_data_ver3
 *
 * Description:   Sends CMD5 Data for Protocol Version 3
 *
 * Arguments:
 *   cmd5_data:   Buffer containing CMD5 Header and its Payload
 *
 *   read_sig_hdr_after_cmd5: Read the Signature Header after sending CMD5 over
 *                            UART
 *
 * Return Value:
 *    0 incase of success.
 *   -1 if CMD5 is not sent
 *   -2 if read_sig_hdr_after_cmd5 fails
 *****************************************************************************/
int bt_send_cmd5_data_ver3(uint8* cmd5_data, BOOLEAN read_sig_hdr_after_cmd5)
{
  BOOLEAN header_sent = FALSE;
  int8 ret_value = -1;
  if (cmd5_data != NULL) {
    while (ret_value != 0) {
      if (fw_upload_WaitFor_Req(0)) {
        if (uiNewLen != 0 && uiNewError == 0) {
          fw_upload_Send_Ack(V3_REQUEST_ACK);
          if (header_sent == FALSE && uiNewLen == HDR_LEN) {
            fw_upload_ComWriteChars(mchar_fd, cmd5_data, uiNewLen);
            header_sent = TRUE;
            VNDDBG("Sent CMD5 Header: %d", uiNewLen);
          } else if (header_sent == TRUE) {
            fw_upload_ComWriteChars(mchar_fd, cmd5_data + HDR_LEN, uiNewLen);
            header_sent = FALSE;
            ret_value = 0;
            cmd5_len = uiNewLen + HDR_LEN;
            VNDDBG("Sent CMD5 payload: %d", uiNewLen);
          } else {
            VNDDBG(
                "Unexpected Error CMD5 uiNewLen = %d uiNewError = %d "
                "header_sent=%d",
                uiNewLen, uiNewError, header_sent);
          }
        } else {
          tcflush(mchar_fd, TCIFLUSH);
          fw_upload_Send_Ack(V3_TIMEOUT_ACK);
          if (uiNewError & BT_MIC_FAIL_BIT) {
            change_baudrata_buffer_len = 0;
            cmd5_len = 0;
            ulCurrFileSize = 0;
            ulLastOffsetToSend = 0xFFFF;
          }
        }
      }
      if ((ret_value != 0) || read_sig_hdr_after_cmd5) {
        if (!fw_upload_WaitForHeaderSignature(TIMEOUT_VAL_MILLISEC)) {
          ret_value = -2;
          break;
        }
      }
    }
  }
  return ret_value;
}

/******************************************************************************
 *
 * Function:      bt_send_cmd5_data_ver1
 *
 * Description:   Sends CMD5 Data for Protocol Version 1
 *
 * Arguments:
 *   cmd5_data:   Buffer containing CMD5 Header and its Payload
 *
 *   read_sig_hdr_after_cmd5: Read the Signature Header after sending CMD5 over
 *                            UART
 *
 * Return Value:
 *    0 if success and read_sig_hdr_after_cmd5 is FALSE.
 *    Len of next header if success and read_sig_hdr_after_cmd5 is true
 *   -1 if CMD5 is not sent
 *****************************************************************************/
int bt_send_cmd5_data_ver1(uint8* cmd5_data, BOOLEAN read_sig_hdr_after_cmd5) {
  int ret_value = -1;
  int8 retry = 3;
  uint16 uiLenToSend = 0;
  if (cmd5_data != NULL) {
    while (retry--) {
      uiLenToSend = fw_upload_WaitFor_Len(NULL);
      if (uiLenToSend != HDR_LEN) {
        PRINT("Unexpected Header Length Received: %d", uiLenToSend);
        /* If expected header length is invalid, check after signature header
         */
        if (fw_upload_WaitForHeaderSignature(TIMEOUT_VAL_MILLISEC) == FALSE) {
          PRINT("Header Signature Timeout CMD5 not Sent");
          return ret_value;
        }
        if (retry == 0) {
          return ret_value;
        }
      } else {
        PRINT("CMD5 bootloader length check completed");
        break;
      }
    }
    tcflush(mchar_fd, TCIFLUSH);
    ret_value =
        fw_upload_SendBuffer(HDR_LEN, cmd5_data, !read_sig_hdr_after_cmd5);
    PRINT("CMD5 sent succesfully");
  }
  return ret_value;
}
/******************************************************************************
 *
 * Function:      fw_loader_get_default_fw_name
 *
 * Description:   Incase bootcode version 3 is used get default FW Path.
 *
 * Arguments:
 * fw_name : Pointer to Firmware Path array.
 * fw_name_size: Size of fw_name.
 *
 * Return Value: NA
 *****************************************************************************/
void fw_loader_get_default_fw_name(char fw_name[], uint32 fw_name_size) {
  if (uiProVer == Ver3) {
    int size_of_array = sizeof(soc_fw_name_dict) / sizeof(soc_fw_name_dict[0]);
    for (int i = 0; i < size_of_array; i++) {
      if (soc_fw_name_dict[i].soc_id == chip_id) {
        memset(fw_name, 0, fw_name_size);
        strcpy(fw_name, FW_DEFAULT_PATH);
        strcat(fw_name, soc_fw_name_dict[i].default_fw_name);
        VNDDBG("Default Firmware selected= %s", fw_name);
        break;
      }
    }
  }
}
#if (UART_DOWNLOAD_FW == TRUE)
/******************************************************************************
 *
 * Function:      bt_get_fw_config_cmd5_data
 *
 * Description:   Fetch FW conig CMD5 based on conf file settings
 *
 * Arguments: NA
 *
 * Return Value: Pointer to FW Config CMD5 if success or NULL
 *****************************************************************************/
static uint8* bt_get_fw_config_cmd5_data(void) {
  VNDDBG("Opening CMD5 file");
  FILE* fp = fopen(pFilename_fw_init_config_bin, "r");
  uint8* ret = NULL;
  if (fp == NULL) {
    VNDDBG("%s file not found : error %s ", pFilename_fw_init_config_bin,
           strerror(errno));
  } else {
    /*Make sure length of file is in limit and copied in fw_init_config_bin */
    long data_size;
    fseek(fp, 0, SEEK_END);
    data_size = ftell(fp);
    if (data_size <= FW_INIT_CONFIG_LEN) {
      fseek(fp, 0, SEEK_SET);
      ret = (fread(fw_init_config_bin, 1, data_size, fp) == data_size)
                ? fw_init_config_bin
                : NULL;
    }
    fclose(fp);
    VNDDBG("Closing CMD5 file");
  }
  return ret;
}
#endif
/******************************************************************************
 *
 * Name: fw_upload_FW
 *
 * Description:
 *   This function performs the task of FW load over UART.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pPortName:       Com port number.
 *   iBaudRate:       the initial baud rate.
 *   ucFlowCtrl:      the flow ctrl of uart.
 *   pFileName:       the file name for downloading.
 *   iSecondBaudRate: the second baud rate.
 *
 * Return Value:
 *   TRUE:            Download successfully
 *   FALSE:           Download unsuccessfully
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint32 fw_upload_FW(int8 *pPortName, int32 iBaudRate, int8 *pFileName, int32 iSecondBaudRate) {
  uint8 *pFileBuffer = NULL;
  uint32 ulReadLen = 0;
  FILE *pFile = NULL;
  BOOLEAN bRetVal = FALSE;
  int32 result = 0;
  uint16 uiLenToSend = 0;
  BOOLEAN bFirstWaitHeaderSignature = TRUE;
  BOOLEAN check_sig_hdr = TRUE;
#if (UART_DOWNLOAD_FW == TRUE)
  /*FW config CMD5 needs to be sent before Helper and Firmware only once*/
  static BOOLEAN send_fw_config_cmd5 = TRUE;
#endif
  // Open File for reading.
  pFile = fopen(pFileName, "rb");

  if (pFile == NULL) {
    PRINT("\nfile open failed\n");
    return OPEN_FILE_FAIL;
  }

  result = fw_Change_Timeout();

  if(result == -1)
  {
    fclose(pFile);
    return START_INDICATION_NOT_FOUND;
  }

  if(result == 0)
  {
    cmd7_change_timeout_len = HDR_LEN;
    bFirstWaitHeaderSignature = FALSE;
  }

  if (iSecondBaudRate != 0) {
    uint32 j = 0;
    result = fw_Change_Baudrate(pPortName, iBaudRate, iSecondBaudRate, bFirstWaitHeaderSignature);
    switch (result) {
      case -1:
        PRINT("\nSecond baud rate %d is not support\n", iSecondBaudRate);
        PRINT("\nFw loader only supports the baud rate as");
        for (j = 0; j < sizeof(UartCfgTbl) / sizeof(UART_BAUDRATE); j++) {
          PRINT(" %d ", UartCfgTbl[j].iBaudRate);
        }
        PRINT("\n");
        break;
      case -2:
        PRINT("\n0xa5 or 0xaa is not received after changing baud rate in 2s.\n");
        break;
      default:
        break;
    }
    if (result != 0) {
      fclose(pFile);
      return CHANGE_BAUDRATE_FAIL;
    }
  }

#if ((UART_DOWNLOAD_FW == TRUE) )
  VNDDBG("Sending FW Config");
  if (send_fw_config_cmd5 == TRUE) {
    if (((uiProVer == Ver1) &&
         ((uiLenToSend = bt_send_cmd5_data_ver1(bt_get_fw_config_cmd5_data(),
                                                TRUE)) == HDR_LEN)) ||
        ((uiProVer == Ver3) &&
         (bt_send_cmd5_data_ver3(bt_get_fw_config_cmd5_data(), TRUE) == 0))) {
      check_sig_hdr = FALSE;
      PRINT("\n ========== Download Complete FW CONFIG=========\n\n");
    } else {
      VNDDBG("Sending FW config CMD5 FAILED protocol version %d", uiProVer + 1);
    }
    send_fw_config_cmd5 = FALSE;
  }
#endif
  // Calculate the size of the file to be downloaded.
  VNDDBG("Opening FW file");
  result = fseek(pFile, 0, SEEK_END);
  if (result != 0) {
    PRINT("\nfseek failed\n");
    fclose(pFile);
    return FEEK_SEEK_ERROR;
  }

  ulTotalFileSize = (long)ftell(pFile);
  if (ulTotalFileSize <= 0) {
    PRINT("\nError:Download Size is 0\n");
    fclose(pFile);
    return FILESIZE_IS_ZERO;
  }

  pFileBuffer = (uint8 *) malloc(ulTotalFileSize);
  if(pFileBuffer == NULL)
  {
    PRINT("malloc() returned NULL while allocating size for file");
    fclose(pFile);
    return MALLOC_RETURNED_NULL;
  }

  result = fseek(pFile, 0, SEEK_SET);
  if (result != 0) {
    PRINT("\nfseek() failed");
    free(pFileBuffer);
    fclose(pFile);
    return FEEK_SEEK_ERROR;
  }

  if (pFileBuffer != (void *)0) {
    ulReadLen = (uint32)fread((void *)pFileBuffer, 1, ulTotalFileSize, pFile);
    if (ulReadLen != ulTotalFileSize) {
      PRINT("\nError:Read File Fail\n");
      free(pFileBuffer);
      fclose(pFile);
      return READ_FILE_FAIL;
    }
  }
   ulCurrFileSize = 0;
  VNDDBG("Closing FW file");
  // Jump to here in case of protocol resync.
  if(setjmp(resync) > 1) {
    PRINT("\nSome error occured");
    free(pFileBuffer);
    return UNEXPECTED_BEHAVIOUR_IN_SETJMP;
  }

  while (!bRetVal) {
    // Wait to Receive 0xa5, 0xaa, 0xab, 0xa7
    if (check_sig_hdr && (!iSecondBaudRate) &&
        (!fw_upload_WaitForHeaderSignature(TIMEOUT_VAL_MILLISEC))) {
      PRINT("\n0xa5,0xaa,0xab or 0xa7 is not received in %d ms\n",
            TIMEOUT_VAL_MILLISEC);
      free(pFileBuffer);
      fclose(pFile);
      return HEADER_SIGNATURE_TIMEOUT;
    }
    if (uiProVer == Ver1) {
      // Read the 'Length' bytes requested by Helper
      if (check_sig_hdr) {
        uiLenToSend = fw_upload_WaitFor_Len(pFile);
      }
      PRINT("Number of bytes to be downloaded: %8ld\r", ulTotalFileSize);
      tcflush(mchar_fd, TCIFLUSH);
      do {
        uiLenToSend = fw_upload_V1SendLenBytes(pFileBuffer, uiLenToSend);
      } while (uiLenToSend != 0);
      PRINT("File downloaded: %8u:%8ld\r", ulCurrFileSize, ulTotalFileSize);
      // If the Length requested is 0, download is complete.
      if (uiLenToSend == 0) {
        bRetVal = TRUE;
        break;
      }
    } else if (uiProVer == Ver3) {
      if(fw_upload_WaitFor_Req(0)) {
        if (uiNewLen != 0) {
          if (uiNewError == 0) {
#ifdef DEBUG_PRINT
            PRINT("\n === Succ: REQ = 0xA7, Errcode = 0 ");
#endif
            fw_upload_Send_Ack(V3_REQUEST_ACK);
            fw_upload_V3SendLenBytes(pFileBuffer, uiNewLen, ulNewOffset);

#ifdef DEBUG_PRINT
            PRINT("\n sent %d bytes..\n", uiNewLen);
#endif
          } else  // NAK,TIMEOUT,INVALID COMMAND...
          {
#ifdef DEBUG_PRINT
            uint8 i;
            PRINT("\n === Fail: REQ = 0xA7, Errcode != 0 ");
            for (i = 0; i < 7; i++) {
              uiErrCnt[i] += (uiNewError >> i) & 0x1;
            }
#endif
            tcflush(mchar_fd, TCIFLUSH);
            fw_upload_Send_Ack(V3_TIMEOUT_ACK);
            if(uiNewError & BT_MIC_FAIL_BIT)
            {
              change_baudrata_buffer_len = 0;
              cmd5_len = 0;
              ulCurrFileSize = 0;
              ulLastOffsetToSend = 0xFFFF;
            }
          }
        } else {
          /* check if download complete */
          if (uiNewError == 0) {
            fw_upload_Send_Ack(V3_REQUEST_ACK);
            bRetVal = TRUE;
            break;
          } else if (uiNewError & BT_MIC_FAIL_BIT) {
  #ifdef DEBUG_PRINT
            uiErrCnt[7] += 1;
  #endif
            fw_upload_Send_Ack(V3_REQUEST_ACK);
            fseek(pFile, 0, SEEK_SET);
            change_baudrata_buffer_len = 0;
            cmd5_len = 0;
            ulCurrFileSize = 0;
            ulLastOffsetToSend = 0xFFFF;
          } else {
            PRINT("Non-empty terminating else statement uiNewError = %d",
                  uiNewError);
          }
        }
      }
      PRINT("File downloaded: %8u:%8ld\r", ulCurrFileSize, ulTotalFileSize);
    } else {
      PRINT("%d Protocol Version not supported", uiProVer);
    }
    iSecondBaudRate = 0;
    check_sig_hdr = TRUE;
  }

  if(pFileBuffer != NULL)
  {
    free(pFileBuffer);
    pFileBuffer = NULL;
  }
  fclose(pFile);
  return DOWNLOAD_SUCCESS;
}

/******************************************************************************
 *
 * Name: fw_upload_check_FW
 *
 * Description:
 *   This function performs the task of FW load over UART.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pPortName:       Com port number.
 *   iBaudRate:       the initial baud rate.
 *   ucFlowCtrl:      the flow ctrl of uart.
 *
 * Return Value:
 *   TRUE:            Need Download FW
 *   FALSE:           No need Download FW
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
BOOLEAN bt_vnd_mrvl_check_fw_status() {
  BOOLEAN bRetVal = FALSE;

  if (mchar_fd < 0) {
    printf("\nPort is not open or file not found\n");
    return bRetVal;
  }

  // Wait to Receive 0xa5, 0xaa, 0xab, 0xa7
  bRetVal = fw_upload_WaitForHeaderSignature(1000);

  printf("fw_upload_WaitForHeaderSignature return %d", bRetVal);

  return bRetVal;
}

/******************************************************************************
*
* Name: bt_vnd_mrvl_download_fw
*
* Description:
*   Wrapper of fw_upload_FW.
*
* Conditions For Use:
*   None.
*
* Arguments:
*   pPortName:       Com port number.
*   iBaudRate:       the initial baud rate.
*   ucFlowCtrl:      the flow ctrl of uart.
*   pFileName:       the file name for downloading.
*   iSecondBaudRate: the second baud rate.
*
* Return Value:
*   0:            Download successfully
*   1:           Download unsuccessfully
*
* Notes:
*   None.
*
*****************************************************************************/
int bt_vnd_mrvl_download_fw(int8 *pPortName, int32 iBaudrate, int8 *pFileName, int32 iSecondBaudrate) {
  uint64 endTime;
  uint64 start;
  uint64 cost;
  uint32 ulResult;

  start = fw_upload_GetTime();

  printf("Protocol: NXP Proprietary\n");
  printf("FW Loader Version: %s\n", VERSION);
  printf("ComPort : %s\n", pPortName);
  printf("BaudRate: %d\n", iBaudrate);
  printf("Filename: %s\n", pFileName);
  printf("iSecondBaudrate: %d\n", iSecondBaudrate);

  ulResult = fw_upload_FW(pPortName, iBaudrate, pFileName, iSecondBaudrate);
  if (ulResult == 0) {
    printf("\nDownload Complete\n");
    cost = fw_upload_GetTime() - start;
    printf("time:%llu\n", cost);
    if (uiProVer == Ver1) {
      fw_upload_DelayInMs(MAX_CTS_TIMEOUT);
      endTime = fw_upload_GetTime() + MAX_CTS_TIMEOUT;
      do {
        if (!fw_upload_ComGetCTS(mchar_fd)) {
          printf("CTS is low\n");
          goto done;
        }
      } while (endTime > fw_upload_GetTime());
      printf("wait CTS low timeout \n");
      goto done;
    } else if (uiProVer == Ver3) {
      endTime = fw_upload_GetTime() + END_SIG_TIMEOUT;
      do {
        if (!fw_upload_ComGetCTS(mchar_fd)) {
          printf("CTS is low\n");
          goto done;
        }
      } while (endTime > fw_upload_GetTime());
      goto done;
    }
  } else {
    PRINT("\nDownload Error, Error code = %d\n", ulResult);
    return ulResult;
  }

done:
  bVerChecked = FALSE;
  return 0;
}
