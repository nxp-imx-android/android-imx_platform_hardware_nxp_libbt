/******************************************************************************
 *
 *  Copyright 2009-2023 NXP
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
 *  Filename:      fw_loader_uart_v2.c
 *
 *  Description:   Firmware loader version 2 functions
 *
 ******************************************************************************/

#define LOG_TAG "fw_loader"

/*============================== Include Files ===============================*/
#include "fw_loader_uart_v2.h"

#include <cutils/properties.h>
#include <errno.h>
#include <malloc.h>
#include <memory.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>

#include "bt_vendor_log.h"
#include "fw_loader_io.h"

#define TIMEOUT_FOR_READ 4000

/*================================== Macros ==================================*/
#define VERSION "M206"
#define MAX_LENGTH 0xFFFF  // Maximum 2 byte value
#define END_SIG_TIMEOUT 2500
#define MAX_CTS_TIMEOUT 5000  // 5s
#define STRING_SIZE 6
#define HDR_LEN 16
#define CMD4 0x4
#define CMD6 0x6
#define CMD7 0x7

#define DELAY_CMD5_PATCH 250  // 250ms
#define POLL_AA_TIMEOUT 200
/* Timeout for getting 0xa5 or 0xaa or 0xa6, 2 times of helper timeout*/
#define TIMEOUT_VAL_MILLISEC 4000

#define BOOT_HEADER 0xa5
#define BOOT_HEADER_ACK 0x5a
#define HELPER_HEADER 0xa6
#define HELPER_HEADR_ACK 0x6a
#define HELPER_TIMEOUT_ACK 0x6b
#define VERSION_HEADER 0xaa

/*================================== Typedefs=================================*/

/*================================ Global Vars================================*/

// Maximum Length that could be asked by the Helper = 2 bytes
static uint8 ucByteBuffer[MAX_LENGTH];

// Size of the File to be downloaded
static uint32 uiTotalFileSize = 0;

// Current size of the Download
static uint32 ulCurrFileSize = 0;
static uint32 ulLastOffsetToSend = 0xFFFF;
static BOOLEAN uiErrCase = FALSE;
static BOOLEAN uiReDownload = FALSE;

// Received Header
static uint8 ucRcvdHeader = 0xFF;
static BOOLEAN ucHelperOn = FALSE;
static uint8 ucString[STRING_SIZE];
static uint8 ucCmd5Sent = 0;
static BOOLEAN b16BytesData = FALSE;

// Handler of File
static FILE* pFile = NULL;

// CMD5 patch to change bootloader timeout to 2 seconds
uint8 ucCmd5Patch[28] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x0C, 0x00, 0x00, 0x00, 0x9D, 0x32,
                         0xBB, 0x11, 0x2C, 0x94, 0x00, 0xA8, 0xEC,
                         0x70, 0x02, 0x00, 0xB4, 0xD9, 0x9D, 0x26};

jmp_buf resync;  // Protocol restart buffer used in timeout cases.
/*============================ Function Prototypes ===========================*/

/*============================== Coded Procedures ============================*/

/******************************************************************************
 *
 * Name: fw_upload_WaitForHeaderSignature()
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
 *   TRUE:   0xa5 or 0xaa or 0xa6 is received.
 *   FALSE:  0xa5 or 0xaa or 0xa6 is not received.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static BOOLEAN fw_upload_WaitForHeaderSignature(uint32 uiMs) {
  uint8 ucDone = 0;  // signature not Received Yet.
  uint64 startTime = 0;
  uint64 currTime = 0;
  BOOLEAN bResult = TRUE;
  ucRcvdHeader = 0xFF;
  startTime = fw_upload_GetTime();
  while (!ucDone) {
    fw_upload_ComReadChars(mchar_fd, (uint8*)&ucRcvdHeader, 1);
    if ((ucRcvdHeader == BOOT_HEADER) || (ucRcvdHeader == VERSION_HEADER) ||
        (ucRcvdHeader == HELPER_HEADER)) {
      ucDone = 1;
      VND_LOGV("Received 0x%x", ucRcvdHeader);
    } else {
      if (uiMs) {
        currTime = fw_upload_GetTime();
        if (currTime - startTime > uiMs) {
          VND_LOGE("Signature wait timmedout %d", uiMs);
          bResult = FALSE;
          break;
        }
      }
      fw_upload_DelayInMs(1);
    }
  }
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
 *   None.
 *
 * Return Value:
 *   2 Byte Length to send back to the Helper.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint16 fw_upload_WaitFor_Len(FILE* pFile) {
  uint8 uiVersion;
  // Length Variables
  uint16 uiLen = 0x0;
  uint16 uiLenComp = 0x0;
  // uiLen and uiLenComp are 1's complement of each other.
  // In such cases, the XOR of uiLen and uiLenComp will be all 1's
  // i.e 0xffff.
  uint16 uiXorOfLen = 0xFFFF;

  // Read the Lengths.
  fw_upload_ComReadChars(mchar_fd, (uint8*)&uiLen, 2);
  fw_upload_ComReadChars(mchar_fd, (uint8*)&uiLenComp, 2);

  // Check if the length is valid.
  if ((uiLen ^ uiLenComp) == uiXorOfLen)  // All 1's
  {
    VND_LOGV("bootloader asks for %d bytes", uiLen);
    // Successful. Send back the ack.
    if ((ucRcvdHeader == BOOT_HEADER) || (ucRcvdHeader == VERSION_HEADER)) {
      fw_upload_ComWriteChar(mchar_fd, (int8)BOOT_HEADER_ACK);
      if (ucRcvdHeader == VERSION_HEADER) {
        // We have received the Chip Id and Rev Num that the
        // helper intended to send. Ignore the received
        // Chip Id, Rev Num and proceed to Download.
        uiVersion = (uiLen >> 8) & 0xF0;
        uiVersion = uiVersion >> 4;
        VND_LOGV("Helper Version is: %d", uiVersion);
        if (ucHelperOn == TRUE) {
          if (fseek(pFile, 0, SEEK_SET) < 0) {
            VND_LOGE("fseek error: %s (%d)", strerror(errno), errno);
          }
          ulCurrFileSize = 0;
          ulLastOffsetToSend = 0xFFFF;
        }
        // Ensure any pending write data is completely written
        if (0 == tcdrain(mchar_fd)) {
          VND_LOGV("tcdrain succeeded");
        } else {
          VND_LOGV("Version ACK, tcdrain failed with errno = %d", errno);
        }

        longjmp(resync, 1);
      }
    }
  } else {
    VND_LOGV("NAK case: bootloader LEN = %x bytes", uiLen);
    VND_LOGV("NAK case: bootloader LENComp = %x bytes", uiLenComp);
    // Failure due to mismatch.
    fw_upload_ComWriteChar(mchar_fd, (int8)0xbf);
    // Start all over again.
    longjmp(resync, 1);
  }
  return uiLen;
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
static void fw_upload_GetHeaderStartBytes(uint8* ucStr) {
  BOOLEAN ucDone = FALSE, ucStringCnt = 0, i;

  while (!ucDone) {
    ucRcvdHeader = 0xFF;
    fw_upload_ComReadChars(mchar_fd, (uint8*)&ucRcvdHeader, 1);

    if (ucRcvdHeader == BOOT_HEADER) {
      ucStr[ucStringCnt++] = ucRcvdHeader;
      ucDone = TRUE;

      VND_LOGV("Received 0x%x", ucRcvdHeader);
    } else {
      fw_upload_DelayInMs(1);
    }
  }
  while (!fw_upload_GetBufferSize(mchar_fd))
    ;
  for (i = 0; i < 4; i++) {
    ucRcvdHeader = 0xFF;
    fw_upload_ComReadChars(mchar_fd, (uint8*)&ucRcvdHeader, 1);
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
static void fw_upload_GetLast5Bytes(uint8* buf) {
  uint8 a5cnt, i;
  uint8 ucTemp[STRING_SIZE];
  uint16 uiTempLen = 0;
  int32 fifosize;
  BOOLEAN alla5times = FALSE;

  // initialise
  memset(ucString, 0x00, STRING_SIZE);

  fifosize = fw_upload_GetBufferSize(mchar_fd);

  fw_upload_GetHeaderStartBytes(ucString);
  fw_upload_lenValid(&uiTempLen, ucString);

  if ((fifosize < 6) &&
      ((uiTempLen == HDR_LEN) || (uiTempLen == fw_upload_GetDataLen(buf)))) {
    VND_LOGV("=========>success case");
    uiErrCase = FALSE;
  } else  // start to get last valid 5 bytes
  {
    VND_LOGV("=========>fail case");
    while (fw_upload_lenValid(&uiTempLen, ucString) == FALSE) {
      fw_upload_GetHeaderStartBytes(ucString);
      fifosize -= 5;
    }
    VND_LOGV("Error cases 1, 2, 3, 4, 5...");
    if (fifosize > 5) {
      fifosize -= 5;
      do {
        do {
          a5cnt = 0;
          do {
            fw_upload_GetHeaderStartBytes(ucTemp);
            fifosize -= 5;
          } while ((fw_upload_lenValid(&uiTempLen, ucTemp) == TRUE) &&
                   (!alla5times) && (fifosize > 5));
          // if 5bytes are all 0xa5, continue to clear 0xa5
          for (i = 0; i < 5; i++) {
            if (ucTemp[i] == BOOT_HEADER) {
              a5cnt++;
            }
          }
          alla5times = TRUE;
        } while (a5cnt == 5);
        VND_LOGV("a5 count in last 5 bytes: %d", a5cnt);
        if (fw_upload_lenValid(&uiTempLen, ucTemp) == FALSE) {
          for (i = 0; i < (5 - a5cnt); i++) {
            ucTemp[i + a5cnt] = fw_upload_ComReadChar(mchar_fd);
          }
          memcpy(ucString, &ucTemp[a5cnt - 1], 5);
        } else {
          memcpy(ucString, ucTemp, 5);
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
 *   uiLenToSend: len of header request.
 *         ucBuf: the buf to be sent.
 * Return Value:
 *   Returns the len of next header request.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/

uint16 fw_upload_SendBuffer(uint16 uiLenToSend, uint8* ucBuf) {
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
        if ((uiFirstChunkSent == 0) ||
            ((uiFirstChunkSent == 1) && uiErrCase == TRUE)) {
          // Write first 16 bytes of buffer
          VND_LOGV("====>  Sending first chunk...");
          VND_LOGV("====>  Sending %d bytes...", uiBytesToSend);
          fw_upload_ComWriteChars(mchar_fd, (uint8*)ucBuf, uiBytesToSend);
          uiBytesToSend = uiDataLen;
          if (uiBytesToSend == HDR_LEN) {
            b16BytesData = TRUE;
          }
          uiFirstChunkSent = 0;
        } else {
          // Done with buffer
          ucSentDone = 1;
          break;
        }
      } else {
        // Write remaining bytes
        VND_LOGV("====>  Sending %d bytes...", uiBytesToSend);
        if (uiBytesToSend != 0) {
          fw_upload_ComWriteChars(mchar_fd, (uint8*)&ucBuf[HDR_LEN],
                                  uiBytesToSend);
          uiFirstChunkSent = 1;
          // We should expect 16, then next block will start
          uiBytesToSend = HDR_LEN;
          b16BytesData = FALSE;
        } else  // end of bin download
        {
          VND_LOGV("========== Download Complete =========");
          return 0;
        }
      }
    } else {
      // Something not good
      if ((uiLenToSend & 0x01) == 0x01) {
        // some kind of error
        if (uiLenToSend == (HDR_LEN + 1)) {
          // Send first chunk again
          VND_LOGV("1. Resending first chunk...");
          fw_upload_ComWriteChars(mchar_fd, (uint8*)ucBuf, (uiLenToSend - 1));
          uiBytesToSend = uiDataLen;
          uiFirstChunkSent = 0;
        } else if (uiLenToSend == (uiDataLen + 1)) {
          // Send second chunk again
          VND_LOGV("2. Resending second chunk...");
          fw_upload_ComWriteChars(mchar_fd, (uint8*)&ucBuf[HDR_LEN],
                                  (uiLenToSend - 1));
          uiBytesToSend = HDR_LEN;
          uiFirstChunkSent = 1;
        }
      } else if (uiLenToSend == HDR_LEN) {
        // Out of sync. Restart sending buffer
        VND_LOGV("3.  Restart sending the buffer...");
        fw_upload_ComWriteChars(mchar_fd, (uint8*)ucBuf, uiLenToSend);
        uiBytesToSend = uiDataLen;
        uiFirstChunkSent = 0;
      }
    }
    // Ensure any pending write data is completely written
    if (0 == tcdrain(mchar_fd)) {
      VND_LOGV("\t tcdrain succeeded");
    } else {
      VND_LOGV("\t tcdrain failed. Errno =%s (%d)", strerror(errno), errno);
    }
    if (!ucCmd5Sent && uiFirstChunkSent == 1) {
      fw_upload_DelayInMs(DELAY_CMD5_PATCH);
    }
    // Get last 5 bytes now
    fw_upload_GetLast5Bytes(ucBuf);
    // Get next length
    uiValidLen = FALSE;
    do {
      if (fw_upload_lenValid(&uiLenToSend, ucString) == TRUE) {
        // Valid length received
        uiValidLen = TRUE;
        VND_LOGV("Valid length = %d", uiLenToSend);

        // ACK the bootloader
        fw_upload_ComWriteChar(mchar_fd, (int8)BOOT_HEADER_ACK);
        VND_LOGV("BOOT_HEADER_ACK 0x5a sent");
      }
    } while (!uiValidLen);
  }
  VND_LOGV("========== Buffer is successfully sent =========");
  return uiLenToSend;
}

/******************************************************************************
 *
 * Name: fw_upload_WaitFor_Offset
 *
 * Description:
 *   This function gets offset value from helper.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   None.
 *
 * Return Value:
 *   offset value.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint32 fw_upload_WaitFor_Offset() {
  uint32 ulOffset = 0x0;
  uint32 ulOffsetComp = 0x0;

  // uiLen and uiLenComp are 1's complement of each other.
  // In such cases, the XOR of uiLen and uiLenComp will be all 1's
  // i.e 0xffff.
  uint32 uiXorOfOffset = 0xFFFFFFFF;

  // Read the Offset.
  fw_upload_ComReadChars(mchar_fd, (uint8*)&ulOffset, 4);
  fw_upload_ComReadChars(mchar_fd, (uint8*)&ulOffsetComp, 4);

  // Check if the length is valid.
  if ((ulOffset ^ ulOffsetComp) == uiXorOfOffset)  // All 1's
  {
    VND_LOGV("Helper ask for offset %d", ulOffset);
  } else {
    VND_LOGV("NAK case: helper Offset = %x bytes", ulOffset);
    VND_LOGV("NAK case: helper OffsetComp = %x bytes", ulOffsetComp);
    // Failure due to mismatch.
    fw_upload_ComWriteChar(mchar_fd, (int8)0xbf);

    // Start all over again.
    longjmp(resync, 1);
  }
  return ulOffset;
}

/******************************************************************************
 *
 * Name: fw_upload_WaitFor_ErrCode
 *
 * Description:
 *   This function gets error code from helper.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   None.
 *
 * Return Value:
 *   error code.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint16 fw_upload_WaitFor_ErrCode() {
  uint16 uiError = 0x0;
  uint16 uiErrorCmp = 0x0;
  uint16 uiXorOfErrCode = 0xFFFF;

  // Read the Error Code.
  fw_upload_ComReadChars(mchar_fd, (uint8*)&uiError, 2);
  fw_upload_ComReadChars(mchar_fd, (uint8*)&uiErrorCmp, 2);

  // Check if the Err Code is valid.
  if ((uiError ^ uiErrorCmp) == uiXorOfErrCode)  // All 1's
  {
    VND_LOGV("Error Code is %d", uiError);
    if (uiError == 0) {
      // Successful. Send back the ack.
      fw_upload_ComWriteChar(mchar_fd, (int8)HELPER_HEADR_ACK);
    } else {
      VND_LOGV("Helper NAK or CRC or Timeout");
      // NAK/CRC/Timeout
      fw_upload_ComWriteChar(mchar_fd, (int8)HELPER_TIMEOUT_ACK);
    }
  } else {
    VND_LOGV("NAK case: helper ErrorCode = %x bytes", uiError);
    VND_LOGV("NAK case: helper ErrorCodeComp = %x bytes", uiErrorCmp);
    // Failure due to mismatch.
    fw_upload_ComWriteChar(mchar_fd, (int8)0xbf);
    // Start all over again.
    longjmp(resync, 1);
  }
  return uiError;
}

/******************************************************************************
 *
 * Name: fw_upload_SendLenBytesToHelper
 *
 * Description:
 *   This function sends Len bytes to the Helper.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pFileBuffer: bin file buffer being sent.
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
static void fw_upload_SendLenBytesToHelper(uint8* pFileBuffer,
                                           uint16 uiLenToSend, uint32 ulOffset)

{
  // Retransmittion of previous block
  if (ulOffset == ulLastOffsetToSend) {
    VND_LOGV("Retx offset %d...", ulOffset);
    fw_upload_ComWriteChars(mchar_fd, (uint8*)ucByteBuffer, uiLenToSend);
  } else {
    // uint16 uiNumRead = 0;
    //  The length requested by the Helper is equal to the Block
    //  sizes used while creating the FW.bin. The usual
    //  block sizes are 128, 256, 512.
    //  uiLenToSend % 16 == 0. This means the previous packet
    //  was error free (CRC ok) or this is the first packet received.
    //   We can clear the ucByteBuffer and populate fresh data.
    memset(ucByteBuffer, 0, sizeof(ucByteBuffer));
    memcpy(ucByteBuffer, pFileBuffer + ulOffset, uiLenToSend);
    ulCurrFileSize += uiLenToSend;
    fw_upload_ComWriteChars(mchar_fd, (uint8*)ucByteBuffer, uiLenToSend);
    ulLastOffsetToSend = ulOffset;
  }
}

/******************************************************************************
 *
 * Name: fw_upload_SendIntBytes
 *
 * Description:
 *   This function sends 4 bytes and 4bytes' compare.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   4 bytes need to be sent.
 *
 * Return Value:
 *   None.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static void fw_upload_SendIntBytes(uint32 ulBytesToSent) {
  uint8 i, uTemp[9], uiLocalCnt = 0;
  uint32 ulBytesToSentCmp;

  ulBytesToSentCmp = ulBytesToSent ^ 0xFFFFFFFF;

  for (i = 0; i < 4; i++) {
    uTemp[uiLocalCnt++] = (uint8)(ulBytesToSent >> (i * 8)) & 0xFF;
  }
  for (i = 0; i < 4; i++) {
    uTemp[uiLocalCnt++] = (uint8)(ulBytesToSentCmp >> (i * 8)) & 0xFF;
  }

  fw_upload_ComWriteChars(mchar_fd, (uint8*)uTemp, 8);
}

/******************************************************************************
 *
 * Name: fw_upload_SendLenBytes
 *
 * Description:
 *   This function sends Len bytes(header+data) to the boot code.
 *
 * Conditions For Use:
 *   None.
 *
 * Arguments:
 *   pFileBuffer: bin file buffer being sent.
 *   uiLenTosend: the length will be sent.
 *
 * Return Value:
 *   the 'len' of next header request.
 *
 * Notes:
 *   None.
 *
 *****************************************************************************/
static uint16 fw_upload_SendLenBytes(uint8* pFileBuffer, uint16 uiLenToSend) {
  uint16 ucDataLen, uiLen;
  // uint16 uiNumRead = 0;
  memset(ucByteBuffer, 0, sizeof(ucByteBuffer));
  if (!ucCmd5Sent) {
    // put header and data into temp buffer first
    memcpy(ucByteBuffer, ucCmd5Patch, uiLenToSend);
    // get data length from header
    ucDataLen = fw_upload_GetDataLen(ucByteBuffer);
    memcpy(&ucByteBuffer[uiLenToSend], &ucCmd5Patch[uiLenToSend], ucDataLen);
    uiLen = fw_upload_SendBuffer(uiLenToSend, ucByteBuffer);
    ucCmd5Sent = 1;
    VND_LOGV("cmd5 patch is sent");
  } else {
    // fread(void *buffer, size_t size, size_t count, FILE *stream)
    memcpy(ucByteBuffer, pFileBuffer + ulCurrFileSize, uiLenToSend);
    ulCurrFileSize += uiLenToSend;
    ucDataLen = fw_upload_GetDataLen(ucByteBuffer);
    memcpy(&ucByteBuffer[uiLenToSend], pFileBuffer + ulCurrFileSize, ucDataLen);
    ulCurrFileSize += ucDataLen;
#ifdef DEBUG_PRINT
    VND_LOGV("The buffer is to be sent: %d", uiLenToSend + ucDataLen);
    for (uint i = 0; i < (uiLenToSend + ucDataLen); i++) {
      if (i % 16 == 0) {
        VND_LOGV("\n");
      }
      VND_LOGV("%02x", ucByteBuffer[i]);
    }
#endif
    // start to send Temp buffer
    uiLen = fw_upload_SendBuffer(uiLenToSend, ucByteBuffer);
    VND_LOGV("File downloaded: %8d:%8d\r", ulCurrFileSize, uiTotalFileSize);
  }
  return uiLen;
}

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
static BOOLEAN fw_upload_FW(int8* pFileName) {
  uint8* pFileBuffer = NULL;
  uint32 ulReadLen = 0;
  BOOLEAN bRetVal = FALSE;
  int32 result = 0;
  uint16 uiLenToSend = 0;
  long TotalFileSize;
  uint32 ulOffsettoSend = 0;
  uint16 uiErrCode = 0;

  // Open File for reading.
  pFile = fopen(pFileName, "rb");

  if ((mchar_fd < 0) || (pFile == NULL)) {
    VND_LOGE("fopen failed for %s", pFileName);
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    return bRetVal;
  }

  // Calculate the size of the file to be downloaded.
  result = fseek(pFile, 0, SEEK_END);
  if (result) {
    VND_LOGE("fseek error: %s (%d)", strerror(errno), errno);
    return bRetVal;
  }

  TotalFileSize = ftell(pFile);
  if (TotalFileSize == -1) {
    VND_LOGE("Invalid Download Size");
    VND_LOGE("Error: %s (%d)", strerror(errno), errno);
    return bRetVal;
  }
  uiTotalFileSize =(uint32) TotalFileSize;

  pFileBuffer = (uint8*)malloc(uiTotalFileSize);

  if (fseek(pFile, 0, SEEK_SET) < 0) {
    VND_LOGE("fseek error: %s (%d)", strerror(errno), errno);
  }
  if (pFileBuffer) {
    ulReadLen = fread(pFileBuffer, 1, uiTotalFileSize, pFile);
    if (ulReadLen != uiTotalFileSize) {
      VND_LOGV("fread error: %s (%d)", strerror(errno), errno);
      return bRetVal;
    }
  }
  ulCurrFileSize = 0;

  // Jump to here in case of protocol resync.
  setjmp(resync);

  while (!bRetVal) {
    // Wait to Receive 0xa5, 0xaa, 0xa6
    if (!fw_upload_WaitForHeaderSignature(TIMEOUT_VAL_MILLISEC)) {
      VND_LOGV("0xa5,0xaa,or 0xa6 is not received in 4s.");
      return bRetVal;
    }

    // Read the 'Length' bytes requested by Helper
    uiLenToSend = fw_upload_WaitFor_Len(pFile);

    if (ucRcvdHeader == HELPER_HEADER) {
      ucHelperOn = TRUE;
      ulOffsettoSend = fw_upload_WaitFor_Offset();
      uiErrCode = fw_upload_WaitFor_ErrCode();
      if (uiErrCode == 0) {
        if (uiLenToSend != 0) {
          fw_upload_SendLenBytesToHelper(pFileBuffer, uiLenToSend,
                                         ulOffsettoSend);
          VND_LOGV("sent %d bytes..", ulOffsettoSend);
        } else  // download complete
        {
          tcflush(mchar_fd, TCIFLUSH);
          fw_upload_SendIntBytes(ulCurrFileSize);
          fw_upload_DelayInMs(20);
          if (fw_upload_GetBufferSize(mchar_fd) == 0) {
            bRetVal = TRUE;
          }
        }
      } else if (uiErrCode > 0) {
        /*delay 20ms to make multiple uiErrCode == 1 has been sent, after 20ms,
         *if get uiErrCode = 1 again, we consider 0x6b is missing.
         */
        fw_upload_DelayInMs(20);
        tcflush(mchar_fd, TCIFLUSH);
        fw_upload_SendIntBytes(ulOffsettoSend);
      }
      VND_LOGV("File downloaded: %8d:%8d\r", ulCurrFileSize, uiTotalFileSize);

      // Ensure any pending write data is completely written
      if (0 == tcdrain(mchar_fd)) {
        VND_LOGV("\t tcdrain succeeded");
      } else {
        VND_LOGV("tcdrain failed. Errno = %s (%d)", strerror(errno), errno);
      }
    }

    if (!ucHelperOn) {
      do {
        uiLenToSend = fw_upload_SendLenBytes(pFileBuffer, uiLenToSend);
      } while (uiLenToSend != 0);
      // If the Length requested is 0, download is complete.
      if (uiLenToSend == 0) {
        bRetVal = TRUE;
        break;
      }
    }
  }
  if (pFileBuffer != NULL) {
    free(pFileBuffer);
    pFileBuffer = NULL;
  }
  return bRetVal;
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
BOOLEAN bt_vnd_mrvl_check_fw_status_v2() {
  BOOLEAN bRetVal = FALSE;

  if (mchar_fd < 0) {
    VND_LOGE("Port is not open or file not found");
    return bRetVal;
  }

  // Wait to Receive 0xa5, 0xaa, 0xa6
  bRetVal = fw_upload_WaitForHeaderSignature(200);

  VND_LOGD("fw_upload_WaitForHeaderSignature return %d", bRetVal);

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
int bt_vnd_mrvl_download_fw_v2(int8* pPortName, int32 iBaudrate,
                               int8* pFileName) {
  uint64 endTime;
  uint64 start;
  uint64 cost;
  uint32 ulResult;
  uint8 ucByte;

  start = fw_upload_GetTime();

  VND_LOGI("Protocol: NXP Proprietary");
  VND_LOGI("FW Loader Version: %s", VERSION);
  VND_LOGD("ComPort : %s", pPortName);
  VND_LOGD("BaudRate: %d", iBaudrate);
  VND_LOGD("Filename: %s", pFileName);

  do {
    ulResult = fw_upload_FW(pFileName);
    if (ulResult) {
      VND_LOGI("Download Complete");
      cost = fw_upload_GetTime() - start;
      VND_LOGI("time:%llu", cost);
      if (ucHelperOn == TRUE) {
        endTime = fw_upload_GetTime() + POLL_AA_TIMEOUT;
        do {
          if (fw_upload_GetBufferSize(mchar_fd) != 0) {
            ucByte = 0xff;
            fw_upload_ComReadChars(mchar_fd, (uint8*)&ucByte, 1);
            if (ucByte == VERSION_HEADER) {
              VND_LOGV("ReDownload");
              uiReDownload = TRUE;
              ulLastOffsetToSend = 0xFFFF;
              memset(ucByteBuffer, 0, sizeof(ucByteBuffer));
            }
            break;
          }
        } while (endTime > fw_upload_GetTime());
      }
      if (uiReDownload == FALSE) {
        endTime = fw_upload_GetTime() + MAX_CTS_TIMEOUT;
        do {
          if (!fw_upload_ComGetCTS(mchar_fd)) {
            VND_LOGV("CTS is low");
            if (pFile) {
              fclose(pFile);
              pFile = NULL;
            }
            goto done;
          }
        } while (endTime > fw_upload_GetTime());
        VND_LOGV("wait CTS low timeout");
        VND_LOGV("Error code is %d", ulResult);
        if (pFile) {
          fclose(pFile);
          pFile = NULL;
        }
        goto done;
      }
    } else {
      VND_LOGE("Download Error");
      return 1;
    }
  } while (uiReDownload);

done:
  return 0;
}
