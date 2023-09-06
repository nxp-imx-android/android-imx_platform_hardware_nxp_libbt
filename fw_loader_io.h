/******************************************************************************
 *
 *  Copyright 2009-2021, 2023 NXP
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
 *  Filename:      fw_loader_io.h
 *
 *  Description:   Kernel Input/Output function declarations
 *
 ******************************************************************************/

#ifndef FW_LOADER_IO_LINUX_H
#define FW_LOADER_IO_LINUX_H
/*============================== Include Files ===============================*/

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "bt_vendor_nxp.h"
/*================================== Macros ==================================*/

/*================================== Typedefs=================================*/

/*================================ Global Vars================================*/

/*============================ Function Prototypes ===========================*/

extern BOOLEAN fw_upload_lenValid(uint16* uiLenToSend, uint8* ucArray);
extern uint16 fw_upload_GetDataLen(uint8* buf);
extern int32 fw_upload_ComReadChar(int32 mchar_fd);
extern void fw_upload_ComWriteChar(int32 mchar_fd, int8 iChar);
extern void fw_upload_ComWriteChars(int32 mchar_fd, uint8* pChBuffer,
                                    uint32 uiLen);
extern void fw_upload_ComReadChars(int32 mchar_fd, uint8* pChBuffer,
                                   uint32 uiCount);
extern void fw_upload_DelayInMs(uint32 uiMs);
extern int32 fw_upload_ComGetCTS(int32 mchar_fd);
extern uint64 fw_upload_GetTime(void);
extern uint32 fw_upload_GetBufferSize(int32 mchar_fd);
#endif  // FW_LOADER_IO_LINUX_H
