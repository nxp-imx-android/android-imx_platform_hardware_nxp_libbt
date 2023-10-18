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
 *  Filename:      fw_loader_uart.h
 *
 *  Description:   Firmware loader version 1 and 3 function declarations
 *
 ******************************************************************************/

#ifndef FW_LOADER_H
#define FW_LOADER_H
/*============================== Include Files ===============================*/
#include "bt_vendor_nxp.h"

/*================================== Typedefs=================================*/

/*================================== Macros ==================================*/

typedef struct {
  uint32 iBaudRate;
  uint32 iUartDivisor;
  uint32 iClkDivisor;
} UART_BAUDRATE;

extern int mchar_fd;
extern uint8_t independent_reset_mode;

/*================================ Global Vars================================*/

/*============================ Function Prototypes ===========================*/

void init_crc8();
BOOLEAN bt_vnd_mrvl_check_fw_status();
int bt_vnd_mrvl_download_fw(int8* pPortName, int32 iBaudRate, int8* pFileName,
                            int32 iSecondBaudRate);
void fw_loader_get_default_fw_name(char fw_name[], uint32 fw_name_size);
#endif  // FW_LOADER_H
