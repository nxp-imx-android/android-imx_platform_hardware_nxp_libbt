/******************************************************************************
 *
 *  Copyright 2022 NXP
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
 *  Filename:      bt_vendor_log.h
 *
 *  Description:   VHAL specific logs definition
 *
 ******************************************************************************/

#ifndef BT_VENDOR_LOG_H
#define BT_VENDOR_LOG_H

/*============================== Include Files ===============================*/

#include <log/log.h>

/*================================ Global Vars================================*/

extern int vhal_trace_level;

/*================================== Typedefs=================================*/

/*================================== Macros ==================================*/
#define BT_TRACE_LEVEL_NONE 0    /* No trace messages to be generated    */
#define BT_TRACE_LEVEL_ERROR 1   /* Error condition trace messages       */
#define BT_TRACE_LEVEL_WARNING 2 /* Warning condition trace messages     */
#define BT_TRACE_LEVEL_INFO 3    /* Generic Info                         */
#define BT_TRACE_LEVEL_DEBUG 4   /* Debug messages for events            */
#define BT_TRACE_LEVEL_VERBOSE 5 /* Full debug messages                  */

#if (BT_TRACE_LEVEL_ERROR <= VHAL_LOG_LEVEL)
#define VND_LOGE(fmt, ...)                                           \
  {                                                                  \
    if (vhal_trace_level >= BT_TRACE_LEVEL_ERROR)                    \
      ALOGE("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
  }
#else
#define VND_LOGE(fmt, ...)
#endif

#if (BT_TRACE_LEVEL_WARNING <= VHAL_LOG_LEVEL)
#define VND_LOGW(fmt, ...)                                           \
  {                                                                  \
    if (vhal_trace_level >= BT_TRACE_LEVEL_WARNING)                  \
      ALOGW("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
  }
#else
#define VND_LOGW(fmt, ...)
#endif

#if (BT_TRACE_LEVEL_INFO <= VHAL_LOG_LEVEL)
#define VND_LOGI(fmt, ...)                                           \
  {                                                                  \
    if (vhal_trace_level >= BT_TRACE_LEVEL_INFO) {                   \
      ALOGI("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }                                                                \
  }
#else
#define VND_LOGI(fmt, ...)
#endif

#if (BT_TRACE_LEVEL_DEBUG <= VHAL_LOG_LEVEL)
#define VND_LOGD(fmt, ...)                                           \
  {                                                                  \
    if (vhal_trace_level >= BT_TRACE_LEVEL_DEBUG) {                  \
      ALOGD("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }                                                                \
  }
#else
#define VND_LOGD(fmt, ...)
#endif

#if (BT_TRACE_LEVEL_VERBOSE <= VHAL_LOG_LEVEL)
#define VND_LOGV(fmt, ...)                                           \
  {                                                                  \
    if (vhal_trace_level >= BT_TRACE_LEVEL_VERBOSE) {                \
      ALOGD("%s(L%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }                                                                \
  }
#else
#define VND_LOGV(fmt, ...)
#endif

#endif  // BT_VENDOR_LOG_H
