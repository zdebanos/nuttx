/****************************************************************************
 * arch/arm/src/str71x/str71x_can.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_CAN_H
#define __ARCH_ARM_SRC_STR71X_STR71X_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers ****************************************************************/

#define STR71X_CAN_CR           (STR71X_CAN_BASE + 0x0000) /* 16-bits wide */
#define STR71X_CAN_SR           (STR71X_CAN_BASE + 0x0004) /* 16-bits wide */
#define STR71X_CAN_ERR          (STR71X_CAN_BASE + 0x0008) /* 16-bits wide */
#define STR71X_CAN_BTR          (STR71X_CAN_BASE + 0x000c) /* 16-bits wide */
#define STR71X_CAN_IDR          (STR71X_CAN_BASE + 0x0010) /* 16-bits wide */
#define STR71X_CAN_TESTR        (STR71X_CAN_BASE + 0x0014) /* 16-bits wide */
#define STR71X_CAN_BRPR         (STR71X_CAN_BASE + 0x0018) /* 16-bits wide */

#define STR71X_CAN_IF1BASE      (STR71X_CAN_BASE + 0x0020)
#define STR71X_CAN_IF2BASE      (STR71X_CAN_BASE + 0x0080)

#define STR71X_CAN_CRR_OFFSET   (0x0000) /* 16-bits wide */
#define STR71X_CAN_CMR_OFFSET   (0x0004) /* 16-bits wide */
#define STR71X_CAN_M1R_OFFSET   (0x0008) /* 16-bits wide */
#define STR71X_CAN_M2R_OFFSET   (0x000c) /* 16-bits wide */
#define STR71X_CAN_A1R_OFFSET   (0x0010) /* 16-bits wide */
#define STR71X_CAN_A2R_OFFSET   (0x0014) /* 16-bits wide */
#define STR71X_CAN_MCR_OFFSET   (0x0018) /* 16-bits wide */
#define STR71X_CAN_DA1R_OFFSET  (0x001c) /* 16-bits wide */
#define STR71X_CAN_DA2R_OFFSET  (0x0020) /* 16-bits wide */
#define STR71X_CAN_DB1R_OFFSET  (0x0024) /* 16-bits wide */
#define STR71X_CAN_DB2R_OFFSET  (0x0028) /* 16-bits wide */

#define STR71X_CAN_CRR(b)       ((b) + STR71X_CAN_CRR_OFFSET)
#define STR71X_CAN_CMR(b)       ((b) + STR71X_CAN_CMR_OFFSET)
#define STR71X_CAN_M1R(b)       ((b) + STR71X_CAN_M1R_OFFSET)
#define STR71X_CAN_M2R(b)       ((b) + STR71X_CAN_M2R_OFFSET)
#define STR71X_CAN_A1R(b)       ((b) + STR71X_CAN_A1R_OFFSET)
#define STR71X_CAN_A2R(b)       ((b) + STR71X_CAN_A2R_OFFSET)
#define STR71X_CAN_MCR(b)       ((b) + STR71X_CAN_MCR_OFFSET)
#define STR71X_CAN_DA1R(b)      ((b) + STR71X_CAN_DA1R_OFFSET)
#define STR71X_CAN_DA2R(b)      ((b) + STR71X_CAN_DA2R_OFFSET)
#define STR71X_CAN_DB1R(b)      ((b) + STR71X_CAN_DB1R_OFFSET)
#define STR71X_CAN_DB2R(b)      ((b) + STR71X_CAN_DB2R_OFFSET)

#define STR71X_CAN_IF1CRR       (STR71X_CAN_IF1BASE + STR71X_CAN_CRR_OFFSET)
#define STR71X_CAN_IF1CMR       (STR71X_CAN_IF1BASE + STR71X_CAN_CMR_OFFSET)
#define STR71X_CAN_IF1M1R       (STR71X_CAN_IF1BASE + STR71X_CAN_M1R_OFFSET)
#define STR71X_CAN_IF1M2R       (STR71X_CAN_IF1BASE + STR71X_CAN_M2R_OFFSET)
#define STR71X_CAN_IF1A1R       (STR71X_CAN_IF1BASE + STR71X_CAN_A1R_OFFSET)
#define STR71X_CAN_IF1A2R       (STR71X_CAN_IF1BASE + STR71X_CAN_A2R_OFFSET)
#define STR71X_CAN_IF1MCR       (STR71X_CAN_IF1BASE + STR71X_CAN_MCR_OFFSET)
#define STR71X_CAN_IF1DA1R      (STR71X_CAN_IF1BASE + STR71X_CAN_DA1R_OFFSET)
#define STR71X_CAN_IF1DA2R      (STR71X_CAN_IF1BASE + STR71X_CAN_DA2R_OFFSET)
#define STR71X_CAN_IF1DB1R      (STR71X_CAN_IF1BASE + STR71X_CAN_DB1R_OFFSET)
#define STR71X_CAN_IF1DB2R      (STR71X_CAN_IF1BASE + STR71X_CAN_DB2R_OFFSET)

#define STR71X_CAN_IF2CRR       (STR71X_CAN_IF2BASE + STR71X_CAN_CRR_OFFSET)
#define STR71X_CAN_IF2CMR       (STR71X_CAN_IF2BASE + STR71X_CAN_CMR_OFFSET)
#define STR71X_CAN_IF2M1R       (STR71X_CAN_IF2BASE + STR71X_CAN_M1R_OFFSET)
#define STR71X_CAN_IF2M2R       (STR71X_CAN_IF2BASE + STR71X_CAN_M2R_OFFSET)
#define STR71X_CAN_IF2A1R       (STR71X_CAN_IF2BASE + STR71X_CAN_A1R_OFFSET)
#define STR71X_CAN_IF2A2R       (STR71X_CAN_IF2BASE + STR71X_CAN_A2R_OFFSET)
#define STR71X_CAN_IF2MCR       (STR71X_CAN_IF2BASE + STR71X_CAN_MCR_OFFSET)
#define STR71X_CAN_IF2DA1R      (STR71X_CAN_IF2BASE + STR71X_CAN_DA1R_OFFSET)
#define STR71X_CAN_IF2DA2R      (STR71X_CAN_IF2BASE + STR71X_CAN_DA2R_OFFSET)
#define STR71X_CAN_IF2DB1R      (STR71X_CAN_IF2BASE + STR71X_CAN_DB1R_OFFSET)
#define STR71X_CAN_IF2DB2R      (STR71X_CAN_IF2BASE + STR71X_CAN_DB2R_OFFSET)

#define STR71X_CAN_TR1R         (STR71X_CAN_BASE + 0x0100) /* 16-bits wide */
#define STR71X_CAN_TR2R         (STR71X_CAN_BASE + 0x0104) /* 16-bits wide */
#define STR71X_CAN_ND1R         (STR71X_CAN_BASE + 0x0120) /* 16-bits wide */
#define STR71X_CAN_ND2R         (STR71X_CAN_BASE + 0x0124) /* 16-bits wide */
#define STR71X_CAN_IP1R         (STR71X_CAN_BASE + 0x0140) /* 16-bits wide */
#define STR71X_CAN_IP2R         (STR71X_CAN_BASE + 0x0144) /* 16-bits wide */
#define STR71X_CAN_MV1R         (STR71X_CAN_BASE + 0x0160) /* 16-bits wide */
#define STR71X_CAN_MV2R         (STR71X_CAN_BASE + 0x0164) /* 16-bits wide */

/* Register bit settings ****************************************************/

/* Control register */

#define STR41X_CANCR_INIT       (0x0001)
#define STR41X_CANCR_IE         (0x0002)
#define STR41X_CANCR_SIE        (0x0004)
#define STR41X_CANCR_EIE        (0x0008)
#define STR41X_CANCR_DAR        (0x0020)
#define STR41X_CANCR_CCE        (0x0040)
#define STR41X_CANCR_TEST       (0x0080)

/* Status register */

#define STR41X_CANSR_LEC        (0x0007)
#define STR41X_CANSR_TXOK       (0x0008)
#define STR41X_CANSR_RXOK       (0x0010)
#define STR41X_CANSR_EPASS      (0x0020)
#define STR41X_CANSR_EWARN      (0x0040)
#define STR41X_CANSR_BOFF       (0x0080)

/* Test register */

#define STR41X_CANTESTR_BASIC   (0x0004)
#define STR41X_CANTESTR_SILENT  (0x0008)
#define STR41X_CANTESTR_LBACK   (0x0010)
#define STR41X_CANTESTR_TX0     (0x0020)
#define STR41X_CANTESTR_TX1     (0x0040)
#define STR41X_CANTESTR_RX      (0x0080)

/* IFn / Command Request register */

#define STR41X_CANCRR_BUSY      (0x8000)

/* IFn / Command Mask register */

#define STR41X_CANCMR_DATAB     (0x0001)
#define STR41X_CANCMR_DATAA     (0x0002)
#define STR41X_CANCMR_TXRQST    (0x0004)
#define STR41X_CANCMR_CLRINTPND (0x0008)
#define STR41X_CANCMR_CONTROL   (0x0010)
#define STR41X_CANCMR_ARB       (0x0020)
#define STR41X_CANCMR_MASK      (0x0040)
#define STR41X_CANCMR_WRRD      (0x0080)

/* IFn / Mask 2 register */

#define STR41X_CANM2R_MXTD      (0x8000)
#define STR41X_CANM2R_MDIR      (0x4000)

/* IFn / Arbitration 2 register */

#define STR41X_CANA2R_DIR       (0x2000)
#define STR41X_CANA2R_XTD       (0x4000)
#define STR41X_CANA2R_MSGVAL    (0x8000)

/* IFn / Message Control register */

#define STR41X_CANMCR_EOB       (0x0080)
#define STR41X_CANMCR_TXRQST    (0x0100)
#define STR41X_CANMCR_RMTEN     (0x0200)
#define STR41X_CANMCR_RXIE      (0x0400)
#define STR41X_CANMCR_TXIE      (0x0800)
#define STR41X_CANMCR_UMASK     (0x1000)
#define STR41X_CANMCR_INTPND    (0x2000)
#define STR41X_CANMCR_MSGLST    (0x4000)
#define STR41X_CANMCR_NEWDAT    (0x8000)

/* Message ID limits */

#define STR41X_CAN_LASTSTDID    ((1 << 11) - 1)
#define STR41X_CAN_LASTEXTID    ((1 << 29) - 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_CAN_H */
