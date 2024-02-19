/****************************************************************************
 * boards/arm/samv7/samocon/src/sam_mcan.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "sam_mcan.h"
#include "samocon.h"

#ifdef CONFIG_SAMV7_MCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize and register MCAN0 or MCAN1
 *
 ****************************************************************************/

int sam_can_setup(void) 
{
#if defined(CONFIG_SAMV7_MCAN0) || defined(CONFIG_SAMV7_MCAN1)
  int ret;
#ifdef CONFIG_SAMV7_MCAN0
  struct can_dev_s *can0;
  can0 = sam_mcan_initialize(0);
  if (can0 == NULL)
    {
      canerr("ERROR: Failed to init MCAN0\n");
      return -ENODEV;
    }
  /* Register this as /dev/can0 */
  ret = can_register("/dev/can0", can0);
  if (ret < 0)
    {
      canerr("ERROR: failed registering /dev/can0\n");
      return ret;
    }
#endif
#ifdef CONFIG_SAMV7_MCAN1
  struct can_dev_s *can1;
  can1 = sam_mcan_initialize(1);
  if (can0 == NULL)
    {
      canerr("ERROR: Failed to init MCAN1\n");
      return -ENODEV;
    }
  /* Register this as /dev/can1 */
  ret = can_register("/dev/can1", can1);
  if (ret < 0)
    {
      canerr("ERROR: failed registering /dev/can1\n");
      return ret;
    }
#endif
  return OK;
#else
  return -ENODEV;
#endif /* if defined(MCAN0) && defined(MCAN1) */
}

#endif /* CONFIG_SAMV7_MCAN */
