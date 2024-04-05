/****************************************************************************
 * arch/arm/src/samv7/sam_qencoder.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qencoder.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/sam_tc.h"
#include "sam_tc.h"
#include "sam_qencoder.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Overall, RAM-based state structure */

struct sam_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  const struct qe_ops_s *ops;  /* Lower half callback structure */

  /* SAMV7 driver-specific fields: */

  uint8_t          tcid;         /* Timer counter ID {0,1,2,3} */
  TC_HANDLE        tch;          /* Handle returned by sam_tc_initialize() */

  bool             inuse;        /* True: The lower-half driver is in-use */

#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  /* qe_index_s IOCTL support */
  uint32_t last_pos;
  uint32_t last_index;
  uint32_t index_cnt;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static struct sam_lowerhalf_s *sam_tc2lower(int tc);

/* Lower-half Quadrature Encoder Driver Methods */

static int sam_setup(struct qe_lowerhalf_s *lower);
static int sam_shutdown(struct qe_lowerhalf_s *lower);
static int sam_position(struct qe_lowerhalf_s *lower, int32_t *pos);
static int sam_reset(struct qe_lowerhalf_s *lower);
static int sam_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                     unsigned long arg);
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
static int sam_qeindex(struct qe_lowerhalf_s *lower, struct qe_index_s *dest);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = sam_setup,
  .shutdown  = sam_shutdown,
  .position  = sam_position,
  .setposmax = NULL,            /* not supported yet */
  .reset     = sam_reset,
  .setindex  = NULL,            /* not supported yet */
  .ioctl     = sam_ioctl,
};

/* Per-timer state structures */

#ifdef CONFIG_SAMV7_TC0_QE
static struct sam_lowerhalf_s g_tc0lower =
{
  .ops      = &g_qecallbacks,
  .tcid     = 0,
  .inuse    = false,
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  .last_pos = 0,
  .last_index = 0,
  .index_cnt = 0
#endif
};
#endif

#ifdef CONFIG_SAMV7_TC1_QE
static struct sam_lowerhalf_s g_tc1lower =
{
  .ops      = &g_qecallbacks,
  .tcid     = 1,
  .inuse    = false,
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  .last_pos = 0,
  .last_index = 0,
  .index_cnt = 0
#endif
};
#endif

#ifdef CONFIG_SAMV7_TC2_QE
static struct sam_lowerhalf_s g_tc2lower =
{
  .ops      = &g_qecallbacks,
  .tcid     = 2,
  .inuse    = false,
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  .last_pos = 0,
  .last_index = 0,
  .index_cnt = 0
#endif
};
#endif

#ifdef CONFIG_SAMV7_TC3_QE
static struct sam_lowerhalf_s g_tc3lower =
{
  .ops      = &g_qecallbacks,
  .tcid     = 3,
  .inuse    = false,
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  .last_pos = 0,
  .last_index = 0,
  .index_cnt = 0
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tc2lower
 *
 * Description:
 *   Map a timer counter number to a device structure
 *
 ****************************************************************************/

static struct sam_lowerhalf_s *sam_tc2lower(int tc)
{
  switch (tc)
    {
#ifdef CONFIG_SAMV7_TC0_QE
    case 0:
      return &g_tc0lower;
#endif
#ifdef CONFIG_SAMV7_TC1_QE
    case 1:
      return &g_tc1lower;
#endif
#ifdef CONFIG_SAMV7_TC2_QE
    case 2:
      return &g_tc2lower;
#endif
#ifdef CONFIG_SAMV7_TC3_QE
    case 3:
      return &g_tc3lower;
#endif
    default:
      return NULL;
    }
}

/****************************************************************************
 * Name: sam_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero.
 *
 ****************************************************************************/

static int sam_setup(struct qe_lowerhalf_s *lower)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;

  /* Start the counter */

  sam_tc_start(priv->tch);

  return OK;
}

/****************************************************************************
 * Name: sam_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware,
 *   and put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int sam_shutdown(struct qe_lowerhalf_s *lower)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;

  sam_tc_stop(priv->tch);

  return OK;
}

#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
static int32_t sam_qe_pos_16to32b(struct qe_lowerhalf_s *lower, uint32_t current_pos)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s*) lower;

  uint32_t new_pos = *(volatile uint32_t*) &priv->last_pos;
  new_pos += (int16_t) (current_pos - new_pos);
  *(volatile uint32_t*) &priv->last_pos = new_pos;

  return (int32_t) new_pos;
}
#endif

/****************************************************************************
 * Name: sam_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int sam_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;
  uint32_t new_pos;
  new_pos = sam_tc_getcounter(priv->tch);

  /* Return the counter value */
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  *pos = sam_qe_pos_16to32b(lower, new_pos);
#else
  *pos = (int32_t) new_pos;
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ****************************************************************************/

static int sam_reset(struct qe_lowerhalf_s *lower)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;

  sninfo("Resetting position to zero\n");
  DEBUGASSERT(lower && priv->inuse);

  sam_tc_stop(priv->tch);

  sam_tc_start(priv->tch);

  return OK;
}

/****************************************************************************
 * Name: sam_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int sam_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  switch (cmd)
    {
      case QEIOC_GETINDEX:
        {
          /* Call the qeindex function */
          sam_qeindex(lower, (struct qe_index_s*) arg);
          return OK;
        }
      default:
        {
          return -ENOTTY;
        }
    }
#else
  return -ENOTTY;
#endif
}


#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
static int sam_qeindex(struct qe_lowerhalf_s *lower, struct qe_index_s *dest)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s*) lower;

  /* Get the current position into a local variable */
  int32_t current_pos;
  sam_position(lower, &current_pos);
  /* Perform the 32bit extension of the timer */
  dest->qenc_pos = current_pos;

  /* Perform the capture logic */
  TC_HANDLE handle = priv->tch;
  uint32_t status = sam_tc_getpending(handle);
  uint32_t current_indx_pos;
  bool captured = false;

  /* Check if something has been captured */
  if (status & 0x20)
    {
      /* The new index pos is in the Capture A register */
      current_indx_pos = sam_tc_getregister(handle, TC_REGA);
      captured = true;
    }
  else if (status & 0x40)
    {
      /* The new index pos is in the Capture B register */
      current_indx_pos = sam_tc_getregister(handle, TC_REGB);
      captured = true;
    }

  if (captured)
    {
      priv->index_cnt++;
      uint32_t new_index = *(volatile uint32_t*) &priv->last_pos;
      new_index += (int16_t) (current_indx_pos - new_index);
      *(volatile uint32_t*) &priv->last_index = new_index;
    }
  dest->indx_pos = priv->last_index;
  dest->indx_cnt = priv->index_cnt;
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be
 *   called from board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tc      - The timer counter number to used.  'tc' must be an element of
 *             {0,1,2,3}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int sam_qeinitialize(const char *devpath, int tc)
{
  struct sam_lowerhalf_s *priv;
  uint32_t mode;
  int ret;

  /* Find the pre-allocated timer state structure corresponding to this
   * timer
   */

  priv = sam_tc2lower(tc);
  if (!priv)
    {
      snerr("ERROR: TC%d support not configured\n", tc);
      return -ENXIO;
    }

  /* Make sure that it is available */

  if (priv->inuse)
    {
      snerr("ERROR: TC%d is in-use\n", tc);
      return -EBUSY;
    }

  /* Allocate the timer/counter and select its mode of operation */

  /* When configuring the timer with no index reset, do not obey
   * the datasheet's QDEC instructions. Do not set the TC_CMR_ABETRG and
   * TC_CMR_ETRGEDG_RISING bits responsible for the counter reset.
   *
   * Due to the internal structure of the Timer/Counter, both Capture
   * registers (A and B) must be used, because of the exclusive access
   * to both Capture registers (refer to section 49-6 in the latest 2023
   * ATSAMV7's datasheet). Only one register could be used when using triggers
   * which we can't use.  
   */
#ifdef CONFIG_SAMV7_QENCODER_ENABLE_GETINDEX
  mode = TC_CMR_TCCLKS_XC0 |
         TC_CMR_CAPTURE |
         TC_CMR_LDRA_RISING |
         TC_CMR_LDRB_RISING |
         TC_CMR_SBSMPLR_ONE;
#else
  mode = TC_CMR_TCCLKS_XC0 |     /* Use XC0 as an external TCCLKS value */
         TC_CMR_ETRGEDG_RISING | /* Select 'Rising edge' as the External Trigger Edge */
         TC_CMR_ABETRG |         /* Select 'TIOAx' as the External Trigger */
         TC_CMR_CAPTURE;         /* Select 'Capture mode' */
#endif

  priv->tch = sam_tc_allocate(tc * SAM_TC_NCHANNELS, mode);
  if (priv->tch == NULL)
    {
      tmrerr("ERROR: Failed to allocate timer channel %d\n",
             tc * SAM_TC_NCHANNELS);
      return -EBUSY;
    }

  /* Define timer block mode */

  mode = TC_BMR_QDEN |      /* Enable Quadrature Decoder */
         TC_BMR_POSEN |     /* Enable Position Measurement on timer channel */
         TC_BMR_EDGPHA |    /* Select Edge Detection Mode */
         TC_BMR_AUTOC |     /* Enable Auto-Correction of Missing Pulses */
#ifdef CONFIG_SAMV7_QENCODER_FILTER
         TC_BMR_MAXFILT(1); /* Define Filtering Capabilities */
#else
         TC_BMR_MAXFILT(0); /* Disable Filtering Capabilities */
#endif

  sam_tc_setblockmode(priv->tch, mode);


  /* Register the upper-half driver */

  ret = qe_register(devpath, (struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      sam_tc_free(priv->tch);
      return ret;
    }

  /* The driver is now in-use */

  priv->inuse = true;
  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
