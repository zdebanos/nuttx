/****************************************************************************
 * boards/arm/samv7/samocon/src/sam_bringup.c
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

#include <sys/mount.h>
#include <sys/param.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/signal.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/fb.h>

#include "sam_twihs.h"
#include "samocon.h"


#ifdef HAVE_HSMCI
#  include "board_hsmci.h"
#endif /* HAVE_HSMCI */

#ifdef HAVE_AUTOMOUNTER
#  include "sam_automount.h"
#endif /* HAVE_AUTOMOUNTER */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAMV71XULT_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAMV71XULT_ROMFS_ROMDISK_SECTSIZE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SAMV7_PROGMEM_OTA_PARTITION)
static struct mtd_partition_s g_mtd_partition_table[] =
{
  {
    .offset  = CONFIG_SAMV7_OTA_PRIMARY_SLOT_OFFSET,
    .size    = CONFIG_SAMV7_OTA_SLOT_SIZE,
    .devpath = CONFIG_SAMV7_OTA_PRIMARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_SAMV7_OTA_SECONDARY_SLOT_OFFSET,
    .size    = CONFIG_SAMV7_OTA_SLOT_SIZE,
    .devpath = CONFIG_SAMV7_OTA_SECONDARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_SAMV7_OTA_SCRATCH_OFFSET,
    .size    = CONFIG_SAMV7_OTA_SCRATCH_SIZE,
    .devpath = CONFIG_SAMV7_OTA_SCRATCH_DEVPATH
  }
};

static const size_t g_mtd_partition_table_size =
    nitems(g_mtd_partition_table);
#else
#  define g_mtd_partition_table         NULL
#  define g_mtd_partition_table_size    0
#endif /* CONFIG_SAMV7_PROGMEM_OTA_PARTITION */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void)
{
  int ret;


  /* Initialize all i2c buses. */
#if defined(HAVE_MAIN_I2C) || defined(HAVE_EXTERNAL_I2C)
  ret = sam_i2c_init();
  printf("i2c ret = %d\n", ret);
#endif

#ifdef CONFIG_SAMV7_MCAN
  /* Initialize CAN and register the CAN driver. */

  ret = sam_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_can_setup failed: %d\n", ret);
    }
#endif

#if defined(HAVE_IRCA_FEEDBACK) || defined(HAVE_IRCB_FEEDBACK)
  /* Configure quadrature encoder feedbacks */
  ret = sam_qencs_initialize();
  if (ret < 0)
    {
      printf("QENC init returned %d\n", ret);
    }
#endif

#ifdef HAVE_MACADDR
  /* SAMV71-XULT reads the address from the AT24 eeprom.
   * In our case, sam_emac0_setmac sets a MAC address
   * temporarily hardcoded in the function itself.
   */

  ret = sam_emac0_setmac();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_emac0_setmac() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, SAMV71_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SAMV71_PROCFS_MOUNTPOINT, ret);
    }
#endif


#ifdef HAVE_HSMCI
  /* Initialize the HSMCI0 driver */

  ret = sam_hsmci_initialize(HSMCI0_SLOTNO, HSMCI0_MINOR, GPIO_HSMCI0_CD,
                             IRQ_HSMCI0_CD, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
             HSMCI0_SLOTNO, HSMCI0_MINOR, ret);
    }

#ifdef CONFIG_SAMV7_HSMCI0_MOUNT
  else
    {
      if (sam_cardinserted(HSMCI0_SLOTNO))
        {
          nxsig_usleep(1000 * 1000);

          /* Mount the volume on HSMCI0 */

          ret = nx_mount(CONFIG_SAMV7_HSMCI0_MOUNT_BLKDEV,
                         CONFIG_SAMV7_HSMCI0_MOUNT_MOUNTPOINT,
                         CONFIG_SAMV7_HSMCI0_MOUNT_FSTYPE,
                         0, NULL);

          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                     CONFIG_SAMV7_HSMCI0_MOUNT_MOUNTPOINT, ret);
            }
        }
    }

#endif /* CONFIG_SAMV7_HSMCI0_MOUNT */
#endif /* HAVE_HSMCI */

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif


#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  sam_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = sam_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start the USB monitor: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#if defined(HAVE_AFEC0) || defined(HAVE_AFEC1)
  sam_adc_init();
#endif

#if defined(HAVE_PWM0) || defined(HAVE_PWM1)
  sam_pwm_init();
#endif

#if defined(HAVE_HALLA) || defined(HAVE_HALLB)
  sam_halls_init();
#endif

  UNUSED(ret);
  return OK;
}
