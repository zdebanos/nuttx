#include <nuttx/config.h>

#include <stdbool.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/ioctl.h>

#include "sam_twihs.h"
#include "samocon.h"

#if defined(HAVE_MAIN_I2C) || defined(HAVE_EXTERNAL_I2C)

/****************************************************************************
 * Name: sam_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

static void sam_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = sam_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          sam_i2cbus_uninitialize(i2c);
        }
    }
}

int sam_i2c_init(void)
{
  int ret;
  /* First of all, register all defined I2Cs as /dev devices */
#ifdef HAVE_MAIN_I2C
  sam_i2c_register(1);
#endif
#ifdef HAVE_EXTERNAL_I2C
  sam_i2c_register(0);
#endif

  /* Now, if HAVE_24XXXX is defined, initialize it on the main i2c bus */
#ifdef HAVE_24XXXX
  struct i2c_master_s *i2cm;
  struct mtd_dev_s *at24;
#endif 

  /* Credits to samv7-xult's sam_at24config */
  /* Get an instance of the TWIHS1 interface (MAIN_I2C) */
  i2cm = sam_i2cbus_initialize(1);
  if (!i2cm)
    {
      ferr("ERROR: Failed to initialize TWI1\n");
      return -ENODEV;
    }

  /* Initialize the AT24 driver */

  at24 = at24c_initialize(i2cm);
  if (!at24)
    {
      ferr("ERROR: Failed to initialize the AT24 driver\n");
      sam_i2cbus_uninitialize(i2cm);
      return -ENODEV;
    }

  /* Make sure that the AT24 is in normal memory access mode */

  ret = at24->ioctl(at24, MTDIOC_EXTENDED, 0);
  if (ret < 0)
    {
      ferr("ERROR: AT24 ioctl(MTDIOC_EXTENDED) failed: %d\n", ret);
    }

  /* Bind the instance of an MTD device to the /dev/config device. */

  ret = mtdconfig_register(at24);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind AT24 driver to the MTD config device\n");
      sam_i2cbus_uninitialize(i2cm);
    }

  return ret;
}

#endif // HAVE_MAIN_I2C || HAVE_EXTERNAL_I2C