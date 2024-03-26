#include <nuttx/config.h>

#include <stdbool.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/eeprom/i2c_xx24xx.h>

#include "sam_twihs.h"
#include "samocon.h"

#if defined(HAVE_MAIN_I2C) || defined(HAVE_EXTERNAL_I2C)

int sam_i2c_init(void)
{
  int ret;
  struct i2c_master_s *i2cm;

  /* 0 is the external I2C */
#ifdef HAVE_EXTERNAL_I2C
  /* Get an instance of the TWIHS0 interface (EXTERNAL_I2C) */
  i2cm = sam_i2cbus_initialize(0);
  if (!i2cm)
    {
      ferr("ERROR: Failed to initialize TWI0\n");
      return -ENODEV;
    }

  ret = i2c_register(i2cm, 0); 
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C0 driver\n");
      sam_i2cbus_uninitialize(i2cm);
      return -ENODEV;
    }
#endif


  /* 1 is the main I2C */
#ifdef HAVE_MAIN_I2C
  /* Get an instance of the TWIHS1 interface (MAIN_I2C) */
  i2cm = sam_i2cbus_initialize(1);
  if (!i2cm)
    {
      ferr("ERROR: Failed to initialize TWI1\n");
      return -ENODEV;
    }

  ret = i2c_register(i2cm, 1); 
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C1 driver\n");
      sam_i2cbus_uninitialize(i2cm);
      return -ENODEV;
    }
  /* Initialize the EEPROM */
  ret = ee24xx_initialize(i2cm, 0x57, "/dev/eeprom0", EEPROM_24XX64, false);
  if (ret < 0)
  {
    ferr("ERROR: Failed to bind I2C0 to the AT24 EEPROM driver\n");
    return -ENODEV;
  }
#endif

  return OK;
}

#endif // HAVE_MAIN_I2C || HAVE_EXTERNAL_I2C
