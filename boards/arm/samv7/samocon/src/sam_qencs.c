#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>

#include "sam_qencoder.h"
#include "sam_gpio.h"
#include "samocon.h"

#ifdef CONFIG_SENSORS_QENCODER

#ifdef HAVE_QENC_FEEDBACK
int sam_qencs_initialize(void)
{
    int ret;
    ret = sam_qeinitialize(IRCA_DEVPATH, IRCA_TC);
    if (ret < 0)
      {
        //printf("Failed to register %s\n", IRCA_DEVPATH);
        return ret;
      }
    ret = sam_qeinitialize(IRCB_DEVPATH, IRCB_TC);
    if (ret < 0)
      {
        //printf("Failed to register %s\n", IRCB_DEVPATH);
        return ret;
      }
    sam_configgpio(GPIO_IRCA_MARK);
    sam_configgpio(GPIO_IRCB_MARK);
    return OK;
}
#endif

#endif
