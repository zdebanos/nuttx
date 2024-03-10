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

#if defined(HAVE_IRCA_FEEDBACK) || defined(HAVE_IRCB_FEEDBACK)
int sam_qencs_init(void)
{
    int ret;
#ifdef HAVE_IRCA_FEEDBACK
    ret = sam_qeinitialize(IRCA_DEVPATH, IRCA_TC);
    if (ret < 0)
      {
        printf("Failed to register %s\n", IRCA_DEVPATH);
        return ret;
      }
    sam_configgpio(GPIO_IRCA_MARK);
#endif
#ifdef HAVE_IRCB_FEEDBACK
    ret = sam_qeinitialize(IRCB_DEVPATH, IRCB_TC);
    if (ret < 0)
      {
        printf("Failed to register %s\n", IRCB_DEVPATH);
        return ret;
      }
    sam_configgpio(GPIO_IRCB_MARK);
#endif
    return OK;
}
#endif // HAVE_IRCA_FEEDBACK || HAVE_IRCB_FEEDBACK

#endif // CONFIG_SENSORS_QENCODER
