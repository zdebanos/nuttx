#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "sam_gpio.h"
#include "samocon.h"

int sam_halls_init(void)
{
#ifdef HAVE_HALLA
  sam_configgpio(GPIO_HALLA_IN0);
  sam_configgpio(GPIO_HALLA_IN1);
  sam_configgpio(GPIO_HALLA_IN2);
#endif

#ifdef HAVE_HALLB
  sam_configgpio(GPIO_HALLB_IN0);
  sam_configgpio(GPIO_HALLB_IN1);
  sam_configgpio(GPIO_HALLB_IN2);
#endif
  return OK;
}