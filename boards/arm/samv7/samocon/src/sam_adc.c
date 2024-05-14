
#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdint.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "sam_afec.h"
#include "samocon.h"


#if defined(HAVE_AFEC0) || defined(HAVE_AFEC1) 

#define AFEC0_CHANNELS 8

#ifdef HAVE_EXTERNAL_3ADC
#define AFEC1_CHANNELS 11
#else
#define AFEC1_CHANNELS 8
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/*
 * Here is a short overview of channels used on the SaMoCon board:
 * AFEC0: 0, 3, 4, 6, 7, 8, 9, 10
 * AFEC1: 0, 1, 2, 4, 5, 6, 9, 10  
 * 3 external ADCs belonging to AFEC1: 7, 8, 11
 */


/* Put the L channels first! So the PWM trigger for L measurements can
 * be as late as possible, using as much PWM duty bandwidth as possible.
 */

static const uint8_t g_chanlist0[AFEC0_CHANNELS] =
{
  9, 3, 7, 6, 10, 0, 8, 4
};


static const uint8_t g_chanlist1[AFEC1_CHANNELS] =
{
  9, 1, 5, 4, 10, 0, 2, 6
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_init
 *
 * Description:
 *   Initialize and register the ADC drivers.
 *
 ****************************************************************************/

int sam_adc_init(void)
{
  struct adc_dev_s *adc;
  int ret;

  /* Call sam_adc_initialize() to get an instance of the ADC interface */

#ifdef HAVE_AFEC0 
  adc = sam_afec_initialize(0, g_chanlist0, AFEC0_CHANNELS);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC0 interface\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc0" */

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register adc0 failed: %d\n", ret);
      return ret;
    }
#endif
#ifdef HAVE_AFEC1
  adc = sam_afec_initialize(1, g_chanlist1, AFEC1_CHANNELS);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC1 interface\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc1" */

  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register adc1 failed: %d\n", ret);
      return ret;
    }
#endif
  return OK;
}

#endif /* HAVE_AFEC0 || HAVE_AFEC1 */
