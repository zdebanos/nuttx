/****************************************************************************
 * boards/arm/samv7/samocon/src/sam_gpio.c
 *
 * Inspired by: TODO
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
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "sam_gpio.h"

#if defined(CONFIG_DEV_GPIO)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct samgpint_dev_s
{
  struct samgpio_dev_s samgpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);
#endif

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);
#endif
#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct gpio_name_s
{
  char *name;
};

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};
#endif

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};
#endif

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};
#endif

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const struct gpio_name_s g_gpioinnames[BOARD_NGPIOIN] =
{
  {"halla_in0"},
  {"halla_in1"},
  {"halla_in2"},
  {"hallb_in0"},
  {"hallb_in1"},
  {"hallb_in2"},
  {"irca_mark"},
  {"ircb_mark"}
};

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_HALLA_IN0,
  GPIO_HALLA_IN1,
  GPIO_HALLA_IN2,
  GPIO_HALLB_IN0,
  GPIO_HALLB_IN1,
  GPIO_HALLB_IN2,
  GPIO_IRCA_MARK,
  GPIO_IRCB_MARK
};

static struct samgpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0
/* This array maps the GPIO pins used as OUTPUT */

static const struct gpio_name_s g_gpiooutnames[BOARD_NGPIOOUT] =
{
  {"pwma_inh0"},
  {"pwma_inh1"},
  {"pwma_inh2"},
  {"pwma_inh3"},
  {"pwmb_inh0"},
  {"pwmb_inh1"},
  {"pwmb_inh2"},
  {"pwmb_inh3"},
};

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_PWMA_L0,
  GPIO_PWMA_L1,
  GPIO_PWMA_H2,
  GPIO_PWMA_L3,
  GPIO_PWMB_L0,
  GPIO_PWMB_L1,
  GPIO_PWMB_L2,
  GPIO_PWMB_L3
};

static struct samgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

//#if BOARD_NGPIOINT > 0
///* This array maps the GPIO pins used as INTERRUPT INPUTS */
//
//static const struct gpio_name_s g_gpiointnames[BOARD_NGPIOOUT] =
//{
//  {"keypad_int"},
//};
//
//static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
//{
//  GPIO_PCF8575_INT,
//};
//
//static const uint32_t g_irqintinputs[BOARD_NGPIOINT] =
//{
//  IRQ_PCF8575_INT,
//};
//
//static struct samgpint_dev_s g_gpint[BOARD_NGPIOINT];
//#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct samgpio_dev_s *samgpio =
                        (FAR struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = sam_gpioread(g_gpioinputs[samgpio->id]);
  return OK;
}
#endif

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct samgpio_dev_s *samgpio =
                        (FAR struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = sam_gpioread(g_gpiooutputs[samgpio->id]);
  return OK;
}

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct samgpio_dev_s *samgpio =
                             (FAR struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  sam_gpiowrite(g_gpiooutputs[samgpio->id], value);
  return OK;
}
#endif

#if BOARD_NGPIOINT > 0
static int gpint_interrupt(int irq, void *context, void *arg)
{
  struct samgpint_dev_s *samgpint =
                              (struct samgpint_dev_s *)arg;

  DEBUGASSERT(samgpint != NULL && samgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", samgpint->callback);

  samgpint->callback(&samgpint->samgpio.gpio,
                         samgpint->samgpio.id);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpint_dev_s *samgpint =
                              (struct samgpint_dev_s *)dev;

  DEBUGASSERT(samgpint != NULL && value != NULL);
  DEBUGASSERT(samgpint->samgpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = sam_gpioread(g_gpiointinputs[samgpint->samgpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct samgpint_dev_s *samgpint =
                             (struct samgpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  gpioinfo("Attach %p\n", callback);
  samgpint->callback = callback;

  irq_attach(g_irqintinputs[samgpint->samgpio.id], gpint_interrupt,
             samgpint);
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct samgpint_dev_s *samgpint =
                              (struct samgpint_dev_s *)dev;

  if (enable)
    {
      if (samgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

           sam_gpioirqenable(g_irqintinputs[samgpint->samgpio.id]);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      sam_gpioirqdisable(g_irqintinputs[samgpint->samgpio.id]);
      irq_detach(g_irqintinputs[samgpint->samgpio.id]);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpio_init
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int sam_gpio_init(void)
{
#if (BOARD_NGPIOIN > 0) || (BOARD_NGPIOOUT > 0) || (BOARD_NGPIOINT > 0)
  int i;
  int pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register_byname(&g_gpin[i].gpio,
                               g_gpioinnames[i].name);

      /* Configure the pin that will be used as input */

      sam_configgpio(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register_byname(&g_gpout[i].gpio,
                               g_gpiooutnames[i].name);

      /* Configure the pin that will be used as output */

      sam_gpiowrite(g_gpiooutputs[i], 0);
      sam_configgpio(g_gpiooutputs[i]);

      pincount++;
    }

#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].samgpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].samgpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].samgpio.id              = i;
      gpio_pin_register_byname(&g_gpint[i].samgpio.gpio,
                               g_gpiointnames[i].name);

      /* Configure the pin that will be used as interrupt input */

      sam_configgpio(g_gpiointinputs[i]);
      sam_gpioirq(g_gpiointinputs[i]);

      pincount++;
    }
#endif

#endif /* (BOARD_NGPIOIN > 0) || (BOARD_NGPIOOUT > 0) || (BOARD_NGPIOINT > 0) */
  return 0;
}
#endif /* CONFIG_DEV_GPIO */
