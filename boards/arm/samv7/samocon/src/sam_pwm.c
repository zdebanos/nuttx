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

#include <arch/board/board.h>
#include <nuttx/timers/pwm.h>

#include "sam_gpio.h"
#include "sam_pwm.h"
#include "samocon.h"

#if defined(HAVE_PWM0) || defined(HAVE_PWM1)

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
 *  Initialize and register PWM0 or PWM1
 *
 ****************************************************************************/

int sam_pwm_init(void) 
{
  int ret;
  struct pwm_lowerhalf_s *pwm;
#ifdef HAVE_PWM0
  pwm = sam_pwminitialize(0);
  if (pwm == NULL)
    {
      pwmerr("ERROR: PWM0 init!\n");
      return -ENODEV;
    }
  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: PWM0 register failed!\n");
      return ret;
    }
#ifndef CONFIG_SAMV7_PWM0_CH0_COMP
  sam_configgpio(GPIO_PWMC0_L0);
  sam_gpiowrite(GPIO_PWMC0_L0, false);
#endif
#ifndef CONFIG_SAMV7_PWM0_CH1_COMP
  sam_configgpio(GPIO_PWMC0_L1);
  sam_gpiowrite(GPIO_PWMC0_L1, false);
#endif
#ifndef CONFIG_SAMV7_PWM0_CH2_COMP
  sam_configgpio(GPIO_PWMC0_L2);
  sam_gpiowrite(GPIO_PWMC0_L2, false);
#endif
#ifndef CONFIG_SAMV7_PWM0_CH3_COMP
  sam_configgpio(GPIO_PWMC0_L3);
  sam_gpiowrite(GPIO_PWMC0_L3, false);
#endif
#endif /* HAVE_PWM0 */

#ifdef HAVE_PWM1
  pwm = sam_pwminitialize(1);
  if (pwm == NULL)
    {
      pwmerr("ERROR: PWM1 init!\n");
      return -ENODEV;
    }
  ret = pwm_register("/dev/pwm1", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: PWM1 register failed!\n");
      return ret;
    }
#ifndef CONFIG_SAMV7_PWM1_CH0_COMP
  sam_configgpio(GPIO_PWMC1_L0);
  sam_gpiowrite(GPIO_PWMC1_L0, false);
#endif
#ifndef CONFIG_SAMV7_PWM1_CH1_COMP
  sam_configgpio(GPIO_PWMC1_L1);
  sam_gpiowrite(GPIO_PWMC1_L1, false);
#endif
#ifndef CONFIG_SAMV7_PWM1_CH2_COMP
  sam_configgpio(GPIO_PWMC1_L2);
  sam_gpiowrite(GPIO_PWMC1_L2, false);
#endif
#ifndef CONFIG_SAMV7_PWM1_CH3_COMP
  sam_configgpio(GPIO_PWMC1_L3);
  sam_gpiowrite(GPIO_PWMC1_L3, false);
#endif
#endif /* HAVE_PWM1 */
  return 0;
}

#endif /* CONFIG_SAMV7_MCAN */
