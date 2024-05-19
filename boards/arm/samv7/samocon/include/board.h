/****************************************************************************
 * boards/arm/samv7/samocon/include/board.h
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

#ifndef __BOARDS_ARM_SAMV7_SAMV71_XULT_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMV7_SAMV71_XULT_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAMV71Q device is running out of the Master
 * Clock using the Fast RC Oscillator running at 4 MHz.
 *
 *   MAINOSC:  Frequency = 12MHz (crystal)
 *
 * 300MHz Settings:
 *   PLLA: PLL Divider = 1, Multiplier = 20 to generate PLLACK = 240MHz
 *   Master Clock (MCK): Source = PLLACK,
 *                       Prescalar = 1 to generate MCK = 120MHz
 *   CPU clock: 120MHz
 *
 * There are two on-board crystals:
 */

#undef BOARD_HAVE_SLOWXTAL         0          /* Slow crystal is populated */
#define BOARD_SLOWCLK_FREQUENCY    (32768)    /* 32.768 kHz slow crystal oscillator */
#define BOARD_MAINOSC_FREQUENCY    (12000000) /* 12 MHz main oscillator */

/* Main oscillator register settings.
 *
 * The main oscillator could be either the embedded 4/8/12 MHz fast RC
 * oscillators or an external 3-20 MHz crystal or ceramic resonator.
 * The external clock source is selected by default in sam_clockconfig.c.
 * Here we need to specify the main oscillator start-up time.
 *
 * REVISIT... this is old information:
 * The start up time should be should be:
 *
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */
#define BOARD_CKGR_MOR_MOSCXTENBY  (PMC_CKGR_MOR_MOSCXTEN)             /* Crystal Oscillator Enable */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multiplier = 25
 *
 * Yields:
 *
 *   PLLACK = 25 * 12MHz / 1 = 300MHz
 */

#define BOARD_CKGR_PLLAR_STMODE    PMC_CKGR_PLLAR_STMODE_FAST
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_MUL       PMC_CKGR_PLLAR_MUL(24)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  BOARD_PMC_MCKR_CSS - The source of main clock input.  This may be one of:
 *
 *    PMC_MCKR_CSS_SLOW   Slow Clock
 *    PMC_MCKR_CSS_MAIN   Main Clock
 *    PMC_MCKR_CSS_PLLA   PLLA Clock
 *    PMC_MCKR_CSS_UPLL   Divided UPLL Clock
 *
 *  BOARD_PMC_MCKR_PRES - Source clock pre-scaler.  May be one of:
 *
 *    PMC_MCKR_PRES_DIV1  Selected clock
 *    PMC_MCKR_PRES_DIV2  Selected clock divided by 2
 *    PMC_MCKR_PRES_DIV4  Selected clock divided by 4
 *    PMC_MCKR_PRES_DIV8  Selected clock divided by 8
 *    PMC_MCKR_PRES_DIV16 Selected clock divided by 16
 *    PMC_MCKR_PRES_DIV32 Selected clock divided by 32
 *    PMC_MCKR_PRES_DIV64 Selected clock divided by 64
 *    PMC_MCKR_PRES_DIV3  Selected clock divided by 3
 *
 *  The prescaler determines (1) the CPU clock and (2) the input into the
 *  second divider that then generates the Master Clock (MCK).  MCK is the
 *  source clock of the peripheral clocks.
 *
 *  BOARD_PMC_MCKR_MDIV - MCK divider.  May be one of:
 *
 *    PMC_MCKR_MDIV_DIV1  Master Clock = Prescaler Output Clock / 1
 *    PMC_MCKR_MDIV_DIV2  Master Clock = Prescaler Output Clock / 2
 *    PMC_MCKR_MDIV_DIV4  Master Clock = Prescaler Output Clock / 4
 *    PMC_MCKR_MDIV_DIV3  Master Clock = Prescaler Output Clock / 3
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA   /* Source = PLLA */
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1  /* Prescaler = /1 */
#define BOARD_PMC_MCKR_MDIV        PMC_MCKR_MDIV_DIV2  /* MCK divider = /2 */

/* USB clocking */

#define BOARD_PMC_MCKR_UPLLDIV2    0           /* UPLL clock not divided by 2 */

/* Resulting frequencies */

#define BOARD_PLLA_FREQUENCY       (300000000) /* PLLACK:  25 * 12Mhz / 1 */
#define BOARD_CPU_FREQUENCY        (300000000) /* CPU:     PLLACK / 1 */
#define BOARD_MCK_FREQUENCY        (150000000) /* MCK:     PLLACK / 1 / 2 */
#undef  BOARD_UPLL_FREQUENCY                   /* To be provided */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *   MCI_SPEED = MCK / (2*CLKDIV + CLOCKODD + 2)
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 150MHz,
 * CLKDIV = 186,
 * MCI_SPEED = 150MHz / (2*186 + 1 + 2) = 400 KHz
 */

#define HSMCI_INIT_CLKDIV          ((186 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 150MHz,
 * CLKDIV = 3 w/CLOCKODD,
 * MCI_SPEED = 150MHz /(2*3 + 0 + 2) = 18.75 MHz
 */

#define HSMCI_MMCXFR_CLKDIV        (2 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 150MHz,
 * CLKDIV = 2,
 * MCI_SPEED = 150MHz /(2*2 + 0 + 2) = 25 MHz
 */

#define HSMCI_SDXFR_CLKDIV         (2 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states.
 *
 * Wait states Max frequency at 105 centigrade (STH conditions)
 *
 *           VDDIO
 *      1.62V     2.7V
 * --- -------  -------
 *  0   26 MHz   30 MHz
 *  1   52 MHz   62 MHz
 *  2   78 MHz   93 MHz
 *  3  104 MHz  124 MHz
 *  4  131 MHz  150 MHz
 *  5  150 MHz  --- MHz
 *
 * Given: VDDIO=3.3V, VDDCORE=1.2V, MCK=150MHz
 */

#define BOARD_FWS                  4


/* PIO Disambiguation *******************************************************/

/* Serial Communication
 *
 * The SaMoCon board includes two serial drivers, one of them being
 * UART3 and the second one being USART2.
 * The UART3 is used as a basic UART for serial console communication
 * while USART2 has RS232 and RS485 converters.
 */

/* TTL Console (UART3).
 *
 *    ------ ------ --------
 *    Pin on SAMV71 SAMV71
 *    conn.  PIO    Function
 *    ------ ------ --------
 *     2     PD31   UTXD3
 *     4     PD28   URXD3
 *    ------ ------ --------
 *    
 *    There are alternative pin selections only for UART3 TXD.
 */


/* UART3_RXD doesn't have to be defined since there's only one option */

#define GPIO_UART3_TXD  GPIO_UART3_TXD_2

/* RS232 and RS485
 *
 *    ------ ------ --------
 *    Pin on SAMV71 SAMV71
 *    conn.  PIO    Function
 *    ------ ------ --------
 *     1     PD16   TXD2
 *     2     PD15   RXD2
 *     3     PD18   RTS2
 *     4     PD19   CTS2
 *    ------ ------ --------
 *
 * No need to define anything. All USART2 signals already defined.
 */

/* CAN drivers
 *
 * The SaMoCon board includes two CAN peripherals for redundancy:
 * CAN0 and CAN1. The CAN transceiver is MCP2562 with RX and TX
 * being galvanically isolated.
 * 
 *   -------- ------
 *   SAMV71   SAMV71
 *   Function PIO
 *   -------- ------
 *   CANRX0   PB3  
 *   CANTX0   PB2 
 *   -------- ------
 *   CANRX1   PC12
 *   CANTX1   PC14
 *  
 * No need to redefine CAN0.
 */

 #define GPIO_MCAN1_RX GPIO_MCAN1_RX_2
 #define GPIO_MCAN1_TX GPIO_MCAN1_TX_2

/* PWMs */
/*
 * PWM0 and PWM1 peripherals are used. H and L complementary channels
 * are routed, expect for PWM1_CH2, where only L channel is routed.
 * It should be possible to make the L channel behave the same as the
 * H channel. When using the default IFX007 power stage board, only H
 * channels are used and the L channels are configured as GPIOs.
 * Fault inputs are used too.
 *
 *   --------- --------- ------
 *   SaMoCon   SAMV71    SAMV71       
 *   Pin       Function  Pin
 *   --------- --------- ------  
 *   PWMA_H0   PWMC1_H0  PA12
 *   PWMA_H1   PWMC1_H1  PA14
 *   PWMA_H2   GPIO      PA15
 *   PWMA_H3   PWMC1_H3  PA8
 *   PWMA_L0   PWMC1_L0  PA11
 *   PWMA_L1   PWMC1_L1  PA13
 *   PWMA_L2   PWMC1_L2  PA23
 *   PWMA_L3   PWMC1_L3  PA5
 *   PWMA_F    PWMC1_F0  PA21
 *   PWMB_H0   PWMC0_H0  PD11
 *   PWMB_H1   PWMC0_H1  PA24
 *   PWMB_H2   PWMC0_H2  PC19
 *   PWMB_H3   PWMC0_H3  PD23
 *   PWMB_L0   PWMC0_L0  PD24
 *   PWMB_L1   PWMC0_L1  PC1
 *   PWMB_L2   PWMC0_L2  PC20
 *   PWMB_L3   PWMC0_L3  PC22
 *   PWMB_F    PWMC0_F0  PA9
 */

#define GPIO_PWMC0_H0 GPIO_PWMC0_H0_6
#define GPIO_PWMC0_H1 GPIO_PWMC0_H1_5
#define GPIO_PWMC0_H2 GPIO_PWMC0_H2_5
#define GPIO_PWMC0_H3 GPIO_PWMC0_H3_1

#ifdef CONFIG_SAMV7_PWM0_CH0_COMP
#define GPIO_PWMC0_L0 GPIO_PWMC0_L0_2
#else
#define GPIO_PWMC0_L0 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | GPIO_PIN24)
#endif

#ifdef CONFIG_SAMV7_PWM0_CH1_COMP
#define GPIO_PWMC0_L1 GPIO_PWMC0_L1_4
#else
#define GPIO_PWMC0_L1 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOC | GPIO_PIN1)
#endif

#ifdef CONFIG_SAMV7_PWM0_CH2_COMP
#define GPIO_PWMC0_L2 GPIO_PWMC0_L2_5
#else
#define GPIO_PWMC0_L2 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOC | GPIO_PIN20)
#endif

#ifdef CONFIG_SAMV7_PWM0_CH3_COMP
#define GPIO_PWMC0_L3 GPIO_PWMC0_L3_3
#else
#define GPIO_PWMC0_L3 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOC | GPIO_PIN22)
#endif

#define GPIO_PWMC1_H0 GPIO_PWMC1_H0_2
#define GPIO_PWMC1_H1 GPIO_PWMC1_H1_2
#define GPIO_PWMC1_H2 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | GPIO_PIN15)
#define GPIO_PWMC1_H3 GPIO_PWMC1_H3_1

#ifdef CONFIG_SAMV7_PWM1_CH0_COMP
#define GPIO_PWMC1_L0 GPIO_PWMC1_L0_2
#else
#define GPIO_PWMC1_L0 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | GPIO_PIN11)
#endif

#ifdef CONFIG_SAMV7_PWM1_CH1_COMP
#define GPIO_PWMC1_L1 GPIO_PWMC1_L1_2
#else
#define GPIO_PWMC1_L1 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | GPIO_PIN13)
#endif

#define GPIO_PWMC1_L2 GPIO_PWMC1_L2_2

#ifdef CONFIG_SAMV7_PWM1_CH3_COMP
#define GPIO_PWMC1_L3 
#else
#define GPIO_PWMC1_L3 \
        (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | GPIO_PIN5)
#endif

#define GPIO_PWMA_H0 GPIO_PWMC1_H0
#define GPIO_PWMA_H1 GPIO_PWMC1_H1
#define GPIO_PWMA_H2 GPIO_PWMC1_H2
#define GPIO_PWMA_H3 GPIO_PWMC1_H3
#define GPIO_PWMA_L0 GPIO_PWMC1_L0
#define GPIO_PWMA_L1 GPIO_PWMC1_L1
#define GPIO_PWMA_L2 GPIO_PWMC1_L2
#define GPIO_PWMA_L3 GPIO_PWMC1_L3

#define GPIO_PWMB_H0 GPIO_PWMC0_H0
#define GPIO_PWMB_H1 GPIO_PWMC0_H1
#define GPIO_PWMB_H2 GPIO_PWMC0_H2
#define GPIO_PWMB_H3 GPIO_PWMC0_H3
#define GPIO_PWMB_L0 GPIO_PWMC0_L0
#define GPIO_PWMB_L1 GPIO_PWMC0_L1
#define GPIO_PWMB_L2 GPIO_PWMC0_L2
#define GPIO_PWMB_L3 GPIO_PWMC0_L3

/* GPIO Hall Sensors Inputs
 *
 * The SaMoCon board includes two inputs for two Hall sensors trios -
 * HALL0 and HALL1.
 * The outputs of the Hall sensors can be used in motor feedback control
 * as the position of the motor's shaft. GPIO inputs are used for this.
 *
 *   --------- ------  
 *   SaMoCon   SAMV71             
 *   Pin       Pin    
 *   --------- ------  
 *   HALLA_IN0 PC17   
 *   HALLA_IN1 PC11   
 *   HALLA_IN2 PC10   
 *   HALLB_IN0 PD13         
 *   HALLB_IN1 PD14           
 *   HALLB_IN2 PD17   
 */

#define HALL0_PORT_TYPE GPIO_PORT_PIOC
#define HALL1_PORT_TYPE GPIO_PORT_PIOD

#define GPIO_HALLA_IN0 (GPIO_INPUT | GPIO_PORT_PIOD | GPIO_PIN13)
#define GPIO_HALLA_IN1 (GPIO_INPUT | GPIO_PORT_PIOD | GPIO_PIN14)
#define GPIO_HALLA_IN2 (GPIO_INPUT | GPIO_PORT_PIOD | GPIO_PIN17)
#define GPIO_HALLB_IN0 (GPIO_INPUT | GPIO_PORT_PIOC | GPIO_PIN17)
#define GPIO_HALLB_IN1 (GPIO_INPUT | GPIO_PORT_PIOC | GPIO_PIN11)
#define GPIO_HALLB_IN2 (GPIO_INPUT | GPIO_PORT_PIOC | GPIO_PIN10)

/* Quadrature Encoder Counters */

#define GPIO_IRCA_MARK (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | \
                        GPIO_PIN10)
#define GPIO_IRCB_MARK (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOC | \
                        GPIO_PIN16)

#define BOARD_NGPIOIN  8 /* HALLs + IRC marks */
#define BOARD_NGPIOOUT 8 /* INH */
#define BOARD_NGPIOINT 0 



/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_SAMV71_XULT_INCLUDE_BOARD_H */
