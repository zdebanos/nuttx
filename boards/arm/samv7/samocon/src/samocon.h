/****************************************************************************
 * boards/arm/samv7/samocon/src/samocon.h
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

#ifndef __BOARDS_ARM_SAMV7_SAMOCON_SRC_SAMOCON_H
#define __BOARDS_ARM_SAMV7_SAMOCON_SRC_SAMOCON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "sam_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration Logic ******************************************************/

/* HSMCI (SD slot) define slot */

/* Can support HSMCI if the SD card is configured */
#define HAVE_HSMCI          1

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_HSMCI0)
#  warning "Enable HSMCI0 peripheral to support MMC/SD!"
#  undef HAVE_HSMCI
#endif
//#define HAVE_AUTOMOUNTER     1

/* The SD_DET pin - detection of the inserted card */
/* The SD_DET pin is at PC18 */

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled. No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need interrupts on GPIOC (PC18) to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_GPIOC_IRQ)
#  warning GPIOC interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* MMC/SD minor numbers */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifndef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#if CONFIG_NSH_MMCSDSLOTNO != 0
#  error SAMV71 has only one MMC/SD slot (CONFIG_NSH_MMCSDSLOTNO)
#  undef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#define HSMCI0_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#define HSMCI0_MINOR  CONFIG_NSH_MMCSDMINOR

/* Automounter.  Currently only works with HSMCI. */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_HSMCI)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
#endif

#ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

/* USB define logic */
#define HAVE_USB             1
#define HAVE_USBDEV          1
#define HAVE_USBMONITOR      1

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* CONFIG_SAMV7_UDP and CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_SAMV7_UDP) || !defined(CONFIG_USBDEV)
#  undef HAVE_USB
#  undef HAVE_USBDEV
#endif

/* Feedback define logic */
#define HAVE_HALL_FEEDBACK   1
#define HAVE_IRCA_FEEDBACK   1
#define HAVE_IRCB_FEEDBACK   1

/* Can't have A QENC feedback if pins are not defined */
#if defined(HAVE_IRCA_FEEDBACK) && \
    (!defined(CONFIG_SAMV7_TC0_TIOA0) || !defined(CONFIG_SAMV7_TC0_TIOB0) || \
    !defined(CONFIG_SAMV7_TC0_TIOB1))
#   warning "Enable TIOA0, TIOB0 and TIOB1 for QENC_A operation!"
#   undef  HAVE_IRCA_FEEDBACK
#endif

#if defined(HAVE_IRCB_FEEDBACK) && \
    (!defined(CONFIG_SAMV7_TC2_TIOA6) || !defined(CONFIG_SAMV7_TC2_TIOB6) || \
    !defined(CONFIG_SAMV7_TC2_TIOB7))
#   warning "Enable TIOA6, TIOB6 and TIOB7 for QENC_B operation!"
#   undef  HAVE_IRCB_FEEDBACK
#endif


/* I2C define logic */
#define HAVE_MAIN_I2C        1
#define HAVE_EXTERNAL_I2C    1
#define HAVE_24XXXX          1
#define HAVE_I2CTOOL         1

#if defined(HAVE_MAIN_I2C) && !defined(CONFIG_SAMV7_TWIHS1)
#   warning "Configure CONFIG_SAMV7_TWIHS1 for main I2C peripheral."
#   undef HAVE_MAIN_I2C
#endif

#if defined(HAVE_EXTERNAL_I2C) && !defined(CONFIG_SAMV7_TWIHS0)
#   warning "Configure CONFIG_SAMV7_TWIHS0 for external I2C peripheral."
#   undef HAVE_EXTERNAL_I2C
#endif

#if defined(HAVE_24XXXX) && !defined(HAVE_MAIN_I2C)
#   warning "Define HAVE_MAIN_I2C in order to use 24XXXX memory."
#   undef HAVE_24XXXX
#endif

/* SPI define logic */
#define HAVE_W25QXXXJV       1
#define HAVE_MAIN_SPI0       1
//#define HAVE_EXTERNAL_SPI1   1
#undef HAVE_EXTERNAL_3ADC

#ifdef CONFIG_SAMV7_QSPI
#  error "Can't have QSPI on the SaMoCon board!!"
#endif 

#if defined(HAVE_W25QXXXJV) && !defined(CONFIG_MTD_W25QXXXJV)
#  warning "Configure CONFIG_MTD_W25QXXXJV to use the W25 flash!"
#  undef HAVE_W25QXXXJV
#endif

#if defined(HAVE_W25QXXXJV) && \
    (!defined(CONFIG_SAMV7_SPI0_MASTER) && !defined(CONFIG_SAMV7_SPI0_SLAVE))
#  undef HAVE_W25QXXXJV
#  warning "SPI0 not selected! Can't have W25Q32JV!"
#endif

/* Network define logic */
#define HAVE_NETWORK         1
#define HAVE_HARDCODED_MAC   1

#if defined(HAVE_HARDCODED_MAC)
#   define HAVE_MACADDR
#endif

#if defined(HAVE_NETWORK) && \
    (!defined(CONFIG_SAMV7_GPIOC_IRQ) || !defined(CONFIG_NET) || \
    !defined(CONFIG_SAMV7_EMAC0))
#  warning "Configure GPIOC_IRQ, CONFIG_NET and CONFIG_SAMV7_EMAC0 to use eth!"
#  undef HAVE_NETWORK
#endif


/* AD converters define logic */
#define HAVE_AFEC0           1
#define HAVE_AFEC1           1
#undef HAVE_EXTERNAL_3ADC    

#if defined(HAVE_AFEC0) && !defined(CONFIG_SAMV7_AFEC0)
#   warning "Configure CONFIG_SAMV7_AFEC0 for AFEC0!"
#   undef HAVE_AFEC0
#endif

#if defined(HAVE_AFEC1) && !defined(CONFIG_SAMV7_AFEC1)
#   warning "Configure CONFIG_SAMV7_AFEC1 for AFEC1!"
#   undef HAVE_AFEC1
#endif

#if defined(HAVE_EXTERNAL_3ADC) && !defined(CONFIG_SAMV7_AFEC1) && \
    !defined(HAVE_AFEC1) 
#   warning "Configure CONFIG_SAMV7_AFEC1 for 3 external AD channels!"
#   undef HAVE_AFEC1
#endif

/* PWM define logic */
#define HAVE_PWM0
#define HAVE_PWM1
#define HAVE_COMPLEMENTARY_PWM0
#define HAVE_COMPLEMENTARY_PWM1

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAMV71_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAMV71_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif


/* On-chip Programming Memory */

#if !defined(CONFIG_SAMV7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif


/* Do we need to register I2C drivers on behalf of the I2C tool? */

#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif


/* Ethernet MAC.
 *
 * KSZ8081RNA Connections (RMII)
 * ------------------------------
 *
 *   ------ --------- ---------
 *   SAMV71 SAMV71    Ethernet  
 *   Pin    Function  Function
 *   ------ --------- --------- 
 *   PD00   GTXCK     REF_CLK   
 *   PD01   GTXEN     TXEN
 *   PD02   GTX0      TXD0
 *   PD03   GTX1      TXD1
 *   PD04   GRXDV     CRS_DV   
 *   PD05   GRX0      RXD0     
 *   PD06   GRX1      RXD1      
 *   PD07   GRXER     RXER      
 *   PD08   GMDC      MDC       
 *   PD09   GMDIO     MDIO
 *   PC25   GPIO      INTERRUPT 
 *   PB13   GPIO      LINK LED
 *   ------ --------- --------- 
 */

#define GPIO_EMAC0_INT    (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                           GPIO_INT_FALLING | GPIO_PORT_PIOC | GPIO_PIN25)
#define IRQ_EMAC0_INT     SAM_IRQ_PC25
#define GPIO_EMAC0_LINK_LED (GPIO_OUTPUT | GPIO_PORT_PIOB | GPIO_PIN13)

/* HSMCI SD Card Detect
 *
 * The SAM V71 Xplained Ultra has one standard SD card connector which is
 * connected to the High Speed Multimedia Card Interface (HSMCI) of the SAM
 * V71. SD card connector:
 *
 *   ------ ----------------- ---------------------
 *   SAMV71 SAMV71            Shared functionality
 *   Pin    Function
 *   ------ ----------------- ---------------------
 *   PA30   MCDA0 (DAT0)
 *   PA31   MCDA1 (DAT1)
 *   PA26   MCDA2 (DAT2)
 *   PA27   MCDA3 (DAT3)      Camera
 *   PA25   MCCK (CLK)        Shield
 *   PA28   MCCDA (CMD)
 *   PD18   Card Detect (C/D) Shield
 *   ------ ----------------- ---------------------
 */

#define GPIO_HSMCI0_CD (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOD | GPIO_PIN18)
#define IRQ_HSMCI0_CD   SAM_IRQ_PD18

/* USB Related
 *
 * The SaMoCon board incorporates an AP2171W power switch driven by USB_EN
 * pin. The switch powers the USB bus if the SaMoCon board acts as a host.
 * Otherwise an antiparallel diode is incorporated if USB device
 * powering is desired. USB_EN is at PC21 and is active high.
 *
 * The power switch also has /FLG pin (SaMoCon name is USB_OVERCURR)
 * indicating overccurent through the power switch or over-temperature.
 * It is an open drain output and is active LOW. The USB_OVERCURR pin
 * is at TODO
 */


#warning "Need to set up USB_OVERCURR in documentation!! Gitlab issue"
#define GPIO_VBUSON (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                     GPIO_PORT_PIOC | GPIO_PIN21)
#define GPIO_USB_OVERCURR (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOD | \
                           GPIO_PIN29 | GPIO_CFG_DEGLITCH | GPIO_INT_BOTHEDGES)
#define GPIO_USB_TUSB321_ID (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | \
                             GPIO_PIN6)
#define IRQ_USB_HOST_OVERCURR SAM_IRQ_PD29



/* Quadrature Encoder Counters */
/*
 * TC0 and TC2 Timer/Counter peripherals are used. A, B are encoder signals,
 * ID is the encoder index signal. A so-called mark used as a GPIO 
 * can be used for another user-defined feedback signal
 * but it's not necessary.
 * 
 *   --------- -------- ------
 *   SaMoCon   SAMV71   SAMV71       
 *   Pin       Function Pin
 *   --------- -------- ------  
 *   IRCA_A    TIOA6    PC5
 *   IRCA_B    TIOB6    PC6
 *   IRCA_ID   TIOB7    PC9
 *   IRCA_MARK GPIO     PA10
 *   IRCB_A    TIOA0    PA0
 *   IRCB_B    TIOB0    PA1
 *   IRCB_ID   TIOB1    PA16
 *   IRCB_MARK GPIO     PC16
 *
 */

/* No need to define any timer pins. Already defined in samv71_pinmap.h */
#define IRCA_TC      2
#define IRCA_DEVPATH "/dev/qe0"
#define IRCB_TC      0
#define IRCB_DEVPATH "/dev/qe1"


/* SPI Peripherals
 * 
 * The SaMoCon board has an 8pin main connected routed on the board
 * itself for the main SPI using SAMV71's SPI0 peripheral.
 * Optionally, SPI1 can be configured to function as an external
 * peripheral. However, 3 ADC channels routed externally too
 * can not be used at the same time because of collisions with SPI1
 * If chip selects for external SPI1 are desired, configure 
 * external GPIOs manually (description of extra GPIOs follows).
 *
 * A table of SPI0 fixed chip selects
 *    ------ ------ ------- ---------
 *    SPIO0  Conn.  SAMV71  SAMV71
 *    CS     Pin    Pin     Function
 *    ------ ------ ------- ---------
 *    CS0    6      PD26    GPIO
 *    CS1    7      PD25    GPIO
 *    CS2    8      PC7     GPIO
 */

#define GPIO_SPI0_CS0 (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOD | GPIO_PIN26)
#define GPIO_SPI0_CS1 (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOD | GPIO_PIN25)
#define GPIO_SPI0_CS2 (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOC | GPIO_PIN7)


/* W25Q32J SPI FLASH
 *
 * SPI0 is used as a main SPI routed onto a main connector.
 * The W25Q32J shares the same signals with SPI0.
 * There is a pair of flash control pins:
 * 
 *   ---------- ------- 
 *   SaMoCon    SAMV71
 *   Pin        Pin
 *   ---------- -------
 *   MEM_HOLD   PD27
 *   SPI_CSMEM  PC28
 */


#define W25QXXXJV_MEM_HOLD  (GPIO_OUTPUT | GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD \
                             GPIO_PIN27)
#define W25QXXXJV_SPI_CSMEM (GPIO_OUTPUT | GPIO_PORT_PIOC | GPIO_PIN28) 

/* HALL inputs */
#define HAVE_HALLA  1
#define HAVE_HALLB  1

/* External Outputs. 
 * See the SaMoCon's wiki for external pinout.
 * TODO
 */

#warning "Unite External pinouts"

/* SPI cannot be combined with external AD converters */
/* Prioritize external AD converters instead of SPI1 */
#if defined(HAVE_EXTERNAL_3ADC) && defined(HAVE_EXTERNAL_SPI1)
#  warning "Cannot have SPI1 and 3 external ADCs at the same time"
#  undef HAVE_EXTERNAL_SPI1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures the on-board SDRAM.  SAMV71 Xplained Ultra features one
 *   external IS42S16100E-7BLI, 512Kx16x2, 10ns, SDRAM. SDRAM0 is connected
 *   to chip select NCS1.
 *
 *  Input Parameters:
 *     None
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.
 *    When we complete initialization of SDRAM and it is ready for use,
 *    we will make DRAM into normal memory.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SDRAMC
void sam_sdram_config(void);
#else
#  define sam_sdram_config(t)
#endif

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int sam_bringup(void);
#endif

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAMV71-XULT board.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SPI
void sam_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN
int sam_can_setup(void);
#endif

/****************************************************************************
 * Name:  sam_usbinitialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup
 *   USB- related GPIO pins for the SAMV71-XULT board.
 *
 ****************************************************************************/

#ifdef HAVE_USB
void sam_usbinitialize(void);
#endif

/****************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

#ifdef HAVE_NETWORK
void sam_netinitialize(void);
#endif

/****************************************************************************
 * Name: sam_emac0_setmac
 *
 * Description:
 *   Read the Ethernet MAC address from the AT24 FLASH and configure the
 *   Ethernet driver with that address.
 *
 ****************************************************************************/

#ifdef HAVE_MACADDR
int sam_emac0_setmac(void);
#endif

#if defined(HAVE_IRCA_FEEDBACK) || defined(HAVE_IRCB_FEEDBACK)
int sam_qencs_init(void);
#endif

#if defined(HAVE_MAIN_I2C) || defined(HAVE_EXTERNAL_I2C)
int sam_i2c_init(void);
#endif

#if defined(HAVE_AFEC0) || defined(HAVE_AFEC1)
int sam_adc_init(void);
#endif

#if defined(HAVE_PWM0) || defined(HAVE_PWM1)
int sam_pwm_init(void);
#endif

#if defined(HAVE_HALLA) || defined(HAVE_HALLB)
int sam_halls_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_SAMV71_XULT_SRC_SAMV71_XULT_H */
