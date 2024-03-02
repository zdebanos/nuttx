macro(list_ap src)
  list(APPEND SRCS ${src})
endmacro()

set(SRCS)
list(
  APPEND
  SRCS
  sam_start.c
  sam_clockconfig.c
  sam_irq.c
  sam_allocateheap.c
  sam_lowputc.c
  sam_serial.c
  sam_gpio.c
  sam_pck.c
  sam_uid.c
  sam_systemreset.c
)

if(NOT CONFIG_SCHED_TICKLESS)
  list_ap(sam_timerisr.c)
endif()

if(CONFIG_ARM_MPU)
  list_ap(sam_mpuinit.c)
  if(CONFIG_BUILD_PROTECTED)
    list_ap(sam_userspace.c)
  endif()
endif()

if(CONFIG_SAMV7_GPIO_IRQ)
  list_ap(sam_gpioirq.c)
endif()

if(CONFIG_SAMV7_XDMAC)
  list_ap(sam_xdmac.c)
endif()

if(CONFIG_SAMV7_WDT)
  list_ap(sam_wdt.c)
endif()

if(CONFIG_SAMV7_RSWDT)
  list_ap(sam_rswdt.c)
endif()

if(CONFIG_SAMV7_1WIREDRIVER)
  list_ap(sam_1wire.c)
endif()

if(CONFIG_SAMV7_USART_IS_SPI_MASTER)
  list_ap(sam_serial_spi.c)
endif()

if(CONFIG_SAMV7_SPI_MASTER)
  list_ap(sam_spi.c)
endif()

if(CONFIG_SAMV7_SPI_SLAVE)
  list_ap(sam_spi_slave.c)
endif()

if(CONFIG_SAMV7_QSPI_SPI_MODE)
  list_ap(sam_qspi_spi.c)
elseif(CONFIG_SAMV7_QSPI)
  list_ap(sam_qspi.c)
endif()

if(CONFIG_SAMV7_TWIHS0 OR CONFIG_SAMV7_TWIHS1 OR CONFIG_SAMV7_TWIHS2)
  list_ap(sam_twihs.c)
endif()

if(CONFIG_SAMV7_SSC)
  list_ap(sam_ssc.c)
endif()

if(CONFIG_SAMV7_HAVE_TC)

  list_ap(sam_tc.c)

  if(CONFIG_TIMER)
    list_ap(sam_tc_lowerhalf.c)
  endif()
  
  if(CONFIG_SAMV7_ONESHOT)
    list_ap(sam_oneshot.c)
    list_ap(sam_oneshot_lowerhalf.c)
  endif()

  if(CONFIG_SAMV7_FREERUN)
    list_ap(sam_freerun.c)
  endif()

  if(CONFIG_SCHED_TICKLESS)
    list_ap(sam_tickless.c)
  endif()

  if(CONFIG_SENSORS_QENCODER)
    list_ap(sam_qencoder.c)
  endif()

endif()

if(CONFIG_SAMV7_HSMCI)
  list_ap(sam_hsmci.c)
  list_ap(sam_hsmci_clkdiv.c)
endif()

if(CONFIG_SAMV7_EMAC)
  list_ap(sam_emac.c)
  list_ap(sam_ethernet.c)
endif()

if(CONFIG_SAMV7_MCAN)
  list_ap(sam_mcan.c)
endif()

if(CONFIG_SAMV7_AFEC)
  list_ap(sam_afec.c)
endif()

if(CONFIG_SAMV7_USBDEVHS)
  list_ap(sam_usbdevhs.c)
endif()

if(CONFIG_SAMV7_TRNG)
  list_ap(sam_trng.c)
endif()

if(CONFIG_SAMV7_PROGMEM)
  list_ap(sam_progmem.c)
  list_ap(sam_eefc.c)
endif()

if(CONFIG_SAMV7_PWM)
  list_ap(sam_pwm.c)
endif()

if(CONFIG_SAMV7_DAC)
  list_ap(sam_dac.c)
endif()

target_sources(arch PRIVATE ${SRCS})
set_property(GLOBAL PROPERTY LD_SCRIPT
  "${NUTTX_BOARD_DIR}/../common/scripts/flash.ld.template")

set_property(GLOBAL PROPERTY LD_SCRIPT_USER
  "${NUTTX_BOARD_DIR}/../common/scripts/user-space.ld")
