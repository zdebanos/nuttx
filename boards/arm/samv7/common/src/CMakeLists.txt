macro(list_ap arg)
  list(APPEND SRCS ${arg})
endmacro()

set(SRCS)
list_ap(sam_reset.c)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_BOARDCTL_BOOT_IMAGE)
    list_ap(sam_boot_image.c)
  endif()

  if(CONFIG_SAMV7_PROGMEM AND CONFIG_MTD_PROGMEM)
    list_ap(sam_progmem.c)
  endif()

  if(CONFIG_SAMV7_HSMCI0)
    list_ap(sam_hsmci.c)
  endif()

  if(CONFIG_FS_AUTOMOUNTER)
    list_ap(sam_automount.c)
  endif()

  if(CONFIG_SAMV7_GPIO_ENC)
    list_ap(sam_gpio_enc.c)
  endif()

endif()

target_sources(board PRIVATE ${SRCS})
