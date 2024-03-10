# ##############################################################################
# boards/arm/samv7/samocon/src/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS sam_boot.c sam_halls.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sam_appinit.c sam_bringup.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS sam_bringup.c)
endif()

if(CONFIG_SAMV7_EMAC)
  list(APPEND SRCS sam_ethernet.c)
endif()

if(CONFIG_SAMV7_USBDEVHS)
  list(APPEND SRCS sam_usbdev.c)
endif()

if(CONFIG_SAMV7_SPI)
  list(APPEND SRCS sam_spi.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS sam_usbmsc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS sam_composite.c)
endif()

if(CONFIG_SAMV7_MCAN)
  list(APPEND SRCS sam_mcan.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS sam_qencs.c)
endif()

if(CONFIG_SAMV7_TWIHS0 OR CONFIG_SAMV7_TWIHS1)
  list(APPEND SRCS sam_i2c.c)
endif()

if(CONFIG_SAMV7_AFEC0 OR CONFIG_SAMV7_AFEC1)
  list(APPEND SRCS sam_adc.c)
endif()

if(CONFIG_SAMV7_PWM0 OR CONFIG_SAMV7_PWM1)
  list(APPEND SRCS sam_pwm.c)
endif()


target_sources(board PRIVATE ${SRCS})
