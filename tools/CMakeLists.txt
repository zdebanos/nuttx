# ##############################################################################
# tools/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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
# Configure project
cmake_minimum_required(VERSION 3.16)
project(nuttx_tools LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

message(STATUS "NuttX Host Tools")

# set basic warnings

add_compile_options(-Wall -Wstrict-prototypes -Wshadow -Wundef)

# configure according to platform

if(MSYS)
  add_compile_definitions(CONFIG_WINDOWS_NATIVE=y)
else()
  # GCC or clang is assumed in all other POSIX environments (Linux, Cygwin,
  # MSYS2, macOS). strtok_r is used in some tools, but does not seem to be
  # available in the MinGW environment.

  add_compile_definitions(HAVE_STRTOK_C=1)

  if(CYGWIN)
    add_compile_definitions(HOST_CYGWIN=1)
  endif()
endif()

# define targets

add_library(csvparser csvparser.c)

add_executable(mksyscall mksyscall.c)
target_link_libraries(mksyscall PRIVATE csvparser)
install(TARGETS mksyscall DESTINATION bin)

add_executable(nxstyle nxstyle.c)
install(TARGETS nxstyle DESTINATION bin)

add_executable(gencromfs gencromfs.c)
install(TARGETS gencromfs DESTINATION bin)

add_executable(mkdeps mkdeps.c)
target_link_libraries(mkdeps PRIVATE csvparser)
install(TARGETS mkdeps DESTINATION bin)

add_executable(incdir incdir.c)
install(TARGETS incdir DESTINATION bin)
