# Copyright 2020 Real-Time Innovations, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(rmw_connextmicro_cpp)

find_package(rmw_connextdds_common_cpp REQUIRED)

load_rtimehome()
if (NOT RTIMEHOME_FOUND)
    message(WARNING "No RTI Connext DDS Micro installation found, "
            "skipping ${PROJECT_NAME}")
    return()
endif()

set(connext_libs
    rti_me${connext_lib_sfx}
    rti_me_netiosdm${connext_lib_sfx}
    rti_me_netioshmem${connext_lib_sfx}
    rti_me_whsm${connext_lib_sfx}
    rti_me_rhsm${connext_lib_sfx}
    rti_me_discdpde${connext_lib_sfx})
set(connext_includes
    $<BUILD_INTERFACE:${RTIMEHOME}/include>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/domain>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/subscription>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/publication>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/infrastructure>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/topic>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/type>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/builtin_ipc>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/reda/sequence>)
set(connext_src
    src/rmw_type_support_rtime.cpp
    src/rtime_ext.c
    src/dds_api_rtime.cpp
    include/rmw_connextdds_cpp/rtime_ext.h
    include/rmw_connextdds_cpp/type_support_impl.hpp)
set(connext_defines )

build_rmw_connext("micro")
