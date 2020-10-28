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

project(rmw_connextdds_micro_cpp)

find_package(ament_cmake REQUIRED)
find_package(rti_connext_dds_cmake_module REQUIRED)

rti_find_connextmicro()

if(NOT RTIConnextDDSMicro_FOUND)
    ament_package()
    message(WARNING
        "RTI Connext DDS Micro not found, skipping ${PROJECT_NAME}")
    return()
endif()

find_package(rmw_connextdds_common_cpp REQUIRED)

set(connext_deps        rti_connext_dds_cmake_module)
set(connext_includes    )
set(connext_api         MICRO)
set(connext_libs        ${RTIME_TARGETS})
set(connext_src
    src/rmw_type_support_rtime.cpp
    src/dds_api_rtime.cpp
    include/rmw_connextdds_cpp/type_support_impl.hpp)

rti_build_rmw_connext()
