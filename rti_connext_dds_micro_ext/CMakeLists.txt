################################################################################
#
# (c) 2020 Copyright, Real-Time Innovations, Inc. (RTI)
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
#
################################################################################

cmake_minimum_required(VERSION 3.5)

project(rti_connext_dds_micro_ext C)

find_package(ament_cmake REQUIRED)
find_package(rti_cmake_helper REQUIRED)
find_package(rti_connext_dds_micro_vendor REQUIRED)

rti_find_connextmicro()

if(NOT RTIConnextDDSMicro_FOUND)
    ament_package()
    message(STATUS
        "RTI Connext DDS Micro not found, skipping ${PROJECT_NAME}")
    return()
endif()

set(RTIME_EXT_SOURCE
    src/rtime_ext.c
    include/rtiros/rtime_ext.h
    include/rtiros/visibility_control.h)

set(RTIME_EXT_INCLUDE_DIRS
    include
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/domain>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/subscription>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/publication>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/infrastructure>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/topic>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/type>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/builtin_ipc>
    $<BUILD_INTERFACE:${RTIMEHOME}/src/reda/sequence>
)

add_library(${PROJECT_NAME} ${RTIME_EXT_SOURCE})
        
target_include_directories(${PROJECT_NAME} PUBLIC
    ${RTIME_EXT_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}  ${RTIME_TARGETS})

ament_export_dependencies(
    rti_cmake_helper
    rti_connext_dds_micro_vendor
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
