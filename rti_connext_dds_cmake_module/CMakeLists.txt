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

cmake_minimum_required(VERSION 3.5)

project(rti_connext_dds_cmake_module)

find_package(ament_cmake REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/rti_build_helper.cmake)

rti_init_env()

rti_find_connextpro()

if(RTIConnextDDS_FOUND)
    message(STATUS "RTI Connext DDS Professional (${CONNEXTDDS_VERSION}, ${CONNEXTDDS_ARCH}): '${CONNEXTDDS_DIR}'")
else()
    message(STATUS "RTI Connext DDS Professional: NOT FOUND")
endif()

set(extra_components)
if(RMW_CONNEXT_ENABLE_SECURITY)
  list(APPEND extra_components "security_plugins")
endif()

rti_find_connextmicro(${extra_components})

if(NOT RTIMEHOME_FOUND)
    message(STATUS "RTI Connext DDS Micro: NOT FOUND")
else()
    if(NOT RTIConnextDDSMicro_FOUND)
        if(NOT RTIME_TARGET_NAME AND CONNEXTDDS_ARCH)
          set(RTIME_TARGET_NAME   "${CONNEXTDDS_ARCH}")
        endif()
        rti_build_connextmicro(${extra_components})
        message(STATUS "RTI Connext DDS Micro: '${RTIMEHOME}' (build)")
    else()
        message(STATUS "RTI Connext DDS Micro: '${RTIMEHOME}'")
    endif()
endif()


if(WIN32)
    # TODO(asorbini): implement support for Windows
    # ament_environment_hooks(env_hook/rti_connext_dds.bat.in)
else()
    ament_environment_hooks(
        env_hook/rti_connext_dds.sh.in
        env_hook/rti_connext_dds_micro.sh.in)
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(
    FILES cmake/rti_build_helper.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake")

ament_package(
    CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)
