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

project(rti_connext_dds_micro_vendor C)

find_package(ament_cmake REQUIRED)
find_package(rti_cmake_helper REQUIRED)

rti_find_connextmicro()
if (NOT RTIConnextDDSMicro_FOUND)
    if(NOT RTIMEHOME_FOUND)
        ament_package()
        message(STATUS "No RTI Connext DDS Micro installation detected. Skipping ${PROJECT_NAME}")
        return()
    endif()
    message(STATUS "Building RTI Connext DDS Micro from source...")
    rti_build_connextmicro()
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
    rti_cmake_helper
)

ament_package(
    CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)
