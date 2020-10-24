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
find_package(rti_connext_dds_micro_ext REQUIRED)

function(rti_load_connextmicro)
    if (NOT RTIMEHOME_FOUND)
        message(WARNING "RTIMEHOME not found")
        set(RTIConnextDDSMicro_FOUND        false   PARENT_SCOPE)
        return()
    else()
        if(NOT rti_connext_dds_micro_ext_FOUND OR
            "${RTIME_EXT_SOURCE}" STREQUAL "")
            message(WARNING "Extra RTIME source code not found")
            set(RTIConnextDDSMicro_FOUND        false   PARENT_SCOPE)
            return()
        endif()

        set(RTIConnextDDSMicro_FOUND        true    PARENT_SCOPE)

        # for some reason, if we don't set this explicitly, cmake will only build
        # a static library
        set(BUILD_SHARED_LIBS       ON)

        rti_build_connextmicro()

        set(rtime_include_dirs)
        foreach(rtime_inc_dir ${RTIME_EXT_INCLUDE_DIRS})
            list(APPEND rtime_include_dirs $<BUILD_INTERFACE:${RTIMEHOME}/${rtime_inc_dir}>)
        endforeach()

        add_library(${PROJECT_NAME} ${RTIME_EXT_SOURCE})
        
        target_include_directories(${PROJECT_NAME} PUBLIC
            ${rtime_include_dirs}
            ${RTIME_EXT_DIR}/include
            $<INSTALL_INTERFACE:include>)

        target_link_libraries(${PROJECT_NAME}   ${RTIME_LIBRARIES})
    endif()
endfunction()

rti_load_rtimehome()
rti_load_connextmicro()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package(
    CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)

if(RTIConnextDDSMicro_FOUND)

    install(
        DIRECTORY ${RTIMEHOME}/include/
                  ${RTIME_EXT_DIR}/include/
        DESTINATION include
    )

    install(
        FILES ${RTIMEHOME}/src/reda/sequence/REDASequence.h
        DESTINATION include
    )

    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()