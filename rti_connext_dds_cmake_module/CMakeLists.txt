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

rti_find_connextpro()

if(RTIConnextDDS_FOUND)
    message(STATUS "RTI Connext DDS Professional (${CONNEXTDDS_ARCH}): '${CONNEXTDDS_DIR}'")
else()
    message(STATUS "RTI Connext DDS Professional: NOT FOUND")
endif()

rti_find_connextmicro()

if(NOT RTIMEHOME_FOUND)
    message(STATUS "RTI Connext DDS Micro: NOT FOUND")
else()
    if(NOT RTIConnextDDSMicro_FOUND)
        rti_build_connextmicro()
        message(STATUS "RTI Connext DDS Micro: '${RTIMEHOME}' (build)")
    else()
        message(STATUS "RTI Connext DDS Micro: '${RTIMEHOME}'")
    endif()

    # set(RTIME_EXT_SOURCE
    #     src/rtime_ext.c
    #     include/rtiros/rtime_ext.h
    #     include/rtiros/visibility_control.h)

    # $<INSTALL_INTERFACE:include>
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # set(RTIME_EXT_INCLUDE_DIRS
    #     include
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/domain>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/subscription>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/publication>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/infrastructure>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/topic>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/type>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/builtin_ipc>
    #     $<BUILD_INTERFACE:${RTIMEHOME}/src/reda/sequence>
    #     )

    set(RTIME_EXT           rti_connextdds_micro_ext)

    set(BUILD_SHARED_LIBS       ON)

    add_library(${RTIME_EXT}
        src/rtime_ext.c
        include/rtiros/rtime_ext.h
        include/rtiros/visibility_control.h)
        
    target_include_directories(${RTIME_EXT}
        PRIVATE "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/domain>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/subscription>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/publication>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/infrastructure>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/topic>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/type>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/dds_c/builtin_ipc>"
                "$<BUILD_INTERFACE:${RTIMEHOME}/src/reda/sequence>")
    
    target_include_directories(${RTIME_EXT}
        PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
               "$<INSTALL_INTERFACE:include>")

    target_link_libraries(${RTIME_EXT}  ${RTIME_TARGETS})


    install(
        DIRECTORY include/
        DESTINATION include
    )

    install(
        TARGETS ${RTIME_EXT}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    ament_export_include_directories(include)
    ament_export_libraries(${RTIME_EXT})

endif()


if(WIN32)
    # TODO implement support for Windows
    # ament_environment_hooks(env_hook/rti_connext_dds.bat.in)
else()
    ament_environment_hooks(env_hook/rti_connext_dds.sh.in)
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package(
    CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)

install(
    FILES cmake/rti_build_helper.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake")
