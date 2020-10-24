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


if(DEFINED rmw_connextdds_common_cpp_DIR AND
    EXISTS "${rmw_connextdds_common_cpp_DIR}")
    get_filename_component(RMW_CONNEXT_DIR 
        "${rmw_connextdds_common_cpp_DIR}/../../.." REALPATH CACHE FORCE)
else()
    get_filename_component(RMW_CONNEXT_DIR
        "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH CACHE FORCE)
endif()

set(RMW_CONNEXT_COMMON_SOURCE
    ${RMW_CONNEXT_DIR}/src/rmw_context.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_discovery.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_graph.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_event.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_impl.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_info.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_node.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_publication.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_serde.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_service.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_subscription.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_waitset.cpp
    ${RMW_CONNEXT_DIR}/src/rmw_type_support.cpp
    ${RMW_CONNEXT_DIR}/src/demangle.cpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/demangle.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/namespace_prefix.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/context.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/discovery.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/graph_cache.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/log.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/resource_limits.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/rmw_impl.hpp
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/static_config.h
    ${RMW_CONNEXT_DIR}/include/rmw_connextdds_cpp/visibility_control.h)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Default build type" FORCE)
endif()

set(link_static                 false)
set(connext_lib_sfx             "")

if(link_static)
    string(APPEND connext_lib_sfx       "z")
endif()
if("${CMAKE_BUILD_TYPE}" MATCHES "[dD]ebug" OR
    "${CMAKE_BUILD_TYPE}" MATCHES "RelWithDebInfo")
    string(APPEND connext_lib_sfx       "d")
endif()

################################################################################
# 
################################################################################
macro(load_rtimehome)
    # Locate Micro's installation from RTIMEHOME
    #message(STATUS "pre RTIMEHOME = ${RTIMEHOME}")
    #message(STATUS "pre ENV{RTIMEHOME} = $ENV{RTIMEHOME}")
    if(NOT DEFINED RTIMEHOME)
        if(NOT "$ENV{RTIMEHOME}" STREQUAL "")
            set(RTIMEHOME       "$ENV{RTIMEHOME}")
        else()
            load_connextddsdir()
            set(RTIMEHOME "${CONNEXTDDS_DIR}/rti_connext_dds_micro-3.0.3")
        endif()
    endif()

    if("${RTIMEHOME}" STREQUAL "" OR
        NOT EXISTS "${RTIMEHOME}")
        set(RTIMEHOME_FOUND false)
    else()
        set(RTIMEHOME_FOUND true)
    endif()
    #message(STATUS "post RTIMEHOME = ${RTIMEHOME}")
endmacro()

macro(load_connextddsdir)
    if(NOT DEFINED CONNEXTDDS_DIR)
        if(NOT "$ENV{CONNEXTDDS_DIR}" STREQUAL "")
            file(TO_CMAKE_PATH "$ENV{CONNEXTDDS_DIR}" connextdds_dir)
        elseif(DEFINED NDDSHOME)
            set(connextdds_dir          "${NDDSHOME}")
        elseif(NOT "$ENV{NDDSHOME}" STREQUAL "")
            file(TO_CMAKE_PATH "$ENV{NDDSHOME}" connextdds_dir)
        endif()
        if("${connextdds_dir}" STREQUAL "")
            message(WARNING "no CONNEXTDDS_DIR or NDDSHOME specified")
        endif()
        set(CONNEXTDDS_DIR "${connextdds_dir}")
    endif()
    set(CONNEXTDDS_DIR "${CONNEXTDDS_DIR}"
        CACHE STRING
        "Installation directory for RTI Connext DDS Professional")
    if("${CONNEXTDDS_DIR}" STREQUAL "" OR
        NOT EXISTS "${CONNEXTDDS_DIR}")
        set(CONNEXTDDS_DIR_FOUND false)
    else()
        set(CONNEXTDDS_DIR_FOUND true)
    endif()
endmacro()

function(build_connextmicro)
    # TODO try to load binary libraries from RTIMEHOME using
    # FindRTIConnextDDSMicro.cmake

    set(RTIME_CMAKE_ROOT            "${CMAKE_CURRENT_BINARY_DIR}/rtime-build")
    if(NOT DEFINED RTIME_TARGET_NAME)    
        if(WIN32 OR CYGWIN)
            set(RTIME_TARGET_NAME           "Windows")
        elseif(APPLE)
            set(RTIME_TARGET_NAME           "Darwin")
        else()
            set(RTIME_TARGET_NAME           "Linux")
        endif()
    endif()
    if(NOT DEFINED RTIME_TARGET)
        set(RTIME_TARGET            "${RTIME_TARGET_NAME}")
    endif()
    set(RTIME_DDS_ENABLE_APPGEN     false CACHE BOOL "")
    set(RTIME_NO_SHARED_LIB         ${link_static} CACHE BOOL "")
    set(RTIME_EXCLUDE_DPSE          true CACHE BOOL "")
    set(RTIME_EXCLUDE_CPP           true CACHE BOOL "")

    add_subdirectory(${RTIMEHOME} rtime)

    if(NOT link_static)
        install(
            TARGETS ${connext_libs}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin)
    endif()
    
    foreach(rtime_lib ${connext_libs})
        set_target_properties(${rtime_lib} PROPERTIES
                POSITION_INDEPENDENT_CODE   ON)
    endforeach()
endfunction()

################################################################################
# 
################################################################################
function(load_connextpro)
    load_connextddsdir()

    if(NOT CONNEXTDDS_DIR_FOUND)
        set(RTIConnextDDS_FOUND false PARENT_SCOPE)
        return()
    endif()

    if (NOT DEFINED CONNEXTDDS_VERSION)
        if(NOT "$ENV{CONNEXTDDS_VERSION}" STREQUAL "")
            set(CONNEXTDDS_VERSION              "$ENV{CONNEXTDDS_VERSION}")
        else()
            set(CONNEXTDDS_VERSION              "6.0.0")
        endif()
    endif()
    list(APPEND CMAKE_MODULE_PATH   "${CONNEXTDDS_DIR}/resource/cmake")

    find_package(RTIConnextDDS  "${CONNEXTDDS_VERSION}"
        COMPONENTS     core)
    
    if (NOT RTIConnextDDS_FOUND)
        message(STATUS "Failed to load RTI Connext DDS libraries from ${CONNEXTDDS_DIR}")
    else()
        message(STATUS "Loaded RTI Connext DDS libraries "
            "from ${CONNEXTDDS_DIR} (version >= ${CONNEXTDDS_VERSION})")
    endif()

    set(RTIConnextDDS_FOUND ${RTIConnextDDS_FOUND} PARENT_SCOPE)
endfunction()

################################################################################
# 
################################################################################
macro(build_rmw_connext)
    # for some reason, if we don't set this explicitly, cmake will only build
    # a static library
    set(BUILD_SHARED_LIBS       ON)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
    endif()
    
    set(RMW_CONNEXT_DDS_API     "RMW_CONNEXT_DDS_API_${connext_api}"
        CACHE STRING "DDS implementation to use")

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Wimplicit-fallthrough)
    endif()

    set(package_deps
        rcutils
        rcpputils
        rmw
        rmw_dds_common
        rosidl_runtime_c
        rosidl_runtime_cpp
        fastcdr
        rosidl_typesupport_fastrtps_c
        rosidl_typesupport_fastrtps_cpp)

    foreach(pkg_dep ${package_deps})
        find_package(${pkg_dep} REQUIRED)
    endforeach()

    ament_export_dependencies(${package_deps})

    add_library(${PROJECT_NAME}
        ${RMW_CONNEXT_COMMON_SOURCE}
        ${connext_src})

    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${RMW_CONNEXT_DIR}/include>
        $<BUILD_INTERFACE:${RMW_CONNEXT_DIR}/src>
        ${connext_includes}
        $<INSTALL_INTERFACE:include>)

    target_link_libraries(${PROJECT_NAME}   ${connext_libs} fastcdr)

    ament_target_dependencies(${PROJECT_NAME} ${package_deps})

    configure_rmw_library(${PROJECT_NAME})

    target_compile_definitions(${PROJECT_NAME}
        PRIVATE
            RMW_VERSION_MAJOR=${rmw_VERSION_MAJOR}
            RMW_VERSION_MINOR=${rmw_VERSION_MINOR}
            RMW_VERSION_PATCH=${rmw_VERSION_PATCH}
            RMW_CONNEXT_DDS_API=${RMW_CONNEXT_DDS_API}
            ${connext_defines}
    )

    if(NOT "${RMW_CONNEXT_RELEASE}" STREQUAL "")
        string(TOUPPER "${RMW_CONNEXT_RELEASE}" rmw_connext_release)
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_RELEASE=RMW_CONNEXT_RELEASE_${rmw_connext_release})
    endif()

    if(NOT "${RMW_CONNEXT_LOG_MODE}" STREQUAL "")
        string(TOUPPER "${RMW_CONNEXT_LOG_MODE}" rmw_connext_log_mode)
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_LOG_MODE=RMW_CONNEXT_LOG_MODE_${rmw_connext_log_mode})
    endif()

    if(NOT "${RMW_CONNEXT_EXPORT_MESSAGE_TYPES}" STREQUAL "")
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_EXPORT_MESSAGE_TYPES=${RMW_CONNEXT_EXPORT_MESSAGE_TYPES})
    endif()

    if(NOT "${RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES}" STREQUAL "")
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES=${RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES})
    endif()

    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE "RMW_CONNEXT_BUILDING_LIBRARY")

    ament_export_include_directories(include ${RMW_CONNEXT_DIR}/include)
    ament_export_libraries(${PROJECT_NAME})

    register_rmw_implementation(
        "c:rosidl_typesupport_c:rosidl_typesupport_fastrtps_c"
        "cpp:rosidl_typesupport_cpp:rosidl_typesupport_fastrtps_cpp")

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
        DIRECTORY ${RMW_CONNEXT_DIR}/include/
        DESTINATION include
    )

    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endmacro()

