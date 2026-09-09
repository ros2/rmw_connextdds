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

cmake_minimum_required(VERSION 3.20)

project(rmw_connextdds)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

################################################################################
# rtirmw_add_library(
#     NAME      <library name>
#     API       PRO|MICRO
#     SOURCES   <source and header files>
#     DEPS      <ament package dependencies>
#     INCLUDES  <extra include directories>
#     LIBRARIES <extra linked libraries>
#     DEFINES   <extra defines>
#     [STATIC]
#     )
################################################################################
function(rtirmw_add_library)
    cmake_parse_arguments(_rti_build
      "" # boolean arguments
      "NAME;API;EXPORT" # single value arguments
      "SOURCES;DEPS;INCLUDES;LIBRARIES;DEFINES" # multi-value arguments
      ${ARGN} # current function arguments
    )

    # Default to C++20
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 20)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Wimplicit-fallthrough)
    endif()

    add_library(${_rti_build_NAME} SHARED ${_rti_build_SOURCES})

    target_include_directories(${_rti_build_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
        ${_rti_build_INCLUDES})

    target_link_libraries(${_rti_build_NAME}
        ${_rti_build_LIBRARIES}
        ${_rti_build_DEPS})

    set(_extra_defines)
    if("${CMAKE_BUILD_TYPE}" MATCHES "[dD]ebug")
      list(APPEND _extra_defines "RMW_CONNEXT_DEBUG=1")
    endif()

    target_compile_definitions(${_rti_build_NAME}
        PUBLIC
            RMW_VERSION_MAJOR=${rmw_VERSION_MAJOR}
            RMW_VERSION_MINOR=${rmw_VERSION_MINOR}
            RMW_VERSION_PATCH=${rmw_VERSION_PATCH}
            RMW_CONNEXT_DDS_API=RMW_CONNEXT_DDS_API_${_rti_build_API}
            $<$<PLATFORM_ID:Windows>:NOMINMAX>
            ${_rti_build_DEFINES}
            ${_extra_defines}
    )

    set(private_defines)

    if(NOT "${RMW_CONNEXT_LOG_MODE}" STREQUAL "")
        string(TOUPPER "${RMW_CONNEXT_LOG_MODE}" rmw_connext_log_mode)
        list(APPEND private_defines
          "RMW_CONNEXT_LOG_MODE=RMW_CONNEXT_LOG_MODE_${rmw_connext_log_mode}")
        set(RMW_CONNEXT_LOG_MODE "${RMW_CONNEXT_LOG_MODE}"
            CACHE INTERNAL "")
    endif()

    target_compile_definitions(${_rti_build_NAME} PRIVATE ${private_defines})

    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    target_compile_definitions(${_rti_build_NAME}
        PRIVATE "RMW_CONNEXTDDS_BUILDING_LIBRARY")

    install(
        TARGETS ${_rti_build_NAME}
        EXPORT ${_rti_build_EXPORT}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endfunction()

################################################################################
# Load external dependencies
################################################################################
set(RMW_CONNEXT_TARGET_DEPS
    rcutils::rcutils
    rcpputils::rcpputils
    rmw::rmw
    rmw_dds_common::rmw_dds_common_library
    tracetools::tracetools
    fastcdr
    rosidl_runtime_c::rosidl_runtime_c
    rosidl_runtime_cpp::rosidl_runtime_cpp
    rosidl_typesupport_fastrtps_c::rosidl_typesupport_fastrtps_c
    rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
    rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp)

set(RMW_CONNEXT_DEPS
    rcutils
    rcpputils
    rmw
    rmw_dds_common
    rmw_security_common
    tracetools
    fastcdr
    rosidl_runtime_c
    rosidl_runtime_cpp
    rosidl_typesupport_fastrtps_c
    rosidl_typesupport_fastrtps_cpp
    rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp
    rti_connext_dds_cmake_module)

foreach(pkg_dep ${RMW_CONNEXT_DEPS})
    find_package(${pkg_dep} REQUIRED)
endforeach()

################################################################################
# Build Connext Pro version
################################################################################
rti_find_connextpro()

if(NOT RTIConnextDDS_FOUND)
    message(STATUS "RTI Connext DDS Professional not found. Skipping ${PROJECT_NAME}.")
    ament_package()
    return()
endif()

set(extra_defines)
if("${CONNEXTDDS_VERSION}" VERSION_LESS "6.0.0")
  list(APPEND extra_defines "RMW_CONNEXT_DDS_API_PRO_LEGACY=1")
endif()
if(CONNEXTDDS_ARCH MATCHES ".*Win.*" AND "${CONNEXTDDS_VERSION}" VERSION_LESS "6.0.0")
  list(APPEND extra_defines "RMW_CONNEXT_BUILTIN_CFT_COMPATIBILITY_MODE=1")
endif()

set(RMW_CONNEXT_COMMON_SOURCE_CPP
    src/common/rmw_context.cpp
    src/common/rmw_discovery.cpp
    src/common/rmw_network_flow_endpoints.cpp
    src/common/rmw_graph.cpp
    src/common/rmw_event.cpp
    src/common/rmw_impl.cpp
    src/common/rmw_impl_waitset_std.cpp
    src/common/rmw_info.cpp
    src/common/rmw_listener.cpp
    src/common/rmw_node.cpp
    src/common/rmw_publication.cpp
    src/common/rmw_qos.cpp
    src/common/rmw_security_log.cpp
    src/common/rmw_serde.cpp
    src/common/rmw_service.cpp
    src/common/rmw_subscription.cpp
    src/common/rmw_waitset.cpp
    src/common/rmw_type_support.cpp
    src/common/demangle.cpp)

set(RMW_CONNEXT_COMMON_SOURCE_HPP
    include/rmw_connextdds/context.hpp
    include/rmw_connextdds/dds_api.hpp
    include/rmw_connextdds/demangle.hpp
    include/rmw_connextdds/discovery.hpp
    include/rmw_connextdds/graph_cache.hpp
    include/rmw_connextdds/log.hpp
    include/rmw_connextdds/namespace_prefix.hpp
    include/rmw_connextdds/resource_limits.hpp
    include/rmw_connextdds/rmw_impl.hpp
    include/rmw_connextdds/rmw_api_impl.hpp
    include/rmw_connextdds/rmw_waitset_std.hpp
    include/rmw_connextdds/scope_exit.hpp
    include/rmw_connextdds/static_config.hpp
    include/rmw_connextdds/type_support.hpp
    include/rmw_connextdds/visibility_control.h)

set(RMW_CONNEXT_COMMON_SOURCE
    ${RMW_CONNEXT_COMMON_SOURCE_CPP}
    ${RMW_CONNEXT_COMMON_SOURCE_HPP})

rtirmw_add_library(
    NAME      ${PROJECT_NAME}
    API       PRO
    EXPORT    export_${PROJECT_NAME}
    SOURCES   ${RMW_CONNEXT_COMMON_SOURCE}
              src/ndds/rmw_type_support_ndds.cpp
              src/ndds/rmw_typecode.cpp
              src/ndds/dds_api_ndds.cpp
              src/ndds/custom_sql_filter.cpp
              include/rmw_connextdds/typecode.hpp
              include/rmw_connextdds/dds_api_ndds.hpp
              include/rmw_connextdds/custom_sql_filter.hpp
              src/rmw_api_impl_ndds.cpp
    DEPS      ${RMW_CONNEXT_TARGET_DEPS}
    LIBRARIES RTIConnextDDS::c_api
    DEFINES   ${extra_defines})

configure_rmw_library(${PROJECT_NAME})

register_rmw_implementation(
    "c:rosidl_typesupport_fastrtps_c:rosidl_typesupport_introspection_c"
    "cpp:rosidl_typesupport_fastrtps_cpp:rosidl_typesupport_introspection_cpp")

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    find_package(osrf_testing_tools_cpp REQUIRED)
    find_package(rmw REQUIRED)
    find_package(test_msgs REQUIRED)

    ament_add_gtest(test_keyed_topics test/test_keyed_topics.cpp)
    if(TARGET test_keyed_topics)
        target_sources(test_keyed_topics PRIVATE test/test_utils.hpp)
        target_link_libraries(test_keyed_topics
            osrf_testing_tools_cpp::memory_tools
            rcutils::rcutils
            rmw::rmw
            RTIConnextDDS::cpp2_api
            ${PROJECT_NAME}
            ${test_msgs_TARGETS}
        )
    endif()

    ament_add_gtest(test_typeobject_v2 test/test_typeobject_v2.cpp)
    if(TARGET test_typeobject_v2)
        target_sources(test_typeobject_v2 PRIVATE test/test_utils.hpp)
        target_link_libraries(test_typeobject_v2
            osrf_testing_tools_cpp::memory_tools
            rcutils::rcutils
            rmw::rmw
            RTIConnextDDS::cpp2_api
            ${PROJECT_NAME}
            ${test_msgs_TARGETS}
        )
    endif()
endif()

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${RMW_CONNEXT_DEPS})

ament_package(
  CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)

ament_index_register_resource("rmw_output_prefixes"
  CONTENT "RTI Data Distribution Service\nRTI Connext DDS\nExpires on")
