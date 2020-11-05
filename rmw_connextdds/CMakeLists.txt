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

project(rmw_connextdds)

find_package(ament_cmake REQUIRED)
find_package(rti_connext_dds_cmake_module REQUIRED)

set(RMW_CONNEXT_DIR     ${CMAKE_CURRENT_SOURCE_DIR})

set(RMW_CONNEXT_COMMON_SOURCE_CPP
    src/rmw_context.cpp
    src/rmw_discovery.cpp
    src/rmw_graph.cpp
    src/rmw_event.cpp
    src/rmw_impl.cpp
    src/rmw_info.cpp
    src/rmw_node.cpp
    src/rmw_publication.cpp
    src/rmw_serde.cpp
    src/rmw_service.cpp
    src/rmw_subscription.cpp
    src/rmw_waitset.cpp
    src/rmw_type_support.cpp
    src/demangle.cpp)

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
    include/rmw_connextdds/scope_exit.hpp
    include/rmw_connextdds/static_config.hpp
    include/rmw_connextdds/type_support.hpp
    include/rmw_connextdds/visibility_control.h)

set(RMW_CONNEXT_COMMON_SOURCE
    ${RMW_CONNEXT_COMMON_SOURCE_CPP}
    ${RMW_CONNEXT_COMMON_SOURCE_HPP})

install(
    FILES ${RMW_CONNEXT_COMMON_SOURCE_HPP}
    DESTINATION include/rmw_connextdds
)

install(
    FILES ${RMW_CONNEXT_COMMON_SOURCE_CPP}
    DESTINATION src
)

rti_find_connextpro()

if(NOT RTIConnextDDS_FOUND)
    # Skip package if Connext DDS Professional was not found
    message(WARNING
        "RTI Connext DDS Professional not found, skipping ${PROJECT_NAME}")
else()
    set(connext_deps        rti_connext_dds_cmake_module)
    set(connext_api         PRO)
    set(connext_libs        RTIConnextDDS::c_api)
    set(connext_src
        src/rmw_type_support_ndds.cpp
        src/rmw_typecode.cpp
        src/dds_api_ndds.cpp
        include/rmw_connextdds/type_support_impl.hpp
        include/rmw_connextdds/typecode.hpp)

    rti_build_rmw_connext()
endif()

ament_package(
    CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)
