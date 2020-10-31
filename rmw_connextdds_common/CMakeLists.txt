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

project(rmw_connextdds_common)

find_package(ament_cmake REQUIRED)
find_package(rti_connext_dds_cmake_module REQUIRED)

set(RMW_CONNEXT_COMMON_SOURCE
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
    src/demangle.cpp
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

ament_package(
    CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY src/
    DESTINATION src
)
