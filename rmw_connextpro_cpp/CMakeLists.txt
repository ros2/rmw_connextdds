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

project(rmw_connextpro_cpp)

include(../rmw_connextdds_cpp/rmw_connextdds_cpp.cmake)

set(connext_libs        RTIConnextDDS::c_api)
set(connext_includes    )
set(connext_src
    src/rmw_type_support_ndds.cpp
    src/rmw_typecode.cpp
    include/rmw_connextdds_cpp/type_support_ndds.hpp
    include/rmw_connextdds_cpp/typecode.hpp)
set(connext_defines
    RMW_CONNEXT_DDS_API=RMW_CONNEXT_DDS_API_PRO)

build_rmw_connext("pro")