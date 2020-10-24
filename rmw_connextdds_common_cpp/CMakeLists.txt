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

project(rmw_connextdds_common_cpp)

find_package(ament_cmake REQUIRED)

ament_package(
    CONFIG_EXTRAS_POST "rmw_connextdds_common_cpp-extras.cmake.in"
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY src/
    DESTINATION src
)

install(
    FILES cmake/rmw_connextdds_builder.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake")