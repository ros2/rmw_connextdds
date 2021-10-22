// Copyright 2021 Real-Time Innovations, Inc. (RTI)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef RTI_CONNEXTDDS_CUSTOM_SQL_FILTER__VISIBILITY_CONTROL_H_
#define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_EXPORT __attribute__ ((dllexport))
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_EXPORT __declspec(dllexport)
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_BUILDING_LIBRARY
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_EXPORT
  #else
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_IMPORT
  #endif
  #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC_TYPE RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC
  #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_LOCAL
#else
  #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC
    #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_LOCAL
  #endif
  #define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC_TYPE
#endif

#endif  // RTI_CONNEXTDDS_CUSTOM_SQL_FILTER__VISIBILITY_CONTROL_H_
