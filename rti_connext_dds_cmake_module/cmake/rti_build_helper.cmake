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

include(CMakeParseArguments)

################################################################################
# rti_lib_suffix()
################################################################################
function(rti_lib_suffix var)
    set(rti_lib_sfx         "")

    if("${CMAKE_BUILD_TYPE}" MATCHES "[dD]ebug" OR
        "${CMAKE_BUILD_TYPE}" MATCHES "RelWithDebInfo")
        string(APPEND rti_lib_sfx "d")
    endif()

    set(${var} "${rti_lib_sfx}" PARENT_SCOPE)
endfunction()

################################################################################
# rti_init_env()
################################################################################
function(rti_init_env)
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE "Release"
            CACHE STRING "Default build type" FORCE)
    endif()

    set(connext_lib_sfx             "")

    if("${CMAKE_BUILD_TYPE}" MATCHES "[dD]ebug" OR
        "${CMAKE_BUILD_TYPE}" MATCHES "RelWithDebInfo")
        string(APPEND connext_lib_sfx       "d")
    endif()

    set(RTI_LIB_SUFFIX  ${connext_lib_sfx}
        CACHE STRING "Suffix for RTI libraries based on target build")
endfunction()

################################################################################
# _list_first_element(list_var match_prefix out_var)
################################################################################
function(_list_first_element list_var match_prefix out_var)
    file(GLOB ${list_var} "${match_prefix}")
    list(LENGTH ${list_var} ${list_var}_len)
    if(${${list_var}_len} GREATER 0)
        list(GET ${list_var} 0 first_el)
        set(${out_var} "${first_el}" PARENT_SCOPE)
    endif()
endfunction()

################################################################################
# rti_load_rtimehome()
################################################################################
function(rti_load_rtimehome)
    if(NOT DEFINED RTIMEHOME)
        if(NOT "$ENV{RTIMEHOME}" STREQUAL "")
            set(RTIMEHOME       "$ENV{RTIMEHOME}")
        else()
            rti_load_connextddsdir()

            if(CONNEXTDDS_DIR_FOUND)
                _list_first_element(rtime_installations
                    "${CONNEXTDDS_DIR}/rti_connext_dds_micro-*"
                    RTIMEHOME)
            endif()
        endif()
    endif()

    if("${RTIMEHOME}" STREQUAL "" OR
        NOT EXISTS "${RTIMEHOME}" OR
        NOT EXISTS "${RTIMEHOME}/CMakeLists.txt")
        set(RTIMEHOME_FOUND false)
        message(STATUS "Invalid RTIMEHOME = '${RTIMEHOME}'")
    else()
        set(RTIMEHOME_FOUND true)
        message(STATUS "Detected RTIMEHOME = '${RTIMEHOME}'")

        # Detect Micro version
        if(NOT DEFINED RTIME_VERSION)
          get_filename_component(rtimehome_base "${RTIMEHOME}" NAME)
          string(REGEX REPLACE
            "^rti_connext_dds_micro[_a-z]*-([0-9.a-z\\-_]+)$"
            "\\1"
            RTIME_VERSION
            "${rtimehome_base}")
          if("${RTIME_VERSION}" STREQUAL "")
            set(RTIME_VERSION "unknown")
          endif()
        endif()
    endif()

    set(RTIMEHOME "${RTIMEHOME}" PARENT_SCOPE)
    set(RTIMEHOME_FOUND "${RTIMEHOME_FOUND}" PARENT_SCOPE)
    set(RTIME_VERSION "${RTIME_VERSION}" PARENT_SCOPE)
endfunction()

################################################################################
# rti_build_connextmicro()
################################################################################
function(rti_build_connextmicro)
    set(extra_components ${ARGN})
    list(FIND extra_components "security_plugins" component_security)

    set(RTIMEHOME_FOUND false PARENT_SCOPE)
    set(RTIConnextDDSMicro_FOUND false PARENT_SCOPE)
    set(RTIME_TARGETS "" PARENT_SCOPE)

    rti_load_rtimehome()

    if(NOT RTIME_TARGET_NAME)
      if(WIN32 OR CYGWIN)
          set(RTIME_TARGET_NAME       "Windows")
      elseif(APPLE)
          set(RTIME_TARGET_NAME       "Darwin")
      else()
          set(RTIME_TARGET_NAME       "Linux")
      endif()
    endif()
    set(RTIME_TARGET_NAME           "${RTIME_TARGET_NAME}")
    set(RTIME_DDS_ENABLE_APPGEN     false)
    set(RTIME_NO_SHARED_LIB         false)
    set(RTIME_EXCLUDE_DPSE          true)
    set(RTIME_EXCLUDE_CPP           true)
    if(${component_security} GREATER_EQUAL 0)
      set(RTIME_TRUST_INCLUDE_INTERFACES   true)
    endif()

    add_subdirectory(${RTIMEHOME} rtime)

    rti_lib_suffix(rti_lib_sfx)

    set(rtime_libraries
        rti_me${rti_lib_sfx}
        rti_me_netiosdm${rti_lib_sfx}
        rti_me_netioshmem${rti_lib_sfx}
        rti_me_whsm${rti_lib_sfx}
        rti_me_rhsm${rti_lib_sfx}
        rti_me_discdpde${rti_lib_sfx})

    if(TARGET rti_me_ddssecurity${rti_lib_sfx} AND
        TARGET rti_me_seccore${rti_lib_sfx})
      list(APPEND rtime_libraries
          rti_me_ddssecurity${rti_lib_sfx}
          rti_me_seccore${rti_lib_sfx})
      set(rtime_have_security     true)
    endif()

    set(RTIME_TARGETS)
    # define imported targets to match those that would be defined
    # by RTIFindConnextDDSMicro.cmake
    foreach(rtime_lib ${rtime_libraries})
        if("${rtime_lib}" STREQUAL "rti_me${rti_lib_sfx}")
          set(rtime_tgt     "c_api")
        else()
          string(REGEX REPLACE "^rti_me_" "" rtime_tgt "${rtime_lib}")
          if(NOT "${rti_lib_sfx}" STREQUAL "")
            string(REGEX REPLACE "${rti_lib_sfx}$" "" rtime_tgt "${rtime_tgt}")
          endif()
        endif()

        if(NOT TARGET RTIConnextDDSMicro::${rtime_tgt})
            # add_custom_target(RTIConnextDDSMicro::${rtime_tgt}
            #     DEPENDS ${rtime_tgt})
            add_library(RTIConnextDDSMicro::${rtime_tgt} SHARED IMPORTED)
            set_target_properties(RTIConnextDDSMicro::${rtime_tgt}
              PROPERTIES
                IMPORTED_LINK_INTERFACE_LIBRARIES
                  ${rtime_lib}
            )
        endif()
        list(APPEND RTIME_TARGETS RTIConnextDDSMicro::${rtime_tgt})
    endforeach()

    set(RTIME_TARGETS ${RTIME_TARGETS} PARENT_SCOPE)
    set(RTIMEHOME_FOUND true PARENT_SCOPE)
    set(RTIConnextDDSMicro_FOUND true PARENT_SCOPE)
    # Re-export RTIME_TARGET_NAME and RTIMEHOME in case they were
    # automatically detected.
    set(RTIME_TARGET_NAME ${RTIME_TARGET_NAME} PARENT_SCOPE)
    set(RTIMEHOME ${RTIMEHOME} PARENT_SCOPE)
endfunction()

################################################################################
# rti_find_connextmicro_lib(basedir rtime_lib)
################################################################################
function(rti_find_connextmicro_lib basedir rtime_lib)
    find_library(lib${rtime_lib}
        NAME
            ${rtime_lib}
        PATHS
            "${basedir}"
        NO_DEFAULT_PATH
        NO_CMAKE_PATH
        NO_CMAKE_ENVIRONMENT_PATH
        NO_SYSTEM_ENVIRONMENT_PATH
        NO_CMAKE_SYSTEM_PATH)

    if("${lib${rtime_lib}}" MATCHES NOTFOUND)
        message(STATUS "Library ${rtime_lib} not found in ${basedir}. RTI Connext DDS Micro will not be available")
        set(${rtime_lib}_FOUND false PARENT_SCOPE)
        return()
    endif()

    set(lib${rtime_lib} ${lib${rtime_lib}} PARENT_SCOPE)
    message(STATUS "Found library ${rtime_lib}: ${lib${rtime_lib}} (${lib${rtime_lib}-NOTFOUND})")
    set(${rtime_lib}_FOUND true PARENT_SCOPE)
endfunction()

################################################################################
# rti_find_connextmicro()
################################################################################
function(rti_find_connextmicro)
    set(extra_components ${ARGN})

    set(RTIMEHOME_FOUND             false PARENT_SCOPE)
    set(RTIConnextDDSMicro_FOUND    false PARENT_SCOPE)
    set(RTIME_TARGETS               "" PARENT_SCOPE)

    rti_lib_suffix(rti_lib_sfx)

    set(rtime_libraries
        rti_me${rti_lib_sfx}
        rti_me_netiosdm${rti_lib_sfx}
        rti_me_netioshmem${rti_lib_sfx}
        rti_me_whsm${rti_lib_sfx}
        rti_me_rhsm${rti_lib_sfx}
        rti_me_discdpde${rti_lib_sfx})

    list(FIND extra_components "security_plugins" component_security)
    if(${component_security} GREATER_EQUAL 0)
      list(APPEND rtime_libraries
        rti_me_ddssecurity${rti_lib_sfx}
        rti_me_seccore${rti_lib_sfx})
    endif()

    rti_load_rtimehome()

    if(NOT RTIMEHOME_FOUND)
        message(STATUS "RTIMEHOME not found")
        return()
    endif()

    set(RTIMEHOME_FOUND     true PARENT_SCOPE)
    set(RTIMEHOME           "${RTIMEHOME}" PARENT_SCOPE)

    if(NOT DEFINED RTIME_TARGET_NAME)
        _list_first_element(rtime_installations
            "${RTIMEHOME}/lib/*"
            rtime_target_dir)

        if("${rtime_target_dir}" STREQUAL "")
            message(STATUS
              "No RTI Connext DDS Micro libraries found under ${RTIMEHOME}/lib")
        else()
            get_filename_component(RTIME_TARGET_NAME
                "${rtime_target_dir}" NAME CACHE)
            message(STATUS
              "Automatically detected "
              "RTIME_TARGET_NAME = '${RTIME_TARGET_NAME}'")
        endif()
    endif()

    set(rtime_lib_dir       "${RTIMEHOME}/lib/${RTIME_TARGET_NAME}")
    set(rtime_inc_dir       "${RTIMEHOME}/include")

    set(location_property IMPORTED_LOCATION)
    if(WIN32)
        set(location_property IMPORTED_IMPLIB)
    endif()

    list(GET rtime_libraries 0 rtime_core)
    set(rtime_extra ${rtime_libraries})
    list(REMOVE_AT rtime_extra 0)

    rti_find_connextmicro_lib(${rtime_lib_dir} ${rtime_core})

    if(NOT ${rtime_core}_FOUND)
        message(STATUS "Required library not found: ${rtime_core}")
        return()
    endif()

    set(rtime_core_inc_dirs
        "${rtime_inc_dir}"
        "${RTIMEHOME}/src/reda/sequence")

    add_library(RTIConnextDDSMicro::c_api SHARED IMPORTED)
    set_target_properties(RTIConnextDDSMicro::c_api
        PROPERTIES
            IMPORTED_NO_SONAME TRUE
            ${location_property}
                "${lib${rtime_core}}"
            INTERFACE_INCLUDE_DIRECTORIES
                "${rtime_core_inc_dirs}")

    if(RTIME_TARGET_NAME MATCHES "Win")
      set_target_properties(RTIConnextDDSMicro::c_api
        PROPERTIES
            INTERFACE_COMPILE_DEFINITIONS
                RTIME_DLL_VARIABLE)
    endif()

    set(rtime_targets     RTIConnextDDSMicro::c_api)

    foreach(rtime_lib ${rtime_extra})
        rti_find_connextmicro_lib(${rtime_lib_dir} ${rtime_lib})

        if(NOT ${rtime_lib}_FOUND)
            message(STATUS "Required library not found: ${rtime_lib}")
            return()
        endif()

        string(REGEX REPLACE "^rti_me_" "" rtime_tgt "${rtime_lib}")
        if(NOT "${rti_lib_sfx}" STREQUAL "")
          string(REGEX REPLACE "${rti_lib_sfx}$" "" rtime_tgt "${rtime_tgt}")
        endif()

        add_library(RTIConnextDDSMicro::${rtime_tgt} SHARED IMPORTED)
        set_target_properties(RTIConnextDDSMicro::${rtime_tgt}
            PROPERTIES
                IMPORTED_NO_SONAME TRUE
                ${location_property}
                    "${lib${rtime_lib}}"
                INTERFACE_INCLUDE_DIRECTORIES
                    "${rtime_inc_dir}"
                INTERFACE_LINK_LIBRARIES
                    RTIConnextDDSMicro::c_api)
        list(APPEND rtime_targets RTIConnextDDSMicro::${rtime_tgt})
    endforeach()

    message(STATUS "RTIME_TARGETS: ${rtime_targets}")

    set(RTIMEHOME_FOUND true PARENT_SCOPE)
    set(RTIMEHOME "${RTIMEHOME}" PARENT_SCOPE)
    set(RTIConnextDDSMicro_FOUND true PARENT_SCOPE)
    set(RTIME_VERSION "${RTIME_VERSION}" PARENT_SCOPE)
    set(RTIME_TARGETS ${rtime_targets} PARENT_SCOPE)
    set(RTIME_TARGET_NAME ${RTIME_TARGET_NAME} PARENT_SCOPE)
endfunction()

################################################################################
# rti_load_connextddsdir()
################################################################################
function(rti_load_connextddsdir)
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
        message(STATUS "Invalid CONNEXTDDS_DIR = '${CONNEXTDDS_DIR}'")
    else()
        set(CONNEXTDDS_DIR_FOUND true)
        message(STATUS "Detected CONNEXTDDS_DIR = '${CONNEXTDDS_DIR}'")
    endif()

    set(CONNEXTDDS_DIR "${CONNEXTDDS_DIR}" PARENT_SCOPE)
    set(CONNEXTDDS_DIR_FOUND "${CONNEXTDDS_DIR_FOUND}" PARENT_SCOPE)
endfunction()

################################################################################
# rti_find_connextpro()
################################################################################
function(rti_find_connextpro)
    set(extra_components ${ARGN})

    rti_load_connextddsdir()

    if(NOT CONNEXTDDS_DIR_FOUND)
        set(RTIConnextDDS_FOUND false)
    else()
        list(APPEND CMAKE_MODULE_PATH
            "${CONNEXTDDS_DIR}/resource/cmake")

        set(CONNEXTDDS_VERSION      "5.3.1")
        find_package(RTIConnextDDS  "${CONNEXTDDS_VERSION}"
            COMPONENTS     core ${extra_components})
    endif()

    if(RTIConnextDDS_FOUND AND NOT TARGET RTIConnextDDS::c_api)
      if("${CONNEXTDDS_DIR}" VERSION_GREATER_EQUAL "6.0.0")
        message(FATAL_ERROR
          "Expected imported targets to be created by "
          "FindRTIConnextDDS.cmake 6.x")
      endif()

      # define imported targets, since they are not defined by
      # FindRTIConnextDDS.cmake before 6.0.0

      if("${CMAKE_BUILD_TYPE}" MATCHES "[dD]ebug")
          set(release_type DEBUG)
      else()
          set(release_type RELEASE)
      endif()

      set(location_property IMPORTED_LOCATION)
      if(WIN32)
          set(location_property IMPORTED_IMPLIB)
      endif()

      add_library(RTIConnextDDS::c_api SHARED IMPORTED)
      list(GET CONNEXTDDS_C_API_LIBRARIES_${release_type}_SHARED 0 libnddsc)
      set(c_api_libs
          ${CONNEXTDDS_C_API_LIBRARIES_${release_type}_SHARED}
          ${CONNEXTDDS_EXTERNAL_LIBS})
      list(REMOVE_AT c_api_libs 0)
      # remove "-D" from defines
      string(REPLACE "-D" "" ndds_defines "${CONNEXTDDS_DEFINITIONS}")
      # expand ndds_defines into a cmake list
      string(REPLACE " " ";" ndds_defines "${ndds_defines}")
      # Detect 64-bit build and remove flag from defines
      list(FIND ndds_defines "-m64" ndds_64bit)
      list(REMOVE_ITEM ndds_defines "-m64")
      set(ndds_compile_opts)
      if(${ndds_64bit} GREATER_EQUAL 0)
        list(APPEND ndds_compile_opts "-m64")
      endif()
      set_target_properties(RTIConnextDDS::c_api
              PROPERTIES
                ${location_property}
                  "${libnddsc}"
                IMPORTED_LINK_INTERFACE_LIBRARIES
                  "${c_api_libs}"
                INTERFACE_INCLUDE_DIRECTORIES
                  "${CONNEXTDDS_INCLUDE_DIRS}"
                INTERFACE_COMPILE_DEFINITIONS
                  "${ndds_defines}"
                INTERFACE_COMPILE_OPTIONS
                  "${ndds_compile_opts}")
    endif()

    set(RTIConnextDDS_FOUND ${RTIConnextDDS_FOUND} PARENT_SCOPE)
    set(CONNEXTDDS_DIR      "${CONNEXTDDS_DIR}" PARENT_SCOPE)
    set(CONNEXTDDS_ARCH     "${CONNEXTDDS_ARCH}" PARENT_SCOPE)
    set(CONNEXTDDS_VERSION  "${RTICONNEXTDDS_VERSION}" PARENT_SCOPE)
endfunction()

