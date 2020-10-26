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

################################################################################
# 
################################################################################
function(rti_load_rmw_source basedir)
    set(sources ${ARGN})

    get_filename_component(RMW_CONNEXT_DIR "${basedir}/../../.." REALPATH CACHE)

    if(NOT EXISTS "${RMW_CONNEXT_DIR}")
        message(FATAL_ERROR "Invalid base directory for RMW common source: ${RMW_CONNEXT_DIR}")
    endif()
    
    set(qsources)
    foreach(src_f ${sources})
        list(APPEND qsources "${RMW_CONNEXT_DIR}/${src_f}")
    endforeach()

    set(RMW_CONNEXT_COMMON_SOURCE ${qsources}
        CACHE INTERNAL "Common source code for RMW implementations" FORCE)

endfunction()

function(rti_lib_suffix var)
    set(rti_lib_sfx         "")

    if("${CMAKE_BUILD_TYPE}" MATCHES "[dD]ebug" OR
        "${CMAKE_BUILD_TYPE}" MATCHES "RelWithDebInfo")
        string(APPEND rti_lib_sfx "d")
    endif()

    set(${var} "${rti_lib_sfx}" PARENT_SCOPE)
endfunction()

################################################################################
# 
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

function(_list_first_element list_var match_prefix out_var)
    file(GLOB ${list_var} "${match_prefix}")
    list(LENGTH ${list_var} ${list_var}_len)
    if (${${list_var}_len} GREATER 0)
        list(GET ${list_var} 0 first_el)
        set(${out_var} "${first_el}" PARENT_SCOPE)
    endif()
endfunction()

################################################################################
# 
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
    endif()

    set(RTIMEHOME "${RTIMEHOME}" PARENT_SCOPE)
    set(RTIMEHOME_FOUND "${RTIMEHOME_FOUND}" PARENT_SCOPE)
endfunction()

################################################################################
# 
################################################################################
function(rti_build_connextmicro)
    set(RTIMEHOME_FOUND false PARENT_SCOPE)
    set(RTIConnextDDSMicro_FOUND false PARENT_SCOPE)
    set(RTIME_TARGETS "" PARENT_SCOPE)

    rti_load_rtimehome()

    if(WIN32 OR CYGWIN)
        set(RTIME_TARGET_NAME       "Windows")
    elseif(APPLE)
        set(RTIME_TARGET_NAME       "Darwin")
    else()
        set(RTIME_TARGET_NAME       "Linux")
    endif()
    set(RTIME_TARGET_NAME           "${RTIME_TARGET_NAME}")
    set(RTIME_DDS_ENABLE_APPGEN     false)
    set(RTIME_NO_SHARED_LIB         false)
    set(RTIME_EXCLUDE_DPSE          true)
    set(RTIME_EXCLUDE_CPP           true)

    add_subdirectory(${RTIMEHOME} rtime)

    rti_lib_suffix(rti_lib_sfx)

    set(rtime_targets
        rti_me${rti_lib_sfx}
        rti_me_netiosdm${rti_lib_sfx}
        rti_me_netioshmem${rti_lib_sfx}
        rti_me_whsm${rti_lib_sfx}
        rti_me_rhsm${rti_lib_sfx}
        rti_me_discdpde${rti_lib_sfx})
    
    set(RTIME_TARGETS ${rtime_targets} PARENT_SCOPE)
    set(RTIMEHOME_FOUND false PARENT_SCOPE)
    set(RTIConnextDDSMicro_FOUND false PARENT_SCOPE)
    set(RTIME_TARGETS "" PARENT_SCOPE)
endfunction()

################################################################################
# 
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
# 
################################################################################
function(rti_find_connextmicro)
    set(lib_prefix      RTIConnextDDSMicro)
    set(RTIMEHOME_FOUND false PARENT_SCOPE)
    set(${lib_prefix}_FOUND false PARENT_SCOPE)
    set(RTIME_TARGETS "" PARENT_SCOPE)

    rti_lib_suffix(rti_lib_sfx)

    set(rtime_libraries
        rti_me${rti_lib_sfx}
        rti_me_netiosdm${rti_lib_sfx}
        rti_me_netioshmem${rti_lib_sfx}
        rti_me_whsm${rti_lib_sfx}
        rti_me_rhsm${rti_lib_sfx}
        rti_me_discdpde${rti_lib_sfx})

    rti_load_rtimehome()

    if (NOT RTIMEHOME_FOUND)
        message(STATUS "RTIMEHOME not found")
        return()
    endif()

    if(NOT DEFINED RTIME_TARGET_NAME)
        _list_first_element(rtime_installations
            "${RTIMEHOME}/lib/*"
            rtime_target_dir)

        if("${rtime_target_dir}" STREQUAL "")
            message(STATUS "No RTI Connext DDS Micro libraries found under ${RTIMEHOME}/lib")
        else()
            get_filename_component(RTIME_TARGET_NAME
                "${rtime_target_dir}" NAME CACHE)
            message(STATUS "Automatically detected RTIME_TARGET_NAME = '${RTIME_TARGET_NAME}'")
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
        return()
    endif()

    set(rtime_core_inc_dirs
            "${rtime_inc_dir}"
            "${RTIMEHOME}/src/reda/sequence")

    add_library(${lib_prefix}::${rtime_core} SHARED IMPORTED)
    set_target_properties(${lib_prefix}::${rtime_core}
        PROPERTIES
            IMPORTED_NO_SONAME TRUE
            ${location_property}
                "${lib${rtime_core}}"
            INTERFACE_INCLUDE_DIRECTORIES
                "${rtime_core_inc_dirs}")

    set(rtime_targets     ${lib_prefix}::${rtime_core})

    foreach(rtime_lib ${rtime_extra})
        rti_find_connextmicro_lib(${rtime_lib_dir} ${rtime_lib})

        if(NOT ${rtime_lib}_FOUND)
            return()
        endif()

        add_library(${lib_prefix}::${rtime_lib} SHARED IMPORTED)
        set_target_properties(${lib_prefix}::${rtime_lib}
            PROPERTIES
                IMPORTED_NO_SONAME TRUE
                ${location_property}
                    "${lib${rtime_lib}}"
                INTERFACE_INCLUDE_DIRECTORIES
                    "${rtime_inc_dir}"
                INTERFACE_LINK_LIBRARIES
                    ${lib_prefix}::${rtime_core})
        list(APPEND rtime_targets ${lib_prefix}::${rtime_lib})
    endforeach()

    set(RTIMEHOME_FOUND true PARENT_SCOPE)
    set(RTIMEHOME "${RTIMEHOME}" PARENT_SCOPE)
    set(${lib_prefix}_FOUND true PARENT_SCOPE)
    set(RTIME_TARGETS ${rtime_targets} PARENT_SCOPE)
endfunction()

################################################################################
# 
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
# 
################################################################################
function(rti_find_connextpro)
    rti_load_connextddsdir()

    if(NOT CONNEXTDDS_DIR_FOUND)
        set(RTIConnextDDS_FOUND false)
    else()
        set(CONNEXTDDS_VERSION      "6.0.0")

        list(APPEND CMAKE_MODULE_PATH
            "${CONNEXTDDS_DIR}/resource/cmake")

        find_package(RTIConnextDDS  "${CONNEXTDDS_VERSION}"
            COMPONENTS     core)
    endif()

    set(RTIConnextDDS_FOUND ${RTIConnextDDS_FOUND} PARENT_SCOPE)
    set(CONNEXTDDS_DIR      "${CONNEXTDDS_DIR}" PARENT_SCOPE)
    set(CONNEXTDDS_ARCH     "${CONNEXTDDS_ARCH}" PARENT_SCOPE)
endfunction()

################################################################################
# 
################################################################################
macro(rti_build_rmw_connext)
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

    ament_export_dependencies(${package_deps} ${connext_deps})

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

    ament_target_dependencies(${PROJECT_NAME} ${package_deps} ${connext_deps})

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
        set(RMW_CONNEXT_RELEASE "${RMW_CONNEXT_RELEASE}"
            CACHE INTERNAL "")
    endif()

    if(NOT "${RMW_CONNEXT_LOG_MODE}" STREQUAL "")
        string(TOUPPER "${RMW_CONNEXT_LOG_MODE}" rmw_connext_log_mode)
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_LOG_MODE=RMW_CONNEXT_LOG_MODE_${rmw_connext_log_mode})
        set(RMW_CONNEXT_LOG_MODE "${RMW_CONNEXT_LOG_MODE}"
            CACHE INTERNAL "")
    endif()

    if(NOT "${RMW_CONNEXT_EXPORT_MESSAGE_TYPES}" STREQUAL "")
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_EXPORT_MESSAGE_TYPES=${RMW_CONNEXT_EXPORT_MESSAGE_TYPES})
        set(RMW_CONNEXT_EXPORT_MESSAGE_TYPES "${RMW_CONNEXT_EXPORT_MESSAGE_TYPES}"
            CACHE INTERNAL "")
    endif()

    if(NOT "${RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES}" STREQUAL "")
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES=${RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES})
        set(RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES "${RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES}"
            CACHE INTERNAL "")
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

