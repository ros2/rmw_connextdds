cmake_minimum_required(VERSION 3.8)
project(rti_connext_dds_custom_sql_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rti_connext_dds_cmake_module REQUIRED)
find_package(rcpputils REQUIRED)

rti_find_connextpro()
if(NOT RTIConnextDDS_FOUND)
  # Skip package if Connext DDS Professional was not found
  message(STATUS
    "RTI Connext DDS Professional not found. "
    "Skipping library ${PROJECT_NAME}")
else()

  add_library(${PROJECT_NAME}
    src/custom_sql_filter.cpp
    include/rti_connext_dds_custom_sql_filter/custom_sql_filter.hpp
    include/rti_connext_dds_custom_sql_filter/visibility_control.h
  )
  target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME} RTIConnextDDS::c_api)
  ament_target_dependencies(${PROJECT_NAME}
    rcpputils)

  # Causes the visibility macros to use dllexport rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(${PROJECT_NAME} PRIVATE "RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_BUILDING_LIBRARY")

  install(
    DIRECTORY include/
    DESTINATION include
  )
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_include_directories(
    include
  )
  ament_export_libraries(
    ${PROJECT_NAME}
  )
  ament_export_targets(
    export_${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
