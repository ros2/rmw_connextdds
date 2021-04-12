cmake_minimum_required(VERSION 3.5)
project(rmw_dds_common_typesupports)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Load type support packages to make sure that they are installed
find_package(rosidl_typesupport_fastrtps_c REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# This is an hack to have the `rosidl_generate_interfaces()` function consider
# all the type supports required by `rmw_connextdds` even though that RMW
# implementation has not yet been registered.
ament_index_register_resource(rmw_typesupport CONTENT
  "rosidl_typesupport_c;rosidl_typesupport_fastrtps_c;rosidl_typesupport_introspection_c;rosidl_typesupport_cpp;rosidl_typesupport_fastrtps_cpp;rosidl_typesupport_introspection_cpp")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
