cmake_minimum_required(VERSION 3.8)
project(ros_gz_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

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

install(
  DIRECTORY
    launch
    models
    worlds
    config
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  PROGRAMS
    scripts/kill_ign.bash
    scripts/kill_ros_gz.bash
  DESTINATION
    "lib/${PROJECT_NAME}"
)
if(DEFINED ENV{GZ_VERSION})
    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.gz.dsv.in")
elseif(DEFINED ENV{IGN_VERSION})
    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.ign.dsv.in")
else()
    message(WARNING "Not defined GZ_VERSION or IGN_VERSION")
endif()

ament_package()
