cmake_minimum_required(VERSION 3.8)
project(gps_mavlink_relay_ros2)
set(CMAKE_BUILD_TYPE RelWithDebInfo) #Debug, Release

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(
  include
)

set(library_name gps_mavlink_relay_core)

set(dependencies
  rclcpp
  rclcpp_components
  px4_msgs
)

add_library(${library_name} SHARED
  src/gps_mavlink_relay.cpp
)
target_link_libraries(${library_name}
  MAVSDK::mavsdk
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(gps_mavlink_relay_node src/gps_mavlink_relay_node.cpp)

target_link_libraries(gps_mavlink_relay_node ${library_name})

ament_target_dependencies(gps_mavlink_relay_node
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "gps_mavlink_relay_ros2::GpsMavlinkRelayer")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gps_mavlink_relay_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()

