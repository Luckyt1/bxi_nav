cmake_minimum_required(VERSION 3.8)
project(gps_waypoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# gps_waypoint: 航点跟随节点
add_executable(gps_waypoint src/main.cpp)
target_link_libraries(gps_waypoint yaml-cpp)
ament_target_dependencies(gps_waypoint
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# rtk_odom: 将 RTK GPS (/fix) 转为本地 ENU 里程计 (/state_estimation)
add_executable(rtk_odom src/rtk_odom.cpp)
ament_target_dependencies(rtk_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

add_executable(gps_relatively src/gps_relatively.cpp)
target_link_libraries(gps_relatively yaml-cpp)
ament_target_dependencies(gps_relatively
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  gps_waypoint
  rtk_odom
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
