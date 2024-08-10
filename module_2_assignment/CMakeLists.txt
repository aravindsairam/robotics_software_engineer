cmake_minimum_required(VERSION 3.8)
project(module_2_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(draw_circle src/draw_circle.cpp)
ament_target_dependencies(draw_circle rclcpp geometry_msgs)

add_executable(draw_spiral src/draw_spiral.cpp)
ament_target_dependencies(draw_spiral rclcpp geometry_msgs)

add_executable(draw_logspiral src/draw_logspiral.cpp)
ament_target_dependencies(draw_logspiral rclcpp geometry_msgs)

add_executable(drive_robot src/drive_robot.cpp)
ament_target_dependencies(drive_robot rclcpp geometry_msgs)

add_executable(change_speed src/control_speeds.cpp)
ament_target_dependencies(change_speed rclcpp geometry_msgs)

install(TARGETS
  draw_circle
  draw_spiral
  draw_logspiral
  drive_robot
  change_speed
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
