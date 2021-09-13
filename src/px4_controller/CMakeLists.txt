cmake_minimum_required(VERSION 3.5)
project(px4_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(set_thrust src/send_controls.cpp)
ament_target_dependencies(set_thrust rclcpp px4_msgs)

add_executable(send_actuator_command src/send_actuator_command.cpp)
ament_target_dependencies(send_actuator_command rclcpp px4_msgs)

add_executable(send_actuator_command_feedback src/send_actuator_command_feedback.cpp src/util_quad.cpp)
ament_target_dependencies(send_actuator_command_feedback rclcpp px4_msgs Eigen3)
target_include_directories(send_actuator_command_feedback PUBLIC
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(send_actuator_command_feedback Eigen3::Eigen)

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

install(TARGETS
  set_thrust
  send_actuator_command
  send_actuator_command_feedback
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
