cmake_minimum_required(VERSION 3.8)
project(action_fleet_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclpy REQUIRED)


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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FleetManagement.action"
)

# Server
#add_executable(fleet_management_server_cli src/fleet_management_server_cli.py)
#ament_target_dependencies(fleet_management_server_cli "rclpy")
#install(TARGETS fleet_management_server_cli DESTINATION lib/${PROJECT_NAME})

# Client
#add_executable(fleet_management_client_cli src/fleet_management_client_cli.py)
#ament_target_dependencies(fleet_management_client_cli "rclpy")
#install(TARGETS fleet_management_client_cli DESTINATION lib/${PROJECT_NAME})



ament_package()
