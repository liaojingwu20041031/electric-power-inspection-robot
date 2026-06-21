cmake_minimum_required(VERSION 3.8)
project(ylhb_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(base_controller src/base_controller.cpp)
ament_target_dependencies(base_controller
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(zlac8015d_canopen_controller src/zlac8015d_canopen_controller.cpp)
target_include_directories(zlac8015d_canopen_controller PRIVATE include)
ament_target_dependencies(zlac8015d_canopen_controller
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(imu_driver src/imu_driver.cpp)
ament_target_dependencies(imu_driver
  rclcpp
  sensor_msgs
  tf2
)

add_library(ylhb_normalize_goal_bt_node SHARED src/normalize_goal_bt_node.cpp)
ament_target_dependencies(ylhb_normalize_goal_bt_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(ylhb_normalize_goal_bt_node BT::behaviortree_cpp_v3)
target_compile_definitions(ylhb_normalize_goal_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  base_controller
  zlac8015d_canopen_controller
  imu_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ylhb_normalize_goal_bt_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vision_node.py
  scripts/auto_align.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/amcl_swing_relocalization_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME amcl_swing_relocalization_node
)

install(PROGRAMS
  scripts/scan_map_relocalization_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_map_relocalization_node
)

install(DIRECTORY
  launch
  config
  urdf
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Keep CI focused on functional checks until the legacy C++ drivers are
  # reformatted in a dedicated mechanical-only change.
  set(ament_cmake_uncrustify_FOUND TRUE)
  # xmllint fetches the ROS package schema over the network in this environment.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_zlac_drive_mapping test/test_zlac_drive_mapping.cpp)
  target_include_directories(test_zlac_drive_mapping PRIVATE include)
  target_compile_definitions(test_zlac_drive_mapping PRIVATE
    BASE_KINEMATICS_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/base_kinematics.yaml")

  ament_add_pytest_test(test_bringup_imu_detection test/test_bringup_imu_detection.py)
  ament_add_pytest_test(test_robot_geometry test/test_robot_geometry.py)
  ament_add_pytest_test(test_slam_config test/test_slam_config.py)
  ament_add_pytest_test(test_nav2_localization_config test/test_nav2_localization_config.py)
  ament_add_pytest_test(test_scan_map_relocalization test/test_scan_map_relocalization.py)
endif()

ament_package()
