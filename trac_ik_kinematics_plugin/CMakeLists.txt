cmake_minimum_required(VERSION 3.5)
project(trac_ik_kinematics_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trac_ik_lib REQUIRED)

set(TRAC_IK_LIBRARY_NAME trac_ik_kinematics_plugin)

add_library(${TRAC_IK_LIBRARY_NAME} SHARED src/trac_ik_kinematics_plugin.cpp)

target_include_directories(${TRAC_IK_LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${TRAC_IK_LIBRARY_NAME}
  rclcpp
  moveit_core
  moveit_msgs
  geometry_msgs
  orocos_kdl
  kdl_parser
  urdf
  pluginlib
  trac_ik_lib
)

pluginlib_export_plugin_description_file(moveit_core trac_ik_kinematics_description.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${TRAC_IK_LIBRARY_NAME}
  EXPORT trac_ik_kinematics_pluginTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(trac_ik_kinematics_pluginTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_msgs
  geometry_msgs
  orocos_kdl
  kdl_parser
  urdf
  pluginlib
  trac_ik_lib
)

ament_package()
