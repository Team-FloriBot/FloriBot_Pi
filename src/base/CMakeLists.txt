cmake_minimum_required(VERSION 3.8)
project(base)

# ============================================================
# Build settings
# ============================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Phidget22 (for hardware_node) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(PHIDGET22 REQUIRED IMPORTED_TARGET libphidget22)

# ============================================================
# 1) Custom message interfaces
#    (make sure the files really exist in ${CMAKE_CURRENT_SOURCE_DIR}/msg)
# ============================================================
set(MSG_FILES
  msg/WheelVelocities.msg
  msg/WheelControllerState.msg
  msg/WheelTicks4.msg
)

foreach(_msg ${MSG_FILES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_msg}")
    message(FATAL_ERROR
      "Missing message file: '${_msg}' (expected at '${CMAKE_CURRENT_SOURCE_DIR}/${_msg}').\n"
      "Fix by either:\n"
      "  1) creating the file in the msg/ folder, or\n"
      "  2) removing it from MSG_FILES, or\n"
      "  3) correcting the filename (case-sensitive).")
  endif()
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ============================================================
# 2) Core math library
# ============================================================
add_library(base_math_lib
  src/library/pid_controller.cpp
  src/library/kinematics_calculator.cpp
)

target_include_directories(base_math_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(base_math_lib
  rclcpp
)

# ============================================================
# Typesupport target for messages (Jazzy-safe pattern)
# ============================================================
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# ============================================================
# 3) Nodes
# ============================================================

# --- Kinematics Node ---
add_executable(kinematics_node
  src/nodes/kinematics_node.cpp
)
target_link_libraries(kinematics_node
  base_math_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- Wheel Controller Node ---
add_executable(wheel_controller_node
  src/nodes/wheel_controller_node.cpp
)
target_link_libraries(wheel_controller_node
  base_math_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(wheel_controller_node
  rclcpp
)

# --- Sensor Aggregator Node ---
add_executable(sensor_aggregator_node
  src/nodes/sensor_aggregator_node.cpp
)
target_link_libraries(sensor_aggregator_node
  base_math_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(sensor_aggregator_node
  rclcpp
  sensor_msgs
)

# --- Hardware Node ---
add_executable(hardware_node
  src/nodes/hardware_node.cpp
)
target_link_libraries(hardware_node
  base_math_lib
  ${cpp_typesupport_target}
  PkgConfig::PHIDGET22
)
ament_target_dependencies(hardware_node
  rclcpp
  std_msgs
)

# ============================================================
# 4) Install
# ============================================================
install(TARGETS
  base_math_lib
  kinematics_node
  wheel_controller_node
  sensor_aggregator_node
  hardware_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 5) Export
# ============================================================
ament_export_include_directories(include)
ament_export_libraries(base_math_lib)
ament_package()
