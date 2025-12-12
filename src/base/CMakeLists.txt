cmake_minimum_required(VERSION 3.8)
project(base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generate Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelVelocities.msg"
  "msg/WheelControllerState.msg"
  DEPENDENCIES std_msgs
)

# 2. Build Core Math Libraries (No ROS dependencies)
add_library(base_math_lib
  src/library/pid_controller.cpp
  src/library/kinematics_calculator.cpp
)
target_include_directories(base_math_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Build Nodes

# --- Kinematics Node ---
add_executable(kinematics_node src/nodes/kinematics_node.cpp)

# Link internal math library
target_link_libraries(kinematics_node base_math_lib)

# Link ROS dependencies using the modern 'ament' macro
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# Link custom messages generated above
rosidl_target_interfaces(kinematics_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# --- Wheel Controller Node ---
add_executable(wheel_controller_node src/nodes/wheel_controller_node.cpp)

# Link internal math library
target_link_libraries(wheel_controller_node base_math_lib)

# Link ROS dependencies
ament_target_dependencies(wheel_controller_node
  rclcpp
)
# Link custom messages generated above
rosidl_target_interfaces(wheel_controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# --- Sensor Aggregator Node ---
add_executable(sensor_aggregator_node src/nodes/sensor_aggregator_node.cpp)
target_link_libraries(sensor_aggregator_node base_math_lib)
ament_target_dependencies(sensor_aggregator_node rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target_aggregator "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(sensor_aggregator_node "${cpp_typesupport_target_aggregator}")

# 4. Install (Install-Abschnitt aktualisieren)
install(TARGETS base_math_lib kinematics_node wheel_controller_node sensor_aggregator_node
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

ament_export_include_directories(include)
ament_export_libraries(base_math_lib)
ament_package()
