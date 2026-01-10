cmake_minimum_required(VERSION 3.8)
project(base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelVelocities.msg"
  "msg/WheelTicks4.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Libraries (no ROS deps)
add_library(base_core
  src/library/kinematics_calculator.cpp
  src/library/pid_controller.cpp
)
target_include_directories(base_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Phidget library (best-effort)
find_library(PHIDGET22_LIB NAMES phidget22)
if(NOT PHIDGET22_LIB)
  message(WARNING "phidget22 library not found. hardware_node may fail to link on this system.")
endif()

# Nodes
add_executable(kinematics_node
  src/nodes/kinematics_node.cpp
)
ament_target_dependencies(kinematics_node rclcpp geometry_msgs)
target_link_libraries(kinematics_node base_core)
rosidl_target_interfaces(kinematics_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(hardware_node
  src/nodes/hardware_node.cpp
)
ament_target_dependencies(hardware_node rclcpp std_msgs)
target_link_libraries(hardware_node base_core)
if(PHIDGET22_LIB)
  target_link_libraries(hardware_node ${PHIDGET22_LIB})
endif()
rosidl_target_interfaces(hardware_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  kinematics_node
  hardware_node
  base_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
