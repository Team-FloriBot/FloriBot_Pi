cmake_minimum_required(VERSION 3.8)
project(base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ---- Phidget22 (no pkg-config required) ----
find_path(PHIDGET22_INCLUDE_DIR
  NAMES phidget22.h
  PATHS /usr/include /usr/local/include
)
find_library(PHIDGET22_LIBRARY
  NAMES phidget22
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/local/lib
)

if(NOT PHIDGET22_INCLUDE_DIR OR NOT PHIDGET22_LIBRARY)
  message(FATAL_ERROR "phidget22 not found (need libphidget22-dev). Include=${PHIDGET22_INCLUDE_DIR} Lib=${PHIDGET22_LIBRARY}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelVelocities.msg"
  "msg/WheelTicks4.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_library(base_lib SHARED
  src/library/pid_controller.cpp
  src/library/kinematics_calculator.cpp
)

target_include_directories(base_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(base_lib
  rclcpp
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(base_lib "${cpp_typesupport_target}")

add_executable(kinematics_node src/nodes/kinematics_node.cpp)
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(kinematics_node base_lib)

add_executable(hardware_node src/nodes/hardware_node.cpp)
ament_target_dependencies(hardware_node
  rclcpp
)
target_include_directories(hardware_node PRIVATE ${PHIDGET22_INCLUDE_DIR})
target_link_libraries(hardware_node
  base_lib
  ${PHIDGET22_LIBRARY}
)

install(TARGETS
  base_lib
  kinematics_node
  hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
