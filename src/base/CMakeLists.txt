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

# Messages
find_package(rosidl_default_generators REQUIRED)

# Optional hardware deps (Phidget22 + I2C)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PHIDGET22 QUIET phidget22)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelVelocities.msg"
  "msg/WheelTicks4.msg"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
)

add_library(base_lib
  src/library/pid_controller.cpp
  src/library/kinematics_calculator.cpp
)
ament_target_dependencies(base_lib
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(kinematics_node
  src/nodes/kinematics_node.cpp
)
target_link_libraries(kinematics_node
  base_lib
)
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
)

add_executable(hardware_node
  src/nodes/hardware_node.cpp
)
target_link_libraries(hardware_node
  base_lib
)
ament_target_dependencies(hardware_node
  rclcpp
)

# Link Phidget22 if available
if(PHIDGET22_FOUND)
  target_include_directories(hardware_node PRIVATE ${PHIDGET22_INCLUDE_DIRS})
  target_link_libraries(hardware_node ${PHIDGET22_LIBRARIES})
  target_compile_definitions(hardware_node PRIVATE HAVE_PHIDGET22=1)
else()
  message(WARNING "phidget22 not found via pkg-config; hardware_node will still build but may fail to link if you use Phidget APIs without installed dev package.")
endif()

# === Jazzy+: replace deprecated rosidl_target_interfaces() ===
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(kinematics_node "${cpp_typesupport_target}")
target_link_libraries(hardware_node   "${cpp_typesupport_target}")

install(TARGETS
  kinematics_node
  hardware_node
  base_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
