cmake_minimum_required(VERSION 3.5)
project(drone)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(base_node_lib SHARED
  src/drone_node.cpp)
target_include_directories(base_node_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(base_node_lib
  PRIVATE "DRONE_NODE_BUILDING_DLL")
ament_target_dependencies(base_node_lib
  "drone_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros")
rclcpp_components_register_node(base_node_lib PLUGIN "drone_node::DroneNode" EXECUTABLE drone_node)

target_link_libraries(base_node_lib
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk
)

install(TARGETS
  base_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
