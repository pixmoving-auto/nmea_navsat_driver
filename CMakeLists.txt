cmake_minimum_required(VERSION 3.5)
project(nmea_udp_navsat_driver_cpp)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_library(nmea_udp_navsat_driver_node SHARED
  src/nmea_udp_navsat_driver_node.cpp
)

rclcpp_components_register_node(nmea_udp_navsat_driver_node
  PLUGIN "NmeaUdpImuDriverNode"
  EXECUTABLE nmea_udp_navsat_driver
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  # config
)
