cmake_minimum_required(VERSION 3.15)
project(msp_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_MSP REQUIRED IMPORTED_TARGET msp)

# find project dependencies
find_package(project_settings REQUIRED)
find_package(acrobat_common REQUIRED)
find_package(acrobat_msgs REQUIRED)

add_library(msp_bridge SHARED src/msp_bridge.cpp)
ament_target_dependencies(msp_bridge "rclcpp" "sensor_msgs" "acrobat_common" "acrobat_msgs" "project_settings")
target_link_libraries(msp_bridge PkgConfig::PKG_MSP)

rclcpp_components_register_nodes(msp_bridge "msp_bridge::MspBridge")

install(TARGETS msp_bridge DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)

  # Get link config files
  symlink_link_configs()

  find_file(CLANG_FORMAT_CONFIG PATHS ${CMAKE_INSTALL_PREFIX}/..)
  ament_clang_format(TESTNAME acrobat_common_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})
endif()

ament_package()
