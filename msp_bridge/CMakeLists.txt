cmake_minimum_required(VERSION 3.15)
project(msp_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_MSP REQUIRED IMPORTED_TARGET msp)

# find project dependencies
find_package(acrobat_common REQUIRED)
find_package(project_settings REQUIRED)

add_executable(msp_bridge src/msp_bridge.cpp)
target_include_directories(msp_bridge PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                             "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(msp_bridge "rclcpp" "project_settings" "sensor_msgs" "acrobat_common")
target_link_libraries(msp_bridge PkgConfig::PKG_MSP)

install(TARGETS msp_bridge DESTINATION lib/${PROJECT_NAME})
install(TARGETS msp_bridge DESTINATION bin)

ament_package()
