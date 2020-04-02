cmake_minimum_required(VERSION 3.15)
project(msp_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msp REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)
find_package(acrobat_common REQUIRED)
find_package(acrobat_msgs REQUIRED)

add_library(msp_bridge SHARED src/msp_bridge.cpp)
target_include_directories(msp_bridge PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                             "$<INSTALL_INTERFACE:include>")
target_compile_definitions(msp_bridge PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(
  msp_bridge
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "acrobat_common"
  "acrobat_msgs"
  "project_settings"
  "msp")
rclcpp_components_register_nodes(msp_bridge "acrobat::msp_bridge::MspBridge")

ament_export_include_directories(include)
ament_export_libraries(msp_bridge)
ament_export_dependencies(rclcpp sensor_msgs acrobat_common acrobat_msgs msp)
ament_export_interfaces(export_msp_bridge HAS_LIBRARY_TARGET)

install(
  TARGETS msp_bridge
  EXPORT export_msp_bridge
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)

  # Get link config files
  symlink_link_configs()

  find_file(CLANG_FORMAT_CONFIG PATHS ${CMAKE_INSTALL_PREFIX}/..)
  ament_clang_format(TESTNAME msp_bridge_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})
endif()

# Install launch and config files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
