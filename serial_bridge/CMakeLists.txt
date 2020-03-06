cmake_minimum_required(VERSION 3.15)
project(serial_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(serial REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)
find_package(acrobat_common REQUIRED)

add_library(serial_bridge SHARED src/serial_bridge.cpp src/encodings.cpp)
target_compile_definitions(serial_bridge PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(serial_bridge "rclcpp" "rclcpp_components" "project_settings" "acrobat_common")
rclcpp_components_register_nodes(serial_bridge "acrobat::serial_bridge::SerialBridge")
target_include_directories(serial_bridge PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:include>")

install(
  TARGETS serial_bridge
  EXPORT export_serial_bridge
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  # Get link config files
  symlink_link_configs()

  find_file(CLANG_FORMAT_CONFIG PATHS ${CMAKE_INSTALL_PREFIX}/..)
  ament_clang_format(TESTNAME serial_bridge_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})

  add_library(serial_bridge_test_utils SHARED test/testing_utils.cpp)
  target_include_directories(serial_bridge_test_utils PUBLIC test)
  ament_target_dependencies(serial_bridge_test_utils "project_settings")
  target_link_libraries(serial_bridge_test_utils serial_bridge)

  ament_add_gtest(test_finalize_message test/test_finalize_message.cpp)
  set_target_properties(test_finalize_message PROPERTIES CXX_CLANG_TIDY "" CMAKE_CXX_CPPCHECK "")
  ament_target_dependencies(test_finalize_message "project_settings")
  target_link_libraries(test_finalize_message serial_bridge serial_bridge_test_utils)

endif()

# Install launch and config files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
