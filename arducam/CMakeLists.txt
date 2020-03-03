cmake_minimum_required(VERSION 3.5)
project(arducam)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(project_settings REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

# Find architecture and choose correct library object
execute_process(
  COMMAND uname -m
  COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE)
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)
if(${ARCHITECTURE} STREQUAL "x86_64")
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/x86_64/libArduCamLib.so ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so
       SYMBOLIC)
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/x86_64/libArduCamLib.so.2 ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so.2
       SYMBOLIC)
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/x86_64/libArduCamLib.so.2.0.0
       ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so.2.0.0 SYMBOLIC)
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/x86_64/libArduCamLib.a ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.a
       SYMBOLIC)
elseif(${ARCHITECTURE} STREQUAL "aarch64")
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/arm64/libArduCamLib.so ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so
       SYMBOLIC)
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/arm64/libArduCamLib.so.2 ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so.2
       SYMBOLIC)
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/arm64/libArduCamLib.so.2.0.0
       ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so.2.0.0 SYMBOLIC)
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/lib/arm64/libArduCamLib.a ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.a SYMBOLIC)
endif()

# Create symlink
set(ARDUCAM_SYMBOLIC_LINK ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so)

# Arducam SDK Target
add_library(arducam_sdk INTERFACE)
target_link_libraries(arducam_sdk INTERFACE ${ARDUCAM_SYMBOLIC_LINK})
target_include_directories(arducam_sdk INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")

# Arducam Config Parser Target
add_library(arducam_config_parser SHARED src/arducam_config_parser/arducam_config_parser.c
                                         src/arducam_config_parser/ini.c)
target_include_directories(arducam_config_parser PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")
set_target_properties(arducam_config_parser PROPERTIES CXX_CLANG_TIDY "" CMAKE_CXX_CPPCHECK "")

# Arducam Driver Target
add_library(arducam_driver SHARED src/arducam_driver.cpp)
target_include_directories(arducam_driver PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(
  arducam_driver
  "project_settings"
  "sensor_msgs"
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "ament_index_cpp")
target_link_libraries(arducam_driver usb-1.0 pthread arducam_config_parser ${ARDUCAM_SYMBOLIC_LINK})
target_compile_definitions(arducam_driver PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(arducam_driver "acrobat::arducam::ArducamDriver")
set(node_plugins "${node_plugins}acrobat::arducam::ArducamDriver;$<TARGET_FILE:arducam_driver>\n")

# Image Viewer Target
add_library(image_viewer SHARED src/image_viewer.cpp)
target_include_directories(image_viewer PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                               "$<INSTALL_INTERFACE:include>")
target_compile_definitions(image_viewer PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(image_viewer "project_settings" "sensor_msgs" "rclcpp" "rclcpp_components" "cv_bridge")
rclcpp_components_register_nodes(image_viewer "acrobat::image_viewer::ImageViewer")
set(node_plugins "${node_plugins}acrobat::image_viewer::ImageViewer;$<TARGET_FILE:image_viewer>\n")

add_executable(manual_arducam src/manual_arducam.cpp)
target_link_libraries(manual_arducam arducam_driver image_viewer)
ament_target_dependencies(manual_arducam "project_settings" "rclcpp")

ament_export_include_directories(include)
ament_export_libraries(arducam_driver image_viewer)
ament_export_interfaces(export_arducam_driver HAS_LIBRARY_TARGET)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS arducam_driver image_viewer arducam_config_parser manual_arducam
  EXPORT export_arducam_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch and config files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)

  # Get link config files
  symlink_link_configs()

  find_file(CLANG_FORMAT_CONFIG PATHS ${CMAKE_INSTALL_PREFIX}/..)
  ament_clang_format(TESTNAME arducam_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})
endif()

ament_package()
