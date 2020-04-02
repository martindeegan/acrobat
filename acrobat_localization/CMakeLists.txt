cmake_minimum_required(VERSION 3.15)
project(acrobat_localization CXX)

# find dependencies find system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)
find_package(acrobat_common REQUIRED)

add_library(acrobat_visual_odometry SHARED src/visual_odometry.cpp)
target_include_directories(acrobat_visual_odometry PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")
target_compile_definitions(acrobat_visual_odometry PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(acrobat_visual_odometry "project_settings" "acrobat_common" "rclcpp" "rclcpp_components"
                          "sensor_msgs")
rclcpp_components_register_nodes(acrobat_visual_odometry "acrobat::localization::VisualOdometry")

ament_export_include_directories(include)
ament_export_libraries(acrobat_visual_odometry)
ament_export_dependencies(rclcpp sensor_msgs acrobat_common)
ament_export_interfaces(export_acrobat_visual_odometry HAS_LIBRARY_TARGET)

install(
  TARGETS acrobat_visual_odometry
  EXPORT export_acrobat_visual_odometry
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment the line when a copyright and license is
  # not present in all source files set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only works
  # in a git repo) uncomment the line when this package is not in a git repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
