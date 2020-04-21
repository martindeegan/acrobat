cmake_minimum_required(VERSION 3.15)
project(acrobat_localization CXX)

# find dependencies find system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(GTSAM REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)
find_package(acrobat_common REQUIRED)

add_library(acrobat_vio SHARED src/visual_odometry.cpp src/imu_propagator.cpp src/frame.cpp src/frontend.cpp
                               src/visualizer.cpp src/backend.cpp)
target_include_directories(acrobat_vio PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                              "$<INSTALL_INTERFACE:include>")
target_compile_definitions(acrobat_vio PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(
  acrobat_vio
  "project_settings"
  "acrobat_common"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "cv_bridge")
target_link_libraries(acrobat_vio gtsam)
rclcpp_components_register_nodes(acrobat_vio "acrobat::localization::VisualOdometry")

add_executable(acrobat_localization_exec src/main.cpp)
target_link_libraries(acrobat_localization_exec acrobat_vio)

ament_export_include_directories(include)
ament_export_libraries(acrobat_vio)
ament_export_dependencies(rclcpp sensor_msgs acrobat_common)
ament_export_interfaces(export_acrobat_localization HAS_LIBRARY_TARGET)

install(
  TARGETS acrobat_vio acrobat_localization_exec
  EXPORT export_acrobat_localization
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  ament_clang_format(TESTNAME acrobat_common_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})
endif()

ament_package()
