cmake_minimum_required(VERSION 3.15)
project(acrobat_common CXX)

# find system dependencies
find_package(ament_cmake REQUIRED)
find_package(Sophus REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)

add_library(
  acrobat_common SHARED
  # Lie groups
  src/lie_groups/lie_group_conversions.cpp src/lie_groups/lie_groups.cpp)

target_include_directories(acrobat_common PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(acrobat_common "project_settings" "tf2_geometry_msgs")
target_link_libraries(acrobat_common Sophus::Sophus)

ament_export_include_directories(include)
ament_export_libraries(acrobat_common)
ament_export_dependencies(sophus tf2_geometry_msgs)
ament_export_interfaces(export_acrobat_common HAS_LIBRARY_TARGET)
install(
  TARGETS acrobat_common
  EXPORT export_acrobat_common
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
  ament_clang_format(TESTNAME acrobat_common_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})

  ament_add_gtest(test_lie_group_conversions test/test_lie_group_conversions.cpp)
  set_target_properties(test_lie_group_conversions PROPERTIES CXX_CLANG_TIDY "" CMAKE_CXX_CPPCHECK "")
  target_link_libraries(test_lie_group_conversions acrobat_common)
endif()

ament_package()
