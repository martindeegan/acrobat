cmake_minimum_required(VERSION 3.5)
project(arducam)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(project_settings REQUIRED)
find_package(vision_opencv REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(arducam_driver SHARED
  src/arducam_driver.cpp
)
target_link_libraries(arducam_driver
  ArduCamLib
  usb-1.0
  pthread
  arducam_config_parser
)
target_compile_definitions(arducam_driver PRIVATE
  "COMPOSITION_BUILDING_DLL"
)
set_target_properties(arducam_driver PROPERTIES CXX_CLANG_TIDY "" CMAKE_CXX_CPPCHECK "")
ament_target_dependencies(arducam_driver
  "project_settings"
  "sensor_msgs"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "vision_opencv"
  "cv_bridge"
)
rclcpp_components_register_nodes(arducam_driver "ArducamDriver")
set(node_plugins "${node_plugins}ArducamDriver;$<TARGET_FILE:arducam_driver>\n")

add_library(image_viewer SHARED
  src/image_viewer.cpp
)
target_compile_definitions(image_viewer PRIVATE
  "COMPOSITION_BUILDING_DLL"
)
set_target_properties(image_viewer PROPERTIES CXX_CLANG_TIDY "" CMAKE_CXX_CPPCHECK "")
ament_target_dependencies(image_viewer
  "project_settings"
  "sensor_msgs"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "vision_opencv"
  "cv_bridge"
)
rclcpp_components_register_nodes(image_viewer "ImageViewer")
set(node_plugins "${node_plugins}ImageViewer;$<TARGET_FILE:image_viewer>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")


add_executable(manual_arducam
  src/manual_arducam.cpp
)
target_link_libraries(manual_arducam
  arducam_driver
  image_viewer
)
ament_target_dependencies(manual_arducam
  "project_settings"
  "rclcpp"
)

# add_executable(dlopen_composition
#   src/dlopen_composition.cpp
# )
# target_link_libraries(dlopen_composition
#   arducam_driver
#   image_viewer
# )
# ament_target_dependencies(dlopen_composition
#   "class_loader"
#   "rclcpp"
#   "rclcpp_components"
#   "project_settings"
# )

install(TARGETS arducam_driver image_viewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  manual_arducam
  # dlopen_composition
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
