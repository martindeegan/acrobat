cmake_minimum_required(VERSION 3.15)
project(acrobat)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)
find_package(arducam REQUIRED)
find_package(msp_bridge REQUIRED)

add_executable(acrobat_executor src/acrobat_executor.cpp)
ament_target_dependencies(acrobat_executor "project_settings" "rclcpp" "arducam" "msp_bridge")

install(TARGETS acrobat_executor RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
