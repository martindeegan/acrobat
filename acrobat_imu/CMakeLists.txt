cmake_minimum_required(VERSION 3.5)
project(acrobat_imu)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# find project dependencies
find_package(project_settings REQUIRED)

add_executable(acrobat_imu src/acrobat_imu.cpp)
ament_target_dependencies(acrobat_imu "rclcpp" "project_settings" "sensor_msgs")
target_link_libraries(acrobat_imu RTIMULib)


install(
  TARGETS acrobat_imu
  EXPORT export_acrobat_imu
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)



ament_package()
