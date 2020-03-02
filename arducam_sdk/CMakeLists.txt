cmake_minimum_required(VERSION 3.15)
project(arducam_sdk)


# find dependencies
find_package(ament_cmake REQUIRED)

# Find architecture and create the symlink to shared library file
execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/x86_64/lib/libArduCamLib.so ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  file(CREATE_LINK ${PROJECT_SOURCE_DIR}/arm64/lib/libArduCamLib.so ${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so)
endif()

add_library(arducam_sdk SHARED IMPORTED)
target_include_directories(arducam_sdk INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")
set_target_properties(arducam_sdk PROPERTIES IMPORTED_LOCATION "${CMAKE_INSTALL_PREFIX}/lib/libArduCamLib.so")

ament_export_include_directories(include)
ament_export_libraries(arducam_sdk)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
