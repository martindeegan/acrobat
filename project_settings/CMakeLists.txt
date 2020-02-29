cmake_minimum_required(VERSION 3.15)
project(project_settings CXX)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_settings INTERFACE)
target_compile_features(
  project_settings INTERFACE cxx_std_17 cxx_auto_type cxx_nullptr
                             cxx_right_angle_brackets)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_settings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_settings)

# enable doxygen
include(cmake/Doxygen.cmake)
enable_doxygen()

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

# Very basic PCH example
option(ENABLE_PCH "Enable Precompiled Headers" ON)
if(ENABLE_PCH)
  # This sets a global PCH parameter, each project will build its own PCH, which
  # is a good idea if any #define's change
  target_precompile_headers(project_settings INTERFACE <vector> <string> <map>
                            <utility>)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_export_interfaces(export_project_settings HAS_LIBRARY_TARGET)
install(
  TARGETS project_settings
  EXPORT export_project_settings
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)

# Install clang-tidy, clang-format, and cmake-format configurations
install(DIRECTORY config DESTINATION share/lint_config)

ament_package()
