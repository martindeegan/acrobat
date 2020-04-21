# Want all projects to be able to find the lint config files
include("${project_settings_DIR}/InstallLintConfigs.cmake")
include("${project_settings_DIR}/StaticAnalyzers.cmake")
include("${project_settings_DIR}/StandardProjectSettings.cmake")

# We want all projects to export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
