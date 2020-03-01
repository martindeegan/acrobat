find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_clang_format REQUIRED)

# Get link config files
symlink_link_configs()

find_file(CLANG_FORMAT_CONFIG PATHS ${CMAKE_INSTALL_PREFIX}/..)
ament_clang_format(TESTNAME acrobat_common_clang_format_test CONFIG_FILE ${CLANG_FORMAT_CONFIG})

ament_add_gtest(test_lie_group_conversions test_lie_group_conversions.cpp)
set_target_properties(test_lie_group_conversions PROPERTIES CXX_CLANG_TIDY "" CMAKE_CXX_CPPCHECK "")
target_link_libraries(test_lie_group_conversions acrobat_common)
