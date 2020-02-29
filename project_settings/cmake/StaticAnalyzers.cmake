option(ENABLE_CPPCHECK "Enable static analysis with cppcheck" ON)
option(ENABLE_CLANG_TIDY "Enable static analysis with clang-tidy" ON)
if(ENABLE_CPPCHECK)
  find_program(CPPCHECK cppcheck)
  if(CPPCHECK)
    set(CMAKE_CXX_CPPCHECK
        ${CPPCHECK}
        --suppress=missingInclude
        --enable=all
        --inconclusive
        -i
        ${CMAKE_SOURCE_DIR}/test
        --suppress=*:/opt/ros/eloquent/src/gtest_vendor/include/gtest/internal/gtest-port.h
        --quiet)
  else()
    message(SEND_ERROR "cppcheck requested but executable not found")
  endif()
endif()

if(ENABLE_CLANG_TIDY)
  find_program(CLANGTIDY clang-tidy)
  if(CLANGTIDY)
    set(CMAKE_CXX_CLANG_TIDY ${CLANGTIDY})
  else()
    message(SEND_ERROR "clang-tidy requested but executable not found")
  endif()
endif()
