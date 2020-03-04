# Locate LibSerial
#
# This module defines
#  LIBSERIAL_FOUND, if false, do not try to link to LibSerial
#  LIBSERIAL_LIBRARY, where to find LibSerial
#  LIBSERIAL_INCLUDE_DIR, where to find SerialPort.h

# find the LibSerial include directory
find_path(libserial_INCLUDE_DIR 
          NAMES libserial/SerialPortConstants.h
          PATH_SUFFIXES include
          PATHS /usr /usr/local)

# find the LibSerial library
find_library(libserial_LIBRARY
             NAMES libserial.so
             PATH_SUFFIXES lib
             PATHS /usr /usr/local)

add_library(libserial INTERFACE)
target_include_directories(libserial INTERFACE ${libserial_INCLUDE_DIR})
target_link_libraries(libserial INTERFACE ${libserial_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set LIBSERIAL_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBSERIAL DEFAULT_MSG libserial_INCLUDE_DIR libserial_LIBRARY)
mark_as_advanced(libserial libserial_INCLUDE_DIR libserial_LIBRARY)
