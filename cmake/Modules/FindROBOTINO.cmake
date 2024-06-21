# This module defines the following variables:
#  ROBOTINO_INCLUDE_DIR - include directories for robotino API2
#  ROBOTINO_LIBRARY - libraries to link against robotino API2
#  ROBOTINO_FOUND - true if robotino API2 has been found and can be used

# HIGHLY TEMPORARY !!!!
unset(ROBOTINO_LIBRARY CACHE)

Message(STATUS "findROBOTINO.cmake is temporarily hardcoded to find the lib in /usr/local/robotino/api2/")
set(ROBOTINO_INCLUDE_DIR "/usr/local/robotino/api2/include")
#unset(ROBOTINO_LIBRARY)
find_library(ROBOTINO_LIBRARY NAMES rec_robotino_api2 PATHS /usr/local/robotino/api2/lib)
if(NOT ROBOTINO_LIBRARY)
    set(ROBOTINO_FOUND FALSE)
    Message(FATAL_ERROR "API2 is not properly installed. Easy install from .deb package at http://wiki.openrobotino.org/index.php?title=Debrepository")
else(NOT ROBOTINO_LIBRARY)
    set(ROBOTINO_FOUND TRUE)
endif(NOT ROBOTINO_LIBRARY)

mark_as_advanced(ROBOTINO_INCLUDE_DIR ROBOTINO_LIBRARY)
