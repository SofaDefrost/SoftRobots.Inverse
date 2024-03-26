# Find the qpOASES includes and libraries.
# The following variables are set if qpOASES is found.  If qpOASES is not
# found, qpOASES_FOUND is set to false.
#  qpOASES_FOUND        - True when the qpOASES include directory is found.
#  qpOASES_INCLUDE_DIRS - the path to where the qpOASES include files are.
#  qpOASES_LIBRARIES    - The libraries to link against qpOASES

include(FindPackageHandleStandardArgs)

find_path(qpOASES_INCLUDE_DIR
  NAMES qpOASES.hpp
  PATH_SUFFIXES include
)

find_library(qpOASES_LIBRARY
  NAMES qpOASES
  PATH_SUFFIXES lib
)

set(qpOASES_INCLUDE_DIRS ${qpOASES_INCLUDE_DIR})
set(qpOASES_LIBRARIES ${qpOASES_LIBRARY})

find_package_handle_standard_args(qpOASES DEFAULT_MSG qpOASES_LIBRARIES qpOASES_INCLUDE_DIRS)
