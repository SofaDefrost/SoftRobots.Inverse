##
##     This file is part of qpOASES.
##
##     qpOASES -- An Implementation of the Online Active Set Strategy.
##     Copyright (C) 2007-2015 by Hans Joachim Ferreau, Andreas Potschka,
##     Christian Kirches et al. All rights reserved.
##
##     qpOASES is free software; you can redistribute it and/or
##     modify it under the terms of the GNU Lesser General Public
##     License as published by the Free Software Foundation; either
##     version 2.1 of the License, or (at your option) any later version.
##
##     qpOASES is distributed in the hope that it will be useful,
##     but WITHOUT ANY WARRANTY; without even the implied warranty of
##     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##     See the GNU Lesser General Public License for more details.
##
##     You should have received a copy of the GNU Lesser General Public
##     License along with qpOASES; if not, write to the Free Software
##     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##     Filename:  CMakeLists.txt
##     Author:    Hans Joachim Ferreau (thanks to Milan Vukov)
##     Version:   3.2
##     Date:      2007-2015
##

cmake_minimum_required(VERSION 3.12)

project(libqpOASES VERSION 1.0)
SET(PACKAGE_NAME "libqpOASES")
SET(PACKAGE_VERSION "3.2.0")
SET(PACKAGE_SO_VERSION "3.2")
SET(PACKAGE_DESCRIPTION "An implementation of the online active set strategy")
SET(PACKAGE_AUTHOR "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
SET(PACKAGE_MAINTAINER "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
SET(PACKAGE_URL "https://projects.coin-or.org/qpOASES")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)

IF( NOT CMAKE_VERBOSE_MAKEFILE )
    SET( CMAKE_VERBOSE_MAKEFILE OFF )
ENDIF( NOT CMAKE_VERBOSE_MAKEFILE )

IF( NOT CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE
        )
ENDIF( NOT CMAKE_BUILD_TYPE )


############################################################
#################### compiler flags ########################
############################################################
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
IF ( UNIX )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -O3 -finline-functions")
ELSEIF( WINDOWS )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32")
ENDIF()

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__DEBUG__")

############################################################
######################## rpath #############################
############################################################
# use, i.e. don't skip the full RPATH for the build tree
#set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
#set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

#set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/casadi")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/casadi")
endif("${isSystemDir}" STREQUAL "-1")



############################################################
#################### build and install #####################
############################################################
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# compile qpOASES libraries
FILE(GLOB SRC src/*.cpp)
FILE(GLOB_RECURSE HEADERS include/*.hpp)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# library
# ADD_LIBRARY(qpOASES STATIC ${SRC} ${HEADERS})
ADD_LIBRARY(libqpOASES SHARED ${SRC} ${HEADERS})
INSTALL(TARGETS libqpOASES
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)
SET_TARGET_PROPERTIES(
    libqpOASES
    PROPERTIES
    SOVERSION ${PACKAGE_SO_VERSION}
    )

# headers
INSTALL(FILES include/qpOASES.hpp
  DESTINATION include)
INSTALL(DIRECTORY include/qpOASES
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*.ipp"
  PATTERN ".svn" EXCLUDE)

sofa_create_package_with_targets(
        PACKAGE_NAME ${PROJECT_NAME}
        PACKAGE_VERSION ${PACKAGE_VERSION}
        TARGETS ${PROJECT_NAME}
        INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
        )

############################################################
######################### examples #########################
############################################################
# compile qpOASES examples
# SET(EXAMPLE_NAMES
    # example1
    # example1a
    # example1b
    # example2
    # example3
    # example3b
    # example4
    # example5
    # exampleLP
    # qrecipe
	# qrecipeSchur
# )

# FOREACH(ELEMENT ${EXAMPLE_NAMES})
    # ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
    # TARGET_LINK_LIBRARIES(${ELEMENT} qpOASES)
# ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})


##
##   end of file
##
