cmake_minimum_required(VERSION 3.5)
project(SoftRobots.Inverse VERSION 1.0)

# CMAKE_PREFIX_PATH must contain:
# - SOFA install dir
# - SoftRobots install dir
# - STLIB install dir

include(cmake/environment.cmake)

find_package(Sofa.Config REQUIRED)
sofa_find_package(SoftRobots REQUIRED)
sofa_find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
sofa_find_package(Sofa.Component.Collision.Response.Contact REQUIRED)

set(SRC_DIR "src/SoftRobots.Inverse")

set(HEADER_FILES
    ${SRC_DIR}/component/config.h.in

    # EFFECTOR
    ${SRC_DIR}/component/behavior/Effector.h
    ${SRC_DIR}/component/behavior/Effector.inl

    ${SRC_DIR}/component/constraint/BarycentricCenterEffector.h
    ${SRC_DIR}/component/constraint/BarycentricCenterEffector.inl
    ${SRC_DIR}/component/constraint/CableEffector.h
    ${SRC_DIR}/component/constraint/CableEffector.inl
    ${SRC_DIR}/component/constraint/PositionEffector.h
    ${SRC_DIR}/component/constraint/PositionEffector.inl
    ${SRC_DIR}/component/constraint/VolumeEffector.h
    ${SRC_DIR}/component/constraint/VolumeEffector.inl

    # ACTUATOR
    ${SRC_DIR}/component/behavior/Actuator.h
    ${SRC_DIR}/component/behavior/Actuator.inl

    ${SRC_DIR}/component/constraint/YoungModulusActuator.h
    ${SRC_DIR}/component/constraint/YoungModulusActuator.inl
    ${SRC_DIR}/component/constraint/CableActuator.h
    ${SRC_DIR}/component/constraint/CableActuator.inl
    ${SRC_DIR}/component/constraint/ForceSurfaceActuator.h
    ${SRC_DIR}/component/constraint/ForceSurfaceActuator.inl
    ${SRC_DIR}/component/constraint/ForcePointActuator.h
    ${SRC_DIR}/component/constraint/ForcePointActuator.inl
    ${SRC_DIR}/component/constraint/JointActuator.h
    ${SRC_DIR}/component/constraint/JointActuator.inl
    ${SRC_DIR}/component/constraint/SlidingActuator.h
    ${SRC_DIR}/component/constraint/SlidingActuator.inl
    ${SRC_DIR}/component/constraint/SurfacePressureActuator.h
    ${SRC_DIR}/component/constraint/SurfacePressureActuator.inl

    # SENSOR
    ${SRC_DIR}/component/behavior/Sensor.h
    ${SRC_DIR}/component/behavior/Sensor.inl

    ${SRC_DIR}/component/constraint/CableSensor.h
    ${SRC_DIR}/component/constraint/CableSensor.inl
    ${SRC_DIR}/component/constraint/SurfacePressureSensor.h
    ${SRC_DIR}/component/constraint/SurfacePressureSensor.inl

    # EQUALITY
    ${SRC_DIR}/component/behavior/Equality.h
    ${SRC_DIR}/component/behavior/Equality.inl

    ${SRC_DIR}/component/constraint/CableEquality.h
    ${SRC_DIR}/component/constraint/CableEquality.inl
    ${SRC_DIR}/component/constraint/PositionEquality.h
    ${SRC_DIR}/component/constraint/PositionEquality.inl
    ${SRC_DIR}/component/constraint/SurfacePressureEquality.h
    ${SRC_DIR}/component/constraint/SurfacePressureEquality.inl

    ${SRC_DIR}/component/solver/QPInverseProblemSolver.h
    ${SRC_DIR}/component/solver/modules/ContactHandler.h
    ${SRC_DIR}/component/solver/modules/ConstraintHandler.h
    ${SRC_DIR}/component/solver/modules/LCPQPSolver.h
    ${SRC_DIR}/component/solver/modules/NLCPSolver.h
    ${SRC_DIR}/component/solver/modules/QPInverseProblem.h
    ${SRC_DIR}/component/solver/modules/QPInverseProblemImpl.h
    ${SRC_DIR}/component/solver/modules/QPMechanicalAccumulateConstraint.h
    ${SRC_DIR}/component/solver/modules/QPMechanicalSetConstraint.h
    )
set(SOURCE_FILES
    ${SRC_DIR}/component/initSoftRobotsInverse.cpp

    # EFFECTOR
    ${SRC_DIR}/component/behavior/Effector.cpp

    ${SRC_DIR}/component/constraint/BarycentricCenterEffector.cpp
    ${SRC_DIR}/component/constraint/CableEffector.cpp
    ${SRC_DIR}/component/constraint/PositionEffector.cpp
    ${SRC_DIR}/component/constraint/VolumeEffector.cpp

    # ACTUATOR
    ${SRC_DIR}/component/behavior/Actuator.cpp

    ${SRC_DIR}/component/constraint/YoungModulusActuator.cpp
    ${SRC_DIR}/component/constraint/CableActuator.cpp
    ${SRC_DIR}/component/constraint/ForceSurfaceActuator.cpp
    ${SRC_DIR}/component/constraint/ForcePointActuator.cpp
    ${SRC_DIR}/component/constraint/JointActuator.cpp
    ${SRC_DIR}/component/constraint/SlidingActuator.cpp
    ${SRC_DIR}/component/constraint/SurfacePressureActuator.cpp

    # SENSOR
    ${SRC_DIR}/component/behavior/Sensor.cpp

    ${SRC_DIR}/component/constraint/CableSensor.cpp
    ${SRC_DIR}/component/constraint/SurfacePressureSensor.cpp

    # EQUALITY
    ${SRC_DIR}/component/behavior/Equality.cpp

    ${SRC_DIR}/component/constraint/CableEquality.cpp
    ${SRC_DIR}/component/constraint/PositionEquality.cpp
    ${SRC_DIR}/component/constraint/SurfacePressureEquality.cpp

    ${SRC_DIR}/component/solver/QPInverseProblemSolver.cpp
    ${SRC_DIR}/component/solver/modules/ContactHandler.cpp
    ${SRC_DIR}/component/solver/modules/ConstraintHandler.cpp
    ${SRC_DIR}/component/solver/modules/LCPQPSolver.cpp
    ${SRC_DIR}/component/solver/modules/NLCPSolver.cpp
    ${SRC_DIR}/component/solver/modules/QPInverseProblem.cpp
    ${SRC_DIR}/component/solver/modules/QPInverseProblemImpl.cpp
    ${SRC_DIR}/component/solver/modules/QPMechanicalAccumulateConstraint.cpp
    ${SRC_DIR}/component/solver/modules/QPMechanicalSetConstraint.cpp
    )

if(SOFA-DEVPLUGIN_BEAMADAPTER)
    list(APPEND HEADER_FILES
        ${SRC_DIR}/component/constraint/BeamRestPositionActuator.h
        ${SRC_DIR}/component/constraint/BeamRestPositionActuator.inl
        )
    list(APPEND SOURCE_FILES
        ${SRC_DIR}/component/constraint/BeamRestPositionActuator.cpp
        )
endif()

set(DOC_FILES README.md)
file(GLOB_RECURSE EXAMPLE_FILES examples "*.pyscn" "*.py" "*.md" "*.psl" "*.pslx" "*.scn" "*.xml")

option(SOFTROBOTSINVERSE_INSTALL_HEADERS "Install the headers" ON)
if(SOFTROBOTSINVERSE_INSTALL_HEADERS)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${DOC_FILES} ${EXAMPLE_FILES} ${HEADER_FILES})
else()
    add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${DOC_FILES} ${EXAMPLE_FILES} "${SRC_DIR}/component/config.h.in")
endif()

find_package(qpOASES)
if(qpOASES_FOUND)
  include_directories(${qpOASES_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${qpOASES_LIBRARIES})
else()
  message("External package qpOASES not found, using embedded version...")
  set(OASES_LIBRARY_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/extlibs/qpOASES-3.2.0/")
  add_subdirectory(${OASES_LIBRARY_DIRECTORY} extlibs/libqpOASES)
  find_package(libqpOASES REQUIRED)
  include_directories(${libqpOASES_INCLUDE_DIRS})
  sofa_install_libraries(PATHS ${libqpOASES_LIBRARY})
  target_link_libraries(${PROJECT_NAME} ${libqpOASES_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME}
    SoftRobots
    Sofa.Component.Constraint.Lagrangian.Solver
    Sofa.Component.Collision.Response.Contact)

# TODO: remove this when SoftRobotsConfig.cmake.in is fixed
message("SOFTROBOTS_HAVE_SOFA_GL = ${SOFTROBOTS_HAVE_SOFA_GL}")
if(SOFTROBOTS_HAVE_SOFA_GL)
    find_package(Sofa.GL REQUIRED)
    target_link_libraries(${PROJECT_NAME} Sofa.GL)
endif()

# Sofa.Component.Compat should permit to avoid this
find_package(Sofa.Component.SolidMechanics.FEM.Elastic)
target_link_libraries(${PROJECT_NAME} Sofa.Component.SolidMechanics.FEM.Elastic)

if(SOFA_HAVE_NEWMAT)
    target_link_libraries(${PROJECT_NAME} SofaDenseSolver)
endif()

## Install rules for the library and headers; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

install(DIRECTORY docs DESTINATION .)
install(FILES "README.md" "LICENSE.AGPLV3.txt" "LICENSE.AGPLV3.Clause11.txt" "CHANGELOG" DESTINATION .)

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
cmake_dependent_option(SOFTROBOTSINVERSE_BUILD_TESTS "Compile the automatic tests" ON "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
if(SOFTROBOTSINVERSE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

include(cmake/packaging.cmake)
