cmake_minimum_required(VERSION 3.5)
# Force cmake to use the NEW policy regaringthe "VERSION" management.
cmake_policy(SET CMP0048 NEW)

project(SoftRobots.Inverse_test VERSION 1.0)
find_package(Sofa.Testing REQUIRED)

set(SOURCE_FILES )
set(HEADER_FILES )

include(component/constraint/ConstraintTest.cmake)
include(component/solver/SolverTest.cmake)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${RC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE "${SoftRobots_INCLUDE_DIRS}")

target_link_libraries(${PROJECT_NAME} Sofa.Testing SoftRobots.Inverse)
add_definitions("-DSOFTROBOTSINVERSE_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

