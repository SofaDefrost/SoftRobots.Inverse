cmake_minimum_required(VERSION 3.12)
project(SoftRobots.Inverse_test VERSION 1.0)
find_package(Sofa.Testing REQUIRED)
sofa_find_package(Sofa.SimpleApi REQUIRED)

set(SOURCE_FILES )
set(HEADER_FILES )

include(component/constraint/ConstraintTest.cmake)
# include(component/solver/SolverTest.cmake)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Sofa.Testing SoftRobots.Inverse Sofa.SimpleApi)
add_definitions("-DSOFTROBOTSINVERSE_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

