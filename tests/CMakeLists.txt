cmake_minimum_required(VERSION 3.5)
# Force cmake to use the NEW policy regaringthe "VERSION" management.
cmake_policy(SET CMP0048 NEW)

project(SoftRobots_test VERSION 1.0)

set(SOURCE_FILES)
set(HEADER_FILES)

include(component/constraint/ConstraintTest.cmake)
include(component/controller/ControllerTest.cmake)
include(component/engine/EngineTest.cmake)
include(component/forcefield/ForcefieldTest.cmake)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${RC_FILES})

find_package(SofaFramework REQUIRED)
find_package(SofaCommon REQUIRED)
find_package(SofaTest REQUIRED)
find_package(SoftRobots REQUIRED)

target_include_directories(${PROJECT_NAME} PUBLIC "${SoftRobots_INCLUDE_DIRS}")

target_link_libraries(${PROJECT_NAME} SofaTest SofaGTestMain SoftRobots SofaCore SofaConstraint SofaBaseMechanics SofaUserInteraction SofaComponentCommon)
add_definitions("-DSOFTROBOTS_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

