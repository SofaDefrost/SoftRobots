cmake_minimum_required(VERSION 3.12)
project(SoftRobots_test VERSION 1.0)
find_package(Sofa.Testing REQUIRED)
sofa_find_package(Sofa.SimpleApi REQUIRED)

set(SOURCE_FILES
    component/constraint/SurfacePressureConstraintTest.cpp
    component/constraint/UnilateralPlaneConstraintTest.cpp
    
    component/controller/AnimationEditorTest.cpp
    component/controller/DataVariationLimiterTest.cpp
    component/controller/SerialPortBridgeGenericTest.cpp
    )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Sofa.Testing SoftRobots Sofa.SimpleApi)
add_definitions("-DSOFTROBOTS_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

