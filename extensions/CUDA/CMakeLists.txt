cmake_minimum_required(VERSION 3.12)
project(SoftRobots.CUDA)

set(HEADER_FILES
    src/SoftRobots.CUDA/init.h
    src/SoftRobots.CUDA/config.h.in
)

set(SOURCE_FILES
    src/SoftRobots.CUDA/init.cpp

    src/SoftRobots.CUDA/component/forcefield/CUDAPartialRigidificationForceField.cpp
    src/SoftRobots.CUDA/component/forcefield/CUDAPipeForceField.cpp
    src/SoftRobots.CUDA/component/forcefield/CUDAPREquivalentStiffnessForceField.cpp
)

sofa_find_package(SoftRobots REQUIRED)
sofa_find_package(SofaCUDA REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SoftRobots)
target_link_libraries(${PROJECT_NAME} SofaCUDA)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
