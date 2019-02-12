cmake_minimum_required(VERSION 3.0)

# Force cmake to use the NEW policy regaringthe "VERSION" management.
cmake_policy(SET CMP0048 NEW)

project(SoftRobots VERSION 1.0)

find_package(SofaFramework REQUIRED)

option(SOFTROBOTS_IGNORE_ERRORS "Enable this option to ignore the recommendations and generate anyway." OFF)

if(SOFA_USE_MASK AND NOT SOFTROBOTS_IGNORE_ERRORS)
    message(SEND_ERROR "
        You should not use the plugin SoftRobots with SOFA_USE_MASK enabled.
        Most components will not work correctly.
        To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
endif()

if(NOT SOFA_BUILD_METIS AND NOT SOFTROBOTS_IGNORE_ERRORS)
    message(SEND_ERROR "
        You should not use the plugin SoftRobots without enabling SOFA_BUILD_METIS.
        Most examples will run slowly.
        To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
endif()

if(NOT PLUGIN_SOFAPYTHON AND NOT SOFTROBOTS_IGNORE_ERRORS)
    message(SEND_ERROR "
        You should not use the plugin SoftRobots without enabling PLUGIN_SOFAPYTHON.
        Most examples will not be loadable.
        To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
endif()

if(NOT MODULE_SOFASPARSESOLVER AND NOT SOFTROBOTS_IGNORE_ERRORS)
    message(SEND_ERROR "
        You should not use the plugin SoftRobots without enabling MODULE_SOFASPARSESOLVER.
        Most examples and some tests will not work.
        To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
endif()



find_package(STLIB QUIET)
if(STLIB_FOUND)
    message("-- Found dependency : 'STLIB' plugin .")
else()
    message("-- The higly recommanded 'STLIB' plugin is missing. You can compile SoftRobots but most of the provided examples will not work. ")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

set(SOFTROBOTS_SOURCE_DIR src/SoftRobots)

set(HEADER_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/initSoftRobots.h
        )

set(SOURCE_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/initSoftRobots.cpp
        )

file(GLOB_RECURSE RESOURCE_FILES  "*.md" "*.psl" "*.py" "*.pyscn" "*.scn" "*.ah")

include(cmake/environment.cmake)
include(cmake/utils.cmake)

include(${SOFTROBOTS_SOURCE_DIR}/component/behavior/Behavior_Files.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/constraint/Constraint_Files.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/controller/Controller_Files.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/engine/Engine_Files.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/forcefield/Forcefield_Files.cmake)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${DOC_FILES} ${RESOURCE_FILES})

include(${SOFTROBOTS_SOURCE_DIR}/component/behavior/Behavior_Links.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/constraint/Constraint_Links.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/controller/Controller_Links.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/engine/Engine_Links.cmake)
include(${SOFTROBOTS_SOURCE_DIR}/component/forcefield/Forcefield_Links.cmake)


set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

find_package(SofaFramework REQUIRED)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SOFTROBOTS")
target_link_libraries(${PROJECT_NAME} SofaCore SofaConstraint SofaBaseMechanics SofaUserInteraction)

# Config files and install rules for pythons scripts
sofa_set_python_directory(${PROJECT_NAME} "python")

## Install rules for the library and headers; CMake package configurations files
sofa_generate_package(NAME SoftRobots VERSION ${PROJECT_VERSION} TARGETS ${PROJECT_NAME} INCLUDE_ROOT_DIR SoftRobots)

if(SOFA_BUILD_TESTS)
    add_subdirectory(tests)
endif()


