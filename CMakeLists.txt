cmake_minimum_required(VERSION 3.0)

# Force cmake to use the NEW policy regaringthe "VERSION" management.
cmake_policy(SET CMP0048 NEW)

project(SoftRobots VERSION 1.0)

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


find_package(SofaPython QUIET)
if(SofaPython_FOUND)
    set(SOFTROBOTS_PYTHON ${SofaPython_FOUND}) # config.h.in
endif()

find_package(STLIB QUIET)
if(STLIB_FOUND)
    message("-- Found dependency : 'STLIB' plugin .")
else()
    message("-- The higly recommanded 'STLIB' plugin is missing. You can compile SoftRobots but most of the provided examples will not work. ")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")


set(HEADER_FILES
#    component/initSoftRobots.h
        )

set(SOURCE_FILES
    component/initSoftRobots.cpp
        )

# initSoftRobots.h
configure_file(component/initSoftRobots.h.in "${CMAKE_BINARY_DIR}/include/SoftRobots/initSoftRobots.h")
install(FILES "${CMAKE_BINARY_DIR}/include/SoftRobots/initSoftRobots.h" DESTINATION "include/SoftRobots" COMPONENT headers)

file(GLOB_RECURSE RESOURCE_FILES  "*.md" "*.psl" "*.py" "*.pyscn" "*.scn" "*.ah")

include(cmake/environment.cmake)
include(cmake/utils.cmake)

include(component/behavior/Behavior_Files.cmake)
include(component/constraint/Constraint_Files.cmake)
include(component/controller/Controller_Files.cmake)
include(component/engine/Engine_Files.cmake)
include(component/forcefield/Forcefield_Files.cmake)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${DOC_FILES} ${RESOURCE_FILES})

include(component/behavior/Behavior_Links.cmake)
include(component/constraint/Constraint_Links.cmake)
include(component/controller/Controller_Links.cmake)
include(component/engine/Engine_Links.cmake)
include(component/forcefield/Forcefield_Links.cmake)

target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")

find_package(SofaFramework REQUIRED)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SOFTROBOTS")
target_link_libraries(${PROJECT_NAME} SofaCore SofaConstraint SofaBaseMechanics SofaUserInteraction)

if(SofaPython_FOUND)
    target_link_libraries(${PROJECT_NAME} SofaPython)
    # Config files and install rules for pythons scripts
    sofa_set_python_directory(${PROJECT_NAME} "python")
endif()

## Install rules for the library and headers; CMake package configurations files
sofa_create_package(SoftRobots ${PROJECT_VERSION} ${PROJECT_NAME} SoftRobots)

install(DIRECTORY docs DESTINATION share/sofa/${PROJECT_NAME})
install(FILES "README.md" "LICENSE" "CHANGELOG" DESTINATION .)

if(SOFA_BUILD_TESTS)
    add_subdirectory(tests)
endif()


#######################
# CPack configuration #
#######################
set(CPACK_GENERATOR ZIP)
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME} v${CPACK_PACKAGE_VERSION}")
set(CPACK_PACKAGE_VENDOR "Defrost team")
set(CPACK_PACKAGE_CONTACT "https://project.inria.fr/softrobot/contact/")
set(CPACK_PACKAGE_DESCRIPTION "Ease the modeling, the simulation and the control of soft-robots in SOFA.")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://project.inria.fr/softrobot")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}_v${CPACK_PACKAGE_VERSION}")
if(WIN32)
#    set(CPACK_PACKAGE_INSTALL_DIRECTORY "SOFA\\\\v${CPACK_PACKAGE_VERSION}\\\\plugins\\\\${PROJECT_NAME}")
    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "AMD64")
        set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}_Win64")
    else()
        set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}_Win32")
    endif()
elseif(UNIX)
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "SOFA/v${CPACK_PACKAGE_VERSION}/plugis/${PROJECT_NAME}")
    if(APPLE)
        set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}_MacOS")
    else()
        set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}_Linux")
    endif()
endif()
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_SOURCE_DIR}/README.md")

include(CPack)
