cmake_minimum_required(VERSION 3.1)
project(SoftRobots VERSION 1.0)

include(cmake/environment.cmake)

find_package(SofaFramework REQUIRED)
find_package(SofaCommon REQUIRED) # SofaRigid
find_package(SofaMisc REQUIRED) # SofaMiscMapping
find_package(SofaSparseSolver REQUIRED)

option(SOFTROBOTS_IGNORE_ERRORS "Enable this option to ignore the recommendations and generate anyway." OFF)
if(NOT SOFTROBOTS_IGNORE_ERRORS)
    if(SOFA_USE_MASK)
        message(SEND_ERROR "
            You should not use the plugin SoftRobots with SOFA_USE_MASK enabled.
            Most components will not work correctly.
            To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
    endif()
    if(NOT PLUGIN_SOFAPYTHON AND NOT PLUGIN_SOFAPYTHON3)
        message(SEND_ERROR "
            You should not use the plugin SoftRobots without enabling PLUGIN_SOFAPYTHON or PLUGIN_SOFAPYTHON3.
            Most examples and tutorials will not be loadable.
            To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
    endif()
    if(NOT MODULE_SOFASPARSESOLVER)
        message(SEND_ERROR "
            You should not use the plugin SoftRobots without enabling MODULE_SOFASPARSESOLVER.
            Most examples and some tests will not work.
            To fix this error you can either follow the recommendation or enable SOFTROBOTS_IGNORE_ERRORS.")
    endif()
endif()

option(SOFTROBOTS_COMMUNICATIONCONTROLLER "Enables Communication component. It requires ZeroMQ library (see controller/README for more info)" OFF)
option(SOFTROBOTS_GAMETRAKCONTROLLER "Enables GameTrak component. It requires HIDAPI library (see controller/README for more info)" OFF)
option(SOFTROBOTS_ROBOTINOCONTROLLER "Enables Robotino component" OFF)
option(SOFTROBOTS_WITH_CAMERA "Enable this option if you want to use the realsense cameras to perform force sensing, and point cloud processing." OFF)

find_package(SofaPython QUIET)
set(SOFTROBOTS_PYTHON ${SofaPython_FOUND}) # config.h.in

find_package(Qt5 REQUIRED COMPONENTS Network)
if(WIN32)
     sofa_copy_libraries(TARGETS Qt5::Network)
endif()
sofa_install_libraries(TARGETS Qt5::Network)

find_package(STLIB QUIET)
if(STLIB_FOUND)
    message("-- Found dependency : 'STLIB' plugin .")
else()
    message("-- The higly recommanded 'STLIB' plugin is missing. You can compile SoftRobots but most of the provided examples will not work. ")
endif()

set(SOFTROBOTS_SOURCE_DIR src/SoftRobots)

set(HEADER_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/initSoftRobots.h.in

    ${SOFTROBOTS_SOURCE_DIR}/component/behavior/SoftRobotsBaseConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/behavior/SoftRobotsConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/behavior/SoftRobotsConstraint.inl
    
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/CableConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/CableConstraint.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/PartialRigidificationConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/PartialRigidificationConstraint.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/SurfacePressureConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/SurfacePressureConstraint.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/UnilateralPlaneConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/UnilateralPlaneConstraint.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/AffineFunctionModel.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/AffineFunctionModel.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/CableModel.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/CableModel.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/SurfacePressureModel.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/SurfacePressureModel.inl

    ${SOFTROBOTS_SOURCE_DIR}/component/controller/AnimationEditor.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/AnimationEditor.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataVariationLimiter.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataVariationLimiter.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/InteractiveControl.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Network.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SplitFloat.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Serial.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/SerialPortBridgeGeneric.h # Communication robot/sofa

    ${SOFTROBOTS_SOURCE_DIR}/component/engine/CenterOfMass.h
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/CenterOfMass.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTriangles.h
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTriangles.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTetrahedrons.h
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTetrahedrons.inl

    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PREquivalentStiffnessForceField.h
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PREquivalentStiffnessForceField.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PartialRigidificationForceField.h
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PartialRigidificationForceField.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PipeForceField.h
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PipeForceField.inl
    )
set(SOURCE_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/initSoftRobots.cpp

    ${SOFTROBOTS_SOURCE_DIR}/component/behavior/SoftRobotsBaseConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/behavior/SoftRobotsConstraint.cpp

    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/CableConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/PartialRigidificationConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/SurfacePressureConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/UnilateralPlaneConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/AffineFunctionModel.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/CableModel.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/SurfacePressureModel.cpp

    ${SOFTROBOTS_SOURCE_DIR}/component/controller/AnimationEditor.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataVariationLimiter.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/InteractiveControl.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Network.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SplitFloat.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Serial.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/SerialPortBridgeGeneric.cpp # Communication robot/sofa

    ${SOFTROBOTS_SOURCE_DIR}/component/engine/CenterOfMass.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTriangles.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTetrahedrons.cpp

    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PREquivalentStiffnessForceField.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PartialRigidificationForceField.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PipeForceField.cpp
    )

if(WIN32)
    list(APPEND HEADER_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SysTimeWin.h
        )
    list(APPEND SOURCE_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SysTimeWin.cpp
        )
endif()

# COMMUNICATION CONTROLLER
if(SOFTROBOTS_COMMUNICATIONCONTROLLER)
    list(APPEND HEADER_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/CommunicationController.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/CommunicationController.inl
        )
    list(APPEND SOURCE_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/CommunicationController.cpp
        )
endif()

# GAMETRAK CONTROLLER
if(SOFTROBOTS_GAMETRAKCONTROLLER)
    list(APPEND HEADER_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/GameTrakController.h

        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/GameTrak.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/HIDAPIGameTrak.h

        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/OneEuroFilter.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/Quaternion.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/stdint.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/TimeStamp.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/URI.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/vecteur3d.h
        )
    list(APPEND SOURCE_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/GameTrakController.cpp

        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/GameTrak.cpp
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/HIDAPIGameTrak.cpp

        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/OneEuroFilter.cpp
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/Quaternion.cpp
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/TimeStamp.cpp
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/URI.cpp
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/libgametrak/utils/vecteur3d.cpp
        )
endif()

# ROBOTINO CONTROLLER
if(SOFTROBOTS_ROBOTINOCONTROLLER)
    list(APPEND HEADER_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataControllerRobot.h
        )
    list(APPEND SOURCE_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataControllerRobot.cpp
        )
endif()

# CAMERA
if (SOFTROBOTS_WITH_CAMERA)
  list(APPEND HEADER_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/camera/PointCloudProcessing.h
    ${SOFTROBOTS_SOURCE_DIR}/component/camera/PointCloudStreaming.h  
    )  
  list(APPEND SOURCE_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/camera/PointCloudProcessing.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/camera/PointCloudStreaming.cpp
    )
endif(SOFTROBOTS_WITH_CAMERA)

# initSoftRobots.h
configure_file(${SOFTROBOTS_SOURCE_DIR}/component/initSoftRobots.h.in "${CMAKE_BINARY_DIR}/include/SoftRobots/initSoftRobots.h")
install(FILES "${CMAKE_BINARY_DIR}/include/SoftRobots/initSoftRobots.h" DESTINATION "include/SoftRobots" COMPONENT headers)

file(GLOB_RECURSE RESOURCE_FILES  "*.md" "*.psl" "*.py" "*.pyscn" "*.scn" "*.ah")

IF(WIN32)
	add_definitions(-D_WINSOCKAPI_)
ENDIF(WIN32)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${DOC_FILES} ${RESOURCE_FILES})

# COMMUNICATION CONTROLLER
if(SOFTROBOTS_COMMUNICATIONCONTROLLER)
    find_package(ZMQ REQUIRED)
    include_directories(${ZMQ_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARY})
endif()

# ROBOTINO CONTROLLER
if(SOFTROBOTS_ROBOTINOCONTROLLER)
    find_package(ROBOTINO REQUIRED)
    include_directories(${ROBOTINO_INCLUDE_DIR})

    message(STATUS ROBOTINO_LIBRARY:${ROBOTINO_LIBRARY})
    target_link_libraries(${PROJECT_NAME} ${ROBOTINO_LIBRARY})
endif()

# GAMETRACK CONTROLLER
if(SOFTROBOTS_GAMETRAKCONTROLLER)
    find_package(HIDAPI REQUIRED)
    include_directories(${HIDAPI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${HIDAPI_LIBRARIES} ${BOOST_THREAD_LIBRARY})
endif()

# CAMERA
if (SOFTROBOTS_WITH_CAMERA)
  find_package( PCL QUIET )
  find_library( REALSENSE_LIBRARY realsense2
    DOC "The realsense2 SDK is required to compile SoftRobots with the camera module.")

  if (NOT REALSENSE_LIBRARY)
    message(FATAL_ERROR "The realsense2 SDK is REQUIRED to compile SoftRobots with the camera module. Please install it on your system")
    endif(NOT REALSENSE_LIBRARY)
  if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(${PCL_LIBRARY_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${REALSENSE_LIBRARY})
  else (PCL_FOUND)
    message( FATAL_ERROR "The Point Cloud Library (PCL) is required to compile SoftRobots with camera. Either turn off the SOFTROBOTS_WITH_CAMERA option, or provide the link to the PCL")
  endif(PCL_FOUND)

endif(SOFTROBOTS_WITH_CAMERA)


target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SOFTROBOTS")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_link_libraries(${PROJECT_NAME} SofaCore SofaConstraint SofaBaseMechanics SofaUserInteraction SofaRigid SofaMiscMapping SofaSparseSolver SofaEngine Sofa.GL)
target_link_libraries(${PROJECT_NAME} Qt5::Network)

if(SofaPython_FOUND)
    target_link_libraries(${PROJECT_NAME} SofaPython)
    # Config files and install rules for pythons scripts
    sofa_install_pythonscripts(PLUGIN_NAME ${PROJECT_NAME} PYTHONSCRIPTS_SOURCE_DIR "python")
endif()

if(SofaPython3_FOUND)
    # Config files and install rules for pythons scripts
    sofa_install_pythonscripts(PLUGIN_NAME ${PROJECT_NAME} PYTHONSCRIPTS_SOURCE_DIR "python3")
endif()

## Install rules for the library and headers; CMake package configurations files
sofa_generate_package(
    NAME SoftRobots
    VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME}
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

install(DIRECTORY docs DESTINATION .)
install(FILES "README.md" "LICENSE" "CHANGELOG" DESTINATION .)

if(SOFA_BUILD_TESTS)
    add_subdirectory(tests)
endif()

include(cmake/packaging.cmake)
