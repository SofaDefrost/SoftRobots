find_package(SofaFramework) # to include SofaMacros

#COMMUNICATION CONTROLLER
if(SOFTROBOTS_COMMUNICATIONCONTROLLER)
    target_link_libraries(${PROJECT_NAME} "-lzmq")
endif()


#ROBOTINO CONTROLLER
if(SOFTROBOTS_ROBOTINOCONTROLLER)
    find_package(ROBOTINO REQUIRED)
    include_directories(${ROBOTINO_INCLUDE_DIR})

    message(STATUS ROBOTINO_LIBRARY:${ROBOTINO_LIBRARY})
    target_link_libraries(${PROJECT_NAME} ${ROBOTINO_LIBRARY})

endif()

find_package(Qt5 COMPONENTS Core QUIET)
if(Qt5Core_FOUND)
    message("SoftRobots: Using Qt5 for some controllers")
    find_package(Qt5 COMPONENTS Network REQUIRED)
    set(QT_TARGETS Qt5::Core Qt5::Network)
    target_link_libraries(${PROJECT_NAME} ${QT_TARGETS})

    if(WIN32)
        sofa_copy_libraries_from_targets(${QT_TARGETS})
    endif()
    sofa_install_libraries_from_targets(${QT_TARGETS})
endif()


if(SOFTROBOTS_GAMETRAKCONTROLLER)
    find_package(HIDAPI REQUIRED)
    include_directories(${HIDAPI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${HIDAPI_LIBRARIES} ${BOOST_THREAD_LIBRARY})
endif()
