#COMMUNICATION CONTROLLER
if(SOFTROBOTS_COMMUNICATIONCONTROLLER)
	find_package(ZMQ REQUIRED)
	include_directories(${ZMQ_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARY})
	IF(WIN32)
		add_definitions(-D_WINSOCKAPI_)
	ENDIF(WIN32)
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
    target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Network)
else()
    message("SoftRobots: Using Qt4 for some controllers")
    find_package(Qt4 COMPONENTS qtcore qtnetwork REQUIRED)
    include_directories(${QT_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTNETWORK_LIBRARY})
endif()


if(SOFTROBOTS_GAMETRAKCONTROLLER)
    find_package(HIDAPI REQUIRED)
    include_directories(${HIDAPI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${HIDAPI_LIBRARIES} ${BOOST_THREAD_LIBRARY})
endif()
