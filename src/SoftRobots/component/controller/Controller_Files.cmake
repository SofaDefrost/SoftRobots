
#OPTIONS
option(SOFTROBOTS_GAMETRAKCONTROLLER "Enables GameTrak component" OFF)
option(SOFTROBOTS_ROBOTINOCONTROLLER "Enables Robotino component" OFF)
option(SOFTROBOTS_COMMUNICATIONCONTROLLER "Enables Communication component, requires to install ZMQ library" OFF)


list(APPEND HEADER_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/controller/AnimationEditor.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/AnimationEditor.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataVariationLimiter.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataVariationLimiter.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/InteractiveControl.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Network.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SplitFloat.h
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Serial.h

    #Communication robot/sofa
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/SerialPortBridgeGeneric.h
    )
	
if(WIN32)	
list(APPEND HEADER_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SysTimeWin.h
	)
endif(WIN32)

list(APPEND SOURCE_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/controller/AnimationEditor.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataVariationLimiter.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/InteractiveControl.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Network.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SplitFloat.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/Serial.cpp

    #Communication robot/sofa
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/SerialPortBridgeGeneric.cpp

    )

if(WIN32)	
list(APPEND SOURCE_FILES
    ${SOFTROBOTS_SOURCE_DIR}/component/controller/modules/SysTimeWin.cpp
	)
endif(WIN32)


#COMMUNICATION CONTROLLER
if(SOFTROBOTS_COMMUNICATIONCONTROLLER)

    list(APPEND HEADER_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/CommunicationController.h
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/CommunicationController.inl
            )

    list(APPEND SOURCE_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/CommunicationController.cpp
            )

endif()


#GAMETRAK CONTROLLER
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


#ROBOTINO CONTROLLER
if(SOFTROBOTS_ROBOTINOCONTROLLER)

    list(APPEND HEADER_FILES
            ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataControllerRobot.h
            )

    list(APPEND SOURCE_FILES
        ${SOFTROBOTS_SOURCE_DIR}/component/controller/DataControllerRobot.cpp
            )

endif()


