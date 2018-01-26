
#OPTIONS
option(SOFTROBOTS_GAMETRAKCONTROLLER "Enables GameTrak component" OFF)
option(SOFTROBOTS_ROBOTINOCONTROLLER "Enables Robotino component" OFF)
option(SOFTROBOTS_COMMUNICATIONCONTROLLER "Enables Communication component, requires to install ZMQ library" OFF)


list(APPEND HEADER_FILES

    component/controller/AnimationEditor.h
    component/controller/AnimationEditor.inl
    component/controller/DataVariationLimiter.h
    component/controller/DataVariationLimiter.inl
    component/controller/InteractiveControl.h
    component/controller/modules/Network.h
    component/controller/modules/SplitFloat.h
    component/controller/modules/Serial.h

    #Communication robot/sofa
    component/controller/SerialPortBridgeGeneric.h
    )
	
if(WIN32)	
list(APPEND HEADER_FILES
    component/controller/modules/SysTimeWin.h
	)
endif(WIN32)

list(APPEND SOURCE_FILES

    component/controller/AnimationEditor.cpp
    component/controller/DataVariationLimiter.cpp
    component/controller/InteractiveControl.cpp
    component/controller/modules/Network.cpp
    component/controller/modules/SplitFloat.cpp
    component/controller/modules/Serial.cpp

    #Communication robot/sofa
    component/controller/SerialPortBridgeGeneric.cpp

    )

if(WIN32)	
list(APPEND SOURCE_FILES
    component/controller/modules/SysTimeWin.cpp
	)
endif(WIN32)


#COMMUNICATION CONTROLLER
if(SOFTROBOTS_COMMUNICATIONCONTROLLER)

    list(APPEND HEADER_FILES
        component/controller/CommunicationController.h
        component/controller/CommunicationController.inl
            )

    list(APPEND SOURCE_FILES
        component/controller/CommunicationController.cpp
            )

endif()


#GAMETRAK CONTROLLER
if(SOFTROBOTS_GAMETRAKCONTROLLER)

    list(APPEND HEADER_FILES
            component/controller/GameTrakController.h

            component/controller/modules/libgametrak/GameTrak.h
            component/controller/modules/libgametrak/HIDAPIGameTrak.h

            component/controller/modules/libgametrak/utils/OneEuroFilter.h
            component/controller/modules/libgametrak/utils/Quaternion.h
            component/controller/modules/libgametrak/utils/stdint.h
            component/controller/modules/libgametrak/utils/TimeStamp.h
            component/controller/modules/libgametrak/utils/URI.h
            component/controller/modules/libgametrak/utils/vecteur3d.h
            )

    list(APPEND SOURCE_FILES
            component/controller/GameTrakController.cpp

            component/controller/modules/libgametrak/GameTrak.cpp
            component/controller/modules/libgametrak/HIDAPIGameTrak.cpp

            component/controller/modules/libgametrak/utils/OneEuroFilter.cpp
            component/controller/modules/libgametrak/utils/Quaternion.cpp
            component/controller/modules/libgametrak/utils/TimeStamp.cpp
            component/controller/modules/libgametrak/utils/URI.cpp
            component/controller/modules/libgametrak/utils/vecteur3d.cpp
            )

endif()


#ROBOTINO CONTROLLER
if(SOFTROBOTS_ROBOTINOCONTROLLER)

    list(APPEND HEADER_FILES
            component/controller/DataControllerRobot.h
            )

    list(APPEND SOURCE_FILES
        component/controller/DataControllerRobot.cpp
            )

endif()


