# This module defines the following variables:
#  VRPN_INCLUDE_DIR - include directories for GLEW
#  VRPN_LIBRARY - libraries to link against GLEW
#  VRPN_FOUND - true if GLEW has been found and can be used


if(WIN32)

    # HIGHLY TEMPORARY !!!!
    Message(WARNING "findVRPN.cmake is temporarily hardcoded to use SOFA-EXTERNAL_VRPN_PATH !")

    if(NOT SOFA-EXTERNAL_VRPN_PATH)
        message(FATAL_ERROR "SoftRobots plugin needs vrpn. Please add the SOFA-EXTERNAL_VRPN_PATH to fix it...")
    endif()

    set(VRPN_INCLUDE_DIR ${SOFA-EXTERNAL_VRPN_PATH})
    set(VRPN_LIBRARY "vrpn")
    set(VRPN_FOUND TRUE)
else()
    find_path(VRPN_INCLUDE_DIR vrpn_Configure.h)
    find_library(VRPN_LIBRARY NAMES vrpn)
    set(VRPN_FOUND TRUE)

endif()
mark_as_advanced(VRPN_INCLUDE_DIR VRPN_LIBRARY)

# et un grand OUI, c'est moche. À mettre au point sur une machine qui dispose de cette lib..
