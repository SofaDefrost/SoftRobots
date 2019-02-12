
list(APPEND HEADER_FILES
   
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/CenterOfMass.h
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/CenterOfMass.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTriangles.h
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTriangles.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTetrahedrons.h
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTetrahedrons.inl

    )

list(APPEND SOURCE_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/engine/CenterOfMass.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTriangles.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/engine/VolumeFromTetrahedrons.cpp

    )
