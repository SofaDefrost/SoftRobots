
list(APPEND HEADER_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/CableConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/CableConstraint.inl

    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/PartialRigidificationConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/PartialRigidificationConstraint.inl

    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/SurfacePressureConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/SurfacePressureConstraint.inl

    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/UnilateralPlaneConstraint.h
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/UnilateralPlaneConstraint.inl

    )

list(APPEND SOURCE_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/CableConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/PartialRigidificationConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/SurfacePressureConstraint.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/constraint/UnilateralPlaneConstraint.cpp
    )

include(${SOFTROBOTS_SOURCE_DIR}/component/constraint/model/Model.cmake)



