
list(APPEND HEADER_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PREquivalentStiffnessForceField.h
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PREquivalentStiffnessForceField.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PartialRigidificationForceField.h
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PartialRigidificationForceField.inl
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PipeForceField.h
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PipeForceField.inl

    )

list(APPEND SOURCE_FILES

    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PREquivalentStiffnessForceField.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PartialRigidificationForceField.cpp
    ${SOFTROBOTS_SOURCE_DIR}/component/forcefield/PipeForceField.cpp

    )
