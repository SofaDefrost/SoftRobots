
list(APPEND HEADER_FILES

    component/forcefield/PREquivalentStiffnessForceField.h
    component/forcefield/PREquivalentStiffnessForceField.inl
    component/forcefield/PartialRigidificationForceField.h
    component/forcefield/PartialRigidificationForceField.inl
    component/forcefield/PipeForceField.h
    component/forcefield/PipeForceField.inl

    )

list(APPEND SOURCE_FILES

    component/forcefield/PREquivalentStiffnessForceField.cpp
    component/forcefield/PartialRigidificationForceField.cpp
    component/forcefield/PipeForceField.cpp

    )

if(SOFA_WITH_EXPERIMENTAL_FEATURES)

    list(APPEND HEADER_FILES
        component/forcefield/MappedMatrixForceField.h
        component/forcefield/MappedMatrixForceField.inl
        )

    list(APPEND SOURCE_FILES
        component/forcefield/MappedMatrixForceField.cpp
        )

endif()
