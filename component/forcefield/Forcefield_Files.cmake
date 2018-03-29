
list(APPEND HEADER_FILES

    component/forcefield/PREquivalentStiffnessForceField.h
    component/forcefield/PREquivalentStiffnessForceField.inl
    component/forcefield/PartialRigidificationForceField.h
    component/forcefield/PartialRigidificationForceField.inl
    component/forcefield/PipeForceField.h
    component/forcefield/PipeForceField.inl
	component/forcefield/AffineRestShapeSpringForceField.h
	component/forcefield/AffineRestShapeSpringForceField.inl

    )

list(APPEND SOURCE_FILES

    component/forcefield/PREquivalentStiffnessForceField.cpp
    component/forcefield/PartialRigidificationForceField.cpp
    component/forcefield/PipeForceField.cpp
	component/forcefield/AffineRestShapeSpringForceField.cpp

    )
