/******************************************************************************
*               SOFA, Simulation Open-Framework Architecture                  *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                           Plugin SoftRobots v1.0                            *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/


#include "CableConstraint.inl"

#include <sofa/core/ObjectFactory.h>

#define STIFFNESS_CONSTRAINT_TOLERANCE 1E-4

namespace sofa
{

namespace component
{

namespace constraintset
{

//////////////////////////////////////CableConstraintConstraintResolution1Dof/////////////////////////////////////////////
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

//----------- Displacement constraint --------------
CableDisplacementConstraintResolution::CableDisplacementConstraintResolution(double& imposedDisplacement, double* force)
    : m_imposedDisplacement(imposedDisplacement)
    , m_force(force)
{ }


void CableDisplacementConstraintResolution::init(int line, double** w, double * lambda)
{
    SOFA_UNUSED(lambda);

    m_wActuatorActuator = w[line][line];

}


void CableDisplacementConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);

    // da=Waa*(lambda_a) + Sum Wai * lambda_i  = m_imposedDisplacement

    lambda[line] -= (d[line]-m_imposedDisplacement) / m_wActuatorActuator;

    if (lambda[line]<0)
        lambda[line]=0;

    storeForce(line, lambda);
}


void CableDisplacementConstraintResolution::storeForce(int line,  double* lambda)
{
    *m_force = lambda[line];
}


//--------------- Force constraint -------------
CableForceConstraintResolution::CableForceConstraintResolution(double& imposedForce, double *displacement)
    : m_imposedForce(imposedForce)
    , m_displacement(displacement)
{ }


void CableForceConstraintResolution::init(int line, double** w, double * lambda)
{
    SOFA_UNUSED(lambda);

    m_wActuatorActuator = w[line][line];
}

 void CableForceConstraintResolution::initForce(int line, double* lambda)
{
     if (m_imposedForce<0.0)
         m_imposedForce=0.0;

     lambda[line] = m_imposedForce;
}

void CableForceConstraintResolution::resolution(int line, double**, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(d);
    SOFA_UNUSED(line);

    if (m_imposedForce<0.0)
        m_imposedForce=0.0;

    lambda[line] = m_imposedForce;
    storeDisplacement(line, d);
}


void CableForceConstraintResolution::storeDisplacement(int line,  double* d)
{
    *m_displacement = d[line];
}

//------------- Stiffness Constraint -----------
CableStiffnessConstraintResolution::CableStiffnessConstraintResolution(double& imposedNeutralPosition, double& imposedStiffness, double* displacement, double* force)
	: m_imposedNeutralPosition(imposedNeutralPosition)
	, m_imposedStiffness(imposedStiffness)
	, m_displacement(displacement)
	, m_force(force)
{}

void CableStiffnessConstraintResolution::init(int line, double** w, double * lambda)
{
	SOFA_UNUSED(lambda);

	m_wActuatorActuator = w[line][line];
}

void CableStiffnessConstraintResolution::resolution(int line, double**, double* d, double* lambda, double* dfree)
{
	SOFA_UNUSED(dfree);

	if (abs(m_imposedStiffness*m_wActuatorActuator - 1) < STIFFNESS_CONSTRAINT_TOLERANCE)
	{
		// TODO MISK msg_warning() << "Commanded stiffness matches natural stiffness. Stiffness constraint is not imposed (no force was added).";
		std::cout << "Commanded stiffness matches natural stiffness. Stiffness constraint is not imposed." << std::endl;
	}
	else
	{
		double lastLambda = lambda[line];
		lambda[line] = m_imposedStiffness*(m_wActuatorActuator*lastLambda + m_imposedNeutralPosition - d[line]) / (m_wActuatorActuator*m_imposedStiffness - 1);
		if (lambda[line] < 0.0)
			lambda[line] = 0.0;
		d[line] += m_wActuatorActuator*(lambda[line] - lastLambda);
	}
	storeForceAndDisplacement(line, d, lambda);
}


void CableStiffnessConstraintResolution::storeForceAndDisplacement(int line, double* d, double* lambda)
{
	*m_displacement = d[line];
	*m_force = lambda[line];

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
// 2-RegisterObject("description") + .add<> : Register the component
// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(CableConstraint)

int CableConstraintClass = RegisterObject("Simulate cable actuation.")
#ifdef SOFA_WITH_DOUBLE
.add< CableConstraint<Vec3dTypes> >(true)
#endif
#ifdef SOFA_WITH_FLOAT
.add< CableConstraint<Vec3fTypes> >()
#endif
;
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_WITH_DOUBLE
template class CableConstraint<Vec3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
template class CableConstraint<Vec3fTypes>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

