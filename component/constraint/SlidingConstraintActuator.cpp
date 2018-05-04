/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <sofa/helper/logging/Messaging.h>
#include "SlidingConstraintActuator.inl"

#include <sofa/core/ObjectFactory.h>

#define STIFFNESS_CONSTRAINT_TOLERANCE 1E-2

namespace sofa
{

namespace component
{

namespace constraintset
{

//////////////////////////////////////SlidingConstraintConstraintResolution1Dof/////////////////////////////////////////////
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

//----------- Displacement constraint --------------
SlidingDisplacementConstraintResolution::SlidingDisplacementConstraintResolution(double& imposedDisplacement, double* force)
    : m_imposedDisplacement(imposedDisplacement)
    , m_force(force)
{ }


void SlidingDisplacementConstraintResolution::init(int line, double** w, double * lambda)
{
    SOFA_UNUSED(lambda);

    m_wActuatorActuator = w[line][line];

}


void SlidingDisplacementConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);

    // da=Waa*(lambda_a) + Sum Wai * lambda_i  = m_imposedDisplacement

    lambda[line] -= (d[line]-m_imposedDisplacement) / m_wActuatorActuator;

    storeForce(line, lambda);
}


void SlidingDisplacementConstraintResolution::storeForce(int line,  double* lambda)
{
    *m_force = lambda[line];
}


//--------------- Force constraint -------------
SlidingForceConstraintResolution::SlidingForceConstraintResolution(double& imposedForce, double *displacement)
    : m_imposedForce(imposedForce)
    , m_displacement(displacement)
{ }


void SlidingForceConstraintResolution::init(int line, double** w, double * lambda)
{
    SOFA_UNUSED(lambda);

    m_wActuatorActuator = w[line][line];
}

 void SlidingForceConstraintResolution::initForce(int line, double* lambda)
{

     lambda[line] = m_imposedForce;
}

void SlidingForceConstraintResolution::resolution(int line, double**, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(d);
    SOFA_UNUSED(line);


    lambda[line] = m_imposedForce;
    storeDisplacement(line, d);
}


void SlidingForceConstraintResolution::storeDisplacement(int line,  double* d)
{
    *m_displacement = d[line];
}


//------------- Stiffness Constraint -----------
SlidingStiffnessConstraintResolution::SlidingStiffnessConstraintResolution(double& imposedNeutralPosition, double& imposedStiffness, double* displacement, double* force)
	: m_imposedNeutralPosition(imposedNeutralPosition)
	, m_imposedStiffness(imposedStiffness)
	, m_displacement(displacement)
	, m_force(force)	
{}

void SlidingStiffnessConstraintResolution::init(int line, double** w, double * lambda)
{
	SOFA_UNUSED(lambda);

	m_wActuatorActuator = w[line][line];
}

void SlidingStiffnessConstraintResolution::resolution(int line, double**, double* d, double* lambda, double* dfree)
{
	SOFA_UNUSED(dfree);
	std::cout << "Denominator: " << abs(m_imposedStiffness*m_wActuatorActuator - 1) << " \t\t";
	//std::cout << "NeutPos: " << m_imposedNeutralPosition << "\t Waa" << m_wActuatorActuator << 
	if (abs(m_imposedStiffness*m_wActuatorActuator - 1) < STIFFNESS_CONSTRAINT_TOLERANCE)
	{
		// TODO MISK msg_warning() << "Commanded stiffness matches natural stiffness. Stiffness constraint is not imposed (no force was added).";
		std::cout << "Commanded stiffness matches natural stiffness. Stiffness constraint is not imposed." << std::endl;
	}
	else
	{
		double lastLambda = lambda[line];
		lambda[line] = m_imposedStiffness*(m_wActuatorActuator*lastLambda + m_imposedNeutralPosition - d[line]) / (m_wActuatorActuator*m_imposedStiffness - 1);
		d[line] += m_wActuatorActuator*(lambda[line] - lastLambda);
	}
	storeForceAndDisplacement(line, d, lambda);
	std::cout << "Lambda: " << lambda[line] << "\tDelta: " << d[line] << "\t";
	std::cout << "Compliance: " << (d[line]-m_imposedNeutralPosition) / lambda[line] << std::endl;
}


void SlidingStiffnessConstraintResolution::storeForceAndDisplacement(int line, double* d, double* lambda)
{
	*m_displacement = d[line];
	*m_force = lambda[line];

	std::cout << "Compliance in storeForceAndDisplacement: " << (d[line] - m_imposedNeutralPosition) / lambda[line] << std::endl;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
// 2-RegisterObject("description") + .add<> : Register the component
// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(SlidingConstraintActuator)

int SlidingConstraintActuatorClass = RegisterObject("Simulate sliding (or rotating) actuation.")
#ifdef SOFA_WITH_DOUBLE
.add< SlidingConstraintActuator<Vec3dTypes> >(true)
.add< SlidingConstraintActuator<Rigid3dTypes> >()
#endif
#ifdef SOFA_WITH_FLOAT
.add< SlidingConstraintActuator<Vec3fTypes> >()
#endif
;
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_WITH_DOUBLE
template class SlidingConstraintActuator<Vec3dTypes>;
template class SlidingConstraintActuator<Rigid3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
template class SlidingConstraintActuator<Vec3fTypes>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa
