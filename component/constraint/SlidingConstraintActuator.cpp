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

#define STIFFNESS_CONSTRAINT_TOLERANCE 1E-4

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
    //SOFA_UNUSED(d);
    //SOFA_UNUSED(line);

	double lastLambda = lambda[line];
    lambda[line] = m_imposedForce;
	d[line] += m_wActuatorActuator * (lambda[line] - lastLambda);
    storeDisplacement(line, d);
}


void SlidingForceConstraintResolution::storeDisplacement(int line,  double* d)
{
    *m_displacement = d[line];
}


//------------- Stiffness Constraint -----------
SlidingStiffnessConstraintResolution::SlidingStiffnessConstraintResolution(double& imposedBiasForce, double& imposedStiffness, double* displacement, double* force, double& neutralEffectorPosition, double* neutralActuatorPosition, double* Wea, double* Waa)
	: m_imposedBiasForce(imposedBiasForce) // TEST
	, m_imposedStiffness(imposedStiffness)
	, m_displacement(displacement)
	, m_force(force)
	, m_neutralEffectorPosition(neutralEffectorPosition)
	, m_neutralActuatorPosition(neutralActuatorPosition)
	, m_Wea(Wea)
	, m_Waa(Waa)
{}

void SlidingStiffnessConstraintResolution::init(int line, double** w, double * lambda)
{
	SOFA_UNUSED(lambda);
	int otherLine = 0; // TEST
	if (line == 0) // TEST
		otherLine = 1; // TEST
	m_wActuatorActuator = w[line][line];
	m_wEffectorActuator = w[otherLine][line]; // TEST

}

void SlidingStiffnessConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
	// TEST SOFA_UNUSED(dfree);
	int otherLine = 0;
	if (line == 0) // TEST
		otherLine = 1;
	//*m_neutralEffectorPosition = m_wEffectorActuator*m_imposedBiasForce + dfree[otherLine]; // TEST
	//*m_neutralActuatorPosition = m_wActuatorActuator*m_imposedBiasForce + dfree[line]; // TEST
	//*m_Wea = m_wEffectorActuator;
	//*m_Waa = m_wActuatorActuator;
	//double m_imposedNeutralPosition = *m_neutralActuatorPosition - m_imposedBiasForce / m_imposedStiffness; // TEST
	if (abs(m_imposedStiffness*m_wActuatorActuator - 1) < STIFFNESS_CONSTRAINT_TOLERANCE)
	{
		// TODO MISK msg_warning() << "Commanded stiffness matches natural stiffness. Stiffness constraint is not imposed (no force was added).";
		std::cout << "Commanded stiffness matches natural stiffness. Stiffness constraint is not imposed." << std::endl;
	}
	else
	{
		//double lastLambda = lambda[line];
		//lambda[line] = m_imposedStiffness*(m_wActuatorActuator*lastLambda + m_imposedNeutralPosition - d[line]) / (m_wActuatorActuator*m_imposedStiffness - 1);
		//d[line] += m_wActuatorActuator*(lambda[line] - lastLambda);

		double d0 = w[line][otherLine] * lambda[otherLine];// +dfree[line];
		double lambda_buf = m_imposedStiffness / (1 - m_imposedStiffness*m_wActuatorActuator) *d0;
		lambda[line] = lambda_buf;
		d[line] = m_wActuatorActuator * lambda_buf + d0 + dfree[line];

		//double dLambda = (lambda[line] - m_imposedStiffness*d[line]) / (m_imposedStiffness*m_wActuatorActuator - 1);
		//lambda[line] += dLambda;
		//d[line] += m_wActuatorActuator*dLambda;
	}
	double Wee = w[otherLine][otherLine];
	double Wea = w[otherLine][line];
	double Wae = w[line][otherLine];
	double Waa = w[line][line];
	//lambda[line] += (-dfree[otherLine] + m_neutralEffectorPosition) / w[otherLine][line];
	lambda[line] += -dfree[line] / w[line][line];

	double delta_e = w[otherLine][otherLine] * lambda[otherLine] + w[otherLine][line] * lambda[line] +dfree[otherLine];
	double K_e = lambda[otherLine] / (delta_e- m_neutralEffectorPosition);
	
	double W_e_expected = Wee + Wea*(1 / ((1 / m_imposedStiffness) - Waa))*Wae;
	 
	double delta_e_expected = W_e_expected *lambda[otherLine] + m_neutralEffectorPosition;

	//lambda[line] += (delta_e_expected - delta_e) / w[otherLine][line]; 
	d[line] = w[line][otherLine] * lambda[otherLine] + w[line][line] * lambda[line] + dfree[line];



	std::cout << K_e << "\t" << 1/W_e_expected << std::endl;
	storeForceAndDisplacement(line, d, lambda);
}


void SlidingStiffnessConstraintResolution::storeForceAndDisplacement(int line, double* d, double* lambda)
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
