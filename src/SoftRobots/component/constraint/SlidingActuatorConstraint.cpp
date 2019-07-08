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

#include "SlidingActuatorConstraint.inl"

#include <sofa/core/ObjectFactory.h>

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
    : ConstraintResolution(1)
	, m_imposedDisplacement(imposedDisplacement)
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
	: ConstraintResolution(1)
	, m_imposedForce(imposedForce)
    , m_displacement(displacement)
{ }


void SlidingForceConstraintResolution::init(int line, double** w, double * lambda)
{
    SOFA_UNUSED(lambda);

    m_wActuatorActuator = w[line][line];
}

void SlidingForceConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
	SOFA_UNUSED(w);

	double lastLambda = lambda[line];
    lambda[line] = m_imposedForce;
	d[line] += m_wActuatorActuator * (lambda[line] - lastLambda);
    storeDisplacement(line, d);
}


void SlidingForceConstraintResolution::storeDisplacement(int line,  double* d)
{
    *m_displacement = d[line];
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int SlidingActuatorConstraintClass = RegisterObject("Simulate sliding (or rotating) actuation.")
.add< SlidingActuatorConstraint<Vec3Types> >(true)
.add< SlidingActuatorConstraint<Rigid3Types> >()
;
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SlidingActuatorConstraint<Vec3Types>;
template class SlidingActuatorConstraint<Rigid3Types>;

} // namespace constraintset

} // namespace component

} // namespace sofa
