
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
* Authors: Yinoussa Adagolodjo                                                *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/ObjectFactory.h>
#include <iostream>
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

#define SOFTROBOTS_CONSTRAINT_QPSLIDINGCONSTRAINT_NOEXTERN
#include "QPSlidingConstraint.inl"

namespace sofa
{

namespace component
{

namespace constraintset
{
using sofa::defaulttype::Rigid3Types;



//--------------- Force constraint -------------
SlidingForceConstraintResolution::SlidingForceConstraintResolution(const double &imposedForce, const double& min, const double& max)
    : ConstraintResolution(1)
    , m_imposedForce(imposedForce)
    , m_minDisplacement(min)
    , m_maxDisplacement(max)
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

    double displacement = m_wActuatorActuator*m_imposedForce + d[line];

    if (displacement<m_minDisplacement)
    {
        displacement=m_minDisplacement;
        lambda[line] -= (d[line]-displacement) / m_wActuatorActuator;
    }
    else if (displacement>m_maxDisplacement)
    {
        displacement=m_maxDisplacement;
        lambda[line] -= (d[line]-displacement) / m_wActuatorActuator;
    }
    else
        lambda[line] = m_imposedForce;
}


////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int QPSlidingConstraintClass = RegisterObject("Simulate cable actuation.")
.add< QPSlidingConstraint<Vec3Types> >(true)

;
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class QPSlidingConstraint<Vec3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa


