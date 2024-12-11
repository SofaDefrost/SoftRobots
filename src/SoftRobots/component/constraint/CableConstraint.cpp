/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
*                           Plugin SoftRobots                                 *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
******************************************************************************/
#define SOFTROBOTS_CABLECONSTRAINT_CPP

#include <SoftRobots/component/constraint/CableConstraint.inl>

#include <sofa/core/ObjectFactory.h>

namespace softrobots::constraint
{

//////////////////////////////////////CableConstraintConstraintResolution1Dof/////////////////////////////////////////////
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

//----------- Displacement constraint --------------
CableDisplacementConstraintResolution::CableDisplacementConstraintResolution(const SReal& imposedDisplacement, const SReal &min, const SReal &max)
    : sofa::core::behavior::ConstraintResolution(1)
    , m_imposedDisplacement(imposedDisplacement)
    , m_minForce(min)
    , m_maxForce(max)
{ }


void CableDisplacementConstraintResolution::init(int line, SReal** w, SReal * lambda)
{
    SOFA_UNUSED(lambda);
    m_wActuatorActuator = w[line][line];
}


void CableDisplacementConstraintResolution::resolution(int line, SReal** w, SReal* d, SReal* lambda, SReal* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);

    // da=Waa*(lambda_a) + Sum Wai * lambda_i  = m_imposedDisplacement
    lambda[line] -= (d[line]-m_imposedDisplacement) / m_wActuatorActuator;

    if (lambda[line]<m_minForce)
        lambda[line]=m_minForce;

    if (lambda[line]>m_maxForce)
        lambda[line]=m_maxForce;
}


//--------------- Force constraint -------------
CableForceConstraintResolution::CableForceConstraintResolution(const SReal &imposedForce, const SReal& min, const SReal& max)
    : ConstraintResolution(1)
    , m_imposedForce(imposedForce)
    , m_minDisplacement(min)
    , m_maxDisplacement(max)
{ }


void CableForceConstraintResolution::init(int line, SReal** w, SReal * lambda)
{
    SOFA_UNUSED(lambda);
    m_wActuatorActuator = w[line][line];
}

void CableForceConstraintResolution::resolution(int line, SReal** w, SReal* d, SReal* lambda, SReal* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);

    SReal displacement = m_wActuatorActuator*m_imposedForce + d[line];

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void registerCableConstraint(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Simulate a cable.")
    .add< CableConstraint<Vec3Types> >(true)
    .add< CableConstraint<Vec2Types> >());
}


template class SOFA_SOFTROBOTS_API CableConstraint<Vec3Types>;
template class SOFA_SOFTROBOTS_API CableConstraint<Vec2Types>;


} // namespace

