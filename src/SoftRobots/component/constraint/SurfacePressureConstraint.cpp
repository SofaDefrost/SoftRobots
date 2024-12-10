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
#define SOFTROBOTS_SURFACEPRESSURECONSTRAINT_CPP
#include <SoftRobots/component/constraint/SurfacePressureConstraint.inl>

#include <sofa/core/ObjectFactory.h>

namespace softrobots::constraint
{

/////////////////////////////////// SurfacePressureConstant ///////////////////////////////////////
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

SurfacePressureConstraintResolution::SurfacePressureConstraintResolution(const SReal& imposedPressure, const SReal &minVolumeGrowth, const SReal &maxVolumeGrowth)
    : ConstraintResolution(1)
    , m_imposedPressure(imposedPressure)
    , m_minVolumeGrowth(minVolumeGrowth)
    , m_maxVolumeGrowth(maxVolumeGrowth)
{
}

void SurfacePressureConstraintResolution::init(int line, SReal**w, SReal*force)
{
    SOFA_UNUSED(force);

    m_wActuatorActuator = w[line][line];
}

void SurfacePressureConstraintResolution::resolution(int line, SReal** w, SReal* d, SReal* force, SReal* dfree)
{
    SOFA_UNUSED(w);
    SOFA_UNUSED(d);
    SOFA_UNUSED(dfree);


    double volumeGrowth = m_wActuatorActuator*m_imposedPressure + d[line];

    if(volumeGrowth<m_minVolumeGrowth)
    {
        volumeGrowth = m_minVolumeGrowth;
        force[line] -= (d[line]-volumeGrowth) / m_wActuatorActuator ;
    }
    if(volumeGrowth>m_maxVolumeGrowth)
    {
        volumeGrowth = m_maxVolumeGrowth;
        force[line] -= (d[line]-volumeGrowth) / m_wActuatorActuator ;
    }
    else
        force[line] = m_imposedPressure ;


}

/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////// VolumeGrowthConstraintResolution ///////////////////////////////////////

VolumeGrowthConstraintResolution::VolumeGrowthConstraintResolution(const SReal &imposedVolumeGrowth, const SReal &minPressure, const SReal &maxPressure)
    : ConstraintResolution(1)
    , m_imposedVolumeGrowth(imposedVolumeGrowth)
    , m_minPressure(minPressure)
    , m_maxPressure(maxPressure)
{
}

void VolumeGrowthConstraintResolution::init(int line, SReal**w, SReal*force)
{
    SOFA_UNUSED(force);

    m_wActuatorActuator = w[line][line];
}

void VolumeGrowthConstraintResolution::resolution(int line, SReal** w, SReal* d, SReal* lambda, SReal* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);

    // da=Waa*(lambda_a) + Sum Wai * lambda_i  = m_imposedVolumeGrowth
    lambda[line] -= (d[line]-m_imposedVolumeGrowth) / m_wActuatorActuator ;

    if(lambda[line]<m_minPressure)
        lambda[line] = m_minPressure;
    if(lambda[line]>m_maxPressure)
        lambda[line] = m_maxPressure;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////// FACTORY //////////////////////////////////////////////////
void registerSurfacePressureConstraint(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("This component constrains a model by applying "
                                                          "pressure on surfaces (for example cavities)")
    .add< SurfacePressureConstraint<Vec3Types> >(true));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
template class SOFA_SOFTROBOTS_API SurfacePressureConstraint<Vec3Types>;

} // namespace


