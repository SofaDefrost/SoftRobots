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
#define SOFTROBOTS_SURFACEPRESSURECONSTRAINT_CPP
#include <SoftRobots/component/constraint/SurfacePressureConstraint.inl>

#include <sofa/core/ObjectFactory.h>

namespace sofa::component::constraintset
{

namespace _surfacepressureconstraint_
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
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int SurfacePressureConstraintClass = core::RegisterObject("This component constrains a model by applying "
                                                          "pressure on surfaces (for exemple cavities)")
.add< SurfacePressureConstraint<Vec3Types> >(true)//Set Vec3d to default template

;
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_API SurfacePressureConstraint<Vec3Types>;


}

} // namespace


