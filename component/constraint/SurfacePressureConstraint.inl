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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURECONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURECONSTRAINT_INL

#include "SurfacePressureConstraint.h"

#include <sofa/helper/logging/Messaging.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

namespace _surfacepressureconstraint_
{

using sofa::core::objectmodel::ComponentState ;
using sofa::helper::WriteAccessor ;
using sofa::defaulttype::Vec3d;
using sofa::helper::vector ;
using sofa::helper::OptionsGroup;

template<class DataTypes>
SurfacePressureConstraint<DataTypes>::SurfacePressureConstraint(MechanicalState* object)
    : Inherit1(object)
    , d_value(initData(&d_value, "value",
                                "List of choices for volume growth or pressure to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, OptionsGroup(2,"pressure","volumeGrowth"), "valueType",
                                          "volumeGrowth = the contstraint will impose the volume growth provided in data value[valueIndex] \n"
                                          "pressure = the contstraint will impose the pressure provided in data value[valueIndex] \n"
                                          "If unspecified, the default value is pressure"))
{
}


template<class DataTypes>
SurfacePressureConstraint<DataTypes>::~SurfacePressureConstraint()
{
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::init()
{
    Inherit1::init();
    initData();

    m_initialValue = d_value.getValue();
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::reinit()
{
    Inherit1::reinit();
    initData();
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::reset()
{
    Inherit1::reset();
    d_value.setValue(m_initialValue);
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::initData()
{
    if(d_value.getValue().size()==0)
    {
        WriteAccessor<Data<vector<Real>>> value = d_value;
        value.resize(1,0.);
    }

    // check for errors in the initialization
    if(d_value.getValue().size()<d_valueIndex.getValue())
    {
        msg_warning() <<"Bad size for data value (size="<< d_value.getValue().size()<<"), or wrong value for data valueIndex (valueIndex="<<d_valueIndex.getValue()<<"). Set default valueIndex=0.";
        d_valueIndex.setValue(0);
    }
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                                                   unsigned int& offset)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    double imposedValue = d_value.getValue()[d_valueIndex.getValue()];

    if(d_valueType.getValue().getSelectedItem() == "volumeGrowth")
    {
        double maxPressure = std::numeric_limits<double>::max();
        double minPressure = -maxPressure;
        setUpVolumeLimits(imposedValue,minPressure,maxPressure);

        VolumeGrowthConstraintResolution *cr=  new VolumeGrowthConstraintResolution(imposedValue, minPressure, maxPressure);
        resTab[offset++] =cr;
    }
    else // pressure
    {
        double maxVolumeGrowth = std::numeric_limits<double>::max();
        double minVolumeGrowth = -maxVolumeGrowth;
        setUpPressureLimits(imposedValue,minVolumeGrowth,maxVolumeGrowth);

        SurfacePressureConstraintResolution *cr=  new SurfacePressureConstraintResolution(imposedValue, minVolumeGrowth, maxVolumeGrowth);
        resTab[offset++] = cr;
    }
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::setUpVolumeLimits(double& imposedValue, double& minPressure, double& maxPressure)
{
    if(d_maxVolumeGrowthVariation.isSet())
    {
        double displacement = d_volumeGrowth.getValue();
        if(imposedValue > displacement && imposedValue-displacement>d_maxVolumeGrowthVariation.getValue())
            imposedValue = displacement+d_maxVolumeGrowthVariation.getValue();

        if(imposedValue < displacement && imposedValue-displacement<-d_maxVolumeGrowthVariation.getValue())
            imposedValue = displacement-d_maxVolumeGrowthVariation.getValue();
    }

    if(d_maxVolumeGrowth.isSet() && imposedValue>d_maxVolumeGrowth.getValue())
        imposedValue = d_maxVolumeGrowth.getValue();

    if(d_minVolumeGrowth.isSet() && imposedValue<d_minVolumeGrowth.getValue())
        imposedValue = d_minVolumeGrowth.getValue();

    if(d_minPressure.isSet())
        minPressure=d_minPressure.getValue();
    if(d_maxPressure.isSet())
        maxPressure=d_maxPressure.getValue();
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::setUpPressureLimits(double& imposedValue, double& minVolumeGrowth, double& maxVolumeGrowth)
{
    if(d_maxPressure.isSet() && imposedValue>d_maxPressure.getValue())
        imposedValue = d_maxPressure.getValue();

    if(d_minPressure.isSet() && imposedValue<d_minPressure.getValue())
        imposedValue = d_minPressure.getValue();

    if(d_minVolumeGrowth.isSet())
        minVolumeGrowth = d_minVolumeGrowth.getValue();
    if(d_maxVolumeGrowth.isSet())
        maxVolumeGrowth =d_maxVolumeGrowth.getValue();
}

}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
