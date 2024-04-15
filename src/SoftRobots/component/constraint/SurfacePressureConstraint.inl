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
#pragma once

#include <SoftRobots/component/constraint/SurfacePressureConstraint.h>

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState ;
using sofa::helper::WriteAccessor ;
using sofa::type::Vec3d;
using sofa::type::vector ;
using sofa::helper::OptionsGroup;

template<class DataTypes>
SurfacePressureConstraint<DataTypes>::SurfacePressureConstraint(MechanicalState* object)
    : Inherit1(object)
    , d_value(initData(&d_value, "value",
                                "List of choices for volume growth or pressure to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, {"pressure","volumeGrowth"}, "valueType",
                                          "volumeGrowth = the constraint will impose the volume growth provided in data value[valueIndex] \n"
                                          "pressure = the constraint will impose the pressure provided in data value[valueIndex] \n"
                                          "If unspecified, the default value is pressure"))
{
    d_eqVolumeGrowth.setDisplayed(false);
    d_eqPressure.setDisplayed(false);
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
    if (d_value.getValue().empty())
    {
        WriteAccessor<Data<vector<Real>>> value = d_value;
        value.resize(1);
    }

    // check for errors in the initialization
    if (d_value.getValue().size() < d_valueIndex.getValue())
    {
        msg_warning() <<"Bad size for data value (size="<< d_value.getValue().size()<<"), or wrong value for data valueIndex (valueIndex="<<d_valueIndex.getValue()<<"). Set default valueIndex=0.";
        d_valueIndex.setValue(0);
    }
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                                                   unsigned int& offset)
{
    if (!this->isComponentStateValid())
    {
        return;
    }

    Real imposedValue = d_value.getValue()[d_valueIndex.getValue()];

    if (d_valueType.getValue().getSelectedItem() == "volumeGrowth")
    {
        Real maxPressure = std::numeric_limits<Real>::max();
        Real minPressure = -maxPressure;
        setUpVolumeLimits(imposedValue, minPressure, maxPressure);

        VolumeGrowthConstraintResolution *cr=  new VolumeGrowthConstraintResolution(imposedValue, minPressure, maxPressure);
        resTab[offset++] = cr;
    }
    else // pressure
    {
        Real maxVolumeGrowth = std::numeric_limits<Real>::max();
        Real minVolumeGrowth = -maxVolumeGrowth;
        setUpPressureLimits(imposedValue,minVolumeGrowth,maxVolumeGrowth);

        SurfacePressureConstraintResolution *cr=  new SurfacePressureConstraintResolution(imposedValue, minVolumeGrowth, maxVolumeGrowth);
        resTab[offset++] = cr;
    }
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::setUpVolumeLimits(Real& imposedValue, Real& minPressure, Real& maxPressure)
{
    if(d_maxVolumeGrowthVariation.isSet())
    {
        double volumeGrowth = d_volumeGrowth.getValue();
        if(imposedValue > volumeGrowth && imposedValue-volumeGrowth>d_maxVolumeGrowthVariation.getValue())
            imposedValue = volumeGrowth+d_maxVolumeGrowthVariation.getValue();

        if(imposedValue < volumeGrowth && imposedValue-volumeGrowth<-d_maxVolumeGrowthVariation.getValue())
            imposedValue = volumeGrowth-d_maxVolumeGrowthVariation.getValue();
    }

    if(d_maxVolumeGrowth.isSet() && imposedValue>d_maxVolumeGrowth.getValue())
        imposedValue = d_maxVolumeGrowth.getValue();

    if(d_minVolumeGrowth.isSet() && imposedValue<d_minVolumeGrowth.getValue())
        imposedValue = d_minVolumeGrowth.getValue();

    if(d_minPressure.isSet())
        minPressure = d_minPressure.getValue();

    if(d_maxPressure.isSet())
        maxPressure = d_maxPressure.getValue();
}


template<class DataTypes>
void SurfacePressureConstraint<DataTypes>::setUpPressureLimits(Real& imposedValue, Real& minVolumeGrowth, Real& maxVolumeGrowth)
{
    if (d_maxPressureVariation.isSet())
    {
        double pressure = this->d_pressure.getValue();
        if (imposedValue > pressure && imposedValue - pressure > d_maxPressureVariation.getValue())
            imposedValue = pressure + d_maxPressureVariation.getValue();

        if (imposedValue < pressure && imposedValue - pressure < -d_maxPressureVariation.getValue())
            imposedValue = pressure - d_maxPressureVariation.getValue();
    }

    if(d_maxPressure.isSet() && imposedValue > d_maxPressure.getValue())
        imposedValue = d_maxPressure.getValue();

    if(d_minPressure.isSet() && imposedValue < d_minPressure.getValue())
        imposedValue = d_minPressure.getValue();

    if(d_minVolumeGrowth.isSet())
        minVolumeGrowth = d_minVolumeGrowth.getValue();

    if(d_maxVolumeGrowth.isSet())
        maxVolumeGrowth = d_maxVolumeGrowth.getValue();
}

} // namespace

