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

#include <SoftRobots/component/constraint/CableConstraint.h>

using sofa::helper::OptionsGroup;

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState;
using sofa::helper::WriteAccessor;

template<class DataTypes>
CableConstraint<DataTypes>::CableConstraint(MechanicalState* object)
    : Inherit1(object)

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, {"displacement","force"}, "valueType",
                                          "displacement = the contstraint will impose the displacement provided in data value[valueIndex] \n"
                                          "force = the contstraint will impose the force provided in data value[valueIndex] \n"
                                          "If unspecified, the default value is displacement"))
{
    d_eqDisplacement.setDisplayed(false);
    d_eqForce.setDisplayed(false);
}

template<class DataTypes>
CableConstraint<DataTypes>::~CableConstraint()
{
}

template<class DataTypes>
void CableConstraint<DataTypes>::init()
{
    Inherit1::init();

    internalInit();
}

template<class DataTypes>
void CableConstraint<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void CableConstraint<DataTypes>::internalInit()
{
    if(d_value.getValue().size()==0)
    {
        WriteAccessor<Data<vector<Real>>> value = d_value;
        value.resize(1);
    }

    // check for errors in the initialization
    if(d_value.getValue().size()<d_valueIndex.getValue())
    {
        msg_warning() << "Bad size for data value (size="<< d_value.getValue().size()<<"), or wrong value for data valueIndex (valueIndex="<<d_valueIndex.getValue()<<"). Set default valueIndex=0.";
        d_valueIndex.setValue(0);
    }
}

template<class DataTypes>
void CableConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParam,
                                                         std::vector<ConstraintResolution*>& resTab,
                                                         unsigned int& offset)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParam);

    Real imposedValue=d_value.getValue()[d_valueIndex.getValue()];

    if(d_valueType.getValue().getSelectedItem() == "displacement") // displacement
    {
        Real maxForce = std::numeric_limits<Real>::max();
        Real minForce = -maxForce;
        setUpDisplacementLimits(imposedValue,minForce,maxForce);

        CableDisplacementConstraintResolution *cr=  new CableDisplacementConstraintResolution(imposedValue, minForce, maxForce);
        resTab[offset++] =cr;
    }
    else // force
    {
        Real maxDisplacement = std::numeric_limits<Real>::max();
        Real minDisplacement = -maxDisplacement;
        setUpForceLimits(imposedValue,minDisplacement,maxDisplacement);

        CableForceConstraintResolution *cr=  new CableForceConstraintResolution(imposedValue, minDisplacement, maxDisplacement);
        resTab[offset++] =cr;
    }
}

template<class DataTypes>
void CableConstraint<DataTypes>::setUpDisplacementLimits(Real& imposedValue, Real& minForce, Real& maxForce)
{
    if(d_maxDispVariation.isSet())
    {
        Real displacement = d_displacement.getValue();
        if(imposedValue > displacement && imposedValue-displacement>d_maxDispVariation.getValue())
            imposedValue = displacement+d_maxDispVariation.getValue();

        if(imposedValue < displacement && imposedValue-displacement<-d_maxDispVariation.getValue())
            imposedValue = displacement-d_maxDispVariation.getValue();
    }

    if(d_maxPositiveDisplacement.isSet() && imposedValue>d_maxPositiveDisplacement.getValue())
        imposedValue = d_maxPositiveDisplacement.getValue();

    if(d_maxNegativeDisplacement.isSet() && imposedValue<-d_maxNegativeDisplacement.getValue())
        imposedValue = -d_maxNegativeDisplacement.getValue();

    if(d_minForce.isSet())
        minForce=d_minForce.getValue();
    if(d_maxForce.isSet())
        maxForce=d_maxForce.getValue();
}

template<class DataTypes>
void CableConstraint<DataTypes>::setUpForceLimits(Real& imposedValue, Real& minDisplacement, Real& maxDisplacement)
{
    if(d_maxForce.isSet() && imposedValue>d_maxForce.getValue())
        imposedValue = d_maxForce.getValue();

    if(d_minForce.isSet() && imposedValue<d_minForce.getValue())
        imposedValue = d_minForce.getValue();

    if(d_maxNegativeDisplacement.isSet())
        minDisplacement=-d_maxNegativeDisplacement.getValue();
    if(d_maxPositiveDisplacement.isSet())
        maxDisplacement=d_maxPositiveDisplacement.getValue();
}


} // namespace

