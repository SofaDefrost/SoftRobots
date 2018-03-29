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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINTACTUATOR_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINTACTUATOR_INL

#include "SlidingConstraintActuator.h"

using sofa::helper::OptionsGroup;

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::helper::WriteAccessor;

template<class DataTypes>
SlidingConstraintActuator<DataTypes>::SlidingConstraintActuator(MechanicalState* object)
    : Inherit(object)

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, OptionsGroup(2,"displacement","force"), "valueType",
                                          "displacement = the contstraint will impose the displacement provided in data d_inputValue[d_iputIndex] \n"
                                          "force = the contstraint will impose the force provided in data d_inputValue[d_iputIndex] \n"
                                          "If unspecified, the default value is displacement"))
{
    d_value.setGroup("Input");
    d_valueIndex.setGroup("Input");
    d_valueType.setGroup("Input");
}


template<class DataTypes>
SlidingConstraintActuator<DataTypes>::SlidingConstraintActuator()
    : Inherit()

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, OptionsGroup(2,"displacement","force"), "valueType",
                                          "displacement = the contstraint will impose the displacement provided in data d_inputValue[d_iputIndex] \n"
                                          "force = the contstraint will impose the force provided in data d_inputValue[d_iputIndex] \n"
                                          "If unspecified, the default value is displacement"))
{
    d_value.setGroup("Input");
    d_valueIndex.setGroup("Input");
    d_valueType.setGroup("Input");
}

template<class DataTypes>
SlidingConstraintActuator<DataTypes>::~SlidingConstraintActuator()
{
}

template<class DataTypes>
void SlidingConstraintActuator<DataTypes>::init()
{
    Inherit::init();
    internalInit();
}

template<class DataTypes>
void SlidingConstraintActuator<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void SlidingConstraintActuator<DataTypes>::internalInit()
{
    // check for errors in the initialization
    if(d_value.getValue().size()==0)
    {
        WriteAccessor<Data<vector<Real>>> inputValue = d_value;
        inputValue.resize(1,0.);
    }

    if(d_value.getValue().size()<d_valueIndex.getValue())
        serr<<"bad size of inputValue ="<< d_value.getValue().size()<<"  or wrong value for inputIndex = "<<d_valueIndex.getValue()<<sendl;

}

template<class DataTypes>
void SlidingConstraintActuator<DataTypes>::getConstraintResolution(const ConstraintParams* cParam,
                                                         std::vector<ConstraintResolution*>& resTab,
                                                         unsigned int& offset)
{
    SOFA_UNUSED(cParam);

    double imposed_value=d_value.getValue()[d_valueIndex.getValue()];

    if(d_valueType.getValue().getSelectedItem() == "displacement") // displacement
    {
        if(d_maxDispVariation.isSet())
        {
            double displacement = d_displacement.getValue();
            if(imposed_value > displacement && imposed_value-displacement>d_maxDispVariation.getValue())
                imposed_value = displacement+d_maxDispVariation.getValue();

            if(imposed_value < displacement && imposed_value-displacement<-d_maxDispVariation.getValue())
                imposed_value = displacement-d_maxDispVariation.getValue();
        }


        SlidingDisplacementConstraintResolution *cr = new SlidingDisplacementConstraintResolution(imposed_value, &m_force);
        resTab[offset++] =cr;
        d_displacement.setValue(imposed_value);
        d_force.setValue(m_force);
    }
    else // force
    {
        SlidingForceConstraintResolution *cr = new SlidingForceConstraintResolution(imposed_value, &m_displacement);
        resTab[offset++] =cr;
        d_force.setValue(imposed_value);
        d_displacement.setValue(m_displacement);
    }
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINTACTUATOR_INL
