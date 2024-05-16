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

#include <SoftRobots/component/constraint/PositionConstraint.h>

using sofa::helper::OptionsGroup;

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;

template<class DataTypes>
PositionConstraint<DataTypes>::PositionConstraint(MechanicalState* object)
    : PositionModel<DataTypes>(object)

    , d_force(initData(&d_force,double(0.0), "force",
                                         "Output force. Warning: to get the actual force you should divide this value by dt."))

    , d_displacement(initData(&d_displacement,double(0.0), "displacement",
                          "Output displacement compared to the initial position."))

    , d_maxForce(initData(&d_maxForce, "maxForce",
                          "Maximum force allowed. \n"
                          "If unspecified no maximum value will be considered."))

    , d_minForce(initData(&d_minForce, "minForce",
                          "Minimum force allowed. \n"
                          "If unspecified no minimum value will be considered."))

    , d_maxPositiveDisplacement(initData(&d_maxPositiveDisplacement,"maxPositiveDisp",
                                         "Maximum displacement in the positive direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxNegativeDisplacement(initData(&d_maxNegativeDisplacement,"maxNegativeDisp",
                                         "Maximum displacement in the negative direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, {"displacement","force"}, "valueType",
                                          "displacement = the constraint will impose the displacement provided in data value[valueIndex] \n"
                                          "force = the constraint will impose the force provided in data value[valueIndex] \n"
                                          "If unspecified, the default value is displacement"))
{
    d_force.setGroup("Vector");
    d_displacement.setGroup("Vector");
    d_force.setReadOnly(true);
    d_displacement.setReadOnly(true);
}

template<class DataTypes>
PositionConstraint<DataTypes>::~PositionConstraint()
{
}

template<class DataTypes>
void PositionConstraint<DataTypes>::init()
{
    Inherit1::init();

    internalInit();

    ReadAccessor<sofa::Data<VecCoord> > x = m_state->readPositions();
    m_x0 = x;
}

template<class DataTypes>
void PositionConstraint<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void PositionConstraint<DataTypes>::internalInit()
{
    if(d_value.getValue().size()==0)
    {
        WriteAccessor<sofa::Data<vector<Real>>> value = d_value;
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
void PositionConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                         BaseVector *resV,
                                                         const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);

    ReadAccessor<sofa::Data<VecCoord>> x = m_state->readPositions() ;
    const auto& useDirections = sofa::helper::getReadAccessor(d_useDirections);
    const auto& directions = sofa::helper::getReadAccessor(d_directions);
    const auto& weight = sofa::helper::getReadAccessor(d_weight);
    const auto& indices = sofa::helper::getReadAccessor(d_indices);
    sofa::Index sizeIndices = indices.size();
    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    int index = 0;
    for (unsigned int i=0; i<sizeIndices; i++)
    {
        Coord pos = x[indices[i]];
        Coord pos0 = m_x0[indices[i]];

        Deriv d = DataTypes::coordDifference(pos, pos0);

        for(sofa::Size j=0; j<DataTypes::Deriv::total_size; j++)
            if(useDirections[j])
            {
                Real dfree = Jdx->element(index) + d*directions[j]*weight[j];
                resV->set(constraintIndex+index, dfree);
                index++;
            }
    }
}

template<class DataTypes>
void PositionConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParam,
                                                         std::vector<ConstraintResolution*>& resTab,
                                                         unsigned int& offset)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParam);

    double imposedValue=d_value.getValue()[d_valueIndex.getValue()];
    sofa::Size nbIndices = d_indices.getValue().size();

    if(d_valueType.getValue().getSelectedItem() == "displacement") // displacement
    {
        double maxForce = std::numeric_limits<double>::max();
        double minForce = -maxForce;
        setUpDisplacementLimits(imposedValue,minForce,maxForce);

        for (sofa::Size i=0; i<nbIndices; i++){
            PositionDisplacementConstraintResolution *cr=  new PositionDisplacementConstraintResolution(imposedValue, minForce, maxForce);
            resTab[offset++] = cr;
        }
    }
    else // force
    {
        double maxDisplacement = std::numeric_limits<double>::max();
        double minDisplacement = -maxDisplacement;
        setUpForceLimits(imposedValue,minDisplacement,maxDisplacement);

        for (sofa::Size i=0; i<nbIndices; i++){
            PositionForceConstraintResolution *cr=  new PositionForceConstraintResolution(imposedValue, minDisplacement, maxDisplacement);
            resTab[offset++] = cr;
        }
    }
}

template<class DataTypes>
void PositionConstraint<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                sofa::core::MultiVecDerivId res,
                                                const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    d_force.setValue(lambda->element(m_constraintIndex.getValue()));
}


template<class DataTypes>
void PositionConstraint<DataTypes>::setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce)
{
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
void PositionConstraint<DataTypes>::setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement)
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

