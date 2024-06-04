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

#include <SoftRobots/component/constraint/JointConstraint.h>

using sofa::helper::OptionsGroup;

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;

template<class DataTypes>
JointConstraint<DataTypes>::JointConstraint(MechanicalState* object)
    : Inherit1(object)

    , d_index(initData(&d_index, "index",
                          "Index of the node subjected to the force. \n"
                          "If no index given, first node of mechanical context considered."))

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

    , d_maxDisplacement(initData(&d_maxDisplacement,"maxDisplacement",
                                 "Maximum displacement. \n"
                                 "If unspecified no maximum value will be considered."))

    , d_minDisplacement(initData(&d_minDisplacement,"minDisplacement",
                                 "Minimum displacement. \n"
                                 "If unspecified no minimum value will be considered."))

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueType(initData(&d_valueType, {"displacement","force"}, "valueType",
                                          "displacement = the constraint will impose the displacement provided in data value[valueIndex] \n"
                                          "force = the constraint will impose the force provided in data value[valueIndex] \n"
                                          "If unspecified, the default value is displacement"))
{
    d_force.setReadOnly(true);
    d_displacement.setReadOnly(true);
}

template<class DataTypes>
JointConstraint<DataTypes>::~JointConstraint()
{
}

template<class DataTypes>
void JointConstraint<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    Inherit1::init();

    if(m_state==nullptr){
        msg_error() << "There is no mechanical state associated with this node. "
                       "the object is deactivated. "
                       "To remove this error message fix your scene possibly by "
                       "adding a MechanicalObject." ;
        return;
    }

    if (!d_index.isSet())
    {
        msg_warning(this)<<"No index of actuation given, set default (first point of context MechanicalState).";
        d_index.setValue(0);
    }
  
    checkIndicesRegardingState();

    internalInit();

    d_componentState.setValue(ComponentState::Valid);
}

template<class DataTypes>
void JointConstraint<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void JointConstraint<DataTypes>::internalInit()
{
    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    m_initDisplacement = positions[d_index.getValue()][0];
    d_displacement.setValue(0);
    d_force.setValue(0);

}

template<class DataTypes>
void JointConstraint<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();

     if (positions.size() <= d_index.getValue())
            msg_error(this) << "Index is too large regarding mechanicalState [position] size" ;

}

template<class DataTypes>
void JointConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                          DataMatrixDeriv &cMatrix,
                                                          unsigned int &cIndex,
                                                          const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    if(!this->isComponentStateValid())
        return ;

    auto constraintIndex = sofa::helper::getWriteAccessor(m_constraintIndex);
    constraintIndex.wref() = cIndex;

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);
    rowIterator.addCol(d_index.getValue(), Deriv(1.));
    cIndex++;

    cMatrix.endEdit();

    m_nbLines = cIndex - constraintIndex;
}

template<class DataTypes>
void JointConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                         BaseVector *resV,
                                                         const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);

    Real dFree = Jdx->element(0) - m_initDisplacement + m_currentDisplacement;
    resV->set(m_constraintIndex.getValue(), dFree);
}

template<class DataTypes>
void JointConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParam,
                                                         std::vector<ConstraintResolution*>& resTab,
                                                         unsigned int& offset)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParam);

    double imposedValue = d_value.getValue();

    if(d_valueType.getValue().getSelectedItem() == "displacement") // displacement
    {
        double maxForce = std::numeric_limits<double>::max();
        double minForce = -maxForce;
        setUpDisplacementLimits(imposedValue, minForce, maxForce);

        JointDisplacementConstraintResolution *cr=  new JointDisplacementConstraintResolution(imposedValue, minForce, maxForce);
        resTab[offset++] = cr;

    }
    else // force
    {
        double maxDisplacement = std::numeric_limits<double>::max();
        double minDisplacement = -maxDisplacement;
        setUpForceLimits(imposedValue, minDisplacement, maxDisplacement);

        JointForceConstraintResolution *cr=  new JointForceConstraintResolution(imposedValue, minDisplacement, maxDisplacement);
        resTab[offset++] = cr;
        
    }
}

template<class DataTypes>
void JointConstraint<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                sofa::core::MultiVecDerivId res,
                                                const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    // Update joint effort
    d_force.setValue(lambda->element(m_constraintIndex.getValue()));

    // Update joint displacement
    ReadAccessor<sofa::Data<VecCoord>> positions = m_state->readPositions();
    m_currentDisplacement = positions[d_index.getValue()][0];
    d_displacement.setValue(- m_initDisplacement + m_currentDisplacement);
}


template<class DataTypes>
void JointConstraint<DataTypes>::setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce)
{
    if(d_maxDisplacement.isSet() && imposedValue>d_maxDisplacement.getValue())
        imposedValue = d_maxDisplacement.getValue();

    if(d_minDisplacement.isSet() && imposedValue<d_minDisplacement.getValue())
        imposedValue = d_minDisplacement.getValue();

    if(d_minForce.isSet())
        minForce=d_minForce.getValue();
    if(d_maxForce.isSet())
        maxForce=d_maxForce.getValue();
}

template<class DataTypes>
void JointConstraint<DataTypes>::setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement)
{
    if(d_maxForce.isSet() && imposedValue>d_maxForce.getValue())
        imposedValue = d_maxForce.getValue();

    if(d_minForce.isSet() && imposedValue<d_minForce.getValue())
        imposedValue = d_minForce.getValue();

    if(d_minDisplacement.isSet())
        minDisplacement=d_minDisplacement.getValue();
    if(d_maxDisplacement.isSet())
        maxDisplacement=d_maxDisplacement.getValue();
}


} // namespace

