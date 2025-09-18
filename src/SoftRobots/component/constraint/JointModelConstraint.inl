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

#include <SoftRobots/component/constraint/JointModelConstraint.h>
#include <sofa/helper/logging/Messaging.h>
#include <algorithm>

namespace softrobots::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;

/////////////////////////////////////////// JointModelConstraint //////////////////////////////////////////

template<class DataTypes>
JointModelConstraint<DataTypes>::JointModelConstraint(MechanicalState* object)
    : JointModel<DataTypes>(object)
    , d_valueType(initData(&d_valueType, sofa::helper::OptionsGroup(2, "displacement", "force"), "valueType", "Control mode: displacement or force"))
{
}

template<class DataTypes>
JointModelConstraint<DataTypes>::~JointModelConstraint()
{
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::init()
{
    JointModel<DataTypes>::init();
    
    ReadAccessor<typename JointModel<DataTypes>::DataVecCoord> positions = m_state->readPositions();
    
    // Initialize displacement tracking
    JointType jointType = this->getJointTypeFromData();
    switch(jointType)
    {
        case JointModel<DataTypes>::REVOLUTE:
        case JointModel<DataTypes>::CYLINDRICAL:
            m_initDisplacement = this->computeJointAngle(positions.ref());
            break;
        case JointModel<DataTypes>::PRISMATIC:
            m_initDisplacement = this->computeJointPosition(positions.ref());
            break;
        default:
            m_initDisplacement = 0.0;
            break;
    }
    
    m_currentDisplacement = m_initDisplacement;
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::reinit()
{
    JointModel<DataTypes>::reinit();
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                           DataMatrixDeriv &cMatrix,
                                                           unsigned int &cIndex,
                                                           const DataVecCoord &x)
{
    // Use the base class implementation
    JointModel<DataTypes>::buildConstraintMatrix(cParams, cMatrix, cIndex, x);
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                            BaseVector *resV,
                                                            const BaseVector *Jdx)
{
    ReadAccessor<typename JointModel<DataTypes>::DataVecCoord> positions = m_state->readPositions();
    
    JointType jointType = this->getJointTypeFromData();
    unsigned int cIndex = d_constraintIndex.getValue();
    
    // Update current displacement/angle
    switch(jointType)
    {
        case JointModel<DataTypes>::REVOLUTE:
        case JointModel<DataTypes>::CYLINDRICAL:
            m_currentDisplacement = this->computeJointAngle(positions.ref());
            break;
        case JointModel<DataTypes>::PRISMATIC:
            m_currentDisplacement = this->computeJointPosition(positions.ref());
            break;
        default:
            m_currentDisplacement = 0.0;
            break;
    }
    
    // Calculate constraint violation based on control mode
    unsigned int valueTypeIndex = d_valueType.getValue().getSelectedId();
    
    if (valueTypeIndex == 0) // displacement mode
    {
        double targetDisplacement = m_initDisplacement + d_displacement.getValue();
        double violation = m_currentDisplacement - targetDisplacement;
        resV->set(cIndex, violation);
    }
    else // force mode
    {
        // For force mode, violation is based on Jdx (free movement)
        if (Jdx)
        {
            resV->set(cIndex, Jdx->element(cIndex));
        }
        else
        {
            resV->set(cIndex, 0.0);
        }
    }
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::getConstraintResolution(const sofa::core::ConstraintParams *cParam,
                                                             std::vector<ConstraintResolution*>& resTab,
                                                             unsigned int& offset)
{
    SOFA_UNUSED(cParam);
    
    unsigned int valueTypeIndex = d_valueType.getValue().getSelectedId();
    
    if (valueTypeIndex == 0) // displacement mode
    {
        double imposedValue;
        double minForce, maxForce;
        setUpDisplacementLimits(imposedValue, minForce, maxForce);
        
        resTab[offset] = new JointModelDisplacementConstraintResolution(imposedValue, minForce, maxForce);
    }
    else // force mode
    {
        double imposedValue;
        double minDisplacement, maxDisplacement;
        setUpForceLimits(imposedValue, minDisplacement, maxDisplacement);
        
        resTab[offset] = new JointModelForceConstraintResolution(imposedValue, minDisplacement, maxDisplacement);
    }
    
    offset += m_nbLines;
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                 sofa::core::MultiVecDerivId res,
                                                 const BaseVector* lambda)
{
    JointModel<DataTypes>::storeLambda(cParams, res, lambda);
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce)
{
    imposedValue = d_displacement.getValue();
    minForce = static_cast<double>(d_minForce.getValue());
    maxForce = static_cast<double>(d_maxForce.getValue());
}

template<class DataTypes>
void JointModelConstraint<DataTypes>::setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement)
{
    imposedValue = d_force.getValue();
    minDisplacement = static_cast<double>(d_minDisplacement.getValue());
    maxDisplacement = static_cast<double>(d_maxDisplacement.getValue());
}

/////////////////////////////////////////// JointModelDisplacementConstraintResolution ///////////////////////////////////////////

JointModelDisplacementConstraintResolution::JointModelDisplacementConstraintResolution(const double &imposedDisplacement,
                                                                                      const double& min,
                                                                                      const double& max)
    : ConstraintResolution(1)
    , m_wActuatorActuator(0.0)
    , m_imposedDisplacement(imposedDisplacement)
    , m_minForce(min)
    , m_maxForce(max)
{
}

void JointModelDisplacementConstraintResolution::init(int line, double** w, double *lambda)
{
    SOFA_UNUSED(lambda);
    m_wActuatorActuator = w[line][line];
}

void JointModelDisplacementConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(w);
    
    // Compute the force needed to achieve the desired displacement
    double force = -dfree[line] / m_wActuatorActuator;
    
    // Apply force limits
    if (m_maxForce != m_minForce)
    {
        force = std::max(m_minForce, std::min(m_maxForce, force));
    }
    
    lambda[line] = force;
    d[line] = m_wActuatorActuator * lambda[line] + dfree[line];
}

/////////////////////////////////////////// JointModelForceConstraintResolution ///////////////////////////////////////////

JointModelForceConstraintResolution::JointModelForceConstraintResolution(const double& imposedForce,
                                                                        const double& min,
                                                                        const double& max)
    : ConstraintResolution(1)
    , m_wActuatorActuator(0.0)
    , m_imposedForce(imposedForce)
    , m_minDisplacement(min)
    , m_maxDisplacement(max)
{
}

void JointModelForceConstraintResolution::init(int line, double** w, double *force)
{
    SOFA_UNUSED(force);
    m_wActuatorActuator = w[line][line];
}

void JointModelForceConstraintResolution::resolution(int line, double** w, double* d, double* force, double* dfree)
{
    SOFA_UNUSED(w);
    
    // Apply the imposed force directly
    force[line] = m_imposedForce;
    
    // Compute resulting displacement
    double displacement = m_wActuatorActuator * force[line] + dfree[line];
    
    // Apply displacement limits if needed
    if (m_maxDisplacement != m_minDisplacement)
    {
        if (displacement > m_maxDisplacement || displacement < m_minDisplacement)
        {
            // Clamp displacement and recompute force
            displacement = std::max(m_minDisplacement, std::min(m_maxDisplacement, displacement));
            force[line] = (displacement - dfree[line]) / m_wActuatorActuator;
        }
    }
    
    d[line] = displacement;
}

} // namespace softrobots::constraint