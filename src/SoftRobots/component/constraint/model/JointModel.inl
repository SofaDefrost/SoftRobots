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

#include <SoftRobots/component/constraint/model/JointModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/type/RGBAColor.h>
#include <cmath>

namespace softrobots::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::helper::AdvancedTimer;
using sofa::helper::WriteAccessor;
using sofa::type::RGBAColor;

template<class DataTypes>
JointModel<DataTypes>::JointModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
    , d_jointType(initData(&d_jointType, sofa::helper::OptionsGroup(6,
                                          "revolute",
                                          "prismatic", 
                                          "spherical",
                                          "planar",
                                          "cylindrical",
                                          "universal"),
                           "jointType", "Type of joint constraint"))
    , d_indices(initData(&d_indices, "indices", "Indices of the two bodies connected by the joint"))
    , d_jointPosition(initData(&d_jointPosition, "jointPosition", "Position of the joint center"))
    , d_jointAxis1(initData(&d_jointAxis1, "jointAxis1", "Primary axis of the joint"))
    , d_jointAxis2(initData(&d_jointAxis2, "jointAxis2", "Secondary axis of the joint"))
    , d_jointAxis3(initData(&d_jointAxis3, "jointAxis3", "Third axis of the joint"))
    , d_jointAngle(initData(&d_jointAngle, Real(0.0), "jointAngle", "Current joint angle"))
    , d_jointPosition1D(initData(&d_jointPosition1D, Real(0.0), "jointPosition1D", "Current joint position"))
    , d_jointAngles3D(initData(&d_jointAngles3D, Vec3(0.0, 0.0, 0.0), "jointAngles3D", "Current joint angles (3D)"))
    , d_minAngle(initData(&d_minAngle, Real(-M_PI), "minAngle", "Minimum joint angle"))
    , d_maxAngle(initData(&d_maxAngle, Real(M_PI), "maxAngle", "Maximum joint angle"))
    , d_minPosition(initData(&d_minPosition, Real(-1.0), "minPosition", "Minimum joint position"))
    , d_maxPosition(initData(&d_maxPosition, Real(1.0), "maxPosition", "Maximum joint position"))
    , d_force(initData(&d_force, double(0.0), "force", "Applied force/torque"))
    , d_displacement(initData(&d_displacement, double(0.0), "displacement", "Joint displacement/rotation"))
    , d_maxForce(initData(&d_maxForce, Real(1e12), "maxForce", "Maximum force/torque"))
    , d_minForce(initData(&d_minForce, Real(-1e12), "minForce", "Minimum force/torque"))
    , d_eqForce(initData(&d_eqForce, Real(0.0), "eqForce", "Equality force/torque"))
    , d_maxDisplacement(initData(&d_maxDisplacement, Real(1e12), "maxDisplacement", "Maximum displacement/rotation"))
    , d_minDisplacement(initData(&d_minDisplacement, Real(-1e12), "minDisplacement", "Minimum displacement/rotation"))
    , d_eqDisplacement(initData(&d_eqDisplacement, Real(0.0), "eqDisplacement", "Equality displacement/rotation"))
    , d_drawJoint(initData(&d_drawJoint, false, "drawJoint", "Draw joint representation"))
    , d_drawAxis(initData(&d_drawAxis, false, "drawAxis", "Draw joint axes"))
    , d_color(initData(&d_color, sofa::type::RGBAColor(1.0f, 0.35f, 0.35f, 1.0f), "color", "Color for visualization"))
{
    setUpData();
}

template<class DataTypes>
JointModel<DataTypes>::~JointModel()
{
}

template<class DataTypes>
void JointModel<DataTypes>::init()
{
    SoftRobotsConstraint<DataTypes>::init();
    internalInit();
}

template<class DataTypes>
void JointModel<DataTypes>::bwdInit()
{
    SoftRobotsConstraint<DataTypes>::bwdInit();
    initJointGeometry();
}

template<class DataTypes>
void JointModel<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void JointModel<DataTypes>::reset()
{
    d_jointAngle.setValue(m_initialAngle);
    d_jointPosition1D.setValue(m_initialPosition);
    d_jointAngles3D.setValue(m_initialAngles3D);
}

template<class DataTypes>
void JointModel<DataTypes>::setUpData()
{
    // Set default values based on joint type
    JointType jointType = getJointTypeFromData();
    
    switch(jointType)
    {
        case REVOLUTE:
            m_nbLines = 1; // One rotational DOF
            break;
        case PRISMATIC:
            m_nbLines = 1; // One translational DOF
            break;
        case SPHERICAL:
            m_nbLines = 3; // Three rotational DOFs
            break;
        case PLANAR:
            m_nbLines = 3; // Two translations + one rotation
            break;
        case CYLINDRICAL:
            m_nbLines = 2; // One translation + one rotation
            break;
        case UNIVERSAL:
            m_nbLines = 2; // Two rotations
            break;
        default:
            m_nbLines = 1;
            break;
    }
}

template<class DataTypes>
void JointModel<DataTypes>::internalInit()
{
    checkIndicesRegardingState();
    setUpData();
    
    if (d_indices.getValue().size() < 2)
    {
        msg_warning() << "JointModel: At least two body indices must be specified. Setting default values.";
        SetIndexArray indices = {0, 1};
        d_indices.setValue(indices);
    }
}

template<class DataTypes>
void JointModel<DataTypes>::checkIndicesRegardingState()
{
    const SetIndexArray &indices = d_indices.getValue();
    
    if (m_state == nullptr)
    {
        msg_warning() << "No mechanical state associated with JointModel.";
        return;
    }
    
    for (unsigned int index : indices)
    {
        if (index >= m_state->getSize())
        {
            msg_warning() << "Index " << index << " is out of bounds. The mechanical object size is " << m_state->getSize();
        }
    }
}

template<class DataTypes>
void JointModel<DataTypes>::initJointGeometry()
{
    ReadAccessor<DataVecCoord> positions = m_state->readPositions();
    
    // Store initial joint configuration
    m_initialAngle = computeJointAngle(positions.ref());
    m_initialPosition = computeJointPosition(positions.ref());
    m_initialAngles3D = computeJointAngles3D(positions.ref());
    
    d_jointAngle.setValue(m_initialAngle);
    d_jointPosition1D.setValue(m_initialPosition);
    d_jointAngles3D.setValue(m_initialAngles3D);
}

template<class DataTypes>
typename JointModel<DataTypes>::JointType JointModel<DataTypes>::getJointTypeFromData()
{
    unsigned int jointTypeIndex = d_jointType.getValue().getSelectedId();
    return static_cast<JointType>(jointTypeIndex);
}

template<class DataTypes>
void JointModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                  DataMatrixDeriv &cMatrix,
                                                  unsigned int &cIndex,
                                                  const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    
    const VecCoord& positions = x.getValue();
    MatrixDerivRowIterator rowIt = cMatrix.writeLine(cIndex);
    
    JointType jointType = getJointTypeFromData();
    
    switch(jointType)
    {
        case REVOLUTE:
            computeRevoluteConstraint(positions, rowIt, nullptr, cIndex);
            break;
        case PRISMATIC:
            computePrismaticConstraint(positions, rowIt, nullptr, cIndex);
            break;
        case SPHERICAL:
            computeSphericalConstraint(positions, rowIt, nullptr, cIndex);
            break;
        default:
            computeRevoluteConstraint(positions, rowIt, nullptr, cIndex);
            break;
    }
    
    cIndex += m_nbLines;
}

template<class DataTypes>
void JointModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                   BaseVector *resV,
                                                   const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(Jdx);
    
    ReadAccessor<DataVecCoord> positions = m_state->readPositions();
    
    JointType jointType = getJointTypeFromData();
    unsigned int cIndex = d_constraintIndex.getValue();
    
    switch(jointType)
    {
        case REVOLUTE:
        {
            Real currentAngle = computeJointAngle(positions.ref());
            Real targetAngle = m_initialAngle + static_cast<Real>(d_displacement.getValue());
            Real violation = currentAngle - targetAngle;
            resV->set(cIndex, violation);
            break;
        }
        case PRISMATIC:
        {
            Real currentPosition = computeJointPosition(positions.ref());
            Real targetPosition = m_initialPosition + static_cast<Real>(d_displacement.getValue());
            Real violation = currentPosition - targetPosition;
            resV->set(cIndex, violation);
            break;
        }
        case SPHERICAL:
        {
            Vec3 currentAngles = computeJointAngles3D(positions.ref());
            Vec3 targetAngles = m_initialAngles3D + Vec3(d_displacement.getValue(), 0.0, 0.0);
            for (unsigned int i = 0; i < 3; i++)
            {
                resV->set(cIndex + i, currentAngles[i] - targetAngles[i]);
            }
            break;
        }
        default:
            break;
    }
}

template<class DataTypes>
void JointModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                        sofa::core::MultiVecDerivId res,
                                        const BaseVector* lambda)
{
    SoftRobotsConstraint<DataTypes>::storeLambda(cParams, res, lambda);
    
    unsigned int cIndex = d_constraintIndex.getValue();
    double force = lambda->element(cIndex);
    d_force.setValue(force);
}

template<class DataTypes>
typename JointModel<DataTypes>::Real JointModel<DataTypes>::computeJointAngle(const VecCoord &positions)
{
    const SetIndexArray &indices = d_indices.getValue();
    if (indices.size() < 2) return Real(0.0);
    
    const Coord& pos1 = positions[indices[0]];
    const Coord& pos2 = positions[indices[1]];
    const Coord& center = d_jointPosition.getValue();
    const Coord& axis = d_jointAxis1.getValue();
    
    // Compute vectors from joint center to bodies
    Coord v1 = pos1 - center;
    Coord v2 = pos2 - center;
    
    // Project vectors onto plane perpendicular to joint axis
    v1 = v1 - axis * (v1 * axis);
    v2 = v2 - axis * (v2 * axis);
    
    // Normalize vectors
    Real norm1 = v1.norm();
    Real norm2 = v2.norm();
    
    if (norm1 < 1e-12 || norm2 < 1e-12)
        return Real(0.0);
        
    v1 /= norm1;
    v2 /= norm2;
    
    // Compute angle using dot product
    Real dot = v1 * v2;
    dot = std::max(Real(-1.0), std::min(Real(1.0), dot)); // Clamp for numerical stability
    Real angle = std::acos(dot);
    
    // Determine sign using cross product
    Coord cross = v1.cross(v2);
    if (cross * axis < 0)
        angle = -angle;
    
    return angle;
}

template<class DataTypes>
typename JointModel<DataTypes>::Real JointModel<DataTypes>::computeJointPosition(const VecCoord &positions)
{
    const SetIndexArray &indices = d_indices.getValue();
    if (indices.size() < 2) return Real(0.0);
    
    const Coord& pos1 = positions[indices[0]];
    const Coord& pos2 = positions[indices[1]];
    const Coord& axis = d_jointAxis1.getValue();
    
    // Project relative position onto joint axis
    Coord relativePos = pos2 - pos1;
    Real position = relativePos * axis;
    
    return position;
}

template<class DataTypes>
sofa::type::Vec3 JointModel<DataTypes>::computeJointAngles3D(const VecCoord &positions)
{
    // For spherical joints, compute Euler angles
    // Simplified implementation - would need proper rotation matrix analysis
    Real angleX = computeJointAngle(positions);
    // Additional angles would require more sophisticated computation
    return Vec3(angleX, 0.0, 0.0);
}

template<class DataTypes>
void JointModel<DataTypes>::computeRevoluteConstraint(const VecCoord &positions,
                                                     MatrixDerivRowIterator &rowIt,
                                                     BaseVector *resV,
                                                     unsigned int &cIndex)
{
    const SetIndexArray &indices = d_indices.getValue();
    if (indices.size() < 2) return;
    
    const Coord& pos1 = positions[indices[0]];
    const Coord& pos2 = positions[indices[1]];
    const Coord& center = d_jointPosition.getValue();
    const Coord& axis = d_jointAxis1.getValue();
    
    // Compute jacobian for revolute joint
    // ∂θ/∂pos1 and ∂θ/∂pos2
    Coord v1 = pos1 - center;
    Coord v2 = pos2 - center;
    
    // Remove component along axis
    v1 = v1 - axis * (v1 * axis);
    v2 = v2 - axis * (v2 * axis);
    
    Real norm1 = v1.norm();
    Real norm2 = v2.norm();
    
    if (norm1 > 1e-12 && norm2 > 1e-12)
    {
        // Jacobian computation for angle constraint
        Coord grad1 = axis.cross(v1) / (norm1 * norm1);
        Coord grad2 = axis.cross(v2) / (norm2 * norm2);
        
        rowIt.setCol(indices[0], -grad1);
        rowIt.setCol(indices[1], grad2);
    }
}

template<class DataTypes>
void JointModel<DataTypes>::computePrismaticConstraint(const VecCoord &positions,
                                                      MatrixDerivRowIterator &rowIt,
                                                      BaseVector *resV,
                                                      unsigned int &cIndex)
{
    const SetIndexArray &indices = d_indices.getValue();
    if (indices.size() < 2) return;
    
    const Coord& axis = d_jointAxis1.getValue();
    
    // Jacobian for prismatic joint is simply the axis direction
    rowIt.setCol(indices[0], -axis);
    rowIt.setCol(indices[1], axis);
}

template<class DataTypes>
void JointModel<DataTypes>::computeSphericalConstraint(const VecCoord &positions,
                                                       MatrixDerivRowIterator &rowIt,
                                                       BaseVector *resV,
                                                       unsigned int &cIndex)
{
    // For spherical joints, we would need 3 constraint equations
    // This is a simplified implementation
    computeRevoluteConstraint(positions, rowIt, resV, cIndex);
}

template<class DataTypes>
void JointModel<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;
        
    if (d_drawJoint.getValue())
        drawJoint(vparams);
        
    if (d_drawAxis.getValue())
        drawAxes(vparams);
}

template<class DataTypes>
void JointModel<DataTypes>::drawJoint(const VisualParams* vparams)
{
    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    vparams->drawTool()->disableLighting();
    
    RGBAColor color = d_color.getValue();
    vparams->drawTool()->setMaterial(color);
    
    const Coord& center = d_jointPosition.getValue();
    Real radius = 0.1; // Fixed radius for visualization
    
    // Draw a sphere at joint center
    vparams->drawTool()->drawSphere(center, radius);
}

template<class DataTypes>
void JointModel<DataTypes>::drawAxes(const VisualParams* vparams)
{
    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    vparams->drawTool()->disableLighting();
    
    const Coord& center = d_jointPosition.getValue();
    const Coord& axis1 = d_jointAxis1.getValue();
    Real axisLength = 0.2; // Fixed length for visualization
    
    // Draw primary axis in red
    vparams->drawTool()->setMaterial(RGBAColor::red());
    vparams->drawTool()->drawArrow(center, center + axis1 * axisLength, axisLength * 0.1);
    
    JointType jointType = getJointTypeFromData();
    if (jointType == UNIVERSAL || jointType == PLANAR || jointType == SPHERICAL)
    {
        const Coord& axis2 = d_jointAxis2.getValue();
        // Draw secondary axis in green
        vparams->drawTool()->setMaterial(RGBAColor::green());
        vparams->drawTool()->drawArrow(center, center + axis2 * axisLength, axisLength * 0.1);
    }
    
    if (jointType == SPHERICAL)
    {
        const Coord& axis3 = d_jointAxis3.getValue();
        // Draw third axis in blue
        vparams->drawTool()->setMaterial(RGBAColor::blue());
        vparams->drawTool()->drawArrow(center, center + axis3 * axisLength, axisLength * 0.1);
    }
}

} // namespace softrobots::constraint