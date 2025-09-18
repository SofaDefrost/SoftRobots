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

#include <SoftRobots/component/initSoftRobots.h>
#include <SoftRobots/component/constraint/model/JointModel.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/behavior/ConstraintResolution.h>

namespace softrobots::constraint
{

using sofa::linearalgebra::BaseVector;
using sofa::core::ConstraintParams;
using sofa::type::Vec;
using sofa::core::visual::VisualParams;
using sofa::core::behavior::ConstraintResolution;

// Forward declarations for constraint resolution classes
class JointModelDisplacementConstraintResolution;
class JointModelForceConstraintResolution;

/**
 * Joint constraint for direct simulation (not inverse problem).
 * This component applies forces/torques or enforces displacements on joints.
 * It supports different types of joints: revolute, prismatic, spherical, planar, cylindrical, and universal.
 * Based on the JointModel class for kinematic computation.
 * Similar to JointConstraint but uses the more comprehensive JointModel.
 */
template< class DataTypes >
class JointModelConstraint : public JointModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(JointModelConstraint,DataTypes), SOFA_TEMPLATE(JointModel,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord> DataVecCoord;
    typedef sofa::Data<VecDeriv> DataVecDeriv;
    typedef sofa::Data<MatrixDeriv> DataMatrixDeriv;

public:
    JointModelConstraint(MechanicalState* object = nullptr);
    ~JointModelConstraint() override;

    /////////////// Inherited from BaseObject //////////////////////
    void init() override;
    void reinit() override;
    ///////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////////// Inherited from BaseConstraint ///////////////
    void getConstraintResolution(const sofa::core::ConstraintParams *cParam,
                                 std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;

    ////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint /////////////
    void storeLambda(const ConstraintParams* cParams,
                     sofa::core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    using JointModel<DataTypes>::m_state;
    using JointModel<DataTypes>::d_constraintIndex;
    using JointModel<DataTypes>::m_nbLines;
    using JointModel<DataTypes>::d_componentState;
    using JointModel<DataTypes>::d_force;
    using JointModel<DataTypes>::d_displacement;
    using JointModel<DataTypes>::d_maxForce;
    using JointModel<DataTypes>::d_minForce;
    using JointModel<DataTypes>::d_maxDisplacement;
    using JointModel<DataTypes>::d_minDisplacement;
    ////////////////////////////////////////////////////////////////////////////

    // Control mode
    sofa::Data<sofa::helper::OptionsGroup> d_valueType; ///< Control mode: displacement or force
    
    double m_initDisplacement{0.};
    double m_currentDisplacement{0.};

private:
    void setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce);
    void setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement);
};

/**
 * Constraint resolution for displacement-controlled joint
 */
class SOFA_SOFTROBOTS_API JointModelDisplacementConstraintResolution : public ConstraintResolution
{
public:
    JointModelDisplacementConstraintResolution(const double &imposedDisplacement, const double& min, const double& max);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, double** w, double *lambda) override;
    void resolution(int line, double** w, double* d, double* lambda, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
    double m_wActuatorActuator;
    double m_imposedDisplacement;
    double m_minForce;
    double m_maxForce;
};

/**
 * Constraint resolution for force-controlled joint
 */
class SOFA_SOFTROBOTS_API JointModelForceConstraintResolution : public ConstraintResolution
{
public:
    JointModelForceConstraintResolution(const double& imposedForce, const double& min, const double& max);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, double** w, double *force) override;
    void resolution(int line, double** w, double* d, double* force, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
    double m_wActuatorActuator;
    double m_imposedForce;
    double m_minDisplacement;
    double m_maxDisplacement;
};

#if !defined(SOFTROBOTS_JOINTMODELCONSTRAINT_CPP)
extern template class SOFA_SOFTROBOTS_API JointModelConstraint<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API JointModelConstraint<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_API JointModelConstraint<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace softrobots::constraint