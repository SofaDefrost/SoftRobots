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

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/type/Vec.h>

namespace softrobots::constraint
{

using softrobots::behavior::SoftRobotsConstraint;
using sofa::core::visual::VisualParams;
using sofa::core::objectmodel::Data;
using sofa::defaulttype::Vec3Types;
using sofa::defaulttype::Vec3fTypes;
using sofa::linearalgebra::BaseVector;
using sofa::core::ConstraintParams;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::type::Vec3;

/**
 * This class contains common implementation of joint constraints for articulated systems.
 * It supports different types of joints: revolute, prismatic, spherical, and planar.
 * The joint model computes the kinematic constraints and jacobians for inverse problem resolution.
 */
template< class DataTypes >
class SOFA_SOFTROBOTS_API JointModel : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(JointModel,DataTypes),
               SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;
    typedef sofa::type::vector<unsigned int> SetIndexArray;

public:
    
    enum JointType
    {
        REVOLUTE = 0,    ///< Revolute joint (rotation around one axis)
        PRISMATIC,       ///< Prismatic joint (translation along one axis)
        SPHERICAL,       ///< Spherical joint (rotation around all axes)
        PLANAR,          ///< Planar joint (motion in a plane)
        CYLINDRICAL,     ///< Cylindrical joint (rotation + translation along same axis)
        UNIVERSAL        ///< Universal joint (rotation around two axes)
    };

    JointModel(MechanicalState* object = nullptr);
    ~JointModel() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Actuator //////////////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                     sofa::core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

protected:

    // Joint configuration
    Data<sofa::helper::OptionsGroup>    d_jointType;        ///< Type of joint (revolute, prismatic, spherical, etc.)
    Data<SetIndexArray>                 d_indices;          ///< Indices of the two bodies connected by the joint
    Data<Coord>                         d_jointPosition;    ///< Position of the joint center
    Data<Coord>                         d_jointAxis1;       ///< Primary axis of the joint (rotation/translation)
    Data<Coord>                         d_jointAxis2;       ///< Secondary axis for universal/planar joints
    Data<Coord>                         d_jointAxis3;       ///< Third axis for spherical joints

    // Joint state
    Data<Real>                          d_jointAngle;       ///< Current joint angle (for revolute/cylindrical)
    Data<Real>                          d_jointPosition1D;  ///< Current joint position (for prismatic/cylindrical)
    Data<Vec3>                          d_jointAngles3D;    ///< Current joint angles (for spherical/universal)

    // Joint limits
    Data<Real>                          d_minAngle;         ///< Minimum joint angle
    Data<Real>                          d_maxAngle;         ///< Maximum joint angle
    Data<Real>                          d_minPosition;      ///< Minimum joint position
    Data<Real>                          d_maxPosition;      ///< Maximum joint position

    // Control parameters
    Data<double>                        d_force;            ///< Applied force/torque
    Data<double>                        d_displacement;     ///< Joint displacement/rotation
    
    // Limits for inverse problem
    Data<Real>                          d_maxForce;         ///< Maximum force/torque
    Data<Real>                          d_minForce;         ///< Minimum force/torque
    Data<Real>                          d_eqForce;          ///< Equality force/torque
    Data<Real>                          d_maxDisplacement;  ///< Maximum displacement/rotation
    Data<Real>                          d_minDisplacement;  ///< Minimum displacement/rotation
    Data<Real>                          d_eqDisplacement;   ///< Equality displacement/rotation

    // Visualization
    Data<bool>                          d_drawJoint;        ///< Draw joint representation
    Data<bool>                          d_drawAxis;         ///< Draw joint axes
    Data<sofa::type::RGBAColor>         d_color;            ///< Color for visualization

    // Joint kinematics computation methods
    Real computeJointAngle(const VecCoord &positions);
    Real computeJointPosition(const VecCoord &positions);
    Vec3 computeJointAngles3D(const VecCoord &positions);
    
    void computeJointJacobian(const VecCoord &positions, 
                             MatrixDerivRowIterator &rowIt, 
                             unsigned int &cIndex);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using SoftRobotsConstraint<DataTypes>::m_nbLines;
    using SoftRobotsConstraint<DataTypes>::d_constraintIndex;
    using SoftRobotsConstraint<DataTypes>::m_state;
    using SoftRobotsConstraint<DataTypes>::d_componentState;
    ////////////////////////////////////////////////////////////////////////////

private:
    void setUpData();
    void internalInit();
    void checkIndicesRegardingState();
    void initJointGeometry();
    
    JointType getJointTypeFromData();
    
    // Helper methods for different joint types
    void computeRevoluteConstraint(const VecCoord &positions, 
                                  MatrixDerivRowIterator &rowIt, 
                                  BaseVector *resV, 
                                  unsigned int &cIndex);
    
    void computePrismaticConstraint(const VecCoord &positions, 
                                   MatrixDerivRowIterator &rowIt, 
                                   BaseVector *resV, 
                                   unsigned int &cIndex);
    
    void computeSphericalConstraint(const VecCoord &positions, 
                                   MatrixDerivRowIterator &rowIt, 
                                   BaseVector *resV, 
                                   unsigned int &cIndex);

    void drawJoint(const VisualParams* vparams);
    void drawAxes(const VisualParams* vparams);

    // Internal state
    Real m_initialAngle;
    Real m_initialPosition;
    Vec3 m_initialAngles3D;
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_JOINTMODEL_CPP)
extern template class SOFA_SOFTROBOTS_API JointModel<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API JointModel<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_API JointModel<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace softrobots::constraint