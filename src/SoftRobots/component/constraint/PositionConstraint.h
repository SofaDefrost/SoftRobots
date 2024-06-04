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

#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/behavior/ConstraintResolution.h>

#include <SoftRobots/component/constraint/model/PositionModel.h>

namespace softrobots::constraint
{
using sofa::type::Vec;
using sofa::type::Vec3d;
using sofa::helper::WriteAccessor;
using sofa::type::vector;
using sofa::core::ConstraintParams;
using sofa::linearalgebra::BaseVector;
using sofa::core::visual::VisualParams ;
using sofa::core::behavior::ConstraintResolution;


class PositionDisplacementConstraintResolution : public ConstraintResolution
{
public:
    PositionDisplacementConstraintResolution(const double &imposedDisplacement, const double& min, const double& max);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, double** w, double *lambda) override;
    void resolution(int line, double** w, double* d, double* lambda, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    double      m_wActuatorActuator;
    double      m_imposedDisplacement;
    double      m_minForce;
    double      m_maxForce;

};

class PositionForceConstraintResolution : public ConstraintResolution
{
public:
    PositionForceConstraintResolution(const double& imposedForce, const double& min, const double& max);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, double** w, double *force) override;
    void resolution(int line, double** w, double* d, double* force, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    double      m_wActuatorActuator;
    double      m_imposedForce;
    double      m_minDisplacement;
    double      m_maxDisplacement;

};




template< class DataTypes >
class PositionConstraint : public PositionModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PositionConstraint,DataTypes), SOFA_TEMPLATE(PositionModel,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;

    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef sofa::Data<VecCoord>		DataVecCoord;
    typedef sofa::Data<VecDeriv>		DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>    DataMatrixDeriv;

public:
    PositionConstraint(MechanicalState* object = nullptr);
    ~PositionConstraint() override;

    /////////////// Inherited from BaseObject //////////////////////
    void init() override;
    void reinit() override;
    ///////////////////////////////////////////////////////////////

    /////////////////// Inherited from BaseConstraint ///////////////
    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;

    void getConstraintResolution(const sofa::core::ConstraintParams *cParam,
                                 std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;

    void storeLambda(const ConstraintParams* cParams,
                     sofa::core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    ////////////////////////////////////////////////////////////////


    ////////////////////////// Inherited attributes ////////////////////////////
    using PositionModel<DataTypes>::d_indices ;
    using PositionModel<DataTypes>::d_componentState ;
    using PositionModel<DataTypes>::d_useDirections ;
    using PositionModel<DataTypes>::d_directions ;
    using PositionModel<DataTypes>::d_weight ;
    using PositionModel<DataTypes>::m_constraintIndex ;
    using PositionModel<DataTypes>::m_state ;
    ///////////////////////////////////////////////////////////////////////////

protected:
    //Input data
    sofa::Data<double>                d_force; ///< force applied on the points
    sofa::Data<double>                d_displacement; ///< displacement of the points

    sofa::Data<Real>                  d_maxForce; ///< maximum force applied on the points
    sofa::Data<Real>                  d_minForce; ///< minimum force applied on the points

    sofa::Data<Real>                  d_maxPositiveDisplacement; ///< maximum displacement of the points in the positive direction
    sofa::Data<Real>                  d_maxNegativeDisplacement; ///< maximum displacement of the points in the negative direction

    sofa::Data<sofa::type::vector< Real > > d_value;
    sofa::Data<unsigned int>                d_valueIndex;
    sofa::Data<sofa::helper::OptionsGroup>  d_valueType;
                                            // displacement = the constraint will impose the displacement provided in data d_inputValue[d_iputIndex]
                                            // force = the constraint will impose the force provided in data d_inputValue[d_iputIndex]

    VecCoord                    m_x0;

    void internalInit();

private:
    void setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce);
    void setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement);
};

#if !defined(SOFTROBOTS_POSITIONCONSTRAINT_CPP)
extern template class PositionConstraint<sofa::defaulttype::Vec3Types>;
extern template class PositionConstraint<sofa::defaulttype::Vec2Types>;
extern template class PositionConstraint<sofa::defaulttype::Rigid3Types>;
#endif


} // namespace

