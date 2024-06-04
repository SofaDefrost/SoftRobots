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
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/behavior/ConstraintResolution.h>
#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

namespace softrobots::constraint
{
using sofa::linearalgebra::BaseVector;
using sofa::core::ConstraintParams;
using sofa::type::Vec;
using sofa::core::visual::VisualParams ;
using sofa::core::behavior::ConstraintResolution;
using sofa::core::behavior::SoftRobotsConstraint;
using sofa::core::behavior::SoftRobotsBaseConstraint;


class JointDisplacementConstraintResolution : public ConstraintResolution
{
public:
    JointDisplacementConstraintResolution(const double &imposedDisplacement, const double& min, const double& max);

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

class JointForceConstraintResolution : public ConstraintResolution
{
public:
    JointForceConstraintResolution(const double& imposedForce, const double& min, const double& max);

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
class JointConstraint : public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(JointConstraint,DataTypes), SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                                  DataVecCoord;
    typedef sofa::Data<VecDeriv>                                  DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                               DataMatrixDeriv;

    typedef sofa::type::vector<unsigned int> SetIndexArray;
    using SoftRobotsBaseConstraint::m_constraintType ;

public:
    JointConstraint(MechanicalState* object = nullptr);
    ~JointConstraint() override;

    /////////////// Inherited from BaseObject //////////////////////
    void init() override;
    void reinit() override;
    ///////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
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
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::m_constraintIndex ;
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////

    //Input data
    sofa::Data<unsigned int>       d_index; ///< indice of considered node

    sofa::Data<double>                d_force; ///< force applied on the points
    sofa::Data<double>                d_displacement; ///< displacement of the points

    sofa::Data<double>                  d_maxForce; ///< maximum force applied on the points
    sofa::Data<double>                  d_minForce; ///< minimum force applied on the points
    
    sofa::Data<double>                  d_maxDisplacement; ///< maximum displacement of the points
    sofa::Data<double>                  d_minDisplacement; ///< minimum displacement of the points

    sofa::Data<double> d_value;
    sofa::Data<sofa::helper::OptionsGroup>  d_valueType;
                                            // displacement = the constraint will impose the displacement provided in data d_inputValue[d_iputIndex]
                                            // force = the constraint will impose the force provided in data d_inputValue[d_iputIndex]
    void internalInit();
    void checkIndicesRegardingState();

    double m_initDisplacement;
    double m_currentDisplacement;

private:
    void setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce);
    void setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement);
};

#if !defined(SOFTROBOTS_JOINTCONSTRAINT_CPP)
extern template class SOFA_SOFTROBOTS_API JointConstraint<sofa::defaulttype::Vec1Types>;
#endif


} // namespace

