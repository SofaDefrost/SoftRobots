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

#include <SoftRobots/component/constraint/model/CableModel.h>

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


class CableDisplacementConstraintResolution : public ConstraintResolution
{
public:
    CableDisplacementConstraintResolution(const SReal &imposedDisplacement, const SReal& min, const SReal& max);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, SReal** w, SReal *lambda) override;
    void resolution(int line, SReal** w, SReal* d, SReal* lambda, SReal* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    SReal m_wActuatorActuator;
    SReal m_imposedDisplacement;
    SReal m_minForce;
    SReal m_maxForce;

};

class CableForceConstraintResolution : public ConstraintResolution
{
public:
    CableForceConstraintResolution(const SReal& imposedForce, const SReal& min, const SReal& max);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, SReal** w, SReal *force) override;
    void resolution(int line, SReal** w, SReal* d, SReal* force, SReal* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    SReal m_wActuatorActuator;
    SReal m_imposedForce;
    SReal m_minDisplacement;
    SReal m_maxDisplacement;

};



/**
 * This component simulates a force exerted by a cable to solve an effector constraint.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class CableConstraint : public CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableConstraint,DataTypes), SOFA_TEMPLATE(CableModel,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;

    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;

public:
    CableConstraint(MechanicalState* object = nullptr);
    ~CableConstraint() override;

    /////////////// Inherited from BaseObject //////////////////////
    void init() override;
    void reinit() override;
    ///////////////////////////////////////////////////////////////


    /////////////////// Inherited from BaseConstraint ///////////////
    void getConstraintResolution(const sofa::core::ConstraintParams *cParam,
                                 std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;
    ////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    using CableModel<DataTypes>::d_maxDispVariation ;
    using CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using CableModel<DataTypes>::d_eqDisplacement ;
    using CableModel<DataTypes>::d_eqForce ;
    using CableModel<DataTypes>::d_maxForce ;
    using CableModel<DataTypes>::d_minForce ;
    using CableModel<DataTypes>::d_displacement ;
    using CableModel<DataTypes>::d_componentState ;
    ///////////////////////////////////////////////////////////////////////////

protected:
    //Input data
    Data<sofa::type::vector< Real > >       d_value;
    Data<unsigned int>                  d_valueIndex;
    Data<sofa::helper::OptionsGroup>          d_valueType;
                                        // displacement = the constraint will impose the displacement provided in data d_inputValue[d_iputIndex]
                                        // force = the constraint will impose the force provided in data d_inputValue[d_iputIndex]

    void internalInit();

private:
    void setUpDisplacementLimits(Real& imposedValue, Real& minForce, Real& maxForce);
    void setUpForceLimits(Real& imposedValue, Real& minDisplacement, Real& maxDisplacement);
};

#if !defined(SOFTROBOTS_CABLECONSTRAINT_CPP)
    extern template class SOFA_SOFTROBOTS_API CableConstraint<sofa::defaulttype::Vec3Types>;
    extern template class SOFA_SOFTROBOTS_API CableConstraint<sofa::defaulttype::Vec2Types>;
#endif

} // namespace
