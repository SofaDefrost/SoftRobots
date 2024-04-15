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

#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>
#include <SoftRobots/component/initSoftRobots.h>

namespace softrobots::constraint
{

using sofa::core::ConstraintParams ;
using sofa::linearalgebra::BaseVector ;
using sofa::core::behavior::ConstraintResolution ;
using sofa::type::vector;

class VolumeGrowthConstraintResolution : public ConstraintResolution
{
public:
    VolumeGrowthConstraintResolution(const SReal& imposedVolumeGrowth, const SReal& minPressure, const SReal& maxPressure);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, SReal** w, SReal *lambda) override;
    void resolution(int line, SReal** w, SReal* d, SReal* lambda, SReal* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
    SReal m_wActuatorActuator;
    SReal m_imposedVolumeGrowth;
    SReal m_minPressure;
    SReal m_maxPressure;
};

class SurfacePressureConstraintResolution : public ConstraintResolution
{
public:
    SurfacePressureConstraintResolution(const SReal& imposedPressure, const SReal& minVolumeGrowth, const SReal& maxVolumeGrowth);

    //////////////////// Inherited from ConstraintResolution ///////////////////
    void init(int line, SReal** w, SReal *force) override;
    void resolution(int line, SReal** w, SReal* d, SReal* force, SReal* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
    SReal m_wActuatorActuator;
    SReal m_imposedPressure;
    SReal m_minVolumeGrowth;
    SReal m_maxVolumeGrowth;
};


/**
 * This component constrains a model by applying pressure on surfaces (for exemple cavities).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class SOFA_SOFTROBOTS_API SurfacePressureConstraint : public SurfacePressureModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureConstraint,DataTypes), SOFA_TEMPLATE(SurfacePressureModel,DataTypes));

    typedef typename DataTypes::VecCoord    VecCoord;
    typedef typename DataTypes::VecDeriv    VecDeriv;
    typedef typename DataTypes::Coord       Coord;
    typedef typename DataTypes::Deriv       Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type      Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>                  DataVecCoord;
    typedef Data<VecDeriv>                  DataVecDeriv;
    typedef Data<MatrixDeriv>               DataMatrixDeriv;
    
public:
    SurfacePressureConstraint(MechanicalState* object = nullptr);

    ~SurfacePressureConstraint() override;

    ////////////////////////////  Inherited from BaseObject //////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    //////////////////////////////////////////////////////////////////////////////


    /////////////////// from sofa::core::behavior::Constraint /////////////////////////
    void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;
    ///////////////////////////////////////////////////////////////////////////////////


protected:

    Data<vector<Real> >                     d_value;
    Data<unsigned int>                      d_valueIndex;
    Data<sofa::helper::OptionsGroup>        d_valueType;

    double                                  m_pressure;
    double                                  m_volumeGrowth;
    vector<Real>                            m_initialValue;

    ////////////////////////// Inherited attributes ////////////////////////////
    using SurfacePressureModel<DataTypes>::d_volumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation ;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_minVolumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_eqVolumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_eqPressure ;
    using SurfacePressureModel<DataTypes>::d_maxPressure ;
    using SurfacePressureModel<DataTypes>::d_minPressure ;
    using SurfacePressureModel<DataTypes>::d_maxPressureVariation;
    using SoftRobotsConstraint<DataTypes>::d_componentState;
    using SurfacePressureModel<DataTypes>::d_pressure ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void initData();
    void setUpVolumeLimits(Real& imposedValue, Real& minPressure, Real& maxPressure);
    void setUpPressureLimits(Real& imposedValue, Real& minVolumeGrowth, Real& maxVolumeGrowth);

};

#if !defined(SOFTROBOTS_SURFACEPRESSURECONSTRAINT_CPP)
using sofa::defaulttype::Vec3Types;
extern template class SOFA_SOFTROBOTS_API SurfacePressureConstraint<Vec3Types>;
#endif

} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using SurfacePressureConstraint SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::constraint::SurfacePressureConstraint<DataTypes>;
}
