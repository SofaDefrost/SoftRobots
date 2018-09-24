/******************************************************************************
*               SOFA, Simulation Open-Framework Architecture                  *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                           Plugin SoftRobots v1.0                            *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/

#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURECONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURECONSTRAINT_H

#include "model/SurfacePressureModel.h"
#include <sofa/helper/OptionsGroup.h>

#include "../initSoftRobots.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

namespace _surfacepressureconstraint_
{

    using sofa::core::ConstraintParams ;
    using sofa::defaulttype::BaseVector ;
    using sofa::core::behavior::ConstraintResolution ;
    using helper::vector;

class VolumeGrowthConstraintResolution : public ConstraintResolution
{
public:
    VolumeGrowthConstraintResolution(const double& imposedVolumeGrowth, double* pressure);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    virtual void init(int line, double** w, double *lambda) override;
    virtual void resolution(int line, double** w, double* d, double* lambda, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
    double      m_wActuatorActuator;
    double      m_imposedVolumeGrowth;
    double*     m_pressure;
};

class SurfacePressureConstraintResolution : public ConstraintResolution
{
public:
    SurfacePressureConstraintResolution(const double& imposedPressure, double* volumeGrowth);

    //////////////////// Inherited from ConstraintResolution ///////////////////
    virtual void init(int line, double** w, double *force) override;
    virtual void resolution(int line, double** w, double* d, double* force, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
    double      m_wActuatorActuator;
    double      m_imposedPressure;
    double*     m_volumeGrowth;
};


/**
 * This component constrains a model by applying pressure on surfaces (for exemple cavities).
 * Description can be found at:
 * https://project.inria.fr/softrobot/documentation/constraint/SurfacePressureConstraint
*/


using helper::vector;

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
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef SurfacePressureModel<DataTypes>           Inherit;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>                  DataVecCoord;
    typedef Data<VecDeriv>                  DataVecDeriv;
    typedef Data<MatrixDeriv>               DataMatrixDeriv;
    
public:
    SurfacePressureConstraint(MechanicalState* object);
    SurfacePressureConstraint();

    virtual ~SurfacePressureConstraint() ;

    ////////////////////////////  Inherited from BaseObject //////////////////////
    virtual void init() override;
    virtual void reinit() override;
    virtual void reset() override;
    //////////////////////////////////////////////////////////////////////////////


    /////////////////// from sofa::core::behavior::Constraint /////////////////////////
    virtual void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                         unsigned int& offset) override;
    ///////////////////////////////////////////////////////////////////////////////////


protected:

    Data<vector<Real> >                     d_value;
    Data<unsigned int>                      d_valueIndex;
    Data<helper::OptionsGroup>              d_valueType;
    Data<bool>                              d_visualization;

    double                                  m_pressure;
    double                                  m_volumeGrowth;
    vector<Real>                            m_initialValue;

    ////////////////////////// Inherited attributes ////////////////////////////
    using SurfacePressureModel<DataTypes>::d_cavityVolume ;
    using SurfacePressureModel<DataTypes>::d_initialCavityVolume ;
    using SurfacePressureModel<DataTypes>::d_pressure ;
    using SurfacePressureModel<DataTypes>::d_volumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation ;
    using SurfacePressureModel<DataTypes>::m_displayedValue ;
    using SurfacePressureModel<DataTypes>::m_columnId;
    using SurfacePressureModel<DataTypes>::m_visualization;
    using SoftRobotsConstraint<DataTypes>::m_componentstate;
    ////////////////////////////////////////////////////////////////////////////

private:
    void initData();

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_EXTERN_TEMPLATE
#ifdef SOFA_WITH_DOUBLE
extern template class SOFA_SOFTROBOTS_API SurfacePressureConstraint<Vec3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
extern template class SOFA_SOFTROBOTS_API SurfacePressureConstraint<Vec3fTypes>;
#endif
#endif

}

using _surfacepressureconstraint_::SurfacePressureConstraint;
using _surfacepressureconstraint_::SurfacePressureConstraintResolution;
using _surfacepressureconstraint_::VolumeGrowthConstraintResolution;

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURECONSTRAINT_H
