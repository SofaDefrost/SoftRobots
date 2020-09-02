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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPSlidingConstraint_H
#define SOFA_COMPONENT_CONSTRAINTSET_QPSlidingConstraint_H

#include <sofa/defaulttype/Vec3Types.h>
#include "CableModel.h"
#include <sofa/helper/OptionsGroup.h>
#include "../../behavior/SoftRobotsConstraint.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::behavior::SoftRobotsConstraint ;
using sofa::core::visual::VisualParams ;
using sofa::core::objectmodel::Data ;
using sofa::defaulttype::Vec3dTypes ;
using sofa::defaulttype::Vec3fTypes ;
using sofa::defaulttype::BaseVector ;
using sofa::core::ConstraintParams ;
using sofa::helper::ReadAccessor ;
using sofa::core::VecCoordId ;

using sofa::core::behavior::ConstraintResolution ;


class SlidingForceConstraintResolution : public ConstraintResolution
{
public:
    SlidingForceConstraintResolution(const double& imposedForce, const double& min, const double& max);

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


/**
 * This class contains common implementation of cable constraints
*/
template< class DataTypes >
class QPSlidingConstraint : public CableModel<DataTypes>
{
public:

    SOFA_CLASS(SOFA_TEMPLATE(QPSlidingConstraint,DataTypes), SOFA_TEMPLATE(CableModel,DataTypes));
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;
    typedef helper::vector<unsigned int> SetIndexArray;


public:
    QPSlidingConstraint(MechanicalState* object = nullptr);

    ~QPSlidingConstraint() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
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
    void getConstraintResolution(const ConstraintParams*,
                                 std::vector<core::behavior::ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;

    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    //    void storeLambda(const ConstraintParams* cParams,
    //                     core::MultiVecDerivId res,
    //                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

protected:
    //Input data
    Data<helper::vector< Real > >       d_value;
    Data<unsigned int>                  d_valueIndex;
    Data<helper::OptionsGroup>          d_valueType;
    // displacement = the constraint will impose the displacement provided in data d_inputValue[d_iputIndex]
    // force = the constraint will impose the force provided in data d_inputValue[d_iputIndex]


protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using SoftRobotsConstraint<DataTypes>::m_state ;
    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using CableModel<DataTypes>::d_maxDispVariation ;
    using CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using CableModel<DataTypes>::d_maxForce ;
    using CableModel<DataTypes>::d_minForce ;
    using CableModel<DataTypes>::d_displacement ;
    using CableModel<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////
    /// \brief internalInit
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintId ;

    void internalInit();

private:
    void setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce);
    void setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement);

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class QPSlidingConstraint<defaulttype::Vec3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_QPSlidingConstraint_H
