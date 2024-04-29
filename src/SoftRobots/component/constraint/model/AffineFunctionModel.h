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

namespace softrobots::constraint
{

using softrobots::behavior::SoftRobotsConstraint ;
using sofa::core::visual::VisualParams ;
using sofa::core::objectmodel::Data ;
using sofa::defaulttype::Vec3dTypes ;
using sofa::defaulttype::Vec3fTypes ;
using sofa::defaulttype::Rigid3dTypes ; 
using sofa::linearalgebra::BaseVector ;
using sofa::core::ConstraintParams ;
using sofa::helper::ReadAccessor ;
using sofa::core::VecCoordId ;


template< class DataTypes >
class SOFA_SOFTROBOTS_API AffineFunctionModel : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(AffineFunctionModel,DataTypes),
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
    AffineFunctionModel(MechanicalState* object = nullptr);
    ~AffineFunctionModel() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
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

    Data<SetIndexArray>         d_indices;
	Data<VecDeriv>				d_coefficients;
	Data<Real>					d_offset;

    Data<Real>                  d_initFunctionValue;
	Data<Real>                  d_functionValue;

    Data<double>                d_force;
    Data<double>                d_displacement;

protected:

    SReal getAffineFunctionValue(const VecCoord &positions);

    ////////////////////////// Inherited attributes ////////////////////////////
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintIndex ;
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void setUpData();
    void internalInit();

	void checkSizes();
};

extern template class AffineFunctionModel<sofa::defaulttype::Vec3Types>;
extern template class AffineFunctionModel<sofa::defaulttype::Rigid3Types>;

} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using AffineFunctionModel SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::constraint::AffineFunctionModel<DataTypes>;
}
