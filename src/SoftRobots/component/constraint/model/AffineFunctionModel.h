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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_AFFINEFUNCTIONMODEL_H
#define SOFA_COMPONENT_CONSTRAINTSET_AFFINEFUNCTIONMODEL_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

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
using sofa::defaulttype::Rigid3dTypes ; 
using sofa::defaulttype::BaseVector ;
using sofa::core::ConstraintParams ;
using sofa::helper::ReadAccessor ;
using sofa::core::VecCoordId ;



/**
 * This class contains common implementation of cable constraints
*/
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
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;
    typedef helper::vector<unsigned int> SetIndexArray;

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
                     core::MultiVecDerivId res,
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
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintId ;
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void setUpData();
    void internalInit();

	void checkSizes();
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class AffineFunctionModel<defaulttype::Vec3Types>;
extern template class AffineFunctionModel<defaulttype::Rigid3Types>;

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_AFFINEFUNCTIONMODEL_H
