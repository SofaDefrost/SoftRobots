/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_H
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_H

#include "../../behavior/SoftRobotsConstraint.h"
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>

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
 * This class contains common implementation of sliding constraints
*/
template< class DataTypes >
class SlidingModel : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SlidingModel,DataTypes),
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
    SlidingModel();
    SlidingModel(MechanicalState* object);
    virtual ~SlidingModel();

    ////////////////////////// Inherited from BaseObject ////////////////////
    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;
    virtual void reset() override;
    virtual void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Actuator //////////////////////
    virtual void buildConstraintMatrix(const ConstraintParams* cParams,
                                       DataMatrixDeriv &cMatrix,
                                       unsigned int &cIndex,
                                       const DataVecCoord &x) override;

    virtual void getConstraintViolation(const ConstraintParams* cParams,
                                        BaseVector *resV,
                                        const DataVecCoord &xfree,
                                        const DataVecDeriv &vfree) override;
    /////////////////////////////////////////////////////////////////////////


	void computeViolation(Deriv &result, const Coord &PosRef, const Coord &PosIn);

protected:

    Data<Deriv>                 d_direction;
    Data<SetIndexArray>         d_indices;

    Data<double>                d_force;
    Data<double>                d_displacement;

    Data<Real>                  d_maxDispVariation;

    Data<bool>                  d_showDirection;
    Data<double>                d_showVisuScale;

    int                         m_columnIndex;


private:
    void initDatas();

    void checkIndicesRegardingState();

protected:
    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::m_hasDeltaMax ;
    using SoftRobotsConstraint<DataTypes>::m_hasLambdaMax ;
    using SoftRobotsConstraint<DataTypes>::m_deltaMax ;
    using SoftRobotsConstraint<DataTypes>::m_lambdaMax ;
    using SoftRobotsConstraint<DataTypes>::addAlias ;
    ////////////////////////////////////////////////////////////////////////////
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_EXTERN_TEMPLATE
#ifdef SOFA_WITH_DOUBLE
extern template class SlidingModel<Vec3dTypes>;
extern template class SlidingModel<Rigid3dTypes>;
#endif

#ifdef SOFA_WITH_FLOAT
extern template class SlidingModel<Vec3fTypes>;
#endif
#endif //SOFA_EXTERN_TEMPLATE

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_H
