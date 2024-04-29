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

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/behavior/ConstraintResolution.h>
#include <sofa/linearalgebra/BaseVector.h>
#include <sofa/type/Vec.h>

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>
#include <SoftRobots/component/initSoftRobots.h>

namespace softrobots::constraint
{

using sofa::core::visual::VisualParams;
using softrobots::behavior::SoftRobotsConstraint;
using sofa::core::ConstraintParams;
using sofa::linearalgebra::BaseVector;
using sofa::type::Vec;

using sofa::core::behavior::ConstraintResolution ;
using sofa::core::visual::VisualParams ;


class SOFA_SOFTROBOTS_API UnilateralPlaneConstraintResolution: public ConstraintResolution
{
public:
    UnilateralPlaneConstraintResolution(const unsigned int nbLines) ;

    /////////////// Inherited from ConstraintResolution ////////////
    void init(int line, SReal** w, SReal* force) override;
    void resolution(int line, SReal** w, SReal* d, SReal* force, SReal* dFree) override;
    ////////////////////////////////////////////////////////////////
};


/**
 * This component is a simple point plane collision model.
 * By providing 4 points to the component, the first point will be constrained to stay in one side of the plane described
 * by the three other points (in the direction of the plane normal). All the four points, the triangle and the normal can be
 * seen by allowing the "Collision Model" in the "View" tab.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class SOFA_SOFTROBOTS_API UnilateralPlaneConstraint : public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(UnilateralPlaneConstraint,DataTypes), SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes));

    typedef typename DataTypes::VecCoord            VecCoord;
    typedef typename DataTypes::VecDeriv            VecDeriv;
    typedef typename DataTypes::Coord               Coord;
    typedef typename DataTypes::Deriv               Deriv;
    typedef typename DataTypes::MatrixDeriv         MatrixDeriv;
    typedef typename Coord::value_type              Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                          DataVecCoord;
    typedef sofa::Data<VecDeriv>                          DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                       DataMatrixDeriv;

public:
    UnilateralPlaneConstraint(MechanicalState* object = nullptr);
    ~UnilateralPlaneConstraint() override;

    /////////////// Inherited from BaseObject  ////////////
    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;
    //////////////////////////////////////////////////////

    /////////////// Inherited from Constraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;

    void getConstraintResolution(const ConstraintParams* cParams,
                                 std::vector<sofa::core::behavior::ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;
    ///////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::m_constraintIndex ;
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////

    sofa::Data<Vec<4,unsigned int>>   d_indices;
    sofa::Data<bool>         d_flipNormal;

    void drawPoints(const VisualParams* vparams);
    void drawTriangles(const VisualParams* vparams);
    void drawArrows(const VisualParams* vparams);

private:
    void checkIndicesRegardingState();
};

#if !defined(SOFTROBOTS_UNILATERALPLANECONSTRAINT_CPP)
extern template class SOFA_SOFTROBOTS_API UnilateralPlaneConstraint<sofa::defaulttype::Vec3Types>;
#endif

} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using UnilateralPlaneConstraint SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::constraint::UnilateralPlaneConstraint<DataTypes>;
}

