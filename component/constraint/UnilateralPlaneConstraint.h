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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_UNILATERALPLANECONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINTSET_UNILATERALPLANECONSTRAINT_H


#include <sofa/core/behavior/Constraint.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/Vec.h>

#include "../initSoftRobots.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::visual::VisualParams;
using sofa::core::behavior::Constraint;
using sofa::core::ConstraintParams;
using sofa::defaulttype::BaseVector;
using sofa::defaulttype::Vec;

using sofa::core::behavior::ConstraintResolution ;
using core::visual::VisualParams ;


class SOFA_SOFTROBOTS_API UnilateralPlaneConstraintResolution: public ConstraintResolution
{
public:
    UnilateralPlaneConstraintResolution(const unsigned int nbLines) ;

    /////////////// Inherited from ConstraintResolution ////////////
    virtual void init(int line, double** w, double* force) override;
    virtual void resolution(int line, double** w, double* d, double* force, double* dFree) override;
    ////////////////////////////////////////////////////////////////
};


/**
 * This component is a simple point plane collision model.
 * By providing 4 points to the component, the first point will be constrained to stay in one side of the plane described
 * by the three other points (in the direction of the plane normal). All the four points, the triangle and the normal can be
 * seen by allowing the "Collision Model" in the "View" tab.
 * Description can be found at:
 * https://project.inria.fr/softrobot/documentation/constraint/UnilateralPlaneConstraint
*/
template< class DataTypes >
class SOFA_SOFTROBOTS_API UnilateralPlaneConstraint : public Constraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(UnilateralPlaneConstraint,DataTypes), SOFA_TEMPLATE(Constraint,DataTypes));

    typedef typename DataTypes::VecCoord            VecCoord;
    typedef typename DataTypes::VecDeriv            VecDeriv;
    typedef typename DataTypes::Coord               Coord;
    typedef typename DataTypes::Deriv               Deriv;
    typedef typename DataTypes::MatrixDeriv         MatrixDeriv;
    typedef typename Coord::value_type              Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>                          DataVecCoord;
    typedef Data<VecDeriv>                          DataVecDeriv;
    typedef Data<MatrixDeriv>                       DataMatrixDeriv;

public:
    UnilateralPlaneConstraint(MechanicalState* object = nullptr);
    virtual ~UnilateralPlaneConstraint() ;

    /////////////// Inherited from BaseObject  ////////////
    virtual void init() override;
    virtual void reinit() override;
    virtual void draw(const VisualParams* vparams) override;
    //////////////////////////////////////////////////////

    /////////////// Inherited from Constraint ////////////
    virtual void buildConstraintMatrix(const ConstraintParams* cParams ,
                                       DataMatrixDeriv &cMatrix,
                                       unsigned int &cIndex,
                                       const DataVecCoord &x) override;

    virtual void getConstraintViolation(const ConstraintParams* cParams ,
                                        BaseVector *resV,
                                        const DataVecCoord &xfree,
                                        const DataVecDeriv &vfree) override;

    virtual void getConstraintResolution(const ConstraintParams* cParams,
                                         std::vector<core::behavior::ConstraintResolution*>& resTab,
                                         unsigned int& offset) override;
    ///////////////////////////////////////////////////////////////

protected:

    Data<Vec<4,unsigned int>>   d_indices;
    Data<bool>         d_flipNormal;
    int                m_columnIndex;

    void drawPoints(const VisualParams* vparams);
    void drawTriangles(const VisualParams* vparams);
    void drawArrows(const VisualParams* vparams);

private:
    void checkIndicesRegardingState();

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Constraint<DataTypes>::mstate ;
    using Constraint<DataTypes>::m_componentstate ;
    ////////////////////////////////////////////////////////////////////////////
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_EXTERN_TEMPLATE
#ifdef SOFA_WITH_DOUBLE
extern template class SOFA_SOFTROBOTS_API UnilateralPlaneConstraint<sofa::defaulttype::Vec3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
extern template class SOFA_SOFTROBOTS_API UnilateralPlaneConstraint<sofa::defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_UNILATERALPLANECONSTRAINT_H
