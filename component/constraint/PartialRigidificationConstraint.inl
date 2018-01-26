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

#ifndef PARTIALRIGIDIFICATIONCONSTRAINT_INL
#define PARTIALRIGIDIFICATIONCONSTRAINT_INL

#include "PartialRigidificationConstraint.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using defaulttype::Rigid3dTypes;

template<class DataTypes>
PartialRigidificationConstraint<DataTypes>::PartialRigidificationConstraint(MechanicalState* object)
    : Inherit(object)
{
}

template<class DataTypes>
PartialRigidificationConstraint<DataTypes>::PartialRigidificationConstraint()
{
}

template<class DataTypes>
PartialRigidificationConstraint<DataTypes>::~PartialRigidificationConstraint()
{
}

template<class DataTypes>
void PartialRigidificationConstraint<DataTypes>::init()
{
    Inherit::init();
}


template<class DataTypes>
void PartialRigidificationConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                                        BaseVector *resV,
                                                                        const DataVecCoord &xfree,
                                                                        const DataVecDeriv &vfree)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(resV);
    SOFA_UNUSED(xfree);
    SOFA_UNUSED(vfree);
}


template<class DataTypes>
void PartialRigidificationConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                                       DataMatrixDeriv &cMatrix,
                                                                       unsigned int &cIndex,
                                                                       const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);

    std::cout<<"step0"<<std::endl;
    const VecCoord X = x.getValue();
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    m_cid = cIndex;
    std::cout<<"step1"<<std::endl;
    const Vec3 cx(1, 0, 0), cy(0, 1, 0), cz(0, 0, 1), vZero(0, 0, 0);
  //  const Vec3 p0p1 = X[1].getCenter() - X[0].getCenter();

//	const Deriv::Rot tau0p1cx = p0p1.cross(-cx);
//	const Deriv::Rot tau0p1cy = p0p1.cross(-cy);
//	const Deriv::Rot tau0p1cz = p0p1.cross(-cz);

    std::cout<<"step2"<<std::endl;

    MatrixDerivRowIterator cit = matrix.writeLine(cIndex);
    cit.addCol(0, Deriv(cx, vZero));
//	cit.addCol(0, Deriv(cx, tau0p1cx));

    cit = matrix.writeLine(cIndex+1);
    cit.setCol(0, Deriv(cy, vZero));
//	cit.addCol(0, Deriv(cy, tau0p1cy));

    cit = matrix.writeLine(cIndex+2);
    cit.setCol(0, Deriv(cz, vZero));
//	cit.addCol(0, Deriv(cz, tau0p1cz));

    cit = matrix.writeLine(cIndex+3);
    cit.setCol(0, Deriv(vZero, cx));
//    cit.addCol(0, Deriv(cx.cross(p0p1), vZero));

    cit = matrix.writeLine(cIndex+4);
    cit.setCol(0, Deriv(vZero, cy));
//    cit.addCol(0, Deriv(cy.cross(p0p1), vZero));

    cit = matrix.writeLine(cIndex+5);
    cit.setCol(0, Deriv(vZero, cz));
//    cit.addCol(0, Deriv(cz.cross(p0p1), vZero));

      std::cout<<"DEBUG/// state"<<mstate->getName()<<std::endl;

      // Indicates the size of the constraint block
      cIndex +=6;
}

template<class DataTypes>
void PartialRigidificationConstraint<DataTypes>::getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                                                         unsigned int& offset)
{
//	for(size_t i = 0 ; i < 6 ; ++i)
    resTab[offset] = new PartialRigidificationConstraintResolution6Dof();

    // indicates the size of the block on which the constraint resoluation works
    offset += 6;
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // PARTIALRIGIDIFICATIONCONSTRAINT_INL
