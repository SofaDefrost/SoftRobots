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

#include <SoftRobots/component/constraint/PartialRigidificationConstraint.h>

namespace softrobots::constraint
{

using sofa::defaulttype::Rigid3dTypes;

template<class DataTypes>
PartialRigidificationConstraint<DataTypes>::PartialRigidificationConstraint(MechanicalState* object)
    : Inherit1(object)
{
}

template<class DataTypes>
PartialRigidificationConstraint<DataTypes>::~PartialRigidificationConstraint()
{
}

template<class DataTypes>
void PartialRigidificationConstraint<DataTypes>::init()
{
    Inherit1::init();
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

    const VecCoord X = x.getValue();
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    m_cid = cIndex;
    const Vec3 cx(1, 0, 0), cy(0, 1, 0), cz(0, 0, 1), vZero(0, 0, 0);

    MatrixDerivRowIterator cit = matrix.writeLine(cIndex);
    cit.addCol(0, Deriv(cx, vZero));

    cit = matrix.writeLine(cIndex+1);
    cit.setCol(0, Deriv(cy, vZero));

    cit = matrix.writeLine(cIndex+2);
    cit.setCol(0, Deriv(cz, vZero));

    cit = matrix.writeLine(cIndex+3);
    cit.setCol(0, Deriv(vZero, cx));

    cit = matrix.writeLine(cIndex+4);
    cit.setCol(0, Deriv(vZero, cy));

    cit = matrix.writeLine(cIndex+5);
    cit.setCol(0, Deriv(vZero, cz));

    // Indicates the size of the constraint block
    cIndex +=6;
}

template<class DataTypes>
void PartialRigidificationConstraint<DataTypes>::getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                                                         unsigned int& offset)
{
    resTab[offset] = new PartialRigidificationConstraintResolution6Dof();

    // Indicates the size of the block on which the constraint resoluation works
    offset += 6;
}
} // namespace

