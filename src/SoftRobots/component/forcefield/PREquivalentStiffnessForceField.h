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

#include <SoftRobots/component/initSoftRobots.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/type/Vec.h>
#include <sofa/type/Mat.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/type/vector.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <string>

namespace softrobots::forcefield
{

using sofa::type::Vec ;
using sofa::type::Mat ;
using sofa::type::vector;
using sofa::core::MechanicalParams;
using sofa::linearalgebra::BaseMatrix;
using sofa::core::behavior::ForceField ;
using sofa::linearalgebra::CompressedRowSparseMatrix ;

template<typename DataTypes>
class PREquivalentStiffnessForceField : public ForceField<DataTypes>
{
public :
    SOFA_CLASS(SOFA_TEMPLATE(PREquivalentStiffnessForceField, DataTypes), SOFA_TEMPLATE(ForceField, DataTypes));

    typedef typename DataTypes::Real    Real;
    typedef typename DataTypes::Coord   Coord;
    typedef typename DataTypes::Deriv   Deriv;
    typedef typename Coord::Pos         Pos;
    typedef typename Coord::Quat        Quaternion;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef sofa::Data<VecCoord>              DataVecCoord;
    typedef sofa::Data<VecDeriv>              DataVecDeriv;

    typedef Vec<3, Real>                Vec3;
    typedef Vec<6, Real>                Vec6;
    typedef Vec<12, Real>               Vec12;
    typedef Mat<3, 3, Real>             Mat33;
    typedef Mat<3, 3, Real>             Mat44;
    typedef Mat<6, 6, Real>             Mat66;
    typedef Mat<12, 12, Real>           Mat12x12;

    typedef CompressedRowSparseMatrix<Mat66> CSRMatB66;

    typedef typename CompressedRowSparseMatrix<Mat66>::ColBlockConstIterator CSRMatB66ColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat66>::RowBlockConstIterator CSRMatB66RowBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat66>::BlockConstAccessor CSRMatB66BlockConstAccessor;
    typedef typename sofa::defaulttype::SolidTypes<Real>::Transform Transform;
    typedef typename sofa::defaulttype::SolidTypes<Real>::SpatialVector SpatialVector;

    typedef sofa::type::vector<Vec6> VecVec6;
    typedef sofa::type::vector<Mat66> VecMat66;
    typedef sofa::type::vector<Mat12x12> VecMat12;


public :
    PREquivalentStiffnessForceField();
    ~PREquivalentStiffnessForceField() override;

    ////////////////////////// Inherited from BaseObject /////////////////////////
    void init() override;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////// Inherited from ForceField /////////
    void addForce(const MechanicalParams* mparams,
                  DataVecDeriv& f ,
                  const DataVecCoord& x ,
                  const DataVecDeriv& v) override;

    void addDForce(const MechanicalParams* mparams,
                   DataVecDeriv&   df ,
                   const DataVecDeriv&   dx ) override;

    double getPotentialEnergy(const MechanicalParams* mparams,
                              const DataVecCoord& x) const override;

    void addKToMatrix(sofa::linearalgebra::BaseMatrix* matrix,
                      double kFact,
                      unsigned int& offset) override;
    ////////////////////////////////////////////////////////////////////////////

    void displaceFrames(const VecCoord& frames, VecCoord& displaced, VecDeriv& dq, const Real epsilon);

protected :
    sofa::Data<std::string>   d_complianceFile;
    sofa::Data<Real>          d_coeff;
    sofa::Data<unsigned int>  d_startIndex;
    VecMat66            m_complianceMat;
    VecMat66            m_CInv;
    VecMat66            m_H;
    VecMat12            m_K;
    VecCoord            m_pos;
    VecCoord            m_restPos;

private :
    void computeForce(const VecCoord& pos, const VecCoord& restPos, VecDeriv& f);

    ////////////////////////// Inherited attributes ////////////////////////////
    using ForceField<DataTypes>::getContext ;
    using ForceField<DataTypes>::d_componentState;
    ////////////////////////////////////////////////////////////////////////////
};

#if !defined(SOFTROBOTS_PREQUIVALENTSTIFFNESSFORCEFIELD_CPP)
extern template class SOFA_SOFTROBOTS_API PREquivalentStiffnessForceField<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace

namespace sofa::component::forcefield
{
    template <class DataTypes>
    using PREquivalentStiffnessForceField SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::forcefield::PREquivalentStiffnessForceField<DataTypes>;
}
