/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture                          *
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
*                           Plugin SoftRobots    v1.0                         *
*				                                              *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_FORCEFIELD_PREQUIVALENTSTIFFNESSFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_PREQUIVALENTSTIFFNESSFORCEFIELD_INL

#include "PREquivalentStiffnessForceField.h"
#include <SofaBaseLinearSolver/FullVector.h>
#include <sofa/core/behavior/MechanicalState.h>
using sofa::core::behavior::MechanicalState ;
using sofa::core::objectmodel::BaseContext ;

#include <sofa/core/behavior/ForceField.inl>
#include <sofa/helper/RandomGenerator.h>
using sofa::helper::ReadAccessor ;
using sofa::helper::WriteAccessor ;

#include <string>
using std::string ;

#include <fstream>
using std::filebuf ;

#include <iostream>
using std::cout ;
using std::endl ;

#include <algorithm>
#include <ctime>

namespace sofa
{

namespace component
{

namespace forcefield
{

using sofa::core::objectmodel::ComponentState;

template<typename DataTypes>
PREquivalentStiffnessForceField<DataTypes>::PREquivalentStiffnessForceField()
    : Inherit1(),
      d_complianceFile(initData(&d_complianceFile, "complianceFile",
                                "Name of the file where the compliance matrix is stored .")),
      d_coeff(initData(&d_coeff, Real(1.0), "coefForce",
                                "Coefficient")),
      d_startIndex(initData(&d_startIndex, (unsigned int)0, "startIndex",
                                "Nodes below this indices won't be computed."))
{

}

template<typename DataTypes>
PREquivalentStiffnessForceField<DataTypes>::~PREquivalentStiffnessForceField()
{

}

template<typename DataTypes>
void PREquivalentStiffnessForceField<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);

    Inherit1::init();
    const string& filename = d_complianceFile.getValue();

    filebuf filebuffer;

    // Read compliance matrix from file
    // getting mstate bound to this forcefield
    MechanicalState<DataTypes>* mstate;
    getContext()->get(mstate, BaseContext::Local);
    if(mstate==nullptr)
    {
        msg_error() << "No mechanical state associated with this component. The component is deactivated.";
        return;
    }

    // Read rest positions
    WriteAccessor<VecCoord> restPosWriter(m_restPos);
    ReadAccessor<DataVecCoord> restPosReader(*mstate->read(core::ConstVecCoordId::restPosition()));

    size_t nFrames = restPosReader.size();
    restPosWriter.resize(nFrames);
    m_pos.resize(nFrames);

    std::copy(restPosReader.begin(), restPosReader.end(), restPosWriter.begin());

    m_complianceMat.resize(nFrames - 1);
    m_CInv.resize(nFrames - 1);
    m_H.resize(nFrames - 1);
    m_K.resize(nFrames - 1);

    for(size_t n = 0 ; n < m_K.size() ; ++n) {
        m_K[n] = Mat12x12();
    }

    if( filebuffer.open(filename.c_str(), std::ios_base::in) ) {
        std::istream file(&filebuffer);
        for(size_t n = 0 ; n < nFrames - 1 ; ++n) {
            if( ! (file >> m_complianceMat[n]) ) {
                msg_warning() << "Unable to read compliance matrix for frames [" << n << " | " << n + 1 << "]";

            } else {

                m_CInv[n].invert(m_complianceMat[n]);
            }
        }
        filebuffer.close();
    } else {
        msg_warning() << "Can not find compliance matrices file : " << filename;
    }

    d_componentState.setValue(ComponentState::Invalid);
}

template<typename DataTypes>
void PREquivalentStiffnessForceField<DataTypes>::addForce(const MechanicalParams*,
        DataVecDeriv& f,
        const DataVecCoord& x,
        const DataVecDeriv& v)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(v);

    const VecCoord& X = x.getValue();
    VecDeriv& force = *f.beginEdit();
    size_t nFrames = X.size();

    std::copy(X.begin(), X.end(), m_pos.begin());

    const Real& coef = d_coeff.getValue();
    const unsigned int& start = d_startIndex.getValue();

    for(size_t n = 0 ; n < nFrames - (start + 1) ; ++n) {
        const Pos& x0Current = X[n + 0 + start].getCenter();
        const Pos& x1Current = X[n + 1 + start].getCenter();

        const Pos& x0Rest = m_restPos[n + 0 + start].getCenter();
        const Pos& x1Rest = m_restPos[n + 1 + start].getCenter();

        const Quaternion& q0Current = X[n + 0 + start].getOrientation();
        const Quaternion& q1Current = X[n + 1 + start].getOrientation();
        const Quaternion& q0Rest = m_restPos[n + 0 + start].getOrientation();
        const Quaternion& q1Rest = m_restPos[n + 1 + start].getOrientation();

        // compute x1 local rigid position and rotation (can be precomputed)
        const Pos& x1l0Rest = q0Rest.inverseRotate(x1Rest - x0Rest);
        const Quaternion& q1l0Rest = q0Rest.inverse() * q1Rest;

        // compute x1 position w.r.t. x0 frame
        const Pos& x1l0Current = q0Current.inverseRotate(x1Current - x0Current);

        // compute the difference between rigid and real positions and orientations
        const Pos& dx = x1l0Rest - x1l0Current;
        //    Quaternion qDiff = q0Current.inverse() * q1th.inverse() * q1Current;
        Quaternion dummy;
        Quaternion q1l0  = q0Current.inverse() * q1Current;
        Quaternion qdiff = q1l0 * q1l0Rest.inverse();
        qdiff.normalize();

        Vec3 dq = dummy.angularDisplacement(q1l0Rest, q1l0);
        Vec6 dX1(dx, dq);

        // test: verification that we obtain the good values for linear and angular displacements in local frames
        Transform World_H_X0_rest(x0Rest,q0Rest);
        Transform World_H_X1_rest(x1Rest,q1Rest);
        Transform X0_H_X1_Rest = World_H_X0_rest.inversed() * World_H_X1_rest;

        Transform World_H_X0_current(x0Current,q0Current);
        Transform World_H_X1_current(x1Current,q1Current);
        Transform X0_H_X1_Current = World_H_X0_current.inversed() * World_H_X1_current;

        /// compute the angular displacement:
        SpatialVector UinX0= X0_H_X1_Current.CreateSpatialVector() - X0_H_X1_Rest.CreateSpatialVector();
        Vec6 Uloc;
        for (unsigned int i=0; i<3; i++)
        {
            Uloc[i] = UinX0.getLinearVelocity()[i];
            Uloc[i+3] = UinX0.getAngularVelocity()[i];
        }
        Vec6 Floc = m_CInv[n] * Uloc;
        Vec3 f_loc(Floc.ptr());
        Vec3 tau_loc(Floc.ptr() + 3);

        SpatialVector F1atX1inX0(f_loc, tau_loc);

        bool rest=true;
        SpatialVector F1atX1inX1,F0atX0inX0;
        if(rest){
            F1atX1inX1.setForce( X0_H_X1_Rest.backProjectVector( F1atX1inX0.getForce()) );
            F1atX1inX1.setTorque( X0_H_X1_Rest.backProjectVector( F1atX1inX0.getTorque()) )  ;
            F0atX0inX0 = X0_H_X1_Rest*F1atX1inX1;
        }
        else
        {
            F1atX1inX1.setForce( X0_H_X1_Current.backProjectVector( F1atX1inX0.getForce()) );
            F1atX1inX1.setTorque( X0_H_X1_Current.backProjectVector( F1atX1inX0.getTorque()) )  ;
            F0atX0inX0 = X0_H_X1_Current*F1atX1inX1;
        }

        SpatialVector F0atX0inWorld(World_H_X0_current.projectVector(F0atX0inX0.getForce()), World_H_X0_current.projectVector(F0atX0inX0.getTorque()))  ;
        SpatialVector F1atX1inWorld(World_H_X0_current.projectVector(F1atX1inX0.getForce()),  World_H_X0_current.projectVector(F1atX1inX0.getTorque()));


        // compute x1 forces in x0's frame and rotate them back to global coordinates
        Vec6 f1l0 = m_CInv[n] * dX1;

        Vec3 F1(f1l0.ptr());
        Vec3 tau1(f1l0.ptr() + 3);
        F1 = q0Current.rotate(F1);
        tau1 = q0Current.rotate(tau1);


        // compute transport matrix
        Vec3 p0p1;
        if(rest)
            p0p1= q0Rest.inverseRotate(x1Rest - x0Rest); // p0^p1 in local frame
        else
            p0p1= q0Current.inverseRotate(x1Current - x0Current); // p0^p1 in local frame

        Mat66 H = Mat66::Identity();
        H(3, 1) = -p0p1.z();
        H(3, 2) = p0p1.y();
        H(4, 0) = p0p1.z();
        H(4, 2) = -p0p1.x();
        H(5, 0) = -p0p1.y();
        H(5, 1) = p0p1.x();


        H = -H;

        // compute f0
        Vec6 f0l0 = H * f1l0;
        Vec3 F0(f0l0.ptr());
        Vec3 tau0(f0l0.ptr() + 3);

        F0 = q0Current.rotate(F0);
        tau0 = q0Current.rotate(tau0);


        Vec6 f0( F0atX0inWorld.getForce(),  F0atX0inWorld.getTorque());
        Vec6 f1(-F1atX1inWorld.getForce(), -F1atX1inWorld.getTorque());

        force[n + 0 + start] += f0 * coef;
        force[n + 1 + start] += f1 * coef;


        Mat66 block = H * m_CInv[n];
        m_K[n + start].setsub(0, 6, block);
        m_K[n + start].setsub(6, 0, block.transposed());

        block =  H * m_CInv[n] * H.transposed();
        m_K[n + start].setsub(0, 0, block);
        m_K[n + start].setsub(6, 6, m_CInv[n]);


        // build rotation matrix 4 3x3 blocks on diagonal
        Mat33 Rn;
        q0Current.toMatrix(Rn);
        Mat12x12 R(.0);
        R.setsub(0, 0, Rn);
        R.setsub(3, 3, Rn);
        R.setsub(6, 6, Rn);
        R.setsub(9, 9, Rn);

        m_K[n + start] = -R * m_K[n + start] * R.transposed() * coef; // modified : wasn't negated
    }
    f.endEdit();
}

template<typename DataTypes>
void PREquivalentStiffnessForceField<DataTypes>::addDForce(const MechanicalParams* mparams,
                                                           DataVecDeriv&  d_f ,
                                                           const DataVecDeriv&  d_x)
{    
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    const VecDeriv& dx = d_x.getValue();
    VecDeriv& dfdq = *d_f.beginEdit();
    const size_t nFrames = dx.size();

    const Real kFact = mparams->kFactor();
    const unsigned int& start = d_startIndex.getValue();

    for(size_t n = 0 ; n < nFrames - (1 + start) ; ++n) {
        Vec12 dq;

        for (unsigned int i=0; i<6; i++)
        {
            dq[i  ] = dx[n + 0 + start][i];
            dq[i+6] = dx[n + 1 + start][i];
        }

        Vec12 df = m_K[n + start] * dq * kFact;

        // separate force vector
        Vec6 df0(df.ptr());
        Vec6 df1(df.ptr() + 6);

        dfdq[n + 0 + start] += df0;
        dfdq[n + 1 + start] += df1;
    }
}

template<typename DataTypes>
double PREquivalentStiffnessForceField<DataTypes>::getPotentialEnergy(const MechanicalParams* mparams,
                                                                      const DataVecCoord& x) const
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(x);

    return 0.0;
}

template<typename DataTypes>
void PREquivalentStiffnessForceField<DataTypes>::addKToMatrix(BaseMatrix* matrix,
                                                              double kFact,
                                                              unsigned int& offset)
{    
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if( CSRMatB66* csrMat = dynamic_cast<CSRMatB66*>(matrix) ) {
        for(size_t n = 0 ; n < m_K.size() ; ++n) {
            const Mat12x12& K = m_K[n];
            Mat66 block;

            K.getsub(0, 0, block);
            *csrMat->wbloc(n, n, true) += block * kFact;

            K.getsub(0, 6, block);
            *csrMat->wbloc(n, n + 1, true) += block * kFact;

            K.getsub(6, 0, block);
            *csrMat->wbloc(n + 1, n, true) += block * kFact;

            K.getsub(6, 6, block);
            *csrMat->wbloc(n + 1, n + 1, true) += block * kFact;
        }
    } else {
        unsigned int kOffset = offset;
        for(size_t n = 0 ; n < m_K.size() ; ++n) {
            const Mat12x12& K = m_K[n];

            for(int i = 0 ; i < 12 ; ++i) {
                for(int j = 0 ; j < 12 ; ++j) {
                    matrix->add(kOffset + i , kOffset + j, K(i, j)*kFact);
                }
            }
            kOffset += 6;
        }
    }
}

template<typename DataTypes>
void PREquivalentStiffnessForceField<DataTypes>::displaceFrames(const VecCoord& frames,
                                                                VecCoord& displaced,
                                                                VecDeriv& dq,
                                                                const Real epsilon)
{
    sofa::helper::RandomGenerator rgen(time(nullptr));
    for(size_t n = 0 ; n < displaced.size() ; ++n) {
        Deriv& dqn = dq[n];
        dqn = Deriv(Vec3(), Vec3());
        for(int i = 3 ; i < 6 ; ++i) {
            dqn[i] = rgen.random(-1.0, 1.0) * epsilon;
        }

        Quaternion q;
        q.axisToQuat(dqn.getVOrientation(), dqn.getVOrientation().norm());
        q.normalize();
        Quaternion qn = (q * frames[n].getOrientation());
        qn.normalize();
        displaced[n] = Coord(frames[n].getCenter() + dqn.getVCenter(), qn);

    }
}

template<typename DataTypes>
void PREquivalentStiffnessForceField<DataTypes>::computeForce(const VecCoord& pos,
                                                              const VecCoord& restPos,
                                                              VecDeriv& f)
{
    const size_t nFrames = pos.size();

    const Real coef = d_coeff.getValue();

    for(size_t n = 0 ; n < nFrames - 1 ; ++n) {

        const Pos& x0Current = pos[n + 0].getCenter();
        const Pos& x1Current = pos[n + 1].getCenter();

        const Pos& x0Rest = restPos[n + 0].getCenter();
        const Pos& x1Rest = restPos[n + 1].getCenter();

        const Quaternion& q0Current = pos[n + 0].getOrientation();
        const Quaternion& q1Current = pos[n + 1].getOrientation();
        const Quaternion& q0Rest = restPos[n + 0].getOrientation();
        const Quaternion& q1Rest = restPos[n + 1].getOrientation();

        // compute x1 local rigid position and rotation (can be precomputed)
        const Pos& x1l0Rest = q0Rest.inverseRotate(x1Rest - x0Rest);
        const Quaternion& q1l0Rest = q0Rest.inverse() * q1Rest;

        // compute x1 position w.r.t. x0 frame
        const Pos& x1l0Current = q0Current.inverseRotate(x1Current - x0Current);

        // compute the difference between rigid and real positions and orientations
        const Pos& dx = x1l0Rest - x1l0Current;
        Quaternion dummy;
        Quaternion q1l0  = q0Current.inverse() * q1Current;
        Quaternion qdiff = q1l0 * q1l0Rest.inverse();
        qdiff.normalize();

        Vec3 dq = dummy.angularDisplacement(q1l0Rest, q1l0);
        Vec6 dX1(dx, dq);

        // compute x1 forces in x0's frame and rotate them back to global coordinates
        Vec6 f1l0 = m_CInv[n] * dX1;
        Vec3 F1(f1l0.ptr());
        Vec3 r1(f1l0.ptr() + 3);
        F1 = q0Current.rotate(F1);
        r1 = q0Current.rotate(r1);

        Vec6 f1(F1, r1);
        // compute transport matrix
        Vec3 p0p1 = q0Current.inverseRotate(x1Rest - x0Rest); // p0^p1 in local frame
        Mat66 H = Mat66::Identity();

        H(3, 1) = -p0p1.z();
        H(3, 2) = p0p1.y();
        H(4, 0) = p0p1.z();
        H(4, 2) = -p0p1.x();
        H(5, 0) = -p0p1.y();
        H(5, 1) = p0p1.x();

        H = -H; // static equilibrium

        // compute f0
        Vec6 f0l0 = H * f1l0;
        Vec3 F0(f0l0.ptr());
        Vec3 r0(f0l0.ptr() + 3);
        F0 = q0Current.rotate(F0);
        r0 = q0Current.rotate(r0);
        Vec6 f0(F0, r0);
        f[n + 0] += f0 * coef;
        f[n + 1] += f1 * coef;
    }
}

} // forcefield

} // component

} // sofa

#endif // PREQUIVALENTSTIFFNESSFORCEFIELD_INL
