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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPSlidingConstraint_INL
#define SOFA_COMPONENT_CONSTRAINTSET_QPSlidingConstraint_INL

#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/Vec.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>

#include "QPSlidingConstraint.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::helper::WriteAccessor;

using sofa::core::objectmodel::ComponentState;
using sofa::core::visual::VisualParams;
using sofa::defaulttype::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::defaulttype::Vec4f;
using sofa::defaulttype::Vector3;
using sofa::helper::vector;
using sofa::helper::OptionsGroup;

template<class DataTypes>
QPSlidingConstraint<DataTypes>::QPSlidingConstraint(MechanicalState* object)
    : Inherit1(object)

    , d_value(initData(&d_value, "value",
                       "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                            "Index of the value (in InputValue vector) that we want to impose \n"
                            "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, OptionsGroup(2,"displacement","force"), "valueType",
                           "displacement = the contstraint will impose the displacement provided in data value[valueIndex] \n"
                           "force = the contstraint will impose the force provided in data value[valueIndex] \n"
                           "If unspecified, the default value is displacement"))
{

}

template<class DataTypes>
QPSlidingConstraint<DataTypes>::~QPSlidingConstraint()
{
}


template<class DataTypes>
void QPSlidingConstraint<DataTypes>::init()
{
    Inherit1::init();

    // To remove in SoftRobots v20.0
    if(!d_minForce.isSet())
        msg_warning() << "An old implementation of CableConstraint was not allowing negative force. This is now possible. "
                      << "However, to limit the force to be strictly positive you now have to set minForce=0.";
    //

    internalInit();
}


template<class DataTypes>
void QPSlidingConstraint<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void QPSlidingConstraint<DataTypes>::internalInit()
{
    if(d_value.getValue().size()==0)
    {
        WriteAccessor<Data<vector<Real>>> value = d_value;
        value.resize(1,0.);
    }

    // check for errors in the initialization
    if(d_value.getValue().size()<d_valueIndex.getValue())
    {
        msg_warning() << "Bad size for data value (size="<< d_value.getValue().size()<<"), or wrong value for data valueIndex (valueIndex="<<d_valueIndex.getValue()<<"). Set default valueIndex=0.";
        d_valueIndex.setValue(0);
    }
}



template<class DataTypes>
void QPSlidingConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams, DataMatrixDeriv &cMatrix, unsigned int &cIndex, const DataVecCoord &x)
{
    if(d_componentState != ComponentState::Valid)
        return ;

    SOFA_UNUSED(cParams);

    MatrixDeriv& matrix = *cMatrix.beginEdit();
    VecCoord positions = x.getValue();
    m_constraintId= cIndex;

    for (unsigned int i=0; i<positions.size(); i++)
    {
        if(i < positions.size()-1){
            MatrixDerivRowIterator c_it = matrix.writeLine(cIndex);
            c_it.addCol(i, Coord(0,1,0)); // instead of vector3(0,1,0) use the directtion of the projection
            MatrixDerivRowIterator c_it_1 = matrix.writeLine(cIndex+1);
            c_it_1.addCol(i, Coord(0,0,1)); // instead of vector3(0,1,0) use the directtion of the projection
            cIndex +=2;
        }else{
            MatrixDerivRowIterator c_it = matrix.writeLine(cIndex);
            c_it.addCol(i, Coord(1,0,0));

            MatrixDerivRowIterator c_it_1 = matrix.writeLine(cIndex+1);
            c_it_1.addCol(i, Coord(0,1,0));
            MatrixDerivRowIterator c_it_2 = matrix.writeLine(cIndex+2);
            c_it_2.addCol(i, Coord(0,0,1));
            cIndex +=3;
        }

    }
    cMatrix.endEdit();
    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void QPSlidingConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                            BaseVector *resV,
                                                            const BaseVector *Jdx)
{
    if(d_componentState != ComponentState::Valid)
        return ;

    SOFA_UNUSED(cParams);
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();

    if(Jdx->size()==0){
        //std::cout << "Size JDX = "<< Jdx->size() << std::endl;
        for (size_t i = 0; i < positions.size(); i++){
            if( i < positions.size()-1){
                Real dfree1 =  positions[i][1];
                Real dfree2 =  positions[i][2];

                resV->set(m_constraintId + 2*i   , dfree1);
                resV->set(m_constraintId + 2*i +1, dfree2);
            }else{
                Real dfree0 =  positions[i][0];
                Real dfree1 =  positions[i][1];
                Real dfree2 =  positions[i][2];

                resV->set(m_constraintId + 2*i   , dfree0);
                resV->set(m_constraintId + 2*i +1, dfree1);
                resV->set(m_constraintId + 2*i +2, dfree2);
            }
        }

    }else{
        //std::cout << "Size JDX = "<< Jdx->size() << std::endl;
        for (size_t i = 0; i < positions.size(); i++){
            if( i < positions.size()-1){
                Real dfree1 = Jdx->element(2*i)   + positions[i][1];
                Real dfree2 = Jdx->element(2*i+1) + positions[i][2];

                resV->set(m_constraintId + 2*i   , dfree1);
                resV->set(m_constraintId + 2*i +1, dfree2);
            }else{
                //std::cout << " The laste position : "<< positions[i] << "; Jdx->element() "<< Jdx->element(2*i) <<" "<< Jdx->element(2*i+1) <<" "<< Jdx->element(2*i+2) << std::endl;
                Real dfree0 = Jdx->element(2*i)   + positions[i][0];
                Real dfree1 = Jdx->element(2*i+1) + positions[i][1];
                Real dfree2 = Jdx->element(2*i+2) + positions[i][2];

                //std::cout << " m_constraintId + 2*i : "<< m_constraintId + 2*i << std::endl;
                resV->set(m_constraintId + 2*i   , dfree0);
                resV->set(m_constraintId + 2*i +1, dfree1);
                resV->set(m_constraintId + 2*i +2, dfree2);
            }
        }
    }
}

template<class DataTypes>
void QPSlidingConstraint<DataTypes>::getConstraintResolution(const ConstraintParams*,
                                                             std::vector<core::behavior::ConstraintResolution*>& resTab,
                                                             unsigned int& offset)
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    //    std::cout << "The position size is : " << positions.size()<< std::endl;
    double imposedValue= 1.0;
    for (size_t i = 0; i < positions.size(); i++){

        resTab[offset++] = new BilateralConstraintResolution();
        resTab[offset++] = new BilateralConstraintResolution();
        if(i == positions.size()-1){
            resTab[offset++] = new BilateralConstraintResolution();
        }
    }
    //    std::cout << "The position size is END " << std::endl;
}


template<class DataTypes>
void QPSlidingConstraint<DataTypes>::setUpDisplacementLimits(double& imposedValue, double& minForce, double& maxForce)
{
    //    if(d_maxDispVariation.isSet())
    //    {
    //        double displacement = d_displacement.getValue();
    //        if(imposedValue > displacement && imposedValue-displacement>d_maxDispVariation.getValue())
    //            imposedValue = displacement+d_maxDispVariation.getValue();

    //        if(imposedValue < displacement && imposedValue-displacement<-d_maxDispVariation.getValue())
    //            imposedValue = displacement-d_maxDispVariation.getValue();
    //    }

    //    if(d_maxPositiveDisplacement.isSet() && imposedValue>d_maxPositiveDisplacement.getValue())
    //        imposedValue = d_maxPositiveDisplacement.getValue();

    //    if(d_maxNegativeDisplacement.isSet() && imposedValue<-d_maxNegativeDisplacement.getValue())
    //        imposedValue = -d_maxNegativeDisplacement.getValue();

    //    if(d_minForce.isSet())
    //        minForce=d_minForce.getValue();
    //    if(d_maxForce.isSet())
    //        maxForce=d_maxForce.getValue();
}

template<class DataTypes>
void QPSlidingConstraint<DataTypes>::setUpForceLimits(double& imposedValue, double& minDisplacement, double& maxDisplacement)
{
    //    if(d_maxForce.isSet() && imposedValue>d_maxForce.getValue())
    //        imposedValue = d_maxForce.getValue();

    //    if(d_minForce.isSet() && imposedValue<d_minForce.getValue())
    //        imposedValue = d_minForce.getValue();

    //    if(d_maxNegativeDisplacement.isSet())
    //        minDisplacement=-d_maxNegativeDisplacement.getValue();
    //    if(d_maxPositiveDisplacement.isSet())
    //        maxDisplacement=d_maxPositiveDisplacement.getValue();
}

template<class DataTypes>
void QPSlidingConstraint<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState != ComponentState::Valid)
        return ;

}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
