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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_INL
//#include <cstdio> MISK: check if these are needed
//#include <cstdlib>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>

#include "SlidingModel.h"
#include <sofa/helper/logging/Messaging.h>

#include <sofa/defaulttype/Vec.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::visual::VisualParams;
using sofa::defaulttype::BaseVector;
using sofa::helper::gl::glVertexT;
using sofa::defaulttype::Vec4f;

template<class DataTypes>
SlidingModel<DataTypes>::SlidingModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
    , d_maxDispVariation(initData(&d_maxDispVariation,(Real)0.0, "maxDispVariation",
                                   "Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

    , d_direction(initData(&d_direction, "direction",
                          "Direction of the actuation."))

    , d_indices(initData(&d_indices, "indices",
                          "Indices of the nodes subjected to the force. \n"
                          "If no indices given, mechanical context considered."))

    , d_force(initData(&d_force,(double)0.0, "force",
                          "Output force."))

    , d_displacement(initData(&d_displacement,(double)0.0, "displacement",
                          "Output displacement compared to the initial position."))

    , d_showDirection(initData(&d_showDirection,false, "showDirection",
                          "Draw the direction."))

    , d_showVisuScale(initData(&d_showVisuScale,(double)0.001, "showVisuScale",
                          "Visualization scale."))

{


  d_maxDispVariation.setGroup("Input");


  d_direction.setGroup("Input");
  d_indices.setGroup("Input");

  d_force.setGroup("Output");
  d_displacement.setGroup("Output");

  d_showDirection.setGroup("Visualization");
  d_showVisuScale.setGroup("Visualization");
}



template<class DataTypes>
SlidingModel<DataTypes>::SlidingModel()
    : 
	  d_maxDispVariation(initData(&d_maxDispVariation,(Real)0.0, "maxDispVariation",
                                   "Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

    , d_direction(initData(&d_direction, "direction",
                          "Direction of the actuation."))

    , d_indices(initData(&d_indices, "indices",
                          "Indices of the nodes subjected to the force. \n"
                          "If no indices given, mechanical context considered."))

    , d_force(initData(&d_force,(double)0.0, "force",
                          "Output force."))

    , d_displacement(initData(&d_displacement,(double)0.0, "displacement",
                          "Output displacement compared to the initial position."))

    , d_showDirection(initData(&d_showDirection,false, "showDirection",
                          "Draw the direction."))

    , d_showVisuScale(initData(&d_showVisuScale,(double)0.001, "showVisuScale",
                          "Visualization scale."))

{

    d_maxDispVariation.setGroup("Input");

    d_direction.setGroup("Input");
    d_indices.setGroup("Input");

    d_force.setGroup("Output");
    d_displacement.setGroup("Output");

    d_showDirection.setGroup("Visualization");
    d_showVisuScale.setGroup("Visualization");
}

template<class DataTypes>
SlidingModel<DataTypes>::~SlidingModel()
{
}

template<class DataTypes>
void SlidingModel<DataTypes>::init()
{
    SoftRobotsConstraint<DataTypes>::init();

    if(m_state==nullptr)
        msg_error(this) << "There is no mechanical state associated with this node. "
                            "the object is deactivated. "
                            "To remove this error message fix your scene possibly by "
                            "adding a MechanicalObject." ;

    if(m_state->read(VecCoordId::position())==nullptr)
        msg_error(this)<<"There is no position vector in the MechanicalState. "
                         "The object is deactivated. " ;

    if (!d_indices.isSet())
    {
        SetIndexArray &list = (*d_indices.beginEdit());
        msg_warning(this)<<"No index of actuation given, set default (all points of context MechanicalState).";

        for(unsigned int i=0; i<m_state->getSize(); i++)
            list.push_back(i);
        d_indices.endEdit();
    }
    else
        checkIndicesRegardingState();

    initDatas();
}

template<class DataTypes>
void SlidingModel<DataTypes>::bwdInit()
{
}

template<class DataTypes>
void SlidingModel<DataTypes>::reinit()
{
    checkIndicesRegardingState();
    initDatas();
}

template<class DataTypes>
void SlidingModel<DataTypes>::reset()
{
    reinit();
}


template<class DataTypes>
void SlidingModel<DataTypes>::initDatas()
{
    if (!d_direction.isSet())
    {
        msg_warning()<<"No direction of actuation provided by user. Default (1. 0. 0.)";
        Deriv x;
        x[0]=1;
        d_direction.setValue(x);
    }
    else
    {
        Deriv direction = d_direction.getValue();
        direction.normalize();
        d_direction.setValue(direction);
    }

    d_displacement.setValue(0.0);
    d_force.setValue(0.0);
}


template<class DataTypes>
void SlidingModel<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<Data<VecCoord> > positions     = *m_state->read(VecCoordId::position());

    for(unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
            msg_error(this) << "Indices at index " << i << " is too large regarding mechanicalState [position] size" ;
        if (d_indices.getValue()[i] < 0)
            msg_error(this) << "Indices at index " << i << " is negative" ;
    }
}

template<class DataTypes>
void SlidingModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams, DataMatrixDeriv &cMatrix, unsigned int &cIndex, const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    m_columnIndex = cIndex;

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(m_columnIndex);

    Deriv direction = d_direction.getValue();

    ReadAccessor<Data<SetIndexArray>> indices = d_indices;

    for(unsigned int i=0; i<indices.size(); i++)
        rowIterator.setCol(indices[i], direction/indices.size());

    cIndex++;
    cMatrix.endEdit();
    m_nbLines = cIndex - m_columnIndex;
}

template<class DataTypes>
void SlidingModel<DataTypes>::computeViolation(Deriv &result, const Coord &PosRef, const Coord &PosIn)
{
    result = PosIn-PosRef;
}

template<class DataTypes>
void SlidingModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams, BaseVector *resV, const DataVecCoord &xfree, const DataVecDeriv &vfree)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(vfree);
    const VecCoord      &positions     = xfree.getValue();
    const VecCoord      &restPositions = m_state->read(core::ConstVecCoordId::restPosition())->getValue();
    const SetIndexArray &indices       = d_indices.getValue();
    const Deriv         &direction     = d_direction.getValue();

    // Projection of the global displacement along the direction (normalized) of the actuation
    Deriv v;
    computeViolation(v, restPositions[indices[0]] ,positions[indices[0]]);
    const int N = Deriv::total_size;
    double n=0.0;

    for (unsigned int i=0; i<N; i++){
        n+= v[i]*direction[i];
    }



    //double n = v[0]*direction[0] + v[1]*direction[1] + v[2]*direction[2];

    if(indices.size())
        resV->set(m_columnIndex, n);
}

template<class DataTypes>
void SlidingModel<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields()) return;
    if (!d_showDirection.getValue()) return;


    ReadAccessor<Data<VecCoord> > positions = *m_state->read(core::ConstVecCoordId::position());
    const SetIndexArray &indices = d_indices.getValue();

    sofa::defaulttype::Vec<3,SReal> bary(0.,0.,0.);
    for(unsigned int i=0; i<indices.size(); i++)
        for (unsigned int j=0; j<3; j++)
            bary[j]+=positions[indices[i]][j]/indices.size();

    sofa::defaulttype::Vec<3,SReal> bary_arrow(0.,0.,0.);
    for (unsigned int j=0; j<3; j++)
        bary_arrow[j] = bary[j]+d_direction.getValue()[j]*d_showVisuScale.getValue();

    vparams->drawTool()->setLightingEnabled(true);
    vparams->drawTool()->drawArrow(bary,bary_arrow,d_showVisuScale.getValue()/20.0f, Vec4f(1,0,0,1));
    vparams->drawTool()->restoreLastState();
}



} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_INL
