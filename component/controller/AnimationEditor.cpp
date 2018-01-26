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
#define SOFA_CONTROLLER_ANIMATIONEDITOR_CPP

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Quat.h>

#include "AnimationEditor.inl"

namespace sofa
{

namespace component
{

namespace controller
{

using sofa::defaulttype::Vec3dTypes;
using sofa::defaulttype::Vec3fTypes;
using sofa::defaulttype::Vec3d;
using sofa::defaulttype::Vec3f;
using sofa::defaulttype::Quat;

using sofa::defaulttype::Rigid3dTypes;
using sofa::defaulttype::Rigid3fTypes;

namespace _animationeditor_ {

template<>
void AnimationEditor<Rigid3fTypes>::updateAnimationWithInterpolation(const int startKey,
                                                                     const int endKey)
{
    if(m_state == nullptr)
        return;

    vector<Coord> previousPositions = m_animation[startKey];
    vector<Coord> currentPositions  = m_animation[endKey];

    if (currentPositions.size() != previousPositions.size())
    {
        msg_warning() <<"This component does not handle mechanical state size changes";
        return;
    }

    int nbPositions = m_state->read(core::ConstVecCoordId::position())->getValue().size();
    int nbStep = endKey - startKey;

    for (int i=0; i<nbStep+1; i++)
    {
        vector<Coord> newPositions;
        for (int k=0; k<nbPositions; k++)
        {
            Vec3f direction = currentPositions[k].getCenter() - previousPositions[k].getCenter();
            double distance = direction.norm();
            double step = distance/nbStep*i;
            direction.normalize();
            Vec3f center =  previousPositions[k].getCenter() + direction*step;

            double h =  (double)i/(double)nbStep;
            Quat orientation;
            for(int l=0; l<4; l++)
                orientation[l] = previousPositions[k].getOrientation()[l]*(1.-h) + currentPositions[k].getOrientation()[l]*h;
            orientation.normalize();

            Coord newPosition;
            Rigid3fTypes::setCPos(newPosition, center);
            Rigid3fTypes::setCRot(newPosition, orientation);
            newPositions.push_back(newPosition);
        }
        m_animation[startKey+i] = newPositions;
    }
}

template<>
void AnimationEditor<Rigid3dTypes>::updateAnimationWithInterpolation(const int startKey,
                                                                     const int endKey)
{
    if(m_state == nullptr)
        return;

    vector<Coord> previousPositions = m_animation[startKey];
    vector<Coord> currentPositions  = m_animation[endKey];

    if (currentPositions.size() != previousPositions.size())
    {
        msg_warning() <<"This component does not handle mechanical state size changes";
        return;
    }

    int nbPositions = m_state->read(core::ConstVecCoordId::position())->getValue().size();
    int nbStep = endKey - startKey;

    for (int i=0; i<nbStep+1; i++)
    {
        vector<Coord> newPositions;
        for (int k=0; k<nbPositions; k++)
        {
            Vec3d direction = currentPositions[k].getCenter() - previousPositions[k].getCenter();
            double distance = direction.norm();
            double step = distance/nbStep*i;
            direction.normalize();
            Vec3f center =  previousPositions[k].getCenter() + direction*step;

            double h =  (double)i/(double)nbStep;
            Quat orientation;
            for(int l=0; l<4; l++)
                orientation[l] = previousPositions[k].getOrientation()[l]*(1.-h) + currentPositions[k].getOrientation()[l]*h;
            orientation.normalize();

            Coord newPosition;
            Rigid3dTypes::setCPos(newPosition, center);
            Rigid3dTypes::setCRot(newPosition, orientation);
            newPositions.push_back(newPosition);
        }
        m_animation[startKey+i] = newPositions;
    }
}

////////////////////////////////////////////    FACTORY    ////////////////////////////////////////////
using sofa::core::RegisterObject ;

// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
// 2-RegisterObject("description") + .add<> : Register the component
// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(AnimationEditor)

int AnimationEditorClass = RegisterObject("Build an animation from key points motion: \n"
                                   "ctrl+a: add keyframe \n"
                                   "ctrl+d: delete keyframe \n"
                                   "ctrl+c: copy keyframe \n"
                                   "ctrl+v: paste keyframe \n"
                                   "ctrl+x: cut keyframe \n"
                                   "ctrl+w: save animation \n"
                                   "ctrl+(left/right)arrow: move the cursor along the timeline \n"
                                   "ctrl+('+'/'-'): move the cursor to the next/previous keyframe")
///////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SOFA_WITH_FLOAT
        .add< AnimationEditor<Vec3fTypes> >()
        .add< AnimationEditor<Rigid3fTypes> >()
#endif
#ifdef SOFA_WITH_DOUBLE
        .add< AnimationEditor<Vec3dTypes> >(true)
        .add< AnimationEditor<Rigid3dTypes> >()
#endif
        ;

#ifdef SOFA_WITH_FLOAT
template class SOFA_SOFTROBOTS_API AnimationEditor<Vec3fTypes>;
template class SOFA_SOFTROBOTS_API AnimationEditor<Rigid3fTypes>;
#endif
#ifdef SOFA_WITH_DOUBLE
template class SOFA_SOFTROBOTS_API AnimationEditor<Vec3dTypes>;
template class SOFA_SOFTROBOTS_API AnimationEditor<Rigid3dTypes>;
#endif


}//namespace _animationeditor_
}//namespace controller
}//namespace component
}//namespace sofa
