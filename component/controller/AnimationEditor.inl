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
#ifndef SOFA_CONTROLLER_ANIMATIONEDITOR_INL
#define SOFA_CONTROLLER_ANIMATIONEDITOR_INL

#include <sofa/config/build_option_opengl.h>

#ifdef SOFA_WITH_OPENGL
#include <sofa/helper/gl/template.h>
using sofa::helper::gl::glVertexT;
#endif

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/Event.h>
#include <fstream>
#include <iomanip>
#include <fstream>

#include "AnimationEditor.h"

#ifdef SOFA_WITH_DACCORD
#include "../../applications/sofa/gui/SofaGuiCommon/editor/editor.h"
using daccord::current::Editor ;
#endif


namespace sofa
{

namespace component
{

namespace controller
{

namespace _animationeditor_
{

using sofa::core::objectmodel::ComponentState;

using sofa::core::objectmodel::KeypressedEvent;
using sofa::simulation::AnimateBeginEvent;

using sofa::core::visual::VisualParams;
using sofa::defaulttype::Mat;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;
using std::endl;
using std::cout;
using std::stringstream;
using sofa::defaulttype::Vec4f;
using sofa::defaulttype::Vec3d;
using std::ifstream;
using std::ofstream;
using sofa::helper::vector;


template<class DataTypes>
AnimationEditor<DataTypes>::AnimationEditor()
    :
      d_maxKeyFrame(initData(&d_maxKeyFrame,(int)150,"maxKeyFrame","Max >= 1, default 150"))
    , d_filename(initData(&d_filename,string("animation.txt"),"filename","If no filename given, set default to animation.txt"))
    , d_loop(initData(&d_loop,false,"loop","If true, will loop on the animation (only in play mode)."))
    , d_load(initData(&d_load,true,"load","If true, will load the animation at init."))
    , d_dx(initData(&d_dx,0.,"dx","Variation of displacement. You can control the animation on displacement instead of time.\n"
                                  "If dx is set, at each time step, the animation will progress in term of displacement/distance.\n"
                                  "A positive dx means move forward and a negative dx means backward (on the timeline)."))
    , d_frameTime(initData(&d_frameTime,0.01,"frameTime","Frame time."))
    , d_drawTimeline(initData(&d_drawTimeline,false,"drawTimeLine",""))
    , d_cursor(initData(&d_cursor,0,"cursor","Current frame of the cursor along the timeline"))
    , m_state(nullptr)
    , m_isFrameDirty(false)
    , m_isPlaying(false)
    , m_maxKeyFrameID(0)
    , m_time(0.)
    , m_dx(0.)
{
    d_drawTimeline.setGroup("Visualization");
}


template<class DataTypes>
AnimationEditor<DataTypes>::~AnimationEditor()
{

}


template<class DataTypes>
void AnimationEditor<DataTypes>::init()
{
    m_componentstate = ComponentState::Invalid;

    m_keyFramesID.clear();

    if(d_maxKeyFrame.getValue()<=0)
        d_maxKeyFrame.setValue(1);

    m_state = dynamic_cast<MechanicalState<DataTypes>*>(getContext()->getMechanicalState());
    if(m_state == nullptr)
    {
        msg_error() <<"This component needs a mechanical state in its context with the correct template. Abort.";
        return;
    }

    m_keyFramesID.push_back(0);
    m_animation.resize(1,m_state->read(core::ConstVecCoordId::position())->getValue());

    if(d_load.getValue())
        loadAnimation();

    m_componentstate = ComponentState::Valid;
}


template<class DataTypes>
void AnimationEditor<DataTypes>::reinit()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if(d_maxKeyFrame.getValue()<=0)
        d_maxKeyFrame.setValue(1);

    int nbFrames = m_animation.size();
    if(d_maxKeyFrame.getValue()<nbFrames)
    {
        m_animation.erase(m_animation.begin()+d_maxKeyFrame.getValue()+1,m_animation.end());
        for (unsigned int i=1; i<m_keyFramesID.size(); i++)
            if (m_keyFramesID[i]>d_maxKeyFrame.getValue())
            {
                m_keyFramesID.erase(m_keyFramesID.begin()+i);
                --i;
            }
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::bwdInit(){}


template<class DataTypes>
void AnimationEditor<DataTypes>::reset()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    d_cursor.setValue(0);
    m_isFrameDirty = true;
    m_time = 0.;
    m_incrKeyFrame = 0;
}


template<class DataTypes>
void AnimationEditor<DataTypes>::loadAnimation()
{
    ifstream file;
    file.open(d_filename.getValue(), ifstream::in);
    if (file.is_open())
    {
        unsigned int nbFrames = 0;
        unsigned int nbPositions = 0;
        string line;
        int lineId = 0;

        while (getline(file,line))
        {
            if(lineId == 0)
            {
                line.erase(line.begin(),line.begin()+6);
                string type = line.c_str();
                if(DataTypes::Name()!=type)
                    msg_error() << "Animation positions are "<< type <<" while mechanical context is templated with "<<DataTypes::Name();
            }

            if(lineId == 1)
            {
                line.erase(line.begin(),line.begin()+17);
                nbFrames = atoi(line.c_str());
            }

            if(lineId == 2)
            {
                line.erase(line.begin(),line.begin()+20);
                nbPositions = atoi(line.c_str());

                if (m_state->getSize() != nbPositions)
                {
                    msg_warning() <<"Size from file does not match mechanical state, abort.";
                    return;
                }
                else
                {
                    msg_info() <<"Loaded animation of "<<nbFrames<<" frames from "<<d_filename.getValue();
                    m_animation.clear();
                    m_animation.resize(nbFrames);
                    m_keyFramesID.clear();
                    d_cursor.setValue(0);
                    if(d_maxKeyFrame.getValue()<(int)nbFrames)
                        d_maxKeyFrame.setValue(nbFrames+nbFrames/2);
                }
            }

            if(lineId == 3)
            {
                line.erase(line.begin(),line.begin()+10);
                string key;
                stringstream s(line);
                for (int i = 0; s >> key; i++)
                {
                    int keyId = atoi(key.c_str());
                    m_keyFramesID.push_back(keyId);
                }
            }

            if(lineId > 3)
            {
                stringstream s(line);
                m_animation[lineId-4].resize(nbPositions);
                Coord position;
                unsigned int i=0;
                while(i<nbPositions)
                {
                    for (unsigned int j=0; j<position.size(); j++)
                        s >> position[j];
                    m_animation[lineId-4][i++] = position;
                }
            }
            lineId++;
        }
        file.close();
        m_isFrameDirty = true;
        m_maxKeyFrameID = getMaxKeyFrameID();
        msg_info() <<"Load file succeeded";
    }
    else
        msg_warning() <<"Error in attempt to load file";
}


template<class DataTypes>
void AnimationEditor<DataTypes>::saveAnimation()
{
    msg_info() <<"Saved animation of "<<m_animation.size()<<" frames in "<<d_filename.getValue();
    ofstream file;
    file.open(d_filename.getValue());
    if(!file.is_open())
    {
        msg_warning() <<"Failed to open file, abort.";
        return;
    }

    file << "Type: "<< DataTypes::Name() << endl;
    file << "Number of frame: " << m_animation.size() << endl;
    file << "Number of effector: " << m_state->getSize() << endl;

    file << "KeyFrames:";
    for(unsigned int i=0; i<m_keyFramesID.size(); i++)
        file << " " << m_keyFramesID[i];
    file << endl;

    for(unsigned int i=0; i<m_animation.size(); i++)
    {
        for(unsigned int j=0; j<m_animation[i].size(); j++)
            for(unsigned int k=0; k<m_animation[i][j].size(); k++)
                file << m_animation[i][j][k] << " ";
        file << endl;
    }

    file.close();
}


template<class DataTypes>
void AnimationEditor<DataTypes>::onBeginAnimationStep(const double dt)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if(d_dx.isSet())
    {
        moveCursorWithdx();
    }
    else if(m_isPlaying)
    {
        m_time+=dt;

        if(m_time>=d_frameTime.getValue())
        {
            m_time = 0.;
            d_cursor.setValue(d_cursor.getValue() + 1);
            if(d_cursor.getValue() > d_maxKeyFrame.getValue())
            {
                d_cursor.setValue(d_maxKeyFrame.getValue());
                m_isPlaying = false;
            }
            else
                m_isFrameDirty = true;

            if(d_loop.getValue() && d_cursor.getValue()>m_maxKeyFrameID)
                d_cursor.setValue(0);
        }
    }

    if(m_isFrameDirty)
    {
        m_isFrameDirty = false;
        if(d_cursor.getValue()<(int)m_animation.size())
            m_state->write(core::VecCoordId::position())->setValue(m_animation[d_cursor.getValue()]);
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::onEndAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);
}


template<class DataTypes>
void AnimationEditor<DataTypes>::handleEvent(Event *event)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if(!d_dx.isSet())
    {
        if (KeypressedEvent* keyEvent = dynamic_cast<KeypressedEvent*>(event))
        {
            if(keyEvent->getKey() == 18 || keyEvent->getKey() == 20) moveCursor(keyEvent->getKey());
            else if(keyEvent->getKey() == 17 || keyEvent->getKey() == 16) moveCursor(keyEvent->getKey());
            else if(keyEvent->getKey() == '+' || keyEvent->getKey() == '-') moveCursor(keyEvent->getKey());
            else if(keyEvent->getKey() == 'A') addKeyFrame();                 // Ctrl+a (add)
            else if(keyEvent->getKey() == 'D') deleteKeyFrame();              // Ctrl+d (delete)
            else if(keyEvent->getKey() == 'W') saveAnimation();               // Ctrl+w (write)
            else if(keyEvent->getKey() == 'C') copyKeyFrame();                // Ctrl+c (copy)
            else if(keyEvent->getKey() == 'V') pasteKeyFrame();               // Ctrl+v (paste)
            else if(keyEvent->getKey() == 'X') cutKeyFrame();                 // Ctrl+x (cut)
            else if(keyEvent->getKey() == 'M') m_isPlaying = !m_isPlaying;    // Ctrl+m (play/pause)
        }
    }

    if (AnimateBeginEvent::checkEventType(event))
    {
        double dt = getContext()->getDt();
        onBeginAnimationStep(dt);
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::moveCursor(const char key)
{
    if(key == 20)//Move cursor to right direction : Ctrl+Right Arrow
    {
        m_isFrameDirty = true;
        d_cursor.setValue(d_cursor.getValue()+1);
    }

    if(key == 18)//Move cursor to left direction : Ctrl+Left Arrow
    {
        m_isFrameDirty = true;
        d_cursor.setValue(d_cursor.getValue()-1);
    }

    if(key == 17)//Move cursor fast to right direction : Ctrl+Fn+Right Arrow
    {
        m_isFrameDirty = true;
        d_cursor.setValue(d_cursor.getValue()+(d_maxKeyFrame.getValue()/20));
    }

    if(key == 16)//Move cursor fast to left direction : Ctrl+Fn+Left Arrow
    {
        m_isFrameDirty = true;
        d_cursor.setValue(d_cursor.getValue()-(d_maxKeyFrame.getValue()/20));
    }

    if(key == '+')//Move cursor to the next nearest key frame : Ctrl+'+'
    {
        if (d_cursor.getValue() < m_maxKeyFrameID)
        {
            int nextKeyFrame = m_maxKeyFrameID;
            for(unsigned int i=0; i<m_keyFramesID.size(); i++)
                if(m_keyFramesID[i]<nextKeyFrame && m_keyFramesID[i]>d_cursor.getValue())
                    nextKeyFrame = m_keyFramesID[i];

            d_cursor.setValue(nextKeyFrame);
            m_isFrameDirty = true;
        }
    }

    if(key == '-')//Move cursor to the previous nearest key frame : Ctrl+'-'
    {
        if (d_cursor.getValue() > 0)
        {
            int previousKeyFrame = 0;
            for(unsigned int i=0; i<m_keyFramesID.size(); i++)
                if(m_keyFramesID[i]>previousKeyFrame && m_keyFramesID[i]<d_cursor.getValue())
                    previousKeyFrame = m_keyFramesID[i];

            d_cursor.setValue(previousKeyFrame);
            m_isFrameDirty = true;
        }
    }

    if (d_cursor.getValue()<0)
    {
        m_isFrameDirty = false;
        d_cursor.setValue(0);
    }

    if (d_cursor.getValue()>d_maxKeyFrame.getValue())
    {
        m_isFrameDirty = false;
        d_cursor.setValue(d_maxKeyFrame.getValue());
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::moveCursorWithdx()
{
    ReadAccessor<Data<double>> dx = d_dx;
    int cursor = d_cursor.getValue();
    if(dx>0)
    {
        if(d_cursor.getValue()<m_maxKeyFrameID)
        {
            vector<Coord> positions1 = m_animation[d_cursor.getValue()];
            d_cursor.setValue(d_cursor.getValue()+1);
            vector<Coord> positions2 = m_animation[d_cursor.getValue()];
            bool cont = true;
            while(cont && d_cursor.getValue()<m_maxKeyFrameID)
            {
                for(unsigned int i=0; i<positions1.size(); i++)
                {
                    double displacement = (positions1[i]-positions2[i]).norm();
                    if(displacement>dx+m_dx)
                    {
                        m_dx += dx;
                        d_cursor.setValue(d_cursor.getValue()-1);
                        cont = false;
                        break;
                    }
                    else
                        m_dx = 0;
                }

                if(cont)
                {
                    d_cursor.setValue(d_cursor.getValue() + 1);
                    positions2 = m_animation[d_cursor.getValue()];
                }
            }
        }
    }
    else if(dx<0)
    {
        if(d_cursor.getValue()>0)
        {
            vector<Coord> positions1 = m_animation[d_cursor.getValue()];
            d_cursor.setValue(d_cursor.getValue() - 1);
            vector<Coord> positions2 = m_animation[d_cursor.getValue()];
            bool cont = true;
            while(cont && d_cursor.getValue()>0)
            {
                for(unsigned int i=0; i<positions1.size(); i++)
                {
                    double displacement = (positions1[i]-positions2[i]).norm();
                    if(displacement>-(dx+m_dx))
                    {
                        m_dx += dx;
                        d_cursor.setValue(d_cursor.getValue() + 1);
                        cont = false;
                        break;
                    }
                    else
                        m_dx = 0;
                }

                if(cont)
                {
                    d_cursor.setValue(d_cursor.getValue()-1);
                    positions2 = m_animation[d_cursor.getValue()];
                }
            }
        }
    }

    if(cursor!=d_cursor.getValue())
        m_isFrameDirty = true;
}


template<class DataTypes>
void AnimationEditor<DataTypes>::addKeyFrame()
{
    int index;
    bool isKey = isCursorKeyFrame(index);

    if (!isKey)
        m_keyFramesID.push_back(d_cursor.getValue());

    if(d_cursor.getValue()>m_maxKeyFrameID)
        m_maxKeyFrameID = d_cursor.getValue();
    updateAnimation(ST_ADD);
}


template<class DataTypes>
void AnimationEditor<DataTypes>::deleteKeyFrame()
{
    int index = 0;
    bool isKey = isCursorKeyFrame(index);

    if (isKey && index!=0)
    {
        updateAnimation(ST_DELETE);
        m_isFrameDirty = true;
        m_keyFramesID.erase(m_keyFramesID.begin() + index);

        if(m_maxKeyFrameID==d_cursor.getValue())
            m_maxKeyFrameID = getMaxKeyFrameID();
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::copyKeyFrame()
{
    int index = 0;
    bool isKey = isCursorKeyFrame(index);

    if (isKey)
        m_frameCopy = m_animation[d_cursor.getValue()];
}


template<class DataTypes>
void AnimationEditor<DataTypes>::cutKeyFrame()
{
    int index = 0;
    bool isKey = isCursorKeyFrame(index);

    if (isKey)
    {
        m_frameCopy = m_animation[d_cursor.getValue()];
        updateAnimation(ST_DELETE);
        m_keyFramesID.erase(m_keyFramesID.begin() + index);

        if(m_maxKeyFrameID==d_cursor.getValue())
            m_maxKeyFrameID = getMaxKeyFrameID();
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::pasteKeyFrame()
{
    if(m_frameCopy.size()==0)
        return;

    int index = 0;
    bool isKey = isCursorKeyFrame(index);

    if (!isKey)
    {
        m_keyFramesID.push_back(d_cursor.getValue());
        m_isFrameDirty = true;
    }

    if(d_cursor.getValue()>m_maxKeyFrameID)
        m_maxKeyFrameID = d_cursor.getValue();
    updateAnimation(ST_PASTE);
}


template<class DataTypes>
bool AnimationEditor<DataTypes>::isCursorKeyFrame(int &index)
{
    bool isKey = false;
    index = 0;
    for (unsigned int i=0; i<m_keyFramesID.size(); i++)
        if (d_cursor.getValue() == m_keyFramesID[i])
        {
            isKey = true;
            index = i;
            break;
        }
    return isKey;
}


template<class DataTypes>
int AnimationEditor<DataTypes>::getMaxKeyFrameID()
{
    int maxID = 0;
    for (int i=0; i<(int)m_keyFramesID.size(); i++)
        if (maxID < m_keyFramesID[i])
            maxID = m_keyFramesID[i];
    return maxID;
}


template<class DataTypes>
void AnimationEditor<DataTypes>::updateAnimation(Status status)
{
    if(m_state == nullptr)
    {
        msg_warning() <<"Failed to fetch a mechanical state, abort.";
        return;
    }

    int nbKeyFrame  = m_keyFramesID.size();
    int currentKey  = d_cursor.getValue();
    int previousKey = 0;
    int nextKey     = d_maxKeyFrame.getValue()+1;

    // Get previous and next keyframe
    for (int i=0; i<nbKeyFrame; i++)
    {
        if(m_keyFramesID[i]<currentKey && m_keyFramesID[i]>previousKey)
            previousKey = m_keyFramesID[i];

        if(m_keyFramesID[i]>currentKey && m_keyFramesID[i]<nextKey)
            nextKey = m_keyFramesID[i];
    }

    switch(status)
    {
    case ST_DELETE:
    {
        if(nextKey != d_maxKeyFrame.getValue()+1) //Key after this one
            updateAnimationWithInterpolation(previousKey, nextKey);
        else //No key after this one
            m_animation.erase(m_animation.begin() + previousKey+1, m_animation.end());
        break;
    }
    case ST_ADD:
    {
        if(nextKey != d_maxKeyFrame.getValue()+1) //Key after this new one
        {
            m_animation[currentKey] = m_state->read(core::ConstVecCoordId::position())->getValue();
            if(currentKey!=0) updateAnimationWithInterpolation(previousKey, currentKey);
            updateAnimationWithInterpolation(currentKey, nextKey);
        }
        else //No key after this new one
        {
            m_animation.resize(currentKey+1);
            m_animation[currentKey] = m_state->read(core::ConstVecCoordId::position())->getValue();
            if(currentKey!=0) updateAnimationWithInterpolation(previousKey, currentKey);
        }
        break;
    }
    case ST_PASTE:
    {
        if(nextKey != d_maxKeyFrame.getValue()+1) //Key after this new one
        {
            m_animation[currentKey] = m_frameCopy;
            if(currentKey!=0) updateAnimationWithInterpolation(previousKey, currentKey);
            updateAnimationWithInterpolation(currentKey, nextKey);
        }
        else //No key after this new one
        {
            m_animation.resize(currentKey+1);
            m_animation[currentKey] = m_frameCopy;
            if(currentKey!=0) updateAnimationWithInterpolation(previousKey, currentKey);
        }
        break;
    }
    default: break;
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::updateAnimationWithInterpolation(const int startKey,
                                                                  const int endKey)
{
    if(m_state == nullptr)
    {
        msg_warning() <<"Failed to fetch a mechanical state, abort.";
        return;
    }

    vector<Coord> previousPositions = m_animation[startKey];
    vector<Coord> currentPositions  = m_animation[endKey];

    if (currentPositions.size() != previousPositions.size())
    {
        msg_warning() <<"This component does not handle mechanical state size changes";
        return;
    }

    int nbPositions = m_state->read(core::ConstVecCoordId::position())->getValue().size();
    int nbStep = endKey - startKey;

    for (int i=0; i<nbStep; i++)
    {
        vector<Coord> newPositions;
        for (int k=0; k<nbPositions; k++)
        {
            Coord direction = currentPositions[k]- previousPositions[k];
            double distance = direction.norm();
            double step = distance/nbStep*i;
            direction.normalize();
            newPositions.push_back(previousPositions[k]+direction*step);
        }
        m_animation[startKey+i] = newPositions;
    }
}


template<class DataTypes>
void AnimationEditor<DataTypes>::draw(const VisualParams* vparams)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if(!d_drawTimeline.getValue()) return;

#ifdef SOFA_WITH_DACCORD
    // If the currently selected object is not a time line... we do nothing.
    if( dynamic_cast<AnimationEditor<DataTypes>*>(Editor::getSelected()) == nullptrptr )
        return ;
#endif // SOFA_WITH_DACCORD

#ifdef SOFA_WITH_OPENGL
    glDisable(GL_LIGHTING);
    int ratio = round(vparams->viewport()[2]/d_maxKeyFrame.getValue());

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, vparams->viewport()[2], 0, vparams->viewport()[3] ,0, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();


    //////////////////////////////AnimationEditor/////////////////////////////
    Vec3d ULPosition = Vec3d(0,vparams->viewport()[3]/12.,0);
    Vec3d URPosition = Vec3d(vparams->viewport()[2],vparams->viewport()[3]/12,0);
    Vec3d DRPosition = Vec3d(vparams->viewport()[2],vparams->viewport()[3]/10.,0);
    Vec3d DLPosition = Vec3d(0,vparams->viewport()[3]/10.,0);

    glColor3f(0.4f, 0.4f, 0.4f);
    glBegin(GL_QUADS);
    glVertex2f(ULPosition[0], ULPosition[1]);
    glVertex2f(URPosition[0], URPosition[1]);
    glVertex2f(DRPosition[0], DRPosition[1]);
    glVertex2f(DLPosition[0], DLPosition[1]);
    glEnd();

    int maxKey = 0;
    for(unsigned int i=0; i<m_keyFramesID.size(); i++)
        if(m_keyFramesID[i]>maxKey)
            maxKey = m_keyFramesID[i];

    ULPosition = Vec3d(ratio,vparams->viewport()[3]/12.,0);
    URPosition = Vec3d((maxKey+1)*ratio,vparams->viewport()[3]/12,0);
    DRPosition = Vec3d((maxKey+1)*ratio,vparams->viewport()[3]/10.,0);
    DLPosition = Vec3d(ratio,vparams->viewport()[3]/10.,0);

    glColor3f(0.8f, 0.7f, 0.6f);
    glBegin(GL_QUADS);
    glVertex2f(ULPosition[0], ULPosition[1]);
    glVertex2f(URPosition[0], URPosition[1]);
    glVertex2f(DRPosition[0], DRPosition[1]);
    glVertex2f(DLPosition[0], DLPosition[1]);
    glEnd();
    ///////////////////////////////////////////////

    //////////////TRIANGLES////////////////////////
    glBegin(GL_TRIANGLES);


    //Initial key frame
    glColor3f(0.6f, 0.6f, 0.6f);
    glVertex2f(ratio, vparams->viewport()[3]/10);
    glVertex2f(ratio+5, vparams->viewport()[3]/10 + 10);
    glVertex2f(ratio-5, vparams->viewport()[3]/10 + 10);

    //Max key frame
    glColor3f(0.4f, 0.f, 0.f);
    glVertex2f((d_maxKeyFrame.getValue()+1)*ratio, vparams->viewport()[3]/10);
    glVertex2f((d_maxKeyFrame.getValue()+1)*ratio+5, vparams->viewport()[3]/10 + 10);
    glVertex2f((d_maxKeyFrame.getValue()+1)*ratio-5, vparams->viewport()[3]/10 + 10);

    //Selected
    glColor3f(0.6f, 0.6f, 0.6f);
    for (unsigned int i=0; i<m_keyFramesID.size(); i++)
    {
        glVertex2f((m_keyFramesID[i]+1)*ratio, vparams->viewport()[3]/10);
        glVertex2f(5+(m_keyFramesID[i]+1)*ratio, vparams->viewport()[3]/10 + 10);
        glVertex2f(-5+(m_keyFramesID[i]+1)*ratio, vparams->viewport()[3]/10 + 10);
    }

    //Current cursor
    glColor3f(0.9f, 0.9f, 0.9f);
    glVertex2f((d_cursor.getValue()+1)*ratio, vparams->viewport()[3]/10);
    glVertex2f(5+(d_cursor.getValue()+1)*ratio, vparams->viewport()[3]/10 + 10);
    glVertex2f(-5+(d_cursor.getValue()+1)*ratio, vparams->viewport()[3]/10 + 10);

    glEnd();

    ///////////////////////////////////////////////////

    /// /////////////////////LINES/////////////////////
    for(unsigned int i=0; i<m_keyFramesID.size(); i++)
    {
        vector<Vec3d> line;
        line.push_back(Vec3d((m_keyFramesID[i]+1)*ratio,vparams->viewport()[3]/12,0.));
        line.push_back(Vec3d((m_keyFramesID[i]+1)*ratio,vparams->viewport()[3]/10,0.));
        vparams->drawTool()->drawLines(line,2,Vec4f(1.,0.9,0.,1.));
    }

    vector<Vec3d> line;
    line.push_back(Vec3d((d_maxKeyFrame.getValue()+1)*ratio,vparams->viewport()[3]/12,0.));
    line.push_back(Vec3d((d_maxKeyFrame.getValue()+1)*ratio,vparams->viewport()[3]/10,0.));
    vparams->drawTool()->drawLines(line,2,Vec4f(0.6,0.,0.,1.));
    ////////////////////////////////////////////////

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
#endif
}

}//namespace _animationeditor_
}//namespace controller
}//namespace component
}//namespace sofa

#endif // SOFA_CONTROLLER_ANIMATIONEDITOR_INL
