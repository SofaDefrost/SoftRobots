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

#include <SoftRobots/component/controller/AnimationEditor.h>
#ifdef SOFA_WITH_DACCORD
#include "../../applications/sofa/gui/SofaGuiCommon/editor/editor.h"
using daccord::current::Editor ;
#endif

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/Event.h>

#if SOFTROBOTS_HAVE_SOFA_GL
#include <sofa/gl/template.h>
using sofa::gl::glVertexT;
#endif

#include <fstream>
#include <iomanip>
#include <fstream>


namespace softrobots::controller
{

using sofa::core::objectmodel::ComponentState;

using sofa::core::objectmodel::KeypressedEvent;
using sofa::core::objectmodel::MouseEvent;
using sofa::simulation::AnimateBeginEvent;

using sofa::core::visual::VisualParams;
using sofa::type::Mat;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;
using std::endl;
using std::cout;
using std::stringstream;
using sofa::type::RGBAColor;
using sofa::type::Vec3d;
using sofa::type::Vec4d;
using std::ifstream;
using std::ofstream;
using sofa::type::vector;


template<class DataTypes>
AnimationEditor<DataTypes>::AnimationEditor()
    :
      d_maxKeyFrame(initData(&d_maxKeyFrame,(unsigned int)150,"maxKeyFrame","Max >= 1, default 150"))
    , d_filename(initData(&d_filename,string("animation.txt"),"filename","If no filename given, set default to animation.txt"))
    , d_loop(initData(&d_loop,false,"loop","If true, will loop on the animation (only in play mode)."))
    , d_load(initData(&d_load,true,"load","If true, will load the animation at init."))
    , d_dx(initData(&d_dx,0.,"dx","Variation of displacement. You can control the animation on displacement instead of time.\n"
                                  "If dx is set, at each time step, the animation will progress in term of displacement/distance.\n"
                                  "A positive dx means move forward and a negative dx means backward (on the timeline)."))
    , d_frameTime(initData(&d_frameTime,0.01,"frameTime","Frame time."))
    , d_drawTimeline(initData(&d_drawTimeline,true,"drawTimeline",""))
    , d_drawSize(initData(&d_drawSize,0.1,"drawSize",""))
    , d_drawTrajectory(initData(&d_drawTrajectory,false,"drawTrajectory",""))
    , d_cursor(initData(&d_cursor,(unsigned int)0,"cursor","Current frame of the cursor along the timeline"))
    , m_state(nullptr)
    , m_isFrameDirty(false)
    , m_isPlaying(false)
    , m_maxKeyFrameID(0)
    , m_time(0.)
    , m_dx(0.)
{
    d_drawTimeline.setGroup("Visualization");
    d_drawTrajectory.setGroup("Visualization");
    d_drawSize.setGroup("Visualization");
    f_listening.setValue(true);
}


template<class DataTypes>
AnimationEditor<DataTypes>::~AnimationEditor()
{

}


template<class DataTypes>
void AnimationEditor<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);

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
    m_animation.resize(1,m_state->readPositions().ref());

    if(d_load.getValue())
        loadAnimation();

    d_componentState.setValue(ComponentState::Valid);
}


template<class DataTypes>
void AnimationEditor<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if(d_maxKeyFrame.getValue()<=0)
        d_maxKeyFrame.setValue(1);

    unsigned int nbFrames = m_animation.size();
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
    if(d_componentState.getValue() != ComponentState::Valid)
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
                    if(d_maxKeyFrame.getValue()<nbFrames)
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
    if(d_componentState.getValue() != ComponentState::Valid)
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
        if(d_cursor.getValue()<m_animation.size())
            m_state->write(sofa::core::VecCoordId::position())->setValue(m_animation[d_cursor.getValue()]);
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
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if(!d_dx.isSet())
    {
        if (KeypressedEvent* keyEvent = dynamic_cast<KeypressedEvent*>(event))
        {
            if(keyEvent->getKey() == 18 || keyEvent->getKey() == 20) moveCursor(keyEvent->getKey());
            else if(keyEvent->getKey() == 17 || keyEvent->getKey() == 16) moveCursor(keyEvent->getKey());
            else if(keyEvent->getKey() == 22 || keyEvent->getKey() == 23) moveCursor(keyEvent->getKey());
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
        if(d_cursor.getValue()>0)
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
        if(d_cursor.getValue()>0)
            d_cursor.setValue(d_cursor.getValue()-(d_maxKeyFrame.getValue()/20));
    }

    if(key == 23)//Move cursor to the next nearest key frame : Ctrl+PgDn
    {
        if (d_cursor.getValue() < m_maxKeyFrameID)
        {
            unsigned int nextKeyFrame = m_maxKeyFrameID;
            for(unsigned int i=0; i<m_keyFramesID.size(); i++)
                if(m_keyFramesID[i]<nextKeyFrame && m_keyFramesID[i]>d_cursor.getValue())
                    nextKeyFrame = m_keyFramesID[i];

            d_cursor.setValue(nextKeyFrame);
            m_isFrameDirty = true;
        }
    }

    if(key == 22)//Move cursor to the previous nearest key frame : Ctrl+PgUp
    {
        if (d_cursor.getValue() > 0)
        {
            unsigned int previousKeyFrame = 0;
            for(unsigned int i=0; i<m_keyFramesID.size(); i++)
                if(m_keyFramesID[i]>previousKeyFrame && m_keyFramesID[i]<d_cursor.getValue())
                    previousKeyFrame = m_keyFramesID[i];

            d_cursor.setValue(previousKeyFrame);
            m_isFrameDirty = true;
        }
    }

    if (d_cursor.getValue()==0)
    {
        m_isFrameDirty = false;
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
    ReadAccessor<sofa::Data<double>> dx = d_dx;
    unsigned int cursor = d_cursor.getValue();
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
            index = int(i);
            break;
        }
    return isKey;
}


template<class DataTypes>
unsigned int AnimationEditor<DataTypes>::getMaxKeyFrameID()
{
    unsigned int maxID = 0;
    for (unsigned int i=0; i<m_keyFramesID.size(); i++)
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

    unsigned int nbKeyFrame  = m_keyFramesID.size();
    unsigned int currentKey  = d_cursor.getValue();
    unsigned int previousKey = 0;
    unsigned int nextKey     = d_maxKeyFrame.getValue()+1;

    // Get previous and next keyframe
    for (unsigned int i=0; i<nbKeyFrame; i++)
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
            m_animation[currentKey] = m_state->readPositions().ref();
            if(currentKey!=0) updateAnimationWithInterpolation(previousKey, currentKey);
            updateAnimationWithInterpolation(currentKey, nextKey);
        }
        else //No key after this new one
        {
            m_animation.resize(currentKey+1);
            m_animation[currentKey] = m_state->readPositions().ref();
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

    int nbPositions = m_state->getSize();
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
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if(d_drawTimeline.getValue())
        drawTimeline(vparams);

    if(d_drawTrajectory.getValue())
        drawTrajectory(vparams);
}

template<class DataTypes>
void AnimationEditor<DataTypes>::drawTimeline(const VisualParams* vparams)
{
#ifdef SOFA_WITH_DACCORD
    // If the currently selected object is not a time line... we do nothing.
    if( dynamic_cast<AnimationEditor<DataTypes>*>(Editor::getSelected()) == nullptrptr )
        return ;
#endif // SOFA_WITH_DACCORD

#if SOFTROBOTS_HAVE_SOFA_GL
    glDisable(GL_LIGHTING);
    unsigned int ratio = round(vparams->viewport()[2]/d_maxKeyFrame.getValue());

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, vparams->viewport()[2], 0, vparams->viewport()[3] ,0, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    double yUPos = 50.;
    double yDPos = 0.;
    double sizeCursor = yUPos/5.;
    double xShift = 25.;

    //////////////////////////////Timeline/////////////////////////////
    vector<Vec3d> quad;
    quad.push_back(Vec3d(0,yUPos,0)); //UR
    quad.push_back(Vec3d(vparams->viewport()[2],yUPos,0)); //UL
    quad.push_back(Vec3d(vparams->viewport()[2],yDPos,0)); //DL
    quad.push_back(Vec3d(0,yDPos,0)); //DR
    vparams->drawTool()->drawQuads(quad,RGBAColor(0.4f,0.4f,0.4f,1.0f));

    unsigned int maxKey = 0;
    for(unsigned int i=0; i<m_keyFramesID.size(); i++)
        if(m_keyFramesID[i]>maxKey)
            maxKey = m_keyFramesID[i];

    quad.clear();
    quad.push_back(Vec3d(ratio+xShift,yUPos,0)); //UR
    quad.push_back(Vec3d((maxKey+1)*ratio+xShift,yUPos,0)); //UL
    quad.push_back(Vec3d((maxKey+1)*ratio+xShift,yDPos,0)); //DL
    quad.push_back(Vec3d(ratio+xShift,yDPos,0)); //DR
    vparams->drawTool()->drawQuads(quad,RGBAColor(0.0f,0.3f,0.0f,1.0f));
    ///////////////////////////////////////////////

    //////////////Cursor////////////////////////
    glBegin(GL_TRIANGLES);

    //Current cursor
    vector<Vec3d> triangle;
    triangle.push_back(Vec3d((d_cursor.getValue()+1)*ratio+xShift, yUPos, 0.));
    triangle.push_back(Vec3d(sizeCursor/2.+(d_cursor.getValue()+1)*ratio+xShift, yUPos + sizeCursor, 0.));
    triangle.push_back(Vec3d(-sizeCursor/2.+(d_cursor.getValue()+1)*ratio+xShift, yUPos + sizeCursor, 0.));
    vparams->drawTool()->drawTriangles(triangle,RGBAColor(0.9f,0.9f,0.9f,1.0f));
    ///////////////////////////////////////////////////

    /// /////////////////////KeyFrames/////////////////////
    for(unsigned int i=0; i<m_keyFramesID.size(); i++)
    {
        vector<Vec3d> line;
        line.push_back(Vec3d((m_keyFramesID[i]+1)*ratio+xShift,yUPos,0.));
        line.push_back(Vec3d((m_keyFramesID[i]+1)*ratio+xShift,yDPos,0.));
        vparams->drawTool()->drawLines(line,2,RGBAColor(0.0f,0.5f,0.0f,1.0f));
    }

    vector<Vec3d> line;
    line.push_back(Vec3d((d_maxKeyFrame.getValue()+1)*ratio+xShift,yUPos,0.));
    line.push_back(Vec3d((d_maxKeyFrame.getValue()+1)*ratio+xShift,yDPos,0.));
    vparams->drawTool()->drawLines(line,2,RGBAColor(0.6f,0.0f,0.0f,1.0f));
    ////////////////////////////////////////////////

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
#endif
}

template<class DataTypes>
void AnimationEditor<DataTypes>::drawTrajectory(const VisualParams* vparams)
{
    vector<Vec3d> points;
    vector<unsigned int> IDSorted = m_keyFramesID;
    unsigned int nbKey = m_keyFramesID.size();
    std::sort(IDSorted.begin(), IDSorted.begin() + nbKey);
    for(unsigned int k=0; k<nbKey; k++)
    {
        for(unsigned int j=0; j<m_animation[k].size(); j++)
            points.push_back(m_animation[IDSorted[k]][j]);
    }

    vector<Vec3d> lines;
    for(unsigned int i=0; i<points.size()-1; i++)
    {
        lines.push_back(points[i]);
        lines.push_back(points[i+1]);
    }

    vparams->drawTool()->drawPoints(points,d_drawSize.getValue()*5.,RGBAColor(0.4,0.4,0.4,1.));
    vparams->drawTool()->drawLines(lines,d_drawSize.getValue()*2.,RGBAColor(0.5,0.5,0.5,1.));
}

} // namespace

