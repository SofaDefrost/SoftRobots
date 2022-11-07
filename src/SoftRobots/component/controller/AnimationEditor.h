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
#ifndef SOFA_CONTROLLER_ANIMATIONEDITOR_H
#define SOFA_CONTROLLER_ANIMATIONEDITOR_H

#include <sofa/component/controller/Controller.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/Vec.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <SoftRobots/component/initSoftRobots.h>

namespace sofa
{

namespace component
{

namespace controller
{

namespace _animationeditor_
{

using sofa::type::Vec;
using core::objectmodel::Event;
using type::vector;
using core::behavior::MechanicalState;
using std::string;
using core::visual::VisualParams;


/**
 * This component is used to build an animation from key points motion, or typically to build effector goals trajectories.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template<class DataTypes>
class SOFA_SOFTROBOTS_API AnimationEditor : public Controller
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(AnimationEditor,DataTypes),Controller);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord    Coord   ;
    typedef typename DataTypes::Deriv    Deriv   ;


    enum Status
    {
        ST_ADD, ST_DELETE, ST_PASTE
    };


    AnimationEditor();
    ~AnimationEditor() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
    void draw(const core::visual::VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Controller ////////////////////
    void onBeginAnimationStep(const double dt) override;
    void onEndAnimationStep(const double dt) override;
    void handleEvent(Event *event) override;
    /////////////////////////////////////////////////////////////////////////

    void setCursor(const int& cursor) {d_cursor.setValue(cursor); m_isFrameDirty=true;}
    int getCursor() {return d_cursor.getValue();}
    void setIsPlaying(const bool& isPlaying) {m_isPlaying = isPlaying;}
    vector<unsigned int> getKeyFramesID() {return m_keyFramesID;}

    Data<unsigned int>      d_maxKeyFrame;
    Data<string>            d_filename;
    Data<bool>              d_loop;
    Data<bool>              d_load;
    Data<double>            d_dx;
    Data<double>            d_frameTime;
    Data<bool>              d_drawTimeline;
    Data<double>            d_drawSize;
    Data<bool>              d_drawTrajectory;
    Data<unsigned int>      d_cursor;

    vector<unsigned int>    m_keyFramesID;
    vector<vector<Coord>>   m_animation;

    void saveAnimation();
    void loadAnimation();
    void addKeyFrame();
    void deleteKeyFrame();
    void copyKeyFrame();
    void pasteKeyFrame();
    void cutKeyFrame();
    void setEnableRepeat(bool repeat){d_loop.setValue(repeat);}
    double getTime(){return m_time;}
    void setPlaying(bool isPlaying){m_isPlaying = isPlaying;}

protected:

    MechanicalState<DataTypes>*      m_state;
    bool                             m_isFrameDirty;
    vector<Coord>                    m_frameCopy;
    bool                             m_isPlaying;
    unsigned int                     m_maxKeyFrameID;
    unsigned int                     m_incrKeyFrame;
    double                           m_time;
    double                           m_dx;

    void updateAnimation(Status s);
    void updateAnimationWithInterpolation(const int startKey,
                                          const int endKey);

    void moveCursor(const char key);
    void moveCursorWithdx();

    bool isCursorKeyFrame(int &index);
    unsigned int getMaxKeyFrameID();

    void drawTimeline(const VisualParams* vparams);
    void drawTrajectory(const VisualParams* vparams);

};   //class AnimationEditor


// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
using defaulttype::Vec3Types;
extern template class SOFA_SOFTROBOTS_API AnimationEditor<Vec3Types>;


}   //namespace _animationeditor_

using _animationeditor_::AnimationEditor;

}   //namespace controller
}   //namespace component
}   //namespace sofa

#endif // SOFA_CONTROLLER_ANIMATIONEDITOR_H
