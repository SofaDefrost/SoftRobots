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

#ifndef SOFA_CONTROLLER_GameTrak_CPP
#define SOFA_CONTROLLER_GameTrak_CPP

#include "GameTrakController.h"
#include <sofa/core/ObjectFactory.h>

#include <fstream>
#include <iomanip>
#include <fstream>

namespace sofa
{

namespace component
{

namespace controller
{

////////////////////////////////////////////    FACTORY    ////////////////////////////////////////////
using sofa::core::RegisterObject ;

// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
// 2-RegisterObject("description") + .add<> : Register the component
// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(GameTrakController)

int GameTrakControllerClass = RegisterObject("Interface for GameTrak device.")
.add< GameTrakController >()
;
///////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace gametrak ;
using std::exception;
using std::runtime_error;
using std::cout;
using std::endl;
using std::cerr;
using sofa::helper::WriteAccessor;
using sofa::helper::vector;
using sofa::defaulttype::Vec3d;
using helper::WriteAccessor;
using helper::OptionsGroup;

GameTrakController::GameTrakController()
    :
      d_translation(initData(&d_translation,Vec3d(0.,0.,0.), "translation","The gametrak zero is located between the two joysticks."))
    , d_offset(initData(&d_offset,Vec3d(0.,0.,0.), "offset","Starting offset between both joysticks."))
    , d_sizeCoeff(initData(&d_sizeCoeff,1.,"sizeCoeff","The gametrak datas are in millimeters."))

    , d_x( initData(&d_x, OptionsGroup(6,"x","-x","y","-y","z","-z"), "x","Option to change the direction of the x axis of gametrak. \n"
                                                                          "Corresponds to the right direction."))
    , d_y( initData(&d_y, OptionsGroup(6,"x","-x","y","-y","z","-z"), "y","Option to change the direction of the y axis of gametrak. \n"
                                                                          "Corresponds to the up lift direction."))
    , d_z( initData(&d_z, OptionsGroup(6,"x","-x","y","-y","z","-z"), "z","Option to change the direction of the z axis of gametrak. \n"
                                                                          "Corresponds to the direction towards the user."))

    , d_deviceURI(initData(&d_deviceURI,"deviceURI",""))
    , d_buttonPressed(initData(&d_buttonPressed, false,"buttonPressed","GameTrak pedal state"))
    , d_output(initData(&d_output,"positions","Output positions of game trak"))
    , m_initLeft(Vec3d(0.,0.,0.))
    , m_initRight(Vec3d(0.,0.,0.))
{
    d_translation.setGroup("Transformation");
    d_offset.setGroup("Transformation");
    d_sizeCoeff.setGroup("Transformation");
    d_x.setGroup("Transformation");
    d_y.setGroup("Transformation");
    d_z.setGroup("Transformation");

    initAxis();
}


GameTrakController::~GameTrakController()
{
    delete m_gameTrak ;
}


void GameTrakController::GameTrakCallbackStatic(void *context,
                                                TimeStamp::inttime timeStamp,
                                                double leftx, double lefty, double leftz,
                                                double rightx, double righty, double rightz,
                                                bool button)
{
    GameTrakController* pointer = static_cast<GameTrakController*>(context);
//    std::cout<<leftx<<std::endl;
    pointer->GameTrakCallback(context, timeStamp, leftx, lefty, leftz, rightx, righty, rightz, button);
}


void GameTrakController::GameTrakCallback(void *context,
                                          TimeStamp::inttime timeStamp,
                                          double leftx, double lefty, double leftz,
                                          double rightx, double righty, double rightz,
                                          bool button)
{
    SOFA_UNUSED(context);

    if (!m_gameTrak) return;

    WriteAccessor<Data<bool>> pedal = d_buttonPressed;

    Vec3d left(leftx,lefty,leftz);
    Vec3d right(rightx,righty,rightz);

    unsigned int xId = d_x.getValue().getSelectedId();
    unsigned int yId = d_y.getValue().getSelectedId();
    unsigned int zId = d_z.getValue().getSelectedId();

    double signx = (xId%2)?1.:-1.;
    double signy = (yId%2)?1.:-1.;
    double signz = (zId%2)?1.:-1.;

    m_output[0][0] = signx*left[xId/2]*d_sizeCoeff.getValue();
    m_output[0][1] = signy*left[yId/2]*d_sizeCoeff.getValue();
    m_output[0][2] = signz*left[zId/2]*d_sizeCoeff.getValue();

    m_output[1][0] = signx*right[xId/2]*d_sizeCoeff.getValue();
    m_output[1][1] = signy*right[yId/2]*d_sizeCoeff.getValue();
    m_output[1][2] = signz*right[zId/2]*d_sizeCoeff.getValue();

    m_isButtonPressed = button ;
    *pedal = button;
    m_lastTime = timeStamp;

}


void GameTrakController::init()
{
    WriteAccessor<Data<vector<Vec3d>>> output = d_output;
    output.resize(2); //TODO autorized multiple gametrak
    m_output.resize(2);

    initAxis();

    m_lastTime = 0;
    m_isButtonPressed = false;

    try
    {
        m_gameTrak = GameTrak::create(d_deviceURI.getValue()) ;
        m_gameTrak->setGameTrakCallback(&GameTrakCallbackStatic, this) ;
    }
    catch (runtime_error &e)
    {
        cerr << "Runtime error: " << e.what() << endl ;
    }
    catch (exception &e)
    {
        cerr << "Exception: " << e.what() << endl ;
    }

    m_initLeft[0]  = output[0][0] + m_initLeft[0];
    m_initRight[0] = output[1][0] + m_initRight[0];

    m_initLeft[1]  = output[0][1] + m_initLeft[1];
    m_initRight[1] = output[1][1] + m_initRight[1];

    m_initLeft[2]  = output[0][2] + m_initLeft[2];
    m_initRight[2] = output[1][2] + m_initRight[2];
}


void GameTrakController::reinit()
{
    if (!m_gameTrak)
    init();
}


void GameTrakController::bwdInit()
{
    if (!m_gameTrak)
    init();
}


void GameTrakController::reset()
{
    m_output.resize(2);

    m_initLeft[0]  = m_output[0][0] + m_initLeft[0];
    m_initRight[0] = m_output[1][0] + m_initRight[0];

    m_initLeft[1]  = m_output[0][1] + m_initLeft[1];
    m_initRight[1] = m_output[1][1] + m_initRight[1];

    m_initLeft[2]  = m_output[0][2] + m_initLeft[2];
    m_initRight[2] = m_output[1][2] + m_initRight[2];
}


void GameTrakController::onBeginAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);
    WriteAccessor<Data<vector<Vec3d>>> output = d_output;
    Vec3d offset = d_offset.getValue();
    for (unsigned int i=0; i<3; i++)
    {
        output[0][i] = m_output[0][i] + d_translation.getValue()[i] - m_initLeft[i] + offset[i];
        output[1][i] = m_output[1][i] + d_translation.getValue()[i] - m_initRight[i];
    }
}


void GameTrakController::onEndAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);
}

void GameTrakController::initAxis()
{
    OptionsGroup option(6,"x","-x","y","-y","z","-z");

    if(!d_x.isSet())
    {
        option.setSelectedItem(0);
        d_x.setValue(option);
    }

    if(!d_y.isSet())
    {
        option.setSelectedItem(2);
        d_y.setValue(option);
    }

    if(!d_z.isSet())
    {
        option.setSelectedItem(4);
        d_z.setValue(option);
    }
}


}//namespace controller
}//namespace component
}//namespace sofa

#endif // SOFA_CONTROLLER_GameTrak_CPP
