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

#pragma once

#include <sofa/component/controller/Controller.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/Vec.h>

#include <SoftRobots/component/controller/modules/libgametrak/GameTrak.h>

#include <sofa/helper/OptionsGroup.h>
#include <SoftRobots/component/initSoftRobots.h>

namespace softrobots::controller
{

/**
 * Interface for GameTrak device.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
class GameTrakController : public sofa::component::controller::Controller
{
public:

    SOFA_CLASS(GameTrakController, sofa::component::controller::Controller);

    GameTrakController();
    ~GameTrakController() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Controller ////////////////////
    void onBeginAnimationStep(const double dt) override;
    void onEndAnimationStep(const double dt) override;
    /////////////////////////////////////////////////////////////////////////

    /// ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using sofa::component::controller::Controller::f_listening ;
    ////////////////////////////////////////////////////////////////////////////


protected:

    sofa::Data<sofa::type::Vec3d>                       d_translation;
    sofa::Data<sofa::type::Vec3d>                       d_offset;
    sofa::Data<double>                                   d_sizeCoeff;

    sofa::Data<sofa::helper::OptionsGroup>                     d_x;
    sofa::Data<sofa::helper::OptionsGroup>                     d_y;
    sofa::Data<sofa::helper::OptionsGroup>                     d_z;

    sofa::Data<std::string>                              d_deviceURI;
    sofa::Data<bool>                                     d_buttonPressed;
    sofa::Data<sofa::type::vector<sofa::type::Vec3d>> d_output;

    gametrak::GameTrak  *m_gameTrak;
    bool                 m_isButtonPressed;

    gametrak::TimeStamp::inttime m_lastTime;

    sofa::type::Vec3d m_initLeft;
    sofa::type::Vec3d m_initRight;
    sofa::type::vector<sofa::type::Vec3d> m_output;


    void GameTrakCallback(void *context,
                          gametrak::TimeStamp::inttime timeStamp,
                          double leftx, double lefty, double leftz,
                          double rightx, double righty, double rightz,
                          bool button);


    static void GameTrakCallbackStatic(void *context,
                                       gametrak::TimeStamp::inttime timeStamp,
                                       double leftx, double lefty, double leftz,
                                       double rightx, double righty, double rightz,
                                       bool button);

    void initAxis();

};   //class GameTrakController

}   //namespace

namespace sofa::component::controller
{
    using GameTrakController SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::controller::GameTrakController;
}
