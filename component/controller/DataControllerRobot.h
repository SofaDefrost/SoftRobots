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

#ifndef SOFA_COMPONENT_CONTROLLER_DataControllerRobot_H
#define SOFA_COMPONENT_CONTROLLER_DataControllerRobot_H
//#include "initPlugin.h"

#include <SofaUserInteraction/Controller.h>
#include "../solver/QPInverseProblemSolver.h"
#include "../constraint/SurfacePressureActuator.h"
#include <sofa/defaulttype/Vec.h>



// ROBOTINO  /////////////

#include "rec/robotino/api2/all.h"
using namespace rec::robotino::api2;

float _rect[2]= {0,0};

bool _run = true;


class MyCom : public Com
{
public:
    MyCom()
        : Com( "rect" )
    {
    }

    void errorEvent( const char* errorString )
    {
        std::cerr << "Error: " << errorString << std::endl;
    }

    void connectedEvent()
    {
        std::cout << "Connected." << std::endl;
    }

    void connectionClosedEvent()
    {
        std::cout << "Connection closed." << std::endl;
    }

    void logEvent( const char* message, int level )
    {
        SOFA_UNUSED(level);
        std::cout << message << std::endl;
    }

    void pingEvent( float timeMs )
    {
        SOFA_UNUSED(timeMs);
        //std::cout << "Ping: " << timeMs << "ms" << std::endl;
    }
};


class MyBumper : public Bumper
{
public:
    MyBumper()
        : bumped( false )
    {
    }

    void bumperEvent( bool hasContact )
    {
        bumped |= hasContact;
        std::cout << "Bumper has " << ( hasContact ? "contact" : "no contact") << std::endl;
    }

    bool bumped;
};


MyCom com;
OmniDrive omniDrive;
MyBumper bumper;
CompactBHA cbha;




// Controller  /////////////


namespace sofa
{

namespace component
{

namespace controller
{
	
class DataControllerRobot : public sofa::component::controller::Controller
{
public:
    SOFA_CLASS(DataControllerRobot,sofa::component::controller::Controller);

    // needs to be changed to QPInverseProblemSolver
//    sofa::helper::vector<sofa::component::constraintset::SurfacePressureActuator<sofa::defaulttype::Vec3dTypes>*> listSPC;
    sofa::component::constraintset::QPInverseProblemSolver* m_QPSolver;

protected:

    DataControllerRobot();

    ~DataControllerRobot();

public:

	void init();

    void onEndAnimationStep(const double dt); // better do it at the end of the animation step

    void drive(const helper::vector<double>&);

    Data<double> coefPressure;

};


} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_DataControllerRobot_H
