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

#define SOFA_COMPONENT_CONTROLLER_DataControllerRobot_CPP
#include "DataControllerRobot.h"

#include <sofa/core/ObjectFactory.h>


namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(DataControllerRobot)

// Register in the Factory
int DataControllerRobotClass = core::RegisterObject(" ")
.add< DataControllerRobot >()
;


DataControllerRobot::DataControllerRobot():
    coefPressure(initData(&coefPressure, 1.0 ,"coefPressure", "coef between sofa and robotino")),
    m_QPSolver(NULL)
{
}

DataControllerRobot::~DataControllerRobot()
{
}

void DataControllerRobot::init()
{
    this->getContext()->get(m_QPSolver, core::objectmodel::BaseContext::SearchRoot);

    // Initialize the actors

    // Connect
    std::cout << "Connecting... ";
    std::string hostname = "172.26.1.1";
    com.setAddress( hostname.c_str() );

    com.connectToServer( true );

    if( false == com.isConnected() )
    {
        std::cout << std::endl << "Could not connect to " << com.address() << std::endl;

        rec::robotino::api2::shutdown();
        exit( 1 );
    }
    else
    {
        std::cout << "success" << std::endl;

    }

}

void DataControllerRobot::onEndAnimationStep(const double /*dt*/)
{
    std::cout<<"DataControllerRobot::onBeginAnimationStep()"<<std::endl;

//    helper::vector<double> pressures;
//    pressures.resize(listSPC.size(),0.);
//    for (int i=0; i<listSPC.size(); i++)
//        listSPC[i]->getActualPressure(pressures[i]);
    helper::vector<double> pressures = m_QPSolver->d_computedForces.getValue();

    if( pressures.size() < 6 )
        return;

    drive(m_QPSolver->d_computedForces.getValue());
}

void DataControllerRobot::drive(const helper::vector<double>& pressures)
{
    //double pressures[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
   // double pots[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //double lastTime=com.msecsElapsed();

     std::cout << "pressures size " << pressures.size() << std::endl;

    if( com.isConnected() && false == bumper.value() && _run )
    {    
        const double maxPressure = 1.5;

        for(int i = 0 ; i < pressures.size() ; ++i)
        {
            std::cout << pressures[i] << std::endl;
        }
        // update the pressure values (sofa)

        //std::cout << "please select pressure values" << std::endl;

        std::cout << "STEP 0" << std::endl;
        pressuresValuesSofa[0]=coefPressure.getValue()*pressures[0+3]*maxPressure/200. ;
        pressuresValuesSofa[1]=coefPressure.getValue()*pressures[1+3]*maxPressure/200. ;/*test here*/
        pressuresValuesSofa[2]=coefPressure.getValue()*pressures[2+3]*maxPressure/200. ;
        pressuresValuesSofa[3]=coefPressure.getValue()*pressures[3+3]*maxPressure/200. ;
        pressuresValuesSofa[4]=coefPressure.getValue()*pressures[4+3]*maxPressure/200. ;
        pressuresValuesSofa[5]=coefPressure.getValue()*pressures[5+3]*maxPressure/200. ;
        pressuresValuesSofa[6]=0.0 ;
        pressuresValuesSofa[7]=0.0 ;

        std::cout << "STEP 1" << std::endl;

        for( unsigned int i = 0; i < 6; ++i )
        {
            if (pressuresValuesSofa[i]>maxPressure)
            {
                pressuresValuesSofa[i]=maxPressure;
            }
        }


        cbha.setCompressorsEnabled( true );
        cbha.setPressures( pressuresValuesSofa );
        //volt to meter for RobotinoXT
        float valuesMeasured[6];
        float valuesPressures[8];

        double sNom= 1.52;   //the max voltage[v]
        double mNom= 500; //maximal length of the wire-potentiometer[mm]
        double lNom[6]= { 109.9, 100.63, 104.0, 198.1, 195.9, 192.3}; //the nominal length corresponding to vnominal [mm]
        double vNom[6];
        double vMes[6]={ 0.625611, 0.630723, 0.537634, 0.806452, 0.835777, 0.884653};
        double dv[6];
        double lengthValue[6];

        std::cout << "STEP 2" << std::endl;

        // synchronize and delay
        //lastTime=com.msecsElapsed();
        com.processEvents();
        //rec::robotino::api2::msleep( 10000 );
        std::cout <<"convert"<< std::endl;
        cbha.stringPots( valuesMeasured );
        cbha.pressures(valuesPressures);
        for( unsigned int i = 0; i < 6; ++i )
        {
            vNom[i]=(sNom*lNom[i])/(mNom);
            dv[i]=-vNom[i]+vMes[i];
            lengthValue[i]= ((valuesMeasured[i]-dv[i])*mNom)/sNom;
            
            std::cout << "voltages measured " << i + 1 << ": " << valuesMeasured[i] << std::endl;
            std::cout << "String pot " << i + 1 << ": " << lengthValue[i] << std::endl;
        }


        for( unsigned int i = 0; i < 8; ++i )
        {
            std::cout << "SOFA pressure B" << i << ": " << pressuresValuesSofa[i] << " bar" << std::endl;
            std::cout << "Pressure sensor: " << i << ": "<< valuesPressures[i] << " bar"  << std::endl;
        }

        std::cout << std::endl;

    }

}


} // namespace controller

} // namespace component

} // namespace sofa
