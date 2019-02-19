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
#include "InteractiveControl.h"

#include <sofa/helper/messaging/FileMessage.h>

namespace sofa
{
namespace component
{
namespace controller
{

using namespace sofa::defaulttype;
using namespace sofa::helper;


int InteractiveControlClass = core::RegisterObject("InteractiveControl (Controls external motors via network)")
        .add< InteractiveControl >();

InteractiveControl::InteractiveControl()
    : d_motorIndex(initData(&d_motorIndex, (unsigned int)0, "motorIndex", "index of controlled motor (0 to 255 max)"))
    , d_mode(initData(&d_mode, (unsigned int)0, "mode", "feedback control mode : 0 - STOP, 1 - VELOCITY(r/s), 2 - TORQUE/CURRENT (A or Nm), 3 - POSITION (degrees or revolutions)"))
    , d_setPoint(initData(&d_setPoint, "setpoint", "desired setpoint (unit depends on mode)"))
    , d_rotorDiameter(initData(&d_rotorDiameter, float(0.008), "rotorDiameter", "Input data : the diameter (m) of the actuator rotor (to compute setpoint from cable displacement)"))
    , d_manualSetpoint(initData(&d_manualSetpoint, float(0.0), "manual setpoint", "for prototyping purpose"))
    , d_address(initData(&d_address, std::string("169.254.8.220"), "address", "IP address for network usage"))
    , d_port(initData(&d_port, (unsigned int) 1994, "port", "network destination port"))
    , d_useNetwork(initData(&d_useNetwork, true, "useNetwork", "if true, use Network, else, use serial connection"))
    , d_printCableDisplacement(initData(&d_printCableDisplacement, false, "printCableDisplacement", "(CABLE SPECIFIC WILL NEED FUTURE CLEANING if true, displays cable displacement"))
{
    this->f_listening.setValue(true);
    m_timeEnd = 0;
    m_timeStart = 0;
    m_socket = new QUdpSocket();
}

InteractiveControl::~InteractiveControl()
{
    m_socket->abort();
}

void InteractiveControl::init()
{
    connect();
}

void InteractiveControl::reinit()
{
    connect();
}

void InteractiveControl::onBeginAnimationStep(const double /*dt*/ )
{
    send();
    processDataReceivedFromHardware();
}

void InteractiveControl::send()
{
    struct timeval tv;
    char buffer[BUFFER_MAX_SIZE];

    gettimeofday(&tv, nullptr);
    m_timeStart = tv.tv_sec;
    if((m_timeStart - m_timeEnd)>2)
    {
        dmsg_info("IneractiveControl") << " DESINHIBE";
    }

    // Send setpoint
    make_new_buffer(buffer, set_setpoint);
    set_field_time(buffer, 0);
    set_field_mode(buffer, d_mode.getValue());
    set_field_inhibition(buffer, 0);
    set_field_motor(buffer, d_motorIndex.getValue());
    set_field_address(buffer, 0x300);
    //temporarily set as defined current value
    set_field_setpoint(buffer, d_manualSetpoint.getValue());
    set_field_id(buffer, make_id(buffer));
    QByteArray data_setpoint(buffer, BUFFER_MAX_SIZE);

    if(d_printCableDisplacement.getValue())
    {
        std::cout << "Deplacement cable : " << d_setPoint.getValue()[0]*1000 << " mm" <<  std::endl;
    }

    try
    {
        m_socket->write(data_setpoint);
        /*        m_socket->write(data_setpoint2)*/;
    }
    catch(std::exception const &e)
    {
        serr << e.what() << sendl;
    }

    gettimeofday(&tv, nullptr);
    m_timeEnd = tv.tv_sec;
}

void InteractiveControl::connect()
{
    sout<<"Connexion @ "<<d_address.getValue().c_str()<<':'<<d_port.getValue()<<" [IN PROGRESS]"<<sendl;

    try
    {
        m_socket->abort();
        m_socket->connectToHost(d_address.getValue().c_str(), d_port.getValue());
    }
    catch(std::exception const& e)
    {
        msg_error("InteractiveControl") <<"Connexion @ "<<d_address.getValue().c_str()<<':'<<d_port.getValue()<<" [FAILED]";
        msg_error("InteractiveControl") << e.what() ;

        return;
    }

    msg_info("InteractiveControl")<<"Connexion @ "<<d_address.getValue().c_str()<<':'<<d_port.getValue()<<" [DONE]";
}

void InteractiveControl::processDataReceivedFromHardware()
{
    char buffer[BUFFER_MAX_SIZE];

    while(m_socket->readDatagram(buffer,BUFFER_MAX_SIZE) != -1)
    {
        char mode;
        mode=get_field_mode(buffer);
        float bob;
        Bytes2Float(&bob, (uint8*)&(buffer[10]));
    }
}

} // sofa::component::controller
} // sofa::component
} // sofa

