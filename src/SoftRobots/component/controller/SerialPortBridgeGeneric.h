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
#ifndef SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_H
#define SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_H

#include <SofaUserInteraction/Controller.h>

#include "modules/Serial.h"

#include <SoftRobots/initSoftRobots.h>

#if defined (_WIN32) || defined( _WIN64)
#define         DEVICE_PORT             "COM1"
#endif

#ifdef __linux__
#define         DEVICE_PORT             "/dev/ttyACM0"                         
#endif

namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;
using std::vector;
using std::string;

/**
 * This component is used to send data (force, displacement, pressure…) through the usb port.
 * Used to send data to an Arduino card to control the real robot.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
class SOFA_SOFTROBOTS_API SerialPortBridgeGeneric : public Controller
{
public:
    SOFA_CLASS(SerialPortBridgeGeneric, Controller);

    SerialPortBridgeGeneric();
    ~SerialPortBridgeGeneric() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    /////////////////////////////////////////////////////////////

    /////////////// Inherited from Controller ////////////////////
    void onBeginAnimationStep(const double dt) override;
    void onEndAnimationStep(const double dt) override;
    /////////////////////////////////////////////////////////////

    //Config
    Data<string>                 d_port;
    Data<unsigned int>           d_baudRate;
    Data<helper::vector<unsigned char>> d_packetOut;
    Data<helper::vector<unsigned char>> d_packetIn;
    // To remove before v20.0 of the plugin
    Data<helper::vector<double>>        d_packetOutDeprecated;
    Data<helper::vector<unsigned char>> d_packetInDeprecated;
    // ////////////////////////////////////
    Data<helper::vector<unsigned char>> d_header;
    Data<unsigned int>           d_size;
    Data<bool>                   d_precise;
    Data<bool>                   d_splitPacket;
    Data<unsigned int>           d_redundancy;
    Data<bool>                   d_doReceive;

protected:

    Serial m_serial;
    helper::vector<unsigned char> m_packetOut; //member variable to enable tests

    void checkConnection();
    void checkData();

    void receivePacket();
    void sendPacket();
    void sendPacketPrecise();

    // To remove before v20.0 of the plugin
    void dataDeprecationManagement();
    void updateLinkToDeprecatedData();
    // ////////////////////////////////////

};  //class SerialPortBridgeGeneric


}   //namespace controller

}   //namespace component

}   //namespace sofa

#endif
