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

#include <sofa/component/controller/Controller.h>

#include <SoftRobots/component/controller/modules/Serial.h>
#include <SoftRobots/component/initSoftRobots.h>

#if defined (_WIN32) || defined( _WIN64)
#define         DEVICE_PORT             "COM1"
#endif

#ifdef __linux__
#define         DEVICE_PORT             "/dev/ttyACM0"                         
#endif

namespace softrobots::controller
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
class SOFA_SOFTROBOTS_API SerialPortBridgeGeneric : public sofa::component::controller::Controller
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
    sofa::Data<string>                 d_port;
    sofa::Data<unsigned int>           d_baudRate;
    sofa::Data<sofa::type::vector<unsigned char>> d_packetOut;
    sofa::Data<sofa::type::vector<unsigned char>> d_packetIn;

    sofa::Data<sofa::type::vector<unsigned char>> d_header;
    sofa::Data<unsigned int>           d_size;
    sofa::Data<bool>                   d_precise;
    sofa::Data<bool>                   d_splitPacket;
    sofa::Data<unsigned int>           d_redundancy;
    sofa::Data<bool>                   d_doReceive;

protected:

    Serial m_serial;
    sofa::type::vector<unsigned char> m_packetOut; //member variable to enable tests

    void checkConnection();
    void checkData();

    void receivePacket();
    void sendPacket();
    void sendPacketPrecise();

    void dataDeprecationManagement();

};

}   // namespace

namespace sofa::component::controller
{
    using SerialPortBridgeGeneric SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::controller::SerialPortBridgeGeneric;
}

