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
#ifndef SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_CPP
#define SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_CPP

#include <SoftRobots/component/controller/SerialPortBridgeGeneric.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>


namespace softrobots::controller
{

using sofa::type::Vec;
using sofa::helper::WriteAccessor;
using sofa::core::objectmodel::ComponentState;

void registerSerialPortBridgeGeneric(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Send data (ex: force, displacement, pressure…) through the usb port. \n"
                                                              "Usually used to send data to an Arduino card to control the real robot.")
    .add< SerialPortBridgeGeneric >());
}

SerialPortBridgeGeneric::SerialPortBridgeGeneric()
    : d_port(initData(&d_port, "port", "Serial port name"))
    , d_baudRate(initData(&d_baudRate, "baudRate", "Transmission speed"))
    , d_packetOut(initData(&d_packetOut, "packetOut", "Data to send: vector of unsigned char, each entry should be an integer between 0 and header-1 <= 255.\n"
                           "The value of 'header' will be sent at the beginning of the sent data,\n"
                           "enabling to implement a header research in the 'receiving' code, for synchronization purposes.\n"
                           ))
    , d_packetIn(initData(&d_packetIn, "packetIn", "Data received: vector of unsigned char, each entry should be an integer between 0 and header-1 <= 255."))
    , d_header(initData(&d_header, sofa::type::vector<unsigned char>{255,254}, "header", "Vector of unsigned char. Only one value is espected, two values if splitPacket = 1."))
    , d_size(initData(&d_size,(unsigned int)0,"size","Size of the arrow to send. Use to check sentData size. \n"
                                            "Will return a warning if sentData size does not match this value."))
    , d_precise(initData(&d_precise,false,"precise","If true, will send the data in the format [header[0],[MSB,LSB]*2*size]"))
    , d_splitPacket(initData(&d_splitPacket,false,"splitPacket","If true, will split the packet in two for lower error rate (only in precise mode),\n"
                               "data will have the format [header[0],[MSB,LSB]*size],[header[1],[MSB,LSB]*size]"))
    , d_redundancy(initData(&d_redundancy,(unsigned int)1,"redundancy","Each packet will be send that number of times (1=default)"))
    , d_doReceive(initData(&d_doReceive,false,"receive","If true, will read from serial port (timeOut = 10ms)"))
{
}


SerialPortBridgeGeneric::~SerialPortBridgeGeneric()
{
}


void SerialPortBridgeGeneric::init()
{
    d_componentState.setValue(ComponentState::Valid);
    dataDeprecationManagement();
    checkConnection();

    if(d_splitPacket.getValue() && !d_precise.getValue())
    {
        msg_warning() << "Option splitPacket should only be used in precise mode. Set default value, false.";
        d_splitPacket.setValue(false);
    }


    if(d_splitPacket.getValue())
    {
        if(d_header.getValue().size()<2)
        {
            msg_warning() << "Problem with header size. Set default [255,254].";
            d_header.setValue(sofa::type::vector<unsigned char>{255,254});
        }

    }
    else if(d_header.getValue().size()==0)
    {
        msg_warning() << "Problem with header size. Set default 255.";
        d_header.setValue(sofa::type::vector<unsigned char>{255,254});
    }


    // Initial value sent to the robot
    //[header[0], 0 .. 0]
    if(d_precise.getValue())
    {
        if(d_splitPacket.getValue())
            m_packetOut.resize(d_size.getValue()*2+2);
        else
            m_packetOut.resize(d_size.getValue()*2+1);
    }
    else
        m_packetOut.resize(d_size.getValue()+1);

    m_packetOut[0] = d_header.getValue()[0];
    for (unsigned int i=1; i<m_packetOut.size(); i++)
        m_packetOut[i] = (unsigned char)0;
    if(d_splitPacket.getValue())
        m_packetOut[m_packetOut.size()/2] = d_header.getValue()[1];

    if (d_redundancy.getValue()<1)
    {
        msg_warning() <<"No valid number of packets set in Redundancy, set automatically to 1";
        d_redundancy.setValue(1);
    }
}

void SerialPortBridgeGeneric::dataDeprecationManagement()
{
    if(!d_header.isSet())
        msg_info() << "An old implementation was using 245 as the default header for sentData. This is not the case anymore. The default header is now equal to 255.";

    if(d_precise.getValue())
        msg_info() << "An old implementation was multiplying the values of sentData by 1000 when setting precise=true. This is not the case anymore.";
}

void SerialPortBridgeGeneric::checkConnection()
{
    char status = m_serial.Open(d_port.getValue().c_str(), d_baudRate.getValue());

    if (status!=1)
    {
        msg_warning() <<"No serial port found";
        d_componentState.setValue(ComponentState::Invalid);
    }
    else
        msg_info() <<"Serial port found";
}


void SerialPortBridgeGeneric::onBeginAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    if(d_doReceive.getValue())
        receivePacket();
}


void SerialPortBridgeGeneric::onEndAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    checkData();

    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    if(d_precise.getValue()) sendPacketPrecise();
    else                     sendPacket();
}


void SerialPortBridgeGeneric::sendPacketPrecise()
{
    //[header[0], [MSB,LSB]*size*2]
    unsigned int packetlength=d_size.getValue();
    m_packetOut.resize(packetlength*2+1);

    m_packetOut[0] = d_header.getValue()[0];
    for (unsigned int i=0; i<packetlength; i++)
    {
        unsigned int value = d_packetOut.getValue()[i];
        unsigned char LSB = value&0xFF;
        unsigned char MSB = value>>8&0xFF;
        m_packetOut[i*2+1]=MSB;
        m_packetOut[i*2+2]=LSB;
    }

    //[header[0], [MSB,LSB]*size], [header[1], [MSB,LSB]*size]
    if(d_splitPacket.getValue())
        m_packetOut.insert(m_packetOut.begin()+packetlength+1, d_header.getValue()[1]);

    // Vector to void*
    void* packetPtr = new unsigned char[m_packetOut.size() * sizeof(m_packetOut)];
    memcpy (packetPtr, m_packetOut.data(), m_packetOut.size() * sizeof(m_packetOut));

    m_serial.FlushReceiver();
    for(unsigned int i=0;i<d_redundancy.getValue(); i++)
        m_serial.Write(packetPtr, m_packetOut.size());
}


void SerialPortBridgeGeneric::sendPacket()
{
    //[header[0], d_packet]
    m_packetOut.resize(d_size.getValue()+1);

    m_packetOut[0] = d_header.getValue()[0];
    for (unsigned int i=0; i<d_size.getValue(); i++)
        m_packetOut[i+1] = d_packetOut.getValue()[i];

    // Vector to void*
    void* packetPtr = new unsigned char[m_packetOut.size() * sizeof(m_packetOut)];
    memcpy (packetPtr, m_packetOut.data(), m_packetOut.size() * sizeof(m_packetOut));

    m_serial.FlushReceiver();
    for(unsigned int i=0;i<d_redundancy.getValue(); i++)
        m_serial.Write(packetPtr, m_packetOut.size());
}


void SerialPortBridgeGeneric::receivePacket()
{
    WriteAccessor<sofa::Data<sofa::type::vector<unsigned char>>> packetIn = d_packetIn;

    unsigned int nbData = 1;
    packetIn.clear();
    packetIn.resize(nbData);

    unsigned int maxNbBytes = nbData * sizeof(unsigned char);
    unsigned char* packetPtr = new unsigned char[nbData];

    unsigned int timeOut_ms = 20;
    int status = m_serial.Read(packetPtr, maxNbBytes, timeOut_ms);
    if(status==1)
    {
        for(unsigned int i=0; i<nbData; i++)
            packetIn[i]=packetPtr[i];
    }

    delete[] packetPtr;
}


void SerialPortBridgeGeneric::checkData()
{
    if(!d_size.isSet())
        msg_warning() <<"Size not set.";

    if(d_packetOut.getValue().size()!=d_size.getValue())
    {
        msg_warning() <<"The user specified a size for packetOut, size="<<d_size.getValue()
                     <<" but packetOut.size="<<d_packetOut.getValue().size()<<"."
                    <<" To remove this warning you can either change the value of 'size' or 'packetOut'."
                   <<" Make sure the size and format of the data correspond to what the microcontroller in the robot is expecting."
                  << "The component will switch to an invalid state to avoid any issues.";
        d_componentState.setValue(ComponentState::Invalid);
    }
}

} // namespace

#endif //SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_CPP

