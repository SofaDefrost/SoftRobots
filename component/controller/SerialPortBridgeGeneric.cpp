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
#ifndef SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_CPP
#define SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_CPP

#include "SerialPortBridgeGeneric.h"
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>

#include <sofa/helper/logging/Messaging.h>


namespace sofa
{

namespace component
{

namespace controller
{

using defaulttype::Vec;
using sofa::helper::WriteAccessor;

SOFA_DECL_CLASS(SerialPortBridgeGeneric)

int SerialPortBridgeGenericClass = core::RegisterObject("Send data (ex: force, displacement, pressure…) through the usb port. \n"
                                                        "Usually used to send data to an Arduino card to control the real robot.")
        .add< SerialPortBridgeGeneric >()
        ;

SerialPortBridgeGeneric::SerialPortBridgeGeneric()
    : d_port(initData(&d_port, "port", "Serial port name"))
    , d_baudRate(initData(&d_baudRate, "baudRate", "Transmission speed"))
    , d_packetOut(initData(&d_packetOut, "sentData", "Data to send: the value 245 will be set at the beginning of the sent data as a header,\n"
                           "enabling to implement a header research in the 'receiving' code, for synchronization purposes."))
    , d_packetIn(initData(&d_packetIn, "receivedData", "Data received: vector of unsigned char, each entry should be an integer between 0 and 244."))
    , d_size(initData(&d_size,(int)0,"size","Size of the arrow to send. Use to check sentData size. \n"
                                            "Will return a warning if sentData size does not match this value."))
    , d_precise(initData(&d_precise,false,"precise","If true, will send the data in the format [245,[MSB,LSB]*2*size,246]"))
    , d_splitPacket(initData(&d_splitPacket,false,"splitPacket","If true, will split the packet in two for lower error rate (only in precise mode)"))
    , d_redundancy(initData(&d_redundancy,1,"redundancy","Each packet will be send that number of times (1=default)"))
    , d_doReceive(initData(&d_doReceive,false,"receive","If true, will read from serial port (timeOut = 10ms)"))
{
}


SerialPortBridgeGeneric::~SerialPortBridgeGeneric()
{
}


void SerialPortBridgeGeneric::init()
{
    checkConnection();

    // Initial value sent to the robot
    //[245, 0 .. 0]
    if(d_precise.getValue())  m_packetOut.resize(d_size.getValue()*2+1);
    else                      m_packetOut.resize(d_size.getValue()+1);

    m_packetOut[0] = (unsigned char)245;
    for (int i=1; i<(int)m_packetOut.size(); i++)
        m_packetOut[i] = (unsigned char)0;
    if (d_redundancy.getValue()<1)
    {
        msg_warning() <<"No valid number of packets set in Redundancy, set automatically to 1";
        d_redundancy.setValue(1);
    }
}


void SerialPortBridgeGeneric::checkConnection()
{
    int status = m_serial.Open(d_port.getValue().c_str() , d_baudRate.getValue());

    if (status!=1)
        msg_warning() <<"No serial port found";
    else
        msg_info() <<"Serial port found";
}


void SerialPortBridgeGeneric::reinit()
{
    m_serial.FlushReceiver();
    m_serial.Close();
    init();
}


void SerialPortBridgeGeneric::onBeginAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    if(d_doReceive.getValue())
        receivePacket();
}


void SerialPortBridgeGeneric::onEndAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    checkData();

    if(d_precise.getValue()) sendPacketPrecise();
    else                     sendPacket();
}


void SerialPortBridgeGeneric::sendPacketPrecise()
{
    //[245, [MSB,LSB]*size*2, 246]
    int packetlength=d_size.getValue();
    m_packetOut.resize(packetlength*2+1);

    m_packetOut[0] = (unsigned char)245;
    for (int i=0; i<packetlength; i++)
    {
        int value = d_packetOut.getValue()[i];
        unsigned char LSB = value&0xFF;
        unsigned char MSB = value>>8&0xFF;
        m_packetOut[i*2+1]=MSB;
        m_packetOut[i*2+2]=LSB;
    }

    if(d_splitPacket.getValue())
        m_packetOut.insert(m_packetOut.begin()+packetlength+1, 246);

    // Vector to void*
    void* packetPtr = new unsigned char[m_packetOut.size() * sizeof(m_packetOut)];
    memcpy (packetPtr, m_packetOut.data(), m_packetOut.size() * sizeof(m_packetOut));

    m_serial.FlushReceiver();
    for(int i=0;i<d_redundancy.getValue(); i++)
        m_serial.Write(packetPtr, m_packetOut.size());
}


void SerialPortBridgeGeneric::sendPacket()
{
    //[245, d_packet]
    m_packetOut.resize(d_size.getValue()+1);

    m_packetOut[0] = (unsigned char)245;
    for (int i=0; i<d_size.getValue(); i++)
        m_packetOut[i+1] = (unsigned char)d_packetOut.getValue()[i];

    // Vector to void*
    void* packetPtr = new unsigned char[m_packetOut.size() * sizeof(m_packetOut)];
    memcpy (packetPtr, m_packetOut.data(), m_packetOut.size() * sizeof(m_packetOut));

    m_serial.FlushReceiver();
    for(int i=0;i<d_redundancy.getValue(); i++)
        m_serial.Write(packetPtr, m_packetOut.size());
}


void SerialPortBridgeGeneric::receivePacket()
{
    WriteAccessor<Data<helper::vector<unsigned char>>> packetIn = d_packetIn;

    int nbData = 1;
    packetIn.clear();
    packetIn.resize(nbData);

    unsigned int maxNbBytes = nbData * sizeof(unsigned char);
    unsigned char* packetPtr = new unsigned char[nbData];

    unsigned int timeOut_ms = 20;
    int status = m_serial.Read(packetPtr, maxNbBytes, timeOut_ms);
    if(status==1)
    {
        for(int i=0; i<nbData; i++)
            packetIn[i]=packetPtr[i];
    }

    delete[] packetPtr;
}


void SerialPortBridgeGeneric::checkData()
{
    if(!d_size.isSet())
        msg_warning() <<"Size not set.";

    if((int)d_packetOut.getValue().size()!=d_size.getValue())
    {
        msg_warning() <<"The user specified a size for sentData, size="<<d_size.getValue()
                          <<" but sentData.size="<<d_packetOut.getValue().size()<<"."
                          <<" To remove this warning you can either change the value of 'size' or 'sentData'."
                          <<" Make sure it corresponds to what the arduino card is expecting.";
        d_size.setValue(d_packetOut.getValue().size());
    }
}


}//namespace controller
}//namespace component
}//namespace sofa

#endif //SOFAVRPNCLIENT_SERIALPORTBRIDGEGENERIC_CPP

