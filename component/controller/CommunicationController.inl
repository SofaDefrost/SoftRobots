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

#ifndef SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_INL
#define SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_INL

#include "CommunicationController.h"

#include <iostream>
#include <sstream>
#include <string>

namespace sofa
{

namespace component
{

namespace controller
{

using sofa::helper::OptionsGroup;
using std::memcpy;
using sofa::helper::vectorData;
using std::stringstream;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;
using std::string;

template<class DataTypes>
CommunicationController<DataTypes>::CommunicationController()
    : Inherited()
    , d_job(initData(&d_job, OptionsGroup(2,"sender","receiver"), "job",
                     "If unspecified, the default value is sender"))
    , d_pattern(initData(&d_pattern, OptionsGroup(2,"publish/subscribe","request/reply"), "pattern",
                         "Pattern used for communication. \n"
                         "publish/subscribe: Messages sent are distributed in a fan out fashion to all connected peers. Never blocks.\n"
                         "request/reply: Message sent are waiting for reply. Allows only an alternating sequence of send\reply calls.\n"
                         "Default is publish/subscribe. WARNING: the pattern should be the same for both sender and receiver to be effective."))
    , d_HWM(initData(&d_HWM, (uint64_t)0, "HWM", "If publisher, you can define the High Water Mark which is a hard limit on the maximum number of outstanding \n"
                                                 "messages shall queue in memory. Default 0 (means no limit)."))
    , d_port(initData(&d_port, string("5556"), "port", "Default value 5556."))
    , d_ipAdress(initData(&d_ipAdress, "ip", "IP adress of the sender. No given adress will set up a local communication."))
    , d_atBeginAnimationStep(initData(&d_atBeginAnimationStep, (bool)1, "atBeginAnimationStep", "If true, will send or receive datas at begin of the animation step (if false, at end of the animation step). Default true."))
    , d_beginAt(initData(&d_beginAt, (double)0, "beginAt", "Time step value to start the communication at."))
    , d_nbDataField(initData(&d_nbDataField, (unsigned int)1, "nbDataField",
                             "Number of field 'data' the user want to send or receive.\n"
                             "Default value is 1."))
    , d_data(this, "data", "Data to send or receive.")
{
    d_data.resize(d_nbDataField.getValue());
}


template<class DataTypes>
CommunicationController<DataTypes>::~CommunicationController()
{
    closeCommunication();
}


template<class DataTypes>
void CommunicationController<DataTypes>::init()
{
    // Should drop sent messages if exceed HWM.
    // WARNING: does not work
    if(d_job.getValue().getSelectedItem() == "receiver" && d_pattern.getValue().getSelectedItem() == "publish/subscribe")
        d_HWM.setDisplayed(false); // Put to true if problem solved
    else
        d_HWM.setDisplayed(false);

    if(d_job.getValue().getSelectedItem() == "sender")
        d_ipAdress.setDisplayed(false);

    d_data.resize(d_nbDataField.getValue());
    openCommunication();
}

template<class DataTypes>
void CommunicationController<DataTypes>::reinit()
{
    //closeCommunication();
    //init();
}

template<class DataTypes>
void CommunicationController<DataTypes>::reset()
{
    //reinit();
}


template<class DataTypes>
void CommunicationController<DataTypes>::openCommunication()
{
    if(d_job.getValue().getSelectedItem() == "sender")
    {
        string adress = "tcp://*:";
        string port = d_port.getValue();
        adress.insert(adress.length(), port);

        if(d_pattern.getValue().getSelectedItem() == "publish/subscribe")
            m_socket = new zmq::socket_t(m_context, ZMQ_PUB);
        else
            m_socket = new zmq::socket_t(m_context, ZMQ_REP);

        m_socket->bind(adress.c_str());
    }
    else
    {
        string IP ="localhost";
        if(d_ipAdress.isSet())
            IP = d_ipAdress.getValue();

        string adress = "tcp://"+IP+":";
        string port = d_port.getValue();
        adress.insert(adress.length(), port);

        if(d_pattern.getValue().getSelectedItem() == "publish/subscribe")
        {
            m_socket = new zmq::socket_t(m_context, ZMQ_SUB);

            // Should drop sent messages if exceed HWM.
            // WARNING: does not work, uncomment if problem solved
            //uint64_t HWM = d_HWM.getValue();
            //m_socket->setsockopt(ZMQ_RCVHWM, &HWM, sizeof(HWM));

            m_socket->connect(adress.c_str());
            m_socket->setsockopt(ZMQ_SUBSCRIBE, "", 0); // Arg2: publisher name - Arg3: size of publisher name
        }
        else
        {
            m_socket = new zmq::socket_t(m_context, ZMQ_REQ);
            m_socket->connect(adress.c_str());
        }
    }
}


template<class DataTypes>
void CommunicationController<DataTypes>::closeCommunication()
{
    m_socket->close();
    delete m_socket;
}


template<class DataTypes>
void CommunicationController<DataTypes>::parse(BaseObjectDescription* arg)
{
    d_data.parseSizeData(arg, d_nbDataField);
    Inherit1::parse(arg);
}


template<class DataTypes>
void CommunicationController<DataTypes>::parseFields(const map<string,string*>& str)
{
    d_data.parseFieldsSizeData(str, d_nbDataField);
    Inherit1::parseFields(str);
}


template<class DataTypes>
void CommunicationController<DataTypes>::onBeginAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    if(d_beginAt.getValue()>m_time)
        return;

    if(d_atBeginAnimationStep.getValue())
    {
        if(d_job.getValue().getSelectedItem() == "sender")
            sendData();

        if(d_job.getValue().getSelectedItem() == "receiver")
            receiveData();
    }
}


template<class DataTypes>
void CommunicationController<DataTypes>::onEndAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    if(d_beginAt.getValue()>m_time)
    {
        m_time+=dt;
        return;
    }

    if(!d_atBeginAnimationStep.getValue())
    {
        if(d_job.getValue().getSelectedItem() == "sender")
            sendData();

        if(d_job.getValue().getSelectedItem() == "receiver")
            receiveData();
    }
}


template<class DataTypes>
void CommunicationController<DataTypes>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<DataTypes>> data = d_data[i];
        messageStr += std::to_string(data) + " ";
    }
}


template<class DataTypes>
void CommunicationController<DataTypes>::convertStringStreamToData(stringstream* stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<DataTypes>> data = d_data[i];
        (*stream) >> data;
    }
}


template<class DataTypes>
void CommunicationController<DataTypes>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(nbDataFieldReceived!=d_nbDataField.getValue())
        msg_warning() << "Something wrong with the size of data received. Please check template.";
}


template<class DataTypes>
void CommunicationController<DataTypes>::sendData()
{
    if(d_pattern.getValue().getSelectedItem() == "request/reply")
        receiveRequest();

    string messageStr;
    convertDataToMessage(messageStr);

    zmq::message_t message(messageStr.length());

    memcpy(message.data(), messageStr.c_str(), messageStr.length());

    bool status = m_socket->send(message);
    if(!status)
        msg_warning() << "Problem with communication";
}


template<class DataTypes>
void CommunicationController<DataTypes>::receiveData()
{
    if(d_pattern.getValue().getSelectedItem() == "request/reply")
        sendRequest();

    zmq::message_t message;
    bool status = m_socket->recv(&message);
    if(status)
    {
        char messageChar[message.size()];
        memcpy(&messageChar, message.data(), message.size());

        stringstream stream;
        unsigned int nbDataFieldReceived = 0;
        for(unsigned int i=0; i<message.size(); i++)
        {
            if(messageChar[i]==' ' || i==message.size()-1)
                nbDataFieldReceived++;

            if(messageChar[i]==',')
                messageChar[i]='.';

            stream << messageChar[i];
        }

        convertStringStreamToData(&stream);
        checkDataSize(nbDataFieldReceived);
    }
    else
        msg_warning() << "Problem with communication";
}


template<class DataTypes>
void CommunicationController<DataTypes>::sendRequest()
{
    zmq::message_t request;
    m_socket->send(request);
}

template<class DataTypes>
void CommunicationController<DataTypes>::receiveRequest()
{
    zmq::message_t request;
    m_socket->recv(&request);
}


}   //namespace controller
}   //namespace component
}   //namespace sofa


#endif // SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_INL
