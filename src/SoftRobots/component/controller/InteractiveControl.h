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

#include <sofa/core/ObjectFactory.h>
#include <sofa/component/controller/Controller.h>

#ifdef WIN32
    #include <time.h>
    #include <sys/timeb.h>
	#include <WinSock2.h>
	#include <Windows.h>
#else
    #include <sys/time.h>
#endif

#include <QObject>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    #include <QtNetwork/QUdpSocket>
#else
    #include <QUdpSocket>
#endif // SOFA_QT5

#include <math.h>

#include <SoftRobots/component/controller/modules/Network.h>
#include <SoftRobots/component/initSoftRobots.h>

namespace softrobots::controller
{

using namespace sofa::defaulttype;

typedef struct
{
    unsigned char index_motor;
    unsigned char mode;
    float setpoint;
}Packet;

class InteractiveControl : public sofa::component::controller::Controller
{

public:
    SOFA_CLASS(InteractiveControl , sofa::component::controller::Controller);

protected:

    sofa::Data<unsigned int> d_motorIndex;
    sofa::Data<unsigned int> d_mode;
    sofa::Data<sofa::type::Vec1f> d_setPoint;
    sofa::Data<float> d_rotorDiameter;
    sofa::Data<float> d_manualSetpoint;
    sofa::Data<std::string> d_address;
    sofa::Data<unsigned int> d_port;
    // Choose if interactive control is made via network or serial
    // network have priority
    sofa::Data<bool> d_useNetwork;
    sofa::Data<bool> d_printCableDisplacement;

    QUdpSocket * m_socket;

    long int m_timeEnd, m_timeStart;

public:

    InteractiveControl();
    ~InteractiveControl() override;

public:

    void init() override;
    void reinit() override;
    void onBeginAnimationStep(const double dt) override;

protected:

    void send();
    void connect();
    void processDataReceivedFromHardware();
};

} // namespace

namespace sofa::component::controller
{
    using InteractiveControl SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::controller::InteractiveControl;
}

