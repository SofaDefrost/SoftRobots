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
#ifndef SOFA_COMPONENT_INTERACTIVE_CONTROL_H
#define SOFA_COMPONENT_INTERACTIVE_CONTROL_H

#include <sofa/core/ObjectFactory.h>
#include <SofaUserInteraction/Controller.h>

#ifdef WIN32
    #include <time.h>
    #include <sys/timeb.h>
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

#include "modules/Network.h"
#include <math.h>

namespace sofa
{
namespace component
{
namespace controller
{

using namespace sofa::defaulttype;

typedef struct
{
    unsigned char index_motor;
    unsigned char mode;
    float setpoint;
}Packet;

class InteractiveControl : public Controller
{

public:
    SOFA_CLASS(InteractiveControl , Controller);

protected:

    Data<unsigned int> d_motorIndex;
    Data<unsigned int> d_mode;
    Data<Vec1f> d_setPoint;
    Data<float> d_rotorDiameter;
    Data<float> d_manualSetpoint;
    Data<std::string> d_address;
    Data<unsigned int> d_port;
    // Choose if interactive control is made via network or serial
    // network have priority
    Data<bool> d_useNetwork;
    Data<bool> d_printCableDisplacement;

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

} // sofa::component::controller
} // sofa::component
} // sofa

#endif // SOFA_COMPONENT_INTERACTIVE_CONTROL_H
