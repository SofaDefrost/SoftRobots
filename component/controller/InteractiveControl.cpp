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

namespace sofa
{
namespace component
{
namespace controller
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

SOFA_DECL_CLASS(InteractiveControl)

int InteractiveControlClass = core::RegisterObject("InteractiveControl (Controls external motors via network)")
        .add< InteractiveControl >();

// // Ce slot est appelé lorsque la connexion au serveur a réussi
// void NetworkHandler::connecte()
// {
//     std::cout << "Connexion réussie !" << std::endl;
// }

// // Ce slot est appelé lorsqu'on est déconnecté du serveur
// void NetworkHandler::deconnecte()
// {
//     std::cout << "Déconnecté du serveur" << std::endl;
// }

// // Ce slot est appelé lorsqu'il y a une erreur
// void NetworkHandler::erreurSocket(QAbstractSocket::SocketError erreur)
// {
//     switch(erreur) // On affiche un message différent selon l'erreur qu'on nous indique
//     {
//         case QAbstractSocket::HostNotFoundError:
//             std::cerr << "ERREUR : le serveur n'a pas pu être trouvé. Vérifiez l'IP et le port." << std::endl;
//             break;
//         case QAbstractSocket::ConnectionRefusedError:
//             std::cerr << "ERREUR : le serveur a refusé la connexion. Vérifiez si le programme \"serveur\" a bien été lancé. Vérifiez aussi l'IP et le port." << std::endl;
//             break;
//         case QAbstractSocket::RemoteHostClosedError:
//             std::cerr << "ERREUR : le serveur a coupé la connexion." << std::endl;
//             break;
//         default:
//             std::cerr << "ERREUR : " << (std::string)_socket->errorString() << std::endl;
//     }
// }

// NetworkHandler::NetworkHandler()
// {

// }

// NetworkHandler::~NetworkHandler()
// {
//     _socket->abort();
// }

// void NetworkHandler::init()
// {
//     std::cout << "Tentative de connexion en cours..." << std::endl;;

//     _socket->abort();
//     _socket->connectToHost("127.0.0.1", 1994);
// }

// void NetworkHandler::send(char* pack)
// {
//     // Sending packet
//     _socket->write(pack);
// }

InteractiveControl::InteractiveControl()
    : d_motorIndex(initData(&d_motorIndex, (unsigned int)0, "motorIndex", "index of controlled motor (0 to 255 max)"))
    , d_mode(initData(&d_mode, (unsigned int)0, "mode", "feedback control mode : 0 - STOP, 1 - VELOCITY(r/s), 2 - TORQUE/CURRENT (A or Nm), 3 - POSITION (degrees or revolutions)"))
    , d_setPoint(initData(&d_setPoint, "setpoint", "desired setpoint (unit depends on mode)"))
    , d_rotorDiameter(initData(&d_rotorDiameter, (float)0.008, "rotorDiameter", "Input data : the diameter (m) of the actuator rotor (to compute setpoint from cable displacement)"))
    , d_manualSetpoint(initData(&d_manualSetpoint, (float)0.000, "manual setpoint", "for prototyping purpose"))
    , d_address(initData(&d_address, (std::string)"169.254.8.220", "address", "IP address for network usage"))
    , d_port(initData(&d_port, (unsigned int) 1994, "port", "network destination port"))
    , d_useNetwork(initData(&d_useNetwork, (bool)true, "useNetwork", "if true, use Network, else, use serial connection"))
    , d_printCableDisplacement(initData(&d_printCableDisplacement, (bool)false, "printCableDisplacement", "(CABLE SPECIFIC WILL NEED FUTURE CLEANING if true, displays cable displacement"))
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

    gettimeofday(&tv, NULL);
    m_timeStart = tv.tv_sec;
    if((m_timeStart - m_timeEnd)>2)
    {
        std::cout << " DESINHIBE" <<  std::endl;

//        // Send mode and inhibition
//        make_new_buffer(buffer, set_mode_inhibition);
//        set_field_time(buffer, 0);
//        set_field_mode(buffer, 4);
//        //temporarily set as current mode
//        //set_field_mode(buffer, 2);
//        set_field_inhibition(buffer, 0);
//        // set_field_motor(buffer, motorIndex.getValue());
//        set_field_motor(buffer, 0);
//        set_field_address(buffer, 0x300);
//        //set_field_address(buffer, d_motorIndex.getValue());
//        set_field_id(buffer, make_id(buffer));
//        QByteArray data_mode_inhibition(buffer, BUFFER_MAX_SIZE);

        //    //FOR MOTOR 2
        //    // Send mode and inhibition
        //    make_new_buffer(buffer, set_mode_inhibition);
        //    set_field_time(buffer, 0);
        //    set_field_mode(buffer, 4);
        //    //temporarily set as current mode
        //    set_field_mode(buffer, 2);
        //    set_field_inhibition(buffer, 0);
        //    // set_field_motor(buffer, motorIndex.getValue());
        //    set_field_motor(buffer, 1);
        //    set_field_address(buffer, 0x300);
        //    //set_field_address(buffer, d_motorIndex.getValue());
        //    set_field_id(buffer, make_id(buffer));
        //    QByteArray data_mode_inhibition2(buffer, BUFFER_MAX_SIZE);

        //m_socket->write(data_mode_inhibition);
        //m_socket->write(data_mode_inhibition2);

    }

    // Send setpoint
    make_new_buffer(buffer, set_setpoint);
    set_field_time(buffer, 0);
    set_field_mode(buffer, d_mode.getValue());
    set_field_inhibition(buffer, 0);
    set_field_motor(buffer, d_motorIndex.getValue());
    set_field_address(buffer, 0x300);
    //set_field_setpoint(buffer, d_setPoint.getValue()[0]/(d_rotorDiameter.getValue()*M_PI));
    //temporarily set as defined current value
    set_field_setpoint(buffer, d_manualSetpoint.getValue());
    set_field_id(buffer, make_id(buffer));
    QByteArray data_setpoint(buffer, BUFFER_MAX_SIZE);

    //    //FOR MOTOR 2
    //    // Send setpoint
    //    make_new_buffer(buffer, set_setpoint);
    //    set_field_time(buffer, 0);
    //    set_field_mode(buffer, d_mode.getValue());
    //    // set_field_motor(buffer, motorIndex.getValue());
    //    set_field_motor(buffer, 1);
    //    set_field_address(buffer, 0x300);
    //    //set_field_setpoint(buffer, d_setPoint.getValue()[1]/(d_rotorDiameter.getValue()*M_PI));
    //    //temporarily set as defined current value
    //    set_field_setpoint(buffer, 0.5);
    //    set_field_id(buffer, make_id(buffer));
    //    QByteArray data_setpoint2(buffer, BUFFER_MAX_SIZE);
    //    // sout << "Call Send Method!" << sendl;

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

    //     Packet pack;
    //     pack.index_motor = (unsigned char)motorIndex.getValue();
    //     pack.mode = (unsigned char)mode.getValue();
    //     pack.setpoint = setpoint.getValue();

    //     QByteArray data((char *)&pack, sizeof(pack));

    //     // sout << "Call Send Method!" << sendl;

    //     // Sending packet
    //     try
    //     {
    //         _socket->write(data);
    //     }
    //     catch(std::exception const &e)
    //     {
    //         serr << e.what() << sendl;
    //     }

    gettimeofday(&tv, NULL);
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
        serr<<"Connexion @ "<<d_address.getValue().c_str()<<':'<<d_port.getValue()<<" [FAILED]"<<sendl;
        serr << e.what() << sendl;

        return;
    }

    sout<<"Connexion @ "<<d_address.getValue().c_str()<<':'<<d_port.getValue()<<" [DONE]"<<sendl;

//    char buffer[BUFFER_MAX_SIZE];
//    make_new_buffer(buffer, set_mode_inhibition);
//    set_field_time(buffer, 0);
//    set_field_mode(buffer, 4);
//    set_field_inhibition(buffer, 0);
//    set_field_motor(buffer, 0);
//    set_field_address(buffer, 0x300);
//    set_field_id(buffer, make_id(buffer));
//    QByteArray data_mode_init(buffer, BUFFER_MAX_SIZE);
//    m_socket->write(data_mode_init);
}

void InteractiveControl::processDataReceivedFromHardware()
{
    //change size to 4 !
    //char littleData[10];

    char buffer[BUFFER_MAX_SIZE];

    while(m_socket->readDatagram(buffer,BUFFER_MAX_SIZE) != -1)
    {
        char mode;
        mode=get_field_mode(buffer);
        float bob;
        Bytes2Float(&bob, (uint8*)&(buffer[10]));
        //feedback is : current in position mode & position in current mode
        std::cout << bob << ((mode==3)?" A":(mode==2)?" rounds": " " ) << std::endl;
    }
}

} // sofa::component::controller
} // sofa::component
} // sofa

