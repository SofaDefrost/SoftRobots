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

#ifndef SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_H
#define SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_H

#include <SofaUserInteraction/Controller.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vectorData.h>

#include <zmq.hpp>
#include <string>

#include <SoftRobots/initSoftRobots.h>

namespace sofa
{

namespace component
{

namespace controller
{


using core::objectmodel::Event;
using core::objectmodel::BaseObjectDescription;
using std::map;
using std::string;
using sofa::helper::vectorData;


/**
 * This component is used to build a communication between two simulations
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template< class DataTypes >
class SOFA_SOFTROBOTS_API CommunicationController : public Controller
{
public:

    typedef Controller Inherited;
    SOFA_CLASS(SOFA_TEMPLATE(CommunicationController,DataTypes), Inherited);

    CommunicationController();
    ~CommunicationController() override;

    //////////////////////////////// Inherited from Base /////////////////////////////////
    std::string getTemplateName() const override {return templateName(this);}
    static std::string templateName(const CommunicationController<DataTypes>* = NULL);
    /////////////////////////////////////////////////////////////////////////////////


    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /// Parse the given description to assign values to this object's fields and potentially other parameters
    void parse(BaseObjectDescription *arg) override;
    /// Assign the field values stored in the given map of name -> value pairs
    void parseFields(const map<string,string*>& str) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Controller ////////////////////
    void onBeginAnimationStep(const double dt) override;
    void onEndAnimationStep(const double dt) override;
    /////////////////////////////////////////////////////////////////////////

    Data<helper::OptionsGroup>              d_job;
    Data<helper::OptionsGroup>              d_pattern;
    Data<uint64_t>                          d_HWM;
    Data<string>                            d_port;
    Data<string>                            d_ipAdress;
    Data<bool>                              d_atBeginAnimationStep;
    Data<double>                            d_beginAt;
    Data<unsigned int>                      d_timeOut;
    double                                  m_time{0.};
    Data<unsigned int>                      d_nbDataField;
    vectorData<DataTypes>                   d_data;

protected:

    zmq::context_t     *m_context;
    zmq::socket_t      *m_socket;

    void sendData();
    void receiveData();
    void sendRequest();
    void receiveRequest();
    void closeCommunication();
    void openCommunication();

    // Factoring for templates
    void convertDataToMessage(string& messageStr);
    void convertStringStreamToData(std::stringstream *stream);

};  //class CommunicationController

}   //namespace controller
}   //namespace component
}   //namespace sofa

#endif // SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_H
