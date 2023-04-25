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
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/objectmodel/vectorData.h>

#include <zmq.hpp>
#include <string>

#include <SoftRobots/component/initSoftRobots.h>

namespace softrobots::controller
{

using sofa::core::objectmodel::Event;
using sofa::core::objectmodel::BaseObjectDescription;
using std::map;
using std::string;
using sofa::core::objectmodel::vectorData;


/**
 * This component is used to build a communication between two simulations
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template< class DataTypes >
class SOFA_SOFTROBOTS_API CommunicationController : public sofa::component::controller::Controller
{
public:

    typedef Controller Inherited;
    SOFA_CLASS(SOFA_TEMPLATE(CommunicationController,DataTypes), Inherited);

    CommunicationController();
    ~CommunicationController() override;

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

    /// Implementing the GetCustomTemplateName is mandatory to have a custom template name paremters
    /// instead of the default one generated automatically by the SOFA_CLASS() macro.
    static std::string GetCustomTemplateName();

    sofa::Data<sofa::helper::OptionsGroup>              d_job;
    sofa::Data<sofa::helper::OptionsGroup>              d_pattern;
    sofa::Data<uint64_t>                          d_HWM;
    sofa::Data<string>                            d_port;
    sofa::Data<string>                            d_ipAdress;
    sofa::Data<bool>                              d_atBeginAnimationStep;
    sofa::Data<double>                            d_beginAt;
    sofa::Data<unsigned int>                      d_timeOut;
    double                                  m_time{0.};
    sofa::Data<unsigned int>                      d_nbDataField;
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

}   //namespace

namespace sofa::component::controller
{
    template <class DataTypes>
    using CommunicationController SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::controller::CommunicationController<DataTypes>;
}

