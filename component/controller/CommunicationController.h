#ifndef SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_H
#define SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_H

#include <SofaUserInteraction/Controller.h>

#include <zmq.hpp>
#include <string>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vectorData.h>

#include "../initSoftRobots.h"

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
 * https://project.inria.fr/softrobot/documentation/controller/CommunicationController
 */
template< class DataTypes >
class SOFA_SOFTROBOTS_API CommunicationController : public Controller
{
public:

    typedef Controller Inherited;
    SOFA_CLASS(SOFA_TEMPLATE(CommunicationController,DataTypes), Inherited);

    CommunicationController    ();
    virtual ~CommunicationController();

    //////////////////////////////// Inherited from Base /////////////////////////////////
    virtual std::string getTemplateName() const {return templateName(this);}
    static std::string templateName(const CommunicationController<DataTypes>* = NULL);
    /////////////////////////////////////////////////////////////////////////////////


    ////////////////////////// Inherited from BaseObject ////////////////////
    virtual void init() override;
    virtual void reinit() override;
    virtual void reset() override;
    /// Parse the given description to assign values to this object's fields and potentially other parameters
    virtual void parse(BaseObjectDescription *arg) override;
    /// Assign the field values stored in the given map of name -> value pairs
    virtual void parseFields(const map<string,string*>& str) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Controller ////////////////////
    virtual void onBeginAnimationStep(const double dt) override;
    virtual void onEndAnimationStep(const double dt) override;
    /////////////////////////////////////////////////////////////////////////

    Data<helper::OptionsGroup>              d_job;
    Data<helper::OptionsGroup>              d_pattern;
    Data<uint64_t>                          d_HWM;
    Data<string>                            d_port;
    Data<string>                            d_ipAdress;
    Data<bool>                              d_atBeginAnimationStep;
    Data<double>                            d_beginAt;
    double                                  m_time{0.};
    Data<unsigned int>                      d_nbDataField;
    vectorData<DataTypes>                   d_data;

protected:

    zmq::context_t     m_context{1};
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
    void checkDataSize(const unsigned int& nbDataFieldReceived);

};  //class CommunicationController

}   //namespace controller
}   //namespace component
}   //namespace sofa

#endif // SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_H
