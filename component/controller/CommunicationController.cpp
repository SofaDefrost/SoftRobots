#ifndef SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP
#define SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP

#include "CommunicationController.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace controller
{

using sofa::defaulttype::Vec3d;
using sofa::defaulttype::Vec3f;
using sofa::defaulttype::Vec1d;
using sofa::defaulttype::Vec1f;
using sofa::defaulttype::Vec;
using helper::vector;

using sofa::defaulttype::Rigid3dTypes;
using sofa::defaulttype::Rigid3fTypes;


///// VEC3D


template<>
void CommunicationController<vector<Vec3d>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Vec3d>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<3; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Vec3d>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Vec3d>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<3; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Vec3d>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Vec3d>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*4*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}


///// VEC3F


template<>
void CommunicationController<vector<Vec3f>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Vec3f>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<3; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Vec3f>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Vec3f>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<3; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Vec3f>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Vec3f>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*4*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}



///// VEC1D


template<>
void CommunicationController<vector<Vec1d>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Vec1d>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<1; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Vec1d>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Vec1d>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<1; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Vec1d>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Vec1d>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*2*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}



///// VEC1F


template<>
void CommunicationController<vector<Vec1f>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Vec1f>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<1; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Vec1f>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Vec1f>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<1; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Vec1f>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Vec1f>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*2*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}



///// VEC1I


template<>
void CommunicationController<vector<Vec<2,int>>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Vec<2,int>>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<2; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Vec<2,int>>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Vec<2,int>>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<2; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Vec<2,int>>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Vec<2,int>>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*3*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}




///// VEC1UI


template<>
void CommunicationController<vector<Vec<2,unsigned int>>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Vec<2,unsigned int>>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<2; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Vec<2,unsigned int>>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Vec<2,unsigned int>>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<2; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Vec<2,unsigned int>>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Vec<2,unsigned int>>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*3*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}




//// RIGID3D

template<>
void CommunicationController<vector<Rigid3dTypes::Coord>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Rigid3dTypes::Coord>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<7; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Rigid3dTypes::Coord>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Rigid3dTypes::Coord>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<7; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Rigid3dTypes::Coord>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Rigid3dTypes::Coord>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*8*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}


//// RIGID3F

template<>
void CommunicationController<vector<Rigid3fTypes::Coord>>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<vector<Rigid3fTypes::Coord>>> data = d_data[i];
        messageStr += std::to_string(data.size()) + " ";
        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<7; k++)
                messageStr += std::to_string(data[j][k]) + " ";
    }
}

template<>
void CommunicationController<vector<Rigid3fTypes::Coord>>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<vector<Rigid3fTypes::Coord>>> data = d_data[i];
        int dataSize = 0;
        (*stream) >> dataSize;
        data.resize(dataSize);

        for(unsigned int j=0; j<data.size(); j++)
            for(int k=0; k<7; k++)
                (*stream) >> data[j][k];
    }
}

template<>
void CommunicationController<vector<Rigid3fTypes::Coord>>::checkDataSize(const unsigned int& nbDataFieldReceived)
{
    if(d_data.size()>0)
    {
        ReadAccessor<Data<vector<Rigid3fTypes::Coord>>> data = d_data[0];
        if(nbDataFieldReceived!=d_nbDataField.getValue()*8*data.size())
            msg_warning(this) << "Something wrong with the size of data received. Please check template.";
    }
}



//////////////////////////////// Template name definition

template<>
std::string CommunicationController<double>::templateName(const CommunicationController<double>* object){
    SOFA_UNUSED(object);
    return "double";
}

template<>
std::string CommunicationController<float>::templateName(const CommunicationController<float>* object){
    SOFA_UNUSED(object);
    return "float";
}

template<>
std::string CommunicationController<int>::templateName(const CommunicationController<int>* object){
    SOFA_UNUSED(object);
    return "int";
}

template<>
std::string CommunicationController<unsigned int>::templateName(const CommunicationController<unsigned int>* object){
    SOFA_UNUSED(object);
    return "unsigned int";
}

template<>
std::string CommunicationController<vector<Vec3d>>::templateName(const CommunicationController<vector<Vec3d>>* object){
    SOFA_UNUSED(object);
    return "Vec3d";
}


template<>
std::string CommunicationController<vector<Vec3f>>::templateName(const CommunicationController<vector<Vec3f>>* object){
    SOFA_UNUSED(object);
    return "Vec3f";
}

template<>
std::string CommunicationController<vector<Vec1d>>::templateName(const CommunicationController<vector<Vec1d>>* object){
    SOFA_UNUSED(object);
    return "Vec1d";
}


template<>
std::string CommunicationController<vector<Vec1f>>::templateName(const CommunicationController<vector<Vec1f>>* object){
    SOFA_UNUSED(object);
    return "Vec1f";
}

template<>
std::string CommunicationController<vector<Vec<2,int>>>::templateName(const CommunicationController<vector<Vec<2,int>>>* object){
    SOFA_UNUSED(object);
    return "Vec2i";
}


template<>
std::string CommunicationController<vector<Vec<2,unsigned int>>>::templateName(const CommunicationController<vector<Vec<2,unsigned int>>>* object){
    SOFA_UNUSED(object);
    return "Vec2ui";
}

template<>
std::string CommunicationController<vector<Rigid3dTypes::Coord>>::templateName(const CommunicationController<vector<Rigid3dTypes::Coord>>* object){
    SOFA_UNUSED(object);
    return "Rigid3d";
}


template<>
std::string CommunicationController<vector<Rigid3fTypes::Coord>>::templateName(const CommunicationController<vector<Rigid3fTypes::Coord>>* object){
    SOFA_UNUSED(object);
    return "Rigid3f";
}



////////////////////////////////////////////    FACTORY    ////////////////////////////////////////////
using sofa::core::RegisterObject ;

// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
// 2-RegisterObject("description") + .add<> : Register the component
// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(CommunicationController)

int CommunicationControllerClass = RegisterObject("This component is used to build a communication between two simulations")

#ifdef SOFA_WITH_DOUBLE
.add< CommunicationController<double> >(true)
.add< CommunicationController<vector<Vec3d>> >()
.add< CommunicationController<vector<Vec1d>> >()
.add< CommunicationController<vector<Rigid3dTypes::Coord>> >()
#endif
#ifdef SOFA_WITH_FLOAT
.add< CommunicationController<float> >()
.add< CommunicationController<vector<Vec3f>> >()
.add< CommunicationController<vector<Vec1f>> >()
.add< CommunicationController<vector<Rigid3fTypes::Coord>> >()
#endif
.add< CommunicationController<int> >()
.add< CommunicationController<unsigned int> >()
.add< CommunicationController<vector<Vec<2,int>>> >()
.add< CommunicationController<vector<Vec<2,unsigned int>>> >()
;

///////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_WITH_DOUBLE
template class SOFA_SOFTROBOTS_API CommunicationController<double>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec3d>>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec1d>>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Rigid3dTypes::Coord>>;
#endif
#ifdef SOFA_WITH_FLOAT
template class SOFA_SOFTROBOTS_API CommunicationController<float>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec3f>>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec1f>>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Rigid3fTypes::Coord>>;
#endif
template class SOFA_SOFTROBOTS_API CommunicationController<int>;
template class SOFA_SOFTROBOTS_API CommunicationController<unsigned int>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec<2,int>> >;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec<2,unsigned>> >;


}   //namespace controller
}   //namespace component
}   //namespace sofa


#endif // SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP
