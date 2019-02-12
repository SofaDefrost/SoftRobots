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

#ifndef SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP
#define SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP

#include "CommunicationController.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

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


template<>
void CommunicationController<double>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<double>> data = d_data[i];
        messageStr += std::to_string(data) + " ";
    }
}

template<>
void CommunicationController<int>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<int>> data = d_data[i];
        messageStr += std::to_string(data) + " ";
    }
}

template<>
void CommunicationController<float>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<float>> data = d_data[i];
        messageStr += std::to_string(data) + " ";
    }
}

template<>
void CommunicationController<unsigned int>::convertDataToMessage(string& messageStr)
{
    for(unsigned int i=0; i<d_data.size(); i++)
    {
        ReadAccessor<Data<unsigned int>> data = d_data[i];
        messageStr += std::to_string(data) + " ";
    }
}

template<>
void CommunicationController<double>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<double>> data = d_data[i];
        (*stream) >> data;
    }
}

template<>
void CommunicationController<float>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<float>> data = d_data[i];
        (*stream) >> data;
    }
}

template<>
void CommunicationController<int>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<int>> data = d_data[i];
        (*stream) >> data;
    }
}

template<>
void CommunicationController<unsigned int>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        WriteAccessor<Data<unsigned int>> data = d_data[i];
        (*stream) >> data;
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
using defaulttype::Rigid3Types;
using defaulttype::Vec3Types;
using defaulttype::Vec1Types;
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int CommunicationControllerClass = RegisterObject("This component is used to build a communication between two simulations")

.add< CommunicationController<double> >(true)
.add< CommunicationController<vector<Vec3Types::Coord>> >()
.add< CommunicationController<vector<Vec1Types::Coord>> >()
.add< CommunicationController<vector<Rigid3Types::Coord>> >()

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
template class SOFA_SOFTROBOTS_API CommunicationController<double>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec3Types::Coord>>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec1Types::Coord>>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Rigid3Types::Coord>>;

template class SOFA_SOFTROBOTS_API CommunicationController<int>;
template class SOFA_SOFTROBOTS_API CommunicationController<unsigned int>;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec<2,int>> >;
template class SOFA_SOFTROBOTS_API CommunicationController<vector<Vec<2,unsigned>> >;


}   //namespace controller
}   //namespace component
}   //namespace sofa


#endif // SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP
