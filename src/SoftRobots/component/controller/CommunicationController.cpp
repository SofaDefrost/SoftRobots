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

#include <SoftRobots/component/controller/CommunicationController.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component::controller
{

using sofa::type::Vec3d;
using sofa::type::Vec3f;
using sofa::type::Vec1d;
using sofa::type::Vec1f;
using sofa::type::Vec;
using type::vector;

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
        double value;
        (*stream) >> value;
        d_data[i]->setValue(value);
    }
}

template<>
void CommunicationController<float>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        float value;
        (*stream) >> value;
        d_data[i]->setValue(value);
    }
}

template<>
void CommunicationController<int>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        int value;
        (*stream) >> value;
        d_data[i]->setValue(value);
    }
}

template<>
void CommunicationController<unsigned int>::convertStringStreamToData(stringstream *stream)
{
    for (unsigned int i= 0; i<d_data.size(); i++)
    {
        unsigned int value;
        (*stream) >> value;
        d_data[i]->setValue(value);
    }
}

//////////////////////////////// Template name definition

template<>
std::string CommunicationController<double>::GetCustomTemplateName(){return "double";}

template<>
std::string CommunicationController<float>::GetCustomTemplateName(){return "float";}

template<>
std::string CommunicationController<int>::GetCustomTemplateName(){return "int";}

template<>
std::string CommunicationController<unsigned int>::GetCustomTemplateName(){return "unsigned int";}

template<>
std::string CommunicationController<vector<Vec3d>>::GetCustomTemplateName(){return "Vec3d";}

template<>
std::string CommunicationController<vector<Vec3f>>::GetCustomTemplateName(){return "Vec3f";}

template<>
std::string CommunicationController<vector<Vec1d>>::GetCustomTemplateName(){return "Vec1d";}

template<>
std::string CommunicationController<vector<Vec1f>>::GetCustomTemplateName(){return "Vec1f";}

template<>
std::string CommunicationController<vector<Vec<2,int>>>::GetCustomTemplateName(){return "Vec2i";}

template<>
std::string CommunicationController<vector<Vec<2,unsigned int>>>::GetCustomTemplateName(){return "Vec2ui";}

template<>
std::string CommunicationController<vector<Rigid3dTypes::Coord>>::GetCustomTemplateName(){return "Rigid3d";}

template<>
std::string CommunicationController<vector<Rigid3fTypes::Coord>>::GetCustomTemplateName(){return "Rigid3f";}


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
template class CommunicationController<double>;
template class CommunicationController<vector<Vec3Types::Coord>>;
template class CommunicationController<vector<Vec1Types::Coord>>;
template class CommunicationController<vector<Rigid3Types::Coord>>;

template class CommunicationController<int>;
template class CommunicationController<unsigned int>;
template class CommunicationController<vector<Vec<2,int>> >;
template class CommunicationController<vector<Vec<2,unsigned>> >;


}   //namespace


#endif // SOFA_CONTROLLER_COMMUNICATIONCONTROLLER_CPP
