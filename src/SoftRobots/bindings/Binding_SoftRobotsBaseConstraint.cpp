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
#include <SoftRobots/bindings/Binding_SoftRobotsBaseConstraint.h>
#include <pybind11/pybind11.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/PythonFactory.h>

#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>

namespace py { using namespace pybind11; }

namespace softrobots::python3 {

void moduleAddSoftRobotsBaseConstraint(py::module &m)
{
    const auto typeName = softrobots::behavior::SoftRobotsBaseConstraint::GetClass()->className;
    py::class_<softrobots::behavior::SoftRobotsBaseConstraint,
               sofa::core::objectmodel::BaseComponent,
               sofapython3::py_shared_ptr<softrobots::behavior::SoftRobotsBaseConstraint> > c(m, typeName.c_str());

    /// register the binding in the downcasting subsystem
    sofapython3::PythonFactory::registerType<softrobots::behavior::SoftRobotsBaseConstraint>([](sofa::core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<softrobots::behavior::SoftRobotsBaseConstraint*>(object));
    });
}

}
