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

#include <sofa/component/controller/Controller.h>

#include <SoftRobots/component/initSoftRobots.h>

namespace sofa
{

namespace component
{

namespace controller
{

/**
 * This component interpolates between two consecutive inputs when a jump is detected.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template <class T>
class SOFA_SOFTROBOTS_API DataVariationLimiter : public Controller
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(DataVariationLimiter,T),Controller);

    typedef typename type::vector<T> VecValue;
    typedef T                        Value;
    typedef typename T::value_type   ValueType;

public:

    DataVariationLimiter();
    ~DataVariationLimiter() override;


    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Controller ////////////////////
    void onBeginAnimationStep(const double dt) override;
    /////////////////////////////////////////////////////////////////////////

    void interpolate(const int index);

    /// Implementing the GetCustomTemplateName is mandatory to have a custom template name paremters
    /// instead of the default one generated automatically by the SOFA_CLASS() macro.
    static std::string GetCustomTemplateName();


protected:

    Data<VecValue>     d_input;
    Data<VecValue>     d_output;
    Data<unsigned int> d_inputSize;
    Data<double>       d_maxJump;
    Data<unsigned int> d_nbStep;

    Data<bool>         d_initOuput;

    type::vector<bool>                   m_isStabilizing;
    type::vector<unsigned int>           m_step;


    VecValue           m_inititialOuput;


private:

    void initData();

};


// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec3d>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec2d>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec1d>;


extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec1i>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec2i>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec3i>;


} // namespace engine

} // namespace component

} // namespace sofa

