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

#ifndef SOFA_COMPONENT_ENGINE_DATAVARIATIONLIMITER_H
#define SOFA_COMPONENT_ENGINE_DATAVARIATIONLIMITER_H

#include <SofaUserInteraction/Controller.h>
#include <SoftRobots/initSoftRobots.h>

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

    typedef typename helper::vector<T> VecValue;
    typedef T                          Value;
    typedef typename T::value_type     ValueType;

public:

    std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const DataVariationLimiter<T>* = nullptr)
    {
        Value v;
        int size = v.size();

        switch(size)
        {
        case 1:
        {
            return defaulttype::DataTypeInfo<defaulttype::Vec<1,ValueType>>::name();
        }
        case 2:
        {
            return defaulttype::DataTypeInfo<defaulttype::Vec<2,ValueType>>::name();
        }
        case 3:
        {
            return defaulttype::DataTypeInfo<defaulttype::Vec<3,ValueType>>::name();
        }
        default: break;
        }

        return "";
    }

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

protected:

    Data<VecValue>     d_input;
    Data<VecValue>     d_output;
    Data<unsigned int> d_inputSize;
    Data<double>       d_maxJump;
    Data<unsigned int> d_nbStep;

    Data<bool>         d_initOuput;

    helper::vector<bool>                   m_isStabilizing;
    helper::vector<unsigned int>           m_step;


    VecValue           m_inititialOuput;


private:

    void initData();

};


// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::defaulttype::Vec3d>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::defaulttype::Vec2d>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::defaulttype::Vec1d>;


extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::defaulttype::Vec1i>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::defaulttype::Vec2i>;
extern template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::defaulttype::Vec3i>;


} // namespace engine

} // namespace component

} // namespace sofa

#endif
