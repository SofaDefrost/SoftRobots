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

#include <SoftRobots/component/controller/DataVariationLimiter.h>

namespace softrobots::controller
{

using std::cout;
using std::endl;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;


template <class DataTypes>
DataVariationLimiter<DataTypes>::DataVariationLimiter()
    : d_input(initData(&d_input,"input"," "))

    , d_output(initData(&d_output,"output"," "))

    , d_inputSize(initData(&d_inputSize,(unsigned int)0,"size","Input size."))

    , d_maxJump(initData(&d_maxJump,double(0.1),"maxJump",
                         "Maximal jump allowed. Default 10% is equivalent to jump = 0.1."))

    , d_nbStep(initData(&d_nbStep,(unsigned int)50,"nbStep",
                        "Number of interpolation steps. Default is 50."))

    , d_initOuput(initData(&d_initOuput, true, "initOutput", "If true, will initialize the output with the input."))
{
    d_inputSize.setReadOnly(true);
}



template <class DataTypes>
DataVariationLimiter<DataTypes>::~DataVariationLimiter()
{
}


template <class DataTypes>
void DataVariationLimiter<DataTypes>::initData()
{
    //Init data
    if(!d_input.isSet())
        msg_warning() << "Input not set.";
    else
    {
        d_inputSize.setValue(d_input.getValue().size());
        if(d_initOuput.getValue())
            d_output.setValue(d_input.getValue());
        else
            d_inputSize.setValue(d_output.getValue().size());
    }

    m_isStabilizing.resize(d_inputSize.getValue());
    m_step.resize(d_inputSize.getValue());
    for(unsigned int i=0; i<d_inputSize.getValue(); i++)
    {
        m_isStabilizing[i] = false;
        m_step[i] = 0;
    }

    //Max jump > 0
    if(d_maxJump.getValue() < 0.)
        d_maxJump.setValue(0.);

    //Nb step > 0
    if(d_nbStep.getValue() < 1)
        d_nbStep.setValue(1);
}


template <class DataTypes>
void DataVariationLimiter<DataTypes>::init()
{
    initData();
    m_inititialOuput = d_output.getValue();
}


template <class DataTypes>
void DataVariationLimiter<DataTypes>::reinit()
{
    //Max jump > 0
    if(d_maxJump.getValue() < 0.)
        d_maxJump.setValue(0.);

    //Nb step > 0
    if(d_nbStep.getValue() < 1)
        d_nbStep.setValue(1);
}


template <class DataTypes>
void DataVariationLimiter<DataTypes>::reset()
{
    initData();
    d_output.setValue(m_inititialOuput);
}


template <class DataTypes>
void DataVariationLimiter<DataTypes>::onBeginAnimationStep(const double dt)
{
    SOFA_UNUSED(dt);

    if(!d_input.isSet())
        return;

    WriteAccessor<sofa::Data<VecValue>> output = d_output;
    ReadAccessor<sofa::Data<VecValue>>  input  = d_input;

    if(input.size()<d_inputSize.getValue())
    {
        msg_warning() << "Problem with input size. Abort.";
        return;
    }

    for(unsigned int i=0; i<d_inputSize.getValue(); i++)
    {
        Value in  = input[i];
        Value out = output[i];
        ValueType jump = (out - in).norm();

        if(jump>d_maxJump.getValue() || m_isStabilizing[i])
            interpolate(i);
        else
            output[i] = input[i];
    }
}


template <class DataTypes>
void DataVariationLimiter<DataTypes>::interpolate(const int index)
{

    WriteAccessor<sofa::Data<VecValue>> output = d_output;
    ReadAccessor<sofa::Data<VecValue>>  input  = d_input;

    if(d_inputSize.getValue() != output.size() || d_inputSize.getValue() != m_step.size() || d_inputSize.getValue() != m_isStabilizing.size())
    {
        dmsg_warning() << "Size problem in interpolate(), size="<<d_inputSize.getValue() << ", output.size=" <<output.size() << ", step.size=" <<m_step.size() << ", isStab.size=" <<m_isStabilizing.size() << ". Abort.";
        return;
    }

    Value out = output[index];
    Value in  = input[index];
    Value direction = in - out;

    ValueType norm = direction.norm()/d_nbStep.getValue()*m_step[index];
    direction.normalize();
    output[index] += direction*norm;
    m_step[index]++;

    if(m_step[index] >= d_nbStep.getValue())
    {
        m_step[index] = 0;
        m_isStabilizing[index] = false;
    }
}

template <class DataTypes>
std::string DataVariationLimiter<DataTypes>::GetCustomTemplateName()
{
    Value v;
    int size = v.size();

    switch(size)
    {
    case 1:
    {
        return sofa::defaulttype::DataTypeInfo<sofa::type::Vec<1,ValueType>>::name();
    }
    case 2:
    {
        return sofa::defaulttype::DataTypeInfo<sofa::type::Vec<2,ValueType>>::name();
    }
    case 3:
    {
        return sofa::defaulttype::DataTypeInfo<sofa::type::Vec<3,ValueType>>::name();
    }
    default: break;
    }

    return "";
}

} // namespace

