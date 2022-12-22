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
#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>

namespace sofa::core::behavior
{

using type::vector;

SoftRobotsBaseConstraint::SoftRobotsBaseConstraint()
    : m_hasDeltaMax(false)
    , m_hasDeltaMin(false)
    , m_hasDeltaEqual(false)
    , m_hasLambdaMax(false)
    , m_hasLambdaMin(false)
    , m_hasLambdaEqual(false)
    , m_hasLambdaInit(false)
    , m_hasEpsilon(false)
{
}

bool SoftRobotsBaseConstraint::hasDeltaMax()
{
    return m_hasDeltaMax;
}

bool SoftRobotsBaseConstraint::hasDeltaMin()
{
    return m_hasDeltaMin;
}

bool SoftRobotsBaseConstraint::hasDeltaEqual()
{
    return m_hasDeltaEqual;
}



bool SoftRobotsBaseConstraint::hasLambdaMax()
{
    return m_hasLambdaMax;
}

bool SoftRobotsBaseConstraint::hasLambdaMin()
{
    return m_hasLambdaMin;
}

bool SoftRobotsBaseConstraint::hasLambdaEqual()
{
    return m_hasLambdaEqual;
}

bool SoftRobotsBaseConstraint::hasLambdaInit()
{
    return m_hasLambdaInit;
}


bool SoftRobotsBaseConstraint::hasEpsilon()
{
    return m_hasEpsilon;
}


SReal SoftRobotsBaseConstraint::getDeltaMax(const size_t i)
{
    return m_deltaMax[i];
}

SReal SoftRobotsBaseConstraint::getDeltaMin(const size_t i)
{
    return m_deltaMin[i];
}

SReal SoftRobotsBaseConstraint::getDeltaEqual(const size_t i)
{
    return m_deltaEqual[i];
}



SReal SoftRobotsBaseConstraint::getLambdaMax(const size_t i)
{
    return m_lambdaMax[i];
}

SReal SoftRobotsBaseConstraint::getLambdaMin(const size_t i)
{
    return m_lambdaMin[i];
}

SReal SoftRobotsBaseConstraint::getLambdaEqual(const size_t i)
{
    return m_lambdaEqual[i];
}

SReal SoftRobotsBaseConstraint::getLambdaInit(const size_t i)
{
    return m_lambdaInit[i];
}


SReal SoftRobotsBaseConstraint::getEpsilon()
{
    return m_epsilon;
}


unsigned int SoftRobotsBaseConstraint::getNbLines()
{
    return m_nbLines;
}

void SoftRobotsBaseConstraint::storeResults(vector<double>& lambda, vector<double> &delta)
{
    SOFA_UNUSED(lambda);
    SOFA_UNUSED(delta);
}

void SoftRobotsBaseConstraint::storeResults(vector<double> &delta)
{
    SOFA_UNUSED(delta);
}

} // namespaces
