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
#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>

namespace softrobots::behavior
{

using sofa::type::vector;

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

bool SoftRobotsBaseConstraint::hasDeltaMax() const
{
    return m_hasDeltaMax;
}

bool SoftRobotsBaseConstraint::hasDeltaMin() const
{
    return m_hasDeltaMin;
}

bool SoftRobotsBaseConstraint::hasDeltaEqual() const
{
    return m_hasDeltaEqual;
}



bool SoftRobotsBaseConstraint::hasLambdaMax() const
{
    return m_hasLambdaMax;
}

bool SoftRobotsBaseConstraint::hasLambdaMin() const
{
    return m_hasLambdaMin;
}

bool SoftRobotsBaseConstraint::hasLambdaEqual() const
{
    return m_hasLambdaEqual;
}

bool SoftRobotsBaseConstraint::hasLambdaInit() const
{
    return m_hasLambdaInit;
}


bool SoftRobotsBaseConstraint::hasEpsilon() const
{
    return m_hasEpsilon;
}


SReal SoftRobotsBaseConstraint::getDeltaMax(const size_t i) const
{
    return m_deltaMax[i];
}

SReal SoftRobotsBaseConstraint::getDeltaMin(const size_t i) const
{
    return m_deltaMin[i];
}

SReal SoftRobotsBaseConstraint::getDeltaEqual(const size_t i) const
{
    return m_deltaEqual[i];
}



SReal SoftRobotsBaseConstraint::getLambdaMax(const size_t i) const
{
    return m_lambdaMax[i];
}

SReal SoftRobotsBaseConstraint::getLambdaMin(const size_t i) const
{
    return m_lambdaMin[i];
}

SReal SoftRobotsBaseConstraint::getLambdaEqual(const size_t i) const
{
    return m_lambdaEqual[i];
}

SReal SoftRobotsBaseConstraint::getLambdaInit(const size_t i) const
{
    return m_lambdaInit[i];
}


SReal SoftRobotsBaseConstraint::getEpsilon() const
{
    return m_epsilon;
}


unsigned int SoftRobotsBaseConstraint::getNbLines() const
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
