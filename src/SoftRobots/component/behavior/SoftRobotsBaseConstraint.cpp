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
    :
      d_lambda(initData(&d_lambda, vector<double>(1, 0.0), "force", "Force. Warning: to get the actual force you should divide this value by dt."))
    , d_delta(initData(&d_delta, vector<double>(1, 0.0), "displacement", "Displacement compared to the initial value."))
    , m_nbLines(1)
    , m_hasDeltaMax(false)
    , m_hasDeltaMin(false)
    , m_hasDeltaEqual(false)
    , m_hasLambdaMax(false)
    , m_hasLambdaMin(false)
    , m_hasLambdaEqual(false)
    , m_hasLambdaInit(false)
    , m_hasEnergyWeight(false)
{
    d_constraintIndex.setReadOnly(true);

    resizeConstraints(m_nbLines);
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


bool SoftRobotsBaseConstraint::hasEnergyWeight() const
{
    return m_hasEnergyWeight;
}


SReal SoftRobotsBaseConstraint::getDeltaMax(const size_t i) const
{
    return m_deltaMax[i];
}

vector<double> SoftRobotsBaseConstraint::getDeltaMax() const
{
    return m_deltaMax;
}

SReal SoftRobotsBaseConstraint::getDeltaMin(const size_t i) const
{
    return m_deltaMin[i];
}

vector<double> SoftRobotsBaseConstraint::getDeltaMin() const
{
    return m_deltaMin;
}

SReal SoftRobotsBaseConstraint::getDeltaEqual(const size_t i) const
{
    return m_deltaEqual[i];
}



SReal SoftRobotsBaseConstraint::getLambdaMax(const size_t i) const
{
    return m_lambdaMax[i];
}

vector<double> SoftRobotsBaseConstraint::getLambdaMax() const
{
    return m_lambdaMax;
}

SReal SoftRobotsBaseConstraint::getLambdaMin(const size_t i) const
{
    return m_lambdaMin[i];
}

vector<double> SoftRobotsBaseConstraint::getLambdaMin() const
{
    return m_lambdaMin;
}

SReal SoftRobotsBaseConstraint::getLambdaEqual(const size_t i) const
{
    return m_lambdaEqual[i];
}

SReal SoftRobotsBaseConstraint::getLambdaInit(const size_t i) const
{
    return m_lambdaInit[i];
}


SReal SoftRobotsBaseConstraint::getEnergyWeight() const
{
    return m_energyWeight;
}

const sofa::core::objectmodel::Data<vector<double>>& SoftRobotsBaseConstraint::getLambda() const
{
    return d_lambda;
}

const sofa::core::objectmodel::Data<vector<double>>& SoftRobotsBaseConstraint::getDelta() const
{
    return d_delta;
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


void SoftRobotsBaseConstraint::setLambdaNameAndHelp(const std::string& name, const std::string& help)
{
    setLambdaName(name);
    setLambdaHelp(help);
}

void SoftRobotsBaseConstraint::setLambdaName(const std::string& name)
{
    addAlias(&d_lambda, name.c_str());
    d_lambda.setName(name);
}

void SoftRobotsBaseConstraint::setLambdaHelp(const std::string& help)
{
    d_lambda.setHelp(help);
}

void SoftRobotsBaseConstraint::setDeltaNameAndHelp(const std::string& name, const std::string& help)
{
    setDeltaName(name);
    setDeltaHelp(help);
}

void SoftRobotsBaseConstraint::setDeltaName(const std::string& name)
{
    addAlias(&d_delta, name.c_str());
    d_delta.setName(name);
}

void SoftRobotsBaseConstraint::setDeltaHelp(const std::string& help)
{
    d_delta.setHelp(help);
}

void SoftRobotsBaseConstraint::resizeConstraints(const sofa::Size& size)
{
    m_nbLines = size;

    m_lambdaInit.resize(size);
    m_lambdaMax.resize(size);
    m_lambdaMin.resize(size);
    m_lambdaEqual.resize(size);

    m_deltaMax.resize(size);
    m_deltaMin.resize(size);
    m_deltaEqual.resize(size);

    auto delta = sofa::helper::getWriteAccessor(d_delta);
    delta.resize(size, 0);
    auto lambda = sofa::helper::getWriteAccessor(d_lambda);
    lambda.resize(size, 0);
}

} // namespaces
