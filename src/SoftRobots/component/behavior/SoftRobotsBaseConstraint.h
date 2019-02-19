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
#ifndef SOFA_CORE_BEHAVIOR_SOFTROBOTSBASECONSTRAINT_H
#define SOFA_CORE_BEHAVIOR_SOFTROBOTSBASECONSTRAINT_H

#include <sofa/core/behavior/BaseConstraint.h>

#include <SoftRobots/initSoftRobots.h>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief Component computing inverse problem constraints within a simulated body.
 *
 *  This class defines the abstract API common to all inverse problem constraints.
 */

class SOFA_SOFTROBOTS_API SoftRobotsBaseConstraint : public BaseConstraint
{
public:

    SOFA_CLASS(SoftRobotsBaseConstraint, BaseConstraint);

    // Used by the constraint solver to build the inverse problem
    enum ConstraintType
    {
        ACTUATOR,
        EFFECTOR,
        SENSOR
    };

    ConstraintType m_constraintType;

    /// Returns if the constraint has a maximum value for delta or not
    bool hasDeltaMax();

    /// Returns if the constraint has a minimum value for delta or not
    bool hasDeltaMin();

    /// Returns if the constraint has an equal constraint value for delta or not
    bool hasDeltaEqual();


    /// Returns if the constraint has a maximum value for lambda or not
    bool hasLambdaMax();

    /// Returns if the constraint has a minimum value for lambda or not
    bool hasLambdaMin();

    /// Returns if the constraint has an equal constraint value for lambda or not
    bool hasLambdaEqual();


    /// Returns if the constraint has an epsilon value that can be used to prioritize the constraint (for actuator with inverse problem)
    bool hasEpsilon();


    /// Accessor to maximum value of delta
    SReal getDeltaMax();

    /// Accessor to minimum value of delta
    SReal getDeltaMin();

    /// Accessor to equal constraint value of delta
    SReal getDeltaEqual();


    /// Accessor to maximum value of lambda
    SReal getLambdaMax();

    /// Accessor to maximum value of lambda
    SReal getLambdaMin();

    /// Accessor to equal constraint value of lambda
    SReal getLambdaEqual();


    /// Accessor to epsilon value that can be used to prioritize the constraint (for actuator with inverse problem)
    SReal getEpsilon();



    /// Accessor to nbLines value
    unsigned int getNbLines();

    /// Allows the constraint of having access to the result.
    virtual void storeResults(helper::vector<double> &lambda, helper::vector<double> &delta);

    virtual void storeResults(helper::vector<double> &delta);

    /// Useful when the Constraint is applied only on a subset of dofs.
    /// It is automatically called by buildConstraintMatrix
    ///
    /// That way, we can optimize the time spent to transfer quantities through the mechanical mappings.
    /// Every Dofs are inserted by default. The Constraint using only a subset of dofs should only insert these dofs in the mask.
    void updateForceMask() override {}


protected:

    SoftRobotsBaseConstraint();

    ~SoftRobotsBaseConstraint() override {}

    bool m_hasDeltaMax;
    bool m_hasDeltaMin;
    bool m_hasDeltaEqual;

    bool m_hasLambdaMax;
    bool m_hasLambdaMin;
    bool m_hasLambdaEqual;

    bool m_hasEpsilon;

    double m_deltaMax;
    double m_deltaMin;
    double m_deltaEqual;

    double m_lambdaMax;
    double m_lambdaMin;
    double m_lambdaEqual;

    double m_epsilon;

    unsigned int m_nbLines; ///< Constraint nbLines in the constraints matrix

};


} // namespace behavior

} // namespace core

} // namespace sofa

#endif
