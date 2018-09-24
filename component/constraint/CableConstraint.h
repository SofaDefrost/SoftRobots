/******************************************************************************
*               SOFA, Simulation Open-Framework Architecture                  *
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
*                           Plugin SoftRobots v1.0                            *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/

#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLECONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINTSET_CABLECONSTRAINT_H

#include "model/CableModel.h"
#include <sofa/helper/OptionsGroup.h>

namespace sofa
{

namespace component
{

namespace constraintset
{
using sofa::defaulttype::Vec;
using sofa::defaulttype::Vec3d;
using sofa::helper::WriteAccessor;
using sofa::helper::vector;
using sofa::core::ConstraintParams;
using sofa::defaulttype::BaseVector;
using sofa::core::visual::VisualParams ;
using sofa::core::behavior::ConstraintResolution;


class CableDisplacementConstraintResolution : public ConstraintResolution
{
public:
    CableDisplacementConstraintResolution(double& imposedDisplacement, double* force);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    virtual void init(int line, double** w, double *lambda) override;
    //virtual void initForce(int line, double* lambda){ }
    virtual void resolution(int line, double** w, double* d, double* lambda, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    void storeForce(int line, double* lambda);

    double      m_wActuatorActuator;
    double      m_imposedDisplacement;
    double*     m_force;

};

class CableForceConstraintResolution : public ConstraintResolution
{
public:
    CableForceConstraintResolution(double& imposedForce, double* displacement);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    virtual void init(int line, double** w, double *force) override;
    virtual void initForce(int line, double* force) override;
    virtual void resolution(int line, double** w, double* d, double* force, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    void storeDisplacement(int line, double* d);

    double      m_wActuatorActuator;
    double      m_imposedForce;
    double*     m_displacement;

};




/**
 * This component simulates a force exerted by a cable to solve an effector constraint.
 * Description can be found at:
 * https://project.inria.fr/softrobot/documentation/constraint/CableConstraint
*/
template< class DataTypes >
class CableConstraint : public CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableConstraint,DataTypes), SOFA_TEMPLATE(CableModel,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;

    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef CableModel<DataTypes> Inherit;

    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;

public:
    CableConstraint(MechanicalState* object);
    CableConstraint();

    virtual ~CableConstraint();

    /////////////// Inherited from BaseObject //////////////////////
    virtual void init() override;
    virtual void reinit() override;
    ///////////////////////////////////////////////////////////////


    /////////////////// Inherited from BaseConstraint ///////////////
    virtual void getConstraintResolution(const core::ConstraintParams *cParam,
                                         std::vector<ConstraintResolution*>& resTab,
                                         unsigned int& offset) override;
    ////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using CableModel<DataTypes>::d_cableLength ;
    using CableModel<DataTypes>::d_cableInitialLength ;
    using CableModel<DataTypes>::d_maxDispVariation ;
    using CableModel<DataTypes>::d_force ;
    using CableModel<DataTypes>::d_displacement ;
    using CableModel<DataTypes>::m_componentstate ;
    ///////////////////////////////////////////////////////////////////////////

protected:
    //Input data
    Data<helper::vector< Real > >       d_value;
    Data<unsigned int>                  d_valueIndex;
    Data<helper::OptionsGroup>          d_valueType;
                                        // displacement = the constraint will impose the displacement provided in data d_inputValue[d_iputIndex]
                                        // force = the constraint will impose the force provided in data d_inputValue[d_iputIndex]

    void internalInit();

    double m_displacement;
    double m_force;

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_EXTERN_TEMPLATE
#ifdef SOFA_WITH_DOUBLE
extern template class CableConstraint<sofa::defaulttype::Vec3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
extern template class CableConstraint<sofa::defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_CABLEConstraint_H
