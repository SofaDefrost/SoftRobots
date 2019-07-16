/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGACTUATORCONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGACTUATORCONSTRAINT_H

#include "model/SlidingModel.h"
#include <sofa/helper/OptionsGroup.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::behavior::ConstraintResolution;


class SlidingDisplacementConstraintResolution : public ConstraintResolution
{
public:
    SlidingDisplacementConstraintResolution(double& imposedDisplacement);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, double** w, double *lambda) override;
    void resolution(int line, double** w, double* d, double* lambda, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
	
    double      m_wActuatorActuator;
    double      m_imposedDisplacement;

};

class SlidingForceConstraintResolution : public ConstraintResolution
{
public:
    SlidingForceConstraintResolution(double& imposedForce);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    void init(int line, double** w, double *force) override;
    void resolution(int line, double** w, double* d, double* force, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:
	
    double      m_wActuatorActuator;
    double      m_imposedForce;

};


/**
 * This component simulates a force exerted by a motor/linear actuator to solve an effector constraint.
 * Description needs to be added: 
 * https://project.inria.fr/softrobot/documentation/constraint/SlidingActuatorConstraint
*/
template< class DataTypes >
class SlidingActuatorConstraint : public SlidingModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SlidingActuatorConstraint,DataTypes), SOFA_TEMPLATE(SlidingModel,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;

    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef SlidingModel<DataTypes> Inherit;

    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>   DataMatrixDeriv;

public:
    SlidingActuatorConstraint(MechanicalState* object = nullptr);
	~SlidingActuatorConstraint() override;

    /////////////// Inherited from BaseObject //////////////////////
    void init() override;
    void reinit() override;

	//void draw(const VisualParams* vparams) override;
    ///////////////////////////////////////////////////////////////


    /////////////////// Inherited from BaseConstraint ///////////////
    void getConstraintResolution(const core::ConstraintParams *cParam,
                                         std::vector<ConstraintResolution*>& resTab,
                                         unsigned int& offset) override;
	/////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using SlidingModel<DataTypes>::d_maxDispVariation ;
    using SlidingModel<DataTypes>::d_force ;
    using SlidingModel<DataTypes>::d_displacement ;
    ///////////////////////////////////////////////////////////////////////////

protected:
    //Input data
    Data<helper::vector< Real > >       d_value;
    Data<unsigned int>                  d_valueIndex;
    Data<helper::OptionsGroup>          d_valueType;
                                        // displacement = the constraint will impose the displacement provided in data d_value[d_valueIndex]
                                        // force = the constraint will impose the force provided in data d_value[d_valueIndex]
										
    void internalInit();

	double m_imposedValue;
	std::string m_type;

// TODO: Margaret Koehler: add displacement and force limits

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class SlidingActuatorConstraint<sofa::defaulttype::Vec3Types>;
extern template class SlidingActuatorConstraint<sofa::defaulttype::Rigid3Types>;

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGACTUATORCONSTRAINT_H
