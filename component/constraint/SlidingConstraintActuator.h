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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINTACTUATOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINTACTUATOR_H

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
    SlidingDisplacementConstraintResolution(double& imposedDisplacement, double* force);

    //////////////////// Inherited from ConstraintResolution ////////////////////
    virtual void init(int line, double** w, double *lambda) override;
    virtual void resolution(int line, double** w, double* d, double* lambda, double* dfree) override;
    /////////////////////////////////////////////////////////////////////////////

protected:

    void storeForce(int line, double* lambda);

    double      m_wActuatorActuator;
    double      m_imposedDisplacement;
    double*     m_force;

};

class SlidingForceConstraintResolution : public ConstraintResolution
{
public:
    SlidingForceConstraintResolution(double& imposedForce, double* displacement);

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

class SlidingStiffnessConstraintResolution : public ConstraintResolution
{
public:
	SlidingStiffnessConstraintResolution(double& imposedNeutralPosition, double& imposedStiffness, double* displacement, double* force, double& neutralEffectorPosition, double* neutralActuatorPosition, double* Wea, double* Waa);
	// TEST

	//////////////////// Inherited from ConstraintResolution ////////////////////
	virtual void init(int line, double** w, double *force) override;
	virtual void resolution(int line, double** w, double* d, double* force, double* dfree) override;
	/////////////////////////////////////////////////////////////////////////////

protected:

	void storeForceAndDisplacement(int line, double* d, double* lambda);

	double      m_wActuatorActuator;
	double      m_imposedStiffness;
	double		m_imposedBiasForce; // TEST (should be m_imposedNeutralPosition)
	double		m_wEffectorActuator; // TEST
	double		m_neutralEffectorPosition; // TEST
	double*		m_neutralActuatorPosition;
	double*		m_Wea;
	double*		m_Waa;
	double*		m_force;
	double*     m_displacement;

};




/**
 * This component simulates a force exerted by a motor/linear actuator to solve an effector constraint.
 * Description needs to be added: 
 * https://project.inria.fr/softrobot/documentation/constraint/CableConstraint
*/
template< class DataTypes >
class SlidingConstraintActuator : public SlidingModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SlidingConstraintActuator,DataTypes), SOFA_TEMPLATE(SlidingModel,DataTypes));

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
    SlidingConstraintActuator(MechanicalState* object);
    SlidingConstraintActuator();

    virtual ~SlidingConstraintActuator();

    /////////////// Inherited from BaseObject //////////////////////
    virtual void init() override;
    virtual void reinit() override;

	virtual void draw(const VisualParams* vparams) override;
    ///////////////////////////////////////////////////////////////


    /////////////////// Inherited from BaseConstraint ///////////////
    virtual void getConstraintResolution(const core::ConstraintParams *cParam,
                                         std::vector<ConstraintResolution*>& resTab,
                                         unsigned int& offset) override;

	virtual void getConstraintViolation(const ConstraintParams* cParams, BaseVector *resV, const DataVecCoord &xfree, const DataVecDeriv &vfree) override;
    ////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using SlidingModel<DataTypes>::d_maxDispVariation ;
    using SlidingModel<DataTypes>::d_force ;
    using SlidingModel<DataTypes>::d_displacement ;
    ///////////////////////////////////////////////////////////////////////////

	Data<double>						d_neutralEffectorPosition; // TEST
	Data<double>						d_neutralActuatorPosition;
	Data<double>						d_Wea;
	Data<double>						d_Waa;

protected:
    //Input data
    Data<helper::vector< Real > >       d_value;
    Data<unsigned int>                  d_valueIndex;
    Data<helper::OptionsGroup>          d_valueType;
                                        // displacement = the constraint will impose the displacement provided in data d_value[d_valueIndex]
                                        // force = the constraint will impose the force provided in data d_value[d_valueIndex]
										// stiffness = the constraint will impose a stiffness. The provided data is a neutral position and a stiffness. 
	Data<double>						d_stiffness;

    void internalInit();

    double m_displacement;
    double m_force;
	double m_imposedValue;
	double m_neutralEffectorPosition;
	double m_neutralActuatorPosition;
	double m_Wea;
	double m_Waa;
	std::string m_type;

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_EXTERN_TEMPLATE
#ifdef SOFA_WITH_DOUBLE
extern template class SlidingConstraintActuator<sofa::defaulttype::Vec3dTypes>;
extern template class SlidingConstraintActuator<sofa::defaulttype::Rigid3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
extern template class SlidingConstraintActuator<sofa::defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINTACTUATOR_H
