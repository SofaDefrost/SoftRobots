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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLECONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINTSET_CABLECONSTRAINT_INL

#include "CableConstraint.h"

using sofa::helper::OptionsGroup;

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::helper::WriteAccessor;

template<class DataTypes>
CableConstraint<DataTypes>::CableConstraint(MechanicalState* object)
    : Inherit(object)

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

    , d_valueType(initData(&d_valueType, OptionsGroup(3,"displacement","force","stiffness"), "valueType",
                                          "displacement = the contstraint will impose the displacement provided in data d_inputValue[d_iputIndex] \n"
                                          "force = the contstraint will impose the force provided in data d_inputValue[d_iputIndex] \n"
										  "stiffness = the constraint will impose a stiffness (centered around a given displacement). The displacement should be in d_inputValue[d_inputIndex]\n"
										  "If unspecified, the default value is displacement"))

	, d_stiffness(initData(&d_stiffness, (double) 0, "stiffness", "stiffness to enforce (when valueType = stiffness)"))
	, d_neutralEffectorPosition(initData(&d_neutralEffectorPosition, (double)0, "neutralEffectorPosition", "FOR TESTING MISK"))
	, d_neutralActuatorPosition(initData(&d_neutralActuatorPosition, (double)0, "neutralActuatorPosition", "FOR TESTING MISK"))
{
    d_value.setGroup("Input");
    d_valueIndex.setGroup("Input");
    d_valueType.setGroup("Input");
	d_stiffness.setGroup("Input");
	d_neutralEffectorPosition.setGroup("Output"); // TEST
	d_neutralActuatorPosition.setGroup("Output");
}


template<class DataTypes>
CableConstraint<DataTypes>::CableConstraint()
    : Inherit()

    , d_value(initData(&d_value, "value",
                                "Displacement or force to impose.\n"))

    , d_valueIndex(initData(&d_valueIndex, (unsigned int) 0, "valueIndex",
                                  "Index of the value (in InputValue vector) that we want to impose \n"
                                  "If unspecified the default value is {0}"))

	, d_valueType(initData(&d_valueType, OptionsGroup(3, "displacement", "force", "stiffness"), "valueType",
		"displacement = the contstraint will impose the displacement provided in data d_inputValue[d_iputIndex] \n"
		"force = the contstraint will impose the force provided in data d_inputValue[d_iputIndex] \n"
		"stiffness = the constraint will impose a stiffness (centered around a given displacement). The displacement should be in d_inputValue[d_inputIndex]\n"
		"If unspecified, the default value is displacement"))

	, d_stiffness(initData(&d_stiffness, (double)0, "stiffness", "stiffness to enforce (when valueType = stiffness)"))
	, d_neutralEffectorPosition(initData(&d_neutralEffectorPosition, (double)0, "neutralEffectorPosition", "FOR TESTING MISK"))
	, d_neutralActuatorPosition(initData(&d_neutralActuatorPosition, (double)0, "neutralActuatorPosition", "FOR TESTING MISK"))
{
	d_value.setGroup("Input");
	d_valueIndex.setGroup("Input");
	d_valueType.setGroup("Input");
	d_stiffness.setGroup("Input");
	d_neutralEffectorPosition.setGroup("Output"); // TEST
	d_neutralActuatorPosition.setGroup("Output");
}

template<class DataTypes>
CableConstraint<DataTypes>::~CableConstraint()
{
}

template<class DataTypes>
void CableConstraint<DataTypes>::init()
{
    Inherit::init();
    internalInit();
}

template<class DataTypes>
void CableConstraint<DataTypes>::reinit()
{
    internalInit();
}

template<class DataTypes>
void CableConstraint<DataTypes>::internalInit()
{
    // check for errors in the initialization
    if(d_value.getValue().size()==0)
    {
        WriteAccessor<Data<vector<Real>>> inputValue = d_value;
        inputValue.resize(1,0.);
    }

    if(d_value.getValue().size()<d_valueIndex.getValue())
        serr<<"bad size of inputValue ="<< d_value.getValue().size()<<"  or wrong value for inputIndex = "<<d_valueIndex.getValue()<<sendl;

}

template<class DataTypes>
void CableConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParam,
                                                         std::vector<ConstraintResolution*>& resTab,
                                                         unsigned int& offset)
{
    SOFA_UNUSED(cParam);

    double imposed_value=d_value.getValue()[d_valueIndex.getValue()];
	m_imposedValue = imposed_value;
	m_type = d_valueType.getValue().getSelectedItem();

    if(m_type == "displacement") // displacement
    {
        if(d_maxDispVariation.isSet())
        {
            double displacement = d_displacement.getValue();
            if(imposed_value > displacement && imposed_value-displacement>d_maxDispVariation.getValue())
                imposed_value = displacement+d_maxDispVariation.getValue();

            if(imposed_value < displacement && imposed_value-displacement<-d_maxDispVariation.getValue())
                imposed_value = displacement-d_maxDispVariation.getValue();
        }


        CableDisplacementConstraintResolution *cr=  new CableDisplacementConstraintResolution(imposed_value, &m_force);
        resTab[offset++] =cr;
    }
    else if (m_type == "force") // force
    {
        CableForceConstraintResolution *cr=  new CableForceConstraintResolution(imposed_value, &m_displacement);
        resTab[offset++] =cr;
    }
	else // stiffness
	{
		double imposed_stiffness = d_stiffness.getValue();
		CableStiffnessConstraintResolution *cr = new CableStiffnessConstraintResolution(imposed_value, imposed_stiffness, &m_displacement, &m_force, &m_neutralEffectorPosition, &m_neutralActuatorPosition); // TEST
		resTab[offset++] = cr;
	}
}

template<class DataTypes>
void CableConstraint<DataTypes>::draw(const VisualParams* vparams)
{
	if (m_type == "displacement")
	{
		d_cableLength.setValue(d_cableInitialLength.getValue() - m_imposedValue);
		d_displacement.setValue(m_imposedValue);
		d_force.setValue(m_force);
	}
	else if (m_type == "force")
	{
		d_cableLength.setValue(d_cableInitialLength.getValue() - m_displacement);
		d_force.setValue(m_imposedValue);
		d_displacement.setValue(m_displacement);
	}
	else
	{
		d_cableLength.setValue(d_cableInitialLength.getValue() - m_displacement);
		d_displacement.setValue(m_displacement);
		d_force.setValue(m_force);
		d_neutralEffectorPosition.setValue(m_neutralEffectorPosition);
		d_neutralActuatorPosition.setValue(m_neutralActuatorPosition);
	}

}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
