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
#pragma once

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

namespace sofa
{

namespace component
{

namespace constraintset
{
using sofa::core::behavior::SoftRobotsConstraint ;
using sofa::core::ConstraintParams ;
using sofa::linearalgebra::BaseVector ;
using sofa::type::Vec ;
using sofa::core::visual::VisualParams ;

/**
 * This class contains common implementation of position constraints
*/
template< class DataTypes >
class PositionModel : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PositionModel,DataTypes),
               SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes));

    typedef typename DataTypes::VecCoord            VecCoord;
    typedef typename DataTypes::VecDeriv            VecDeriv;
    typedef typename DataTypes::Coord               Coord;
    typedef typename DataTypes::Deriv               Deriv;
    typedef typename DataTypes::MatrixDeriv         MatrixDeriv;
    typedef typename Coord::value_type              Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>                          DataVecCoord;
    typedef Data<VecDeriv>                          DataVecDeriv;
    typedef Data<MatrixDeriv>                       DataMatrixDeriv;

public:
    PositionModel(MechanicalState* object = nullptr);
    ~PositionModel() override;

    /////////////// Inherited from BaseObject  ////////////
    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;
    //////////////////////////////////////////////////////

    /////////////// Inherited from Effector ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;
    ///////////////////////////////////////////////////////////////


    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(type::vector<double> &delta) override;
    ///////////////////////////////////////////////////////////////////////////

protected:
    Data<type::vector<unsigned int> >             d_indices;
    Data<Real>                                  d_weight;
    Data<VecDeriv>                                d_directions;
    Data<Vec<Deriv::total_size,bool>>             d_useDirections;
    Data<type::vector<double>>                    d_delta;

    ////////////////////////// Inherited attributes ////////////////////////////
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintId ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    using SoftRobotsConstraint<DataTypes>::m_state ;
    ////////////////////////////////////////////////////////////////////////////

    void setDefaultDirections();
    void setDefaultUseDirections();
    void normalizeDirections();
    void drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  type::RGBAColor& color) ;

private:
    void internalInit();
    void checkIndicesRegardingState();
    void setIndicesDefaultValue();
    void resizeIndicesRegardingState();


};


template<> SOFA_SOFTROBOTS_API
void PositionModel<defaulttype::Rigid3Types>::normalizeDirections();

template<> SOFA_SOFTROBOTS_API
void PositionModel<defaulttype::Vec3Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  type::RGBAColor& color);

template<> SOFA_SOFTROBOTS_API
void PositionModel<defaulttype::Vec2Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  type::RGBAColor& color);

template<> SOFA_SOFTROBOTS_API
void PositionModel<defaulttype::Rigid3Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  type::RGBAColor& color);

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_POSITIONMODEL_CPP)
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

