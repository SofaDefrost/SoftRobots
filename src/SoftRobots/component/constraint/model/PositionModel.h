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

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

namespace softrobots::constraint
{

using softrobots::behavior::SoftRobotsConstraint ;
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
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                          DataVecCoord;
    typedef sofa::Data<VecDeriv>                          DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                       DataMatrixDeriv;

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


    /////////////// Inherited from SoftRobotsBaseConstraint ////////////////
    void storeResults(sofa::type::vector<double> &delta) override;
    ///////////////////////////////////////////////////////////////////////////

protected:
    sofa::Data<sofa::type::vector<unsigned int> >     d_indices;
    sofa::Data<sofa::type::vector<Real>>              d_weight;
    sofa::Data<VecDeriv>                              d_directions;
    sofa::Data<Vec<Deriv::total_size, bool>>          d_useDirections;
    sofa::Data<sofa::type::vector<Real>>              d_delta;

    ////////////////////////// Inherited attributes ////////////////////////////
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintIndex ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    using SoftRobotsConstraint<DataTypes>::m_state ;
    ////////////////////////////////////////////////////////////////////////////

    void setDefaultDirections();
    void setDefaultUseDirections();
    void normalizeDirections();
    void drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  sofa::type::RGBAColor& color) ;

private:
    void internalInit();
    void checkIndicesRegardingState();
    void setIndicesDefaultValue();
    void resizeIndicesRegardingState();


};


template<> SOFA_SOFTROBOTS_API
void PositionModel<sofa::defaulttype::Rigid3Types>::normalizeDirections();

template<> SOFA_SOFTROBOTS_API
void PositionModel<sofa::defaulttype::Vec3Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  sofa::type::RGBAColor& color);

template<> SOFA_SOFTROBOTS_API
void PositionModel<sofa::defaulttype::Vec2Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  sofa::type::RGBAColor& color);

template<> SOFA_SOFTROBOTS_API
void PositionModel<sofa::defaulttype::Rigid3Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  sofa::type::RGBAColor& color);

#if !defined(SOFTROBOTS_POSITIONMODEL_CPP)
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Vec1Types>;
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API PositionModel<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace


