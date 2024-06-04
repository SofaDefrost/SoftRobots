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

#include <sofa/defaulttype/VecTypes.h>
#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>
#include <sofa/helper/OptionsGroup.h>

namespace softrobots::constraint
{

using softrobots::behavior::SoftRobotsConstraint ;
using sofa::core::visual::VisualParams ;
using sofa::core::objectmodel::Data ;
using sofa::defaulttype::Vec3dTypes ;
using sofa::defaulttype::Vec3fTypes ;
using sofa::linearalgebra::BaseVector ;
using sofa::core::ConstraintParams ;
using sofa::helper::ReadAccessor ;
using sofa::core::VecCoordId ;
using sofa::type::Vec3;

using sofa::core::topology::BaseMeshTopology;
using sofa::linearalgebra::BaseVector;

/**
 * This class contains common implementation of cable constraints
*/
template< class DataTypes >
class SOFA_SOFTROBOTS_API CableModel : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableModel,DataTypes),
               SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;
    typedef sofa::type::vector<unsigned int> SetIndexArray;

    typedef typename sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef typename sofa::component::topology::container::dynamic::TriangleSetTopologyContainer TriangleSetTopologyContainer;
    typedef typename TriangleSetTopologyContainer::TrianglesAroundVertex TrianglesAroundVertex;

    typedef typename sofa::core::topology::BaseMeshTopology BaseMeshTopology;

public:
    CableModel(MechanicalState* object = nullptr);
    ~CableModel() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from Actuator //////////////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                     sofa::core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////


protected:

    Data<SetIndexArray>         d_indices; ///< indices of points connected by the cable from extremity to actuated point
    Data<Coord>                 d_pullPoint; ///< coordinate of pulling point
    Data<bool>                  d_hasPullPoint; ///< if false, the pull point is not considered and the cable is entirely mapped

    Data<Real>                  d_cableInitialLength; ///< length of the cable at the start of the simulation
    Data<Real>                  d_cableLength; ///< length of the cable at the end of the time step

    Data<sofa::helper::OptionsGroup>        d_method; ///< method used for computing cable area effect
    Data<VecCoord>              d_centers; ///< list of positions describing attachment of cables on the surface
    bool                        m_hasCenters;
    Data<sofa::type::vector<Real>>    d_radii;  ///< list of radius used to compute pulling application areas from centers
    sofa::SingleLink<CableModel<DataTypes>, sofa::core::topology::BaseMeshTopology, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK> l_surfaceTopology;  ///< link to the topology container of the surface on which the cable is attached
    sofa::type::vector<SetIndexArray> m_areaIndices;
    sofa::type::vector<sofa::type::vector<Real>> m_ratios;

    sofa::type::vector<Real> m_alphaBarycentric;
    sofa::type::vector<Real> m_betaBarycentric;
    sofa::type::vector<Triangle> m_closestTriangle;

    Data<double>                d_force; ///< pulling force applied on the cable
    Data<double>                d_displacement; ///< displacement of the cable

    Data<Real>                  d_maxForce; ///< maximum pulling force applied on the cable
    Data<Real>                  d_minForce; ///< minimum pulling force applied on the cable
    Data<Real>                  d_eqForce; ///< equality force of the cable
    Data<Real>                  d_maxPositiveDisplacement; ///< maximum displacement of the cable in the positive direction
    Data<Real>                  d_maxNegativeDisplacement; ///< maximum displacement of the cable in the negative direction
    Data<Real>                  d_eqDisplacement; ///< equality displacement of the cable
    Data<Real>                  d_maxDispVariation; ///< maximum displacement of the cable allowed

    Data<bool>                  d_drawPullPoint; ///< to draw pull point
    Data<bool>                  d_drawPoints; ///< to draw center points of cable
    Data<bool>                  d_drawPulledAreas; ///< to draw points in cable area effect

    Data<sofa::type::RGBAColor>       d_color;

    bool                        m_hasSlidingPoint;


    SReal getCableLength(const VecCoord &positions);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintIndex ;
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void setUpData();
    void internalInit();

    void checkIndicesRegardingState();
    void initActuatedPoints();
    void initCableActionAreas();
    void computePointsActionArea();
    unsigned int computeClosestIndice(const Coord& position);
    void getPositionFromTopology(Coord& position, sofa::Index index);
    SReal getDistanceToTriangle(const Coord& position, const Triangle& triangle, Coord& projectionOnTriangle);
    void computeBarycentric(const Triangle& triangle, const Coord& p, Real& alpha, Real& beta);

    void drawPullPoint(const VisualParams* vparams);
    void drawPoints(const VisualParams* vparams);
    void drawLinesBetweenPoints(const VisualParams* vparams);
    void drawPulledAreas(const VisualParams* vparams);
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_CABLEMODEL_CPP)
extern template class SOFA_SOFTROBOTS_API CableModel<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API CableModel<sofa::defaulttype::Vec2Types>;
#endif


} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using CableModel SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::constraint::CableModel<DataTypes>;
}

