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

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/VecTypes.h>

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

namespace softrobots::constraint
{

using sofa::core::topology::BaseMeshTopology ;
using softrobots::behavior::SoftRobotsConstraint ;
using sofa::core::visual::VisualParams ;
using sofa::core::objectmodel::Data ;
using sofa::defaulttype::Vec3dTypes ;
using sofa::defaulttype::Vec3fTypes ;
using sofa::linearalgebra::BaseVector ;
using sofa::core::ConstraintParams ;
using sofa::helper::ReadAccessor ;
using sofa::core::VecCoordId ;


/**
 * This class contains common implementation of surface pressure constraints
*/
template< class DataTypes >
class SOFA_SOFTROBOTS_API SurfacePressureModel : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureModel,DataTypes),
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

    typedef sofa::core::topology::BaseMeshTopology::Triangle      Triangle;
    typedef sofa::core::topology::BaseMeshTopology::Quad          Quad;
    typedef sofa::core::topology::BaseMeshTopology::Edge          Edge;

public:
    SurfacePressureModel(MechanicalState* object = nullptr);
    ~SurfacePressureModel() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void bwdInit() override;
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
    Data<sofa::type::vector<Triangle> >     d_triangles;
    Data<sofa::type::vector<Quad> >         d_quads;
    sofa::type::vector<Edge>                m_edges;

    Data<Real>                          d_initialCavityVolume;
    Data<Real>                          d_cavityVolume;
    Data<bool>                          d_flipNormal;

    Data<double>                        d_pressure;
    Data<Real>                          d_maxPressure;
    Data<Real>                          d_minPressure;
    Data<Real>                          d_eqPressure;
    Data<Real>                          d_maxPressureVariation;
    Data<double>                        d_volumeGrowth;
    Data<Real>                          d_maxVolumeGrowth;
    Data<Real>                          d_minVolumeGrowth;
    Data<Real>                          d_eqVolumeGrowth;
    Data<Real>                          d_maxVolumeGrowthVariation;

    Data<bool>                          d_drawPressure;
    Data<Real>                          d_drawScale;


protected:

    SReal getCavityVolume(const VecCoord &positions);
    void drawQuads(const VisualParams* vparams, float red, float green, float blue);
    void drawTriangles(const VisualParams* vparams, float red, float green, float blue);
    void drawLines(const VisualParams* vparams, float red, float green, float blue);
    std::string getValueString(Real pressure);

    ////////////////////////// Inherited attributes ////////////////////////////
    using SoftRobotsConstraint<DataTypes>::m_state ;
    using SoftRobotsConstraint<DataTypes>::getContext ;
    using SoftRobotsConstraint<DataTypes>::m_nbLines ;
    using SoftRobotsConstraint<DataTypes>::m_constraintIndex ;
    using SoftRobotsConstraint<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void setUpData();
    void internalInit();

    void drawValue(const sofa::core::visual::VisualParams* vparams);
    void computeEdges();
};

extern template class SurfacePressureModel<sofa::defaulttype::Vec3Types>;


} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using SurfacePressureModel SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::constraint::SurfacePressureModel<DataTypes>;
}
