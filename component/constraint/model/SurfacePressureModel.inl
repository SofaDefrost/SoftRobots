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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREMODEL_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREMODEL_INL
#include <cstdio>
#include <cstdlib>

#include "SurfacePressureModel.h"
#include <iomanip>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/helper/logging/Messaging.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::visual::VisualParams;
using sofa::helper::vector;
using core::ConstVecCoordId;
using sofa::defaulttype::Mat;
using sofa::defaulttype::Vec3d;
using sofa::defaulttype::Vec4f;
using std::string;
using std::ostringstream;
using sofa::defaulttype::Vector3;

template<class DataTypes>
SurfacePressureModel<DataTypes>::SurfacePressureModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
    , d_triangles(initData(&d_triangles, "triangles",
                            "List of triangles on which the surface pressure is applied.\n"
                            "If no list is given, the component will \n"
                            "fill the two lists with the context topology."))

    , d_quads(initData (&d_quads, "quads",
                            "List of quads on which the surface pressure is applied. \n"
                            "If no list is given, the component will \n"
                            "fill the two lists with the context topology."))

    , d_initialCavityVolume(initData(&d_initialCavityVolume, "initialCavityVolume",
                               "Output volume of the cavity at init (only relevant in case of closed mesh)"))

    , d_cavityVolume(initData(&d_cavityVolume, (Real) 1.0, "cavityVolume",
                               "Output volume of the cavity (only relevant in case of closed mesh)"))

    , d_deltaCavityVolume(initData(&d_deltaCavityVolume, (Real)0.0, "deltaCavityVolume",
                                "Difference between the intial cavity volume and the current one"))

    , d_flipNormal(initData(&d_flipNormal, false, "flipNormal",
                               "Allows to invert cavity faces orientation. \n"
                               "If a positive pressure acts like a depressurization, try to set \n"
                               "flipNormal to true."))

    , d_pressure(initData(&d_pressure, (double) 0, "pressure",
                             "Output pressure."))

    , d_volumeGrowth(initData(&d_volumeGrowth, (double) 0, "volumeGrowth",
                             "Output volume growth."))

    , d_maxVolumeGrowthVariation(initData(&d_maxVolumeGrowthVariation, (Real) 0, "maxVolumeGrowthVariation",
                             "Maximum volume growth variation allowed for actuation. If no value is set by user, no \n"
                             "maximum will be considered. NB: this value has a dependancy with the time step \n"
                             "(volume/dt) in the dynamic case."))

    , d_showVisuScale(initData(&d_showVisuScale, (Real) 0.1, "showVisuScale",
                               "Scale for visualization. If unspecified the default value is {0.1}"))
{
    d_showVisuScale.setGroup("Visualization");

    d_cavityVolume.setReadOnly(true);
    d_initialCavityVolume.setReadOnly(true);
    d_deltaCavityVolume.setReadOnly(true);

    d_pressure.setGroup("Vector");
    d_volumeGrowth.setGroup("Vector");

    d_pressure.setReadOnly(true);
    d_volumeGrowth.setReadOnly(true);

    d_maxVolumeGrowthVariation.setGroup("Input");
}


template<class DataTypes>
SurfacePressureModel<DataTypes>::SurfacePressureModel()
    : SoftRobotsConstraint<DataTypes>()
    , d_triangles(initData(&d_triangles, "triangles",
                            "List of triangles on which the surface pressure is applied.\n"
                            "If no list is given, the component will \n"
                            "fill the two lists with the context topology."))

    , d_quads(initData (&d_quads, "quads",
                            "List of quads on which the surface pressure is applied. \n"
                            "If no list is given, the component will \n"
                            "fill the two lists with the context topology."))

    , d_initialCavityVolume(initData(&d_initialCavityVolume, "initialCavityVolume",
                               "Output volume of the cavity at init (only relevant in case of closed mesh)"))

    , d_cavityVolume(initData(&d_cavityVolume, (Real) 1.0, "cavityVolume",
                               "Output volume of the cavity (only relevant in case of closed mesh)"))

    , d_deltaCavityVolume(initData(&d_deltaCavityVolume, (Real)0.0, "deltaCavityVolume",
                                "Difference between the intial cavity volume and the current one"))

    , d_flipNormal(initData(&d_flipNormal, false, "flipNormal",
                               "Allows to invert cavity faces orientation. \n"
                               "If a positive pressure acts like a depressurization, try to set \n"
                               "flipNormal to true."))

    , d_pressure(initData(&d_pressure, (double) 0, "pressure",
                             "Output pressure."))

    , d_volumeGrowth(initData(&d_volumeGrowth, (double) 0, "volumeGrowth",
                             "Output volume growth."))

    , d_maxVolumeGrowthVariation(initData(&d_maxVolumeGrowthVariation, (Real) 0, "maxVolumeGrowthVariation",
                             "Maximum volume growth variation allowed for actuation. If no value is set by user, no \n"
                             "maximum will be considered. NB: this value has a dependancy with the time step \n"
                             "(volume/dt) in the dynamic case."))

    , d_showVisuScale(initData(&d_showVisuScale, (Real) 0.1, "showVisuScale",
                               "Scale for visualization. If unspecified the default value is {0.1}"))

{
    d_showVisuScale.setGroup("Visualization");

    d_cavityVolume.setReadOnly(true);
    d_initialCavityVolume.setReadOnly(true);
    d_deltaCavityVolume.setReadOnly(true);

    d_pressure.setGroup("Vector");
    d_volumeGrowth.setGroup("Vector");

    d_pressure.setReadOnly(true);
    d_volumeGrowth.setReadOnly(true);

    d_maxVolumeGrowthVariation.setGroup("Input");
}

template<class DataTypes>
SurfacePressureModel<DataTypes>::~SurfacePressureModel()
{
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::init()
{
    m_componentstate = ComponentState::Invalid;
    SoftRobotsConstraint<DataTypes>::init();

    if(m_state==nullptr)
    {
        msg_error(this) << "There is no mechanical state associated with this node. "
                           "The object is then deactivated. "
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }

    if(m_state->read(VecCoordId::position())==nullptr)
    {
        msg_error(this) << "There is no position vector in the MechanicalState. "
                             "The object is deactivated. " ;
        return;
    }

    /// Check that the triangles and quads datafield contain something, otherwise get context
    /// topology
    if(d_triangles.getValue().size() == 0 && d_quads.getValue().size() == 0)
    {
        msg_info(this) <<"No triangles and quads given. Get context topology.";
        BaseMeshTopology* topology = getContext()->getMeshTopology();

        if(topology==nullptr)
        {
            msg_error(this) << "There is no topology state associated with this node. "
                               "To remove this error message, fix your scene possibly by "
                               "adding a Topology in the parent node or by giving a list of triangles"
                               "indices or a list of quads indices as nodes parameters ." ;
            return;
        }

        d_triangles.setValue(topology->getTriangles());
        d_quads.setValue(topology->getQuads());
        m_edges = topology->getEdges();
    }
    else
    {
        computeEdges();
    }


    /// Check that the triangles datafield does not contains indices that would crash the
    /// component.
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(VecCoordId::position());
    int numTris = d_triangles.getValue().size() ;
    auto triangles = d_triangles.getValue() ;
    for(int i=0;i<numTris;i++){
        for(int j=0;j<3;j++){
            if( triangles[i][j] < 0 )
                msg_error(this) << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                                   <<". is too small regarding mechanicalState size of(" << positions.size() << ")" ;

            if( triangles[i][j] >= positions.size() )
                msg_error(this) << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                                   <<". is too large regarding mechanicalState size of(" << positions.size() << ")" ;
        }
    }

    /// Check that the quads datafield does not contains indices that would crash the
    /// component.
    int numQuads = d_quads.getValue().size() ;
    auto quads = d_quads.getValue() ;
    for(int i=0;i<numQuads;i++){
        for(int j=0;j<4;j++){
            if( quads[i][j] < 0 )
                msg_error(this) << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                                   << " is too small regarding mechanicalState size of("
                                   << positions.size() << ")" ;

            if( quads[i][j] >= positions.size() )
                msg_error(this) << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                                   << " is too large regarding mechanicalState size of("
                                   << positions.size() << ")" ;
        }
    }

    Real volume = getCavityVolume(positions.ref());
    d_initialCavityVolume.setValue(volume);
    d_cavityVolume.setValue(volume);
    d_deltaCavityVolume.setValue(d_initialCavityVolume.getValue()-volume);
    m_componentstate = ComponentState::Valid;
}


template<class DataTypes>
void SurfacePressureModel<DataTypes>::reset()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    d_cavityVolume.setValue(d_initialCavityVolume.getValue());
}


template<class DataTypes>
SReal SurfacePressureModel<DataTypes>::getCavityVolume(const VecCoord& positions)
{
    // NB : The computation  of the cavity volume is not relevant
    // in the case of an open mesh.

    ReadAccessor<Data<vector<Triangle> > >  triangles = d_triangles;
    ReadAccessor<Data<vector<Quad> > >      quads     = d_quads;

    Coord p0, p1, p2;
    Real volume = 0;

    for (unsigned int t=0; t<triangles.size(); t++)
    {
        p0 = positions[triangles[t][0]];
        p1 = positions[triangles[t][1]];
        p2 = positions[triangles[t][2]];

        volume += ((p1[1]-p0[1])*(p2[2]-p0[2])-(p2[1]-p0[1])*(p1[2]-p0[2]))*(p0[0]+p1[0]+p2[0])/6;
    }

    for (unsigned int q=0; q<quads.size(); q++)
    {
        p0 = positions[quads[q][0]];
        p1 = positions[quads[q][1]];
        p2 = positions[quads[q][2]];

        volume += ((p1[1]-p0[1])*(p2[2]-p0[2])-(p2[1]-p0[1])*(p1[2]-p0[2]))*(p0[0]+p1[0]+p2[0])/6;

        p0 = positions[quads[q][0]];
        p1 = positions[quads[q][2]];
        p2 = positions[quads[q][3]];

        volume += ((p1[1]-p0[1])*(p2[2]-p0[2])-(p2[1]-p0[1])*(p1[2]-p0[2]))*(p0[0]+p1[0]+p2[0])/6;
    }

    if(volume<0)
        volume = -volume;

    return volume;
}


template<class DataTypes>
void SurfacePressureModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                                BaseVector *resV,
                                                                const DataVecCoord &xfree,
                                                                const DataVecDeriv &vfree)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(vfree);

    Real dfree = SurfacePressureModel<DataTypes>::getCavityVolume(xfree.getValue());
    resV->set(m_columnId, dfree - d_initialCavityVolume.getValue());
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                               DataMatrixDeriv &cMatrix,
                                                               unsigned int &columnIndex,
                                                               const DataVecCoord &x)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    m_columnId = columnIndex;

    ReadAccessor<Data<VecCoord>>         position = x;
    ReadAccessor<Data<vector<Quad>>>     quadList = d_quads;
    ReadAccessor<Data<vector<Triangle>>> triList  = d_triangles;

    MatrixDeriv& matrix = *cMatrix.beginEdit();
    matrix.begin();
    MatrixDerivRowIterator rowIterator = matrix.writeLine(m_columnId);

    columnIndex++;

    for (Quad quad :  quadList)
    {
        Deriv triangle1Normal = cross(position[quad[1]] - position[quad[0]], position[quad[3]] - position[quad[0]])/2.0;
        Deriv triangle2Normal = cross(position[quad[3]] - position[quad[2]], position[quad[1]] - position[quad[2]])/2.0;
        Deriv quadNormal      = triangle1Normal + triangle2Normal;
        if(d_flipNormal.getValue())
            quadNormal = -quadNormal;

        for (unsigned i=0; i<4; i++)
        {
            rowIterator.addCol(quad[i], quadNormal*(1.0/4.0));
        }
    }

    for (Triangle triangle : triList)
    {
        Deriv triangleNormal = cross(position[triangle[1]]- position[triangle[0]], position[triangle[2]] -position[triangle[0]])/2.0;
        if(d_flipNormal.getValue())
            triangleNormal = -triangleNormal;

        for (unsigned i=0; i<3; i++)
        {
            rowIterator.addCol(triangle[i], triangleNormal*(1.0/3.0));
        }
    }

    cMatrix.endEdit();
    m_nbLines = columnIndex - m_columnId;

    // Update cavity volume at each time step
    if(m_state!=nullptr)
        position = m_state->read(core::ConstVecCoordId::position());
    d_cavityVolume.setValue(getCavityVolume(position.ref()));
    d_deltaCavityVolume.setValue(d_initialCavityVolume.getValue() - getCavityVolume(position.ref()));
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::draw(const VisualParams* vparams)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if (m_visualization) drawValue(vparams);
    if (!vparams->displayFlags().getShowInteractionForceFields()) return;

    float red, green, blue;

    if (m_displayedValue > 0)
    {
        red = 1.0;
        green = 0;
        blue = 0;
    }
    else
    {
        red = 0;
        green = 1.0;
        blue = 0;
    }

    drawQuads(vparams, red, green, blue);
    drawTriangles(vparams, red, green, blue);
    drawLines(vparams, 0.5*red, 0.5*green, blue);
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::drawValue(const core::visual::VisualParams* vparams)
{
    float scale = (float)( ( vparams->sceneBBox().maxBBox() - vparams->sceneBBox().minBBox() ).norm() * d_showVisuScale.getValue() );
    string value = getValueString(m_displayedValue);
    vparams->drawTool()->draw3DText(vparams->sceneBBox().maxBBox(),scale,Vec4f(1.,1.,1.,1.),value.c_str());
}

template<class DataTypes>
string SurfacePressureModel<DataTypes>::getValueString(Real pressure)
{
    ostringstream oss;
    oss << std::setprecision(3) << pressure;
    return oss.str();
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::drawQuads(const VisualParams* vparams, float red, float green, float blue)
{
    ReadAccessor<Data<VecCoord>> x = *m_state->read(ConstVecCoordId::position());
    ReadAccessor<Data<vector<Quad>>>   quadList = d_quads;

    vector<Vector3> points(quadList.size()*4);
    for (unsigned int i =0; i<quadList.size(); i++)
    {
        Quad quad = quadList[i];
        points[i*4]  =x[quad[0]];
        points[i*4+1]=x[quad[1]];
        points[i*4+2]=x[quad[2]];
        points[i*4+3]=x[quad[3]];
    }

    vparams->drawTool()->drawQuads(points, Vec4f(red,green,blue,1));
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::drawTriangles(const VisualParams* vparams, float red, float green, float blue)
{
    ReadAccessor<Data<VecCoord>> x = *m_state->read(ConstVecCoordId::position());
    ReadAccessor<Data<vector<Triangle>>> triList  = d_triangles;

    vector<Vector3> points(triList.size()*3);
    for (unsigned int i =0; i<triList.size(); i++)
    {
        Triangle tri = triList[i];
        points[i*3]  =x[tri[0]];
        points[i*3+1]=x[tri[1]];
        points[i*3+2]=x[tri[2]];
    }

    vparams->drawTool()->drawTriangles(points, Vec4f(red,green,blue,1));
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::drawLines(const VisualParams* vparams, float red, float green, float blue)
{
    ReadAccessor<Data<VecCoord>> x = *m_state->read(ConstVecCoordId::position());

    for (Edge e : m_edges)
    {
        vparams->drawTool()->drawLine(x[e[0]], x[e[1]], Vec4f(red,green,blue,1));
    }
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::computeEdges()
{
    ReadAccessor<Data<vector<Triangle>>> triList  = d_triangles;
    ReadAccessor<Data<vector<Quad>>>   quadList = d_quads;

    std::map<Edge,unsigned int> edgeMap;
    unsigned int edgeIndex;
    m_edges.clear();

    for (Triangle t : triList)
    {
        std::map<Edge,unsigned int>::iterator ite;
        Edge e;
        for (unsigned int j=0; j<3; ++j)
        {
            unsigned int v1=t[(j+1)%3];
            unsigned int v2=t[(j+2)%3];
            // sort vertices in lexicographics order
            if (v1<v2)
                e=Edge(v1,v2);
            else
                e=Edge(v2,v1);
            ite=edgeMap.find(e);
            if (ite==edgeMap.end())
            {
                // edge not in edgeMap so create a new one
                edgeIndex=(unsigned int)m_edges.size();
                edgeMap[e]=edgeIndex;
                m_edges.push_back(e);
            }
        }
    }

    for (Quad q : quadList)
    {
        std::map<Edge,unsigned int>::iterator ite;
        Edge e;
        for (unsigned int j=0; j<4; ++j)
        {
            unsigned int v1=q[(j+1)%4];
            unsigned int v2=q[(j+2)%4];
            // sort vertices in lexicographics order
            if (v1<v2)
                e=Edge(v1,v2);
            else
                e=Edge(v2,v1);
            ite=edgeMap.find(e);
            if (ite==edgeMap.end())
            {
                // edge not in edgeMap so create a new one
                edgeIndex=(unsigned int)m_edges.size();
                edgeMap[e]=edgeIndex;
                m_edges.push_back(e);
            }
        }
    }
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
