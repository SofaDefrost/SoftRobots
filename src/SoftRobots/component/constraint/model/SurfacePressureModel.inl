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

#include <iomanip>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/Vec.h>

#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::visual::VisualParams;
using sofa::core::ConstVecCoordId;
using sofa::type::Mat;
using sofa::type::Vec3;
using sofa::type::RGBAColor;
using sofa::type::vector;
using std::string;
using std::ostringstream;

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

    , d_cavityVolume(initData(&d_cavityVolume, Real(1.0), "cavityVolume",
                               "Output volume of the cavity (only relevant in case of closed mesh)"))

    , d_flipNormal(initData(&d_flipNormal, false, "flipNormal",
                               "Allows to invert cavity faces orientation. \n"
                               "If a positive pressure acts like a depressurization, try to set \n"
                               "flipNormal to true."))

    , d_pressure(initData(&d_pressure, double(0.0), "pressure",
                             "Output pressure. Warning: to get the actual pressure you should divide this value by dt."))


    , d_maxPressure(initData(&d_maxPressure, "maxPressure",
                             "Maximum pressure allowed for actuation. If no value is set by user, no \n"
                             "maximum pressure constraint will be considered."))

    , d_minPressure(initData(&d_minPressure, "minPressure",
                             "Minimum pressure allowed for actuation. If no value is set by user, no \n"
                             "minimum pressure constraint will be considered. A negative pressure will empty/drain the cavity."))

    , d_eqPressure(initData(&d_eqPressure, Real(0.0), "eqPressure",
                            "Equality constraint for the pressure. \n"  
                            "Solver will try to maintain the pressure at this value.\n"  
                            "If unspecified, no equality constraint will be considered."))  

    , d_maxPressureVariation(initData(&d_maxPressureVariation, "maxPressureVariation",
                            "Maximum pressure variation allowed for actuation. If no value is set by user, no \n"
                            "maximum will be considered."))

    , d_volumeGrowth(initData(&d_volumeGrowth, double(0.0), "volumeGrowth",
                             "Output volume growth."))

    , d_maxVolumeGrowth(initData(&d_maxVolumeGrowth, "maxVolumeGrowth",
                             "Maximum volume growth allowed for actuation. If no value is set by user, no \n"
                             "maximum will be considered. NB: this value has a dependancy with the time step \n"
                             "(volume/dt) in the dynamic case."))

    , d_minVolumeGrowth(initData(&d_minVolumeGrowth, "minVolumeGrowth",
                             "Minimum volume growth allowed for actuation. If no value is set by user, no \n"
                             "minimum will be considered. NB: this value has a dependancy with the time step \n"
                             "(volume/dt) in the dynamic case."))

    , d_eqVolumeGrowth(initData(&d_eqVolumeGrowth, Real(0.0), "eqVolumeGrowth",
                            "Equality constraint for the volume growth. \n"
                            "Solver will try to maintain the volume growth at this value.\n"
                            "If unspecified, no equality constraint will be considered."))

    , d_maxVolumeGrowthVariation(initData(&d_maxVolumeGrowthVariation, "maxVolumeGrowthVariation",
                             "Maximum volume growth variation allowed for actuation. If no value is set by user, no \n"
                             "maximum will be considered. NB: this value has a dependancy with the time step \n"
                             "(volume/dt) in the dynamic case."))

    , d_drawPressure(initData(&d_drawPressure, false, "drawPressure",
                               "Visualization of the value of pressure. \n"
                               "If unspecified, the default value is {false}"))

    , d_drawScale(initData(&d_drawScale, Real(0.1), "drawScale",
                               "Scale for visualization. If unspecified the default value is {0.1}"))

{
    setUpData();
}

template<class DataTypes>
SurfacePressureModel<DataTypes>::~SurfacePressureModel()
{
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::setUpData()
{
    d_cavityVolume.setReadOnly(true);
    d_initialCavityVolume.setReadOnly(true);

    d_pressure.setGroup("Vector");
    d_volumeGrowth.setGroup("Vector");
    d_pressure.setReadOnly(true);
    d_volumeGrowth.setReadOnly(true);

    d_drawPressure.setGroup("Visualization");
    d_drawScale.setGroup("Visualization");
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    SoftRobotsConstraint<DataTypes>::init();

    internalInit();

    // Compute initial cavity volume and store it (used in reset())
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(VecCoordId::position());
    Real volume = getCavityVolume(positions.ref());
    d_initialCavityVolume.setValue(volume);
    d_cavityVolume.setValue(volume);

    d_componentState.setValue(ComponentState::Valid);
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::bwdInit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    // The initial volume is computed in bwdInit so the mapping (if there is any)
    // will be considered
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(VecCoordId::position());
    Real volume = getCavityVolume(positions.ref());
    d_initialCavityVolume.setValue(volume);
    d_cavityVolume.setValue(volume);
}


template<class DataTypes>
void SurfacePressureModel<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    internalInit();
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::reset()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    d_cavityVolume.setValue(d_initialCavityVolume.getValue());
    d_pressure.setValue(0.0);
    d_volumeGrowth.setValue(0.0);
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::internalInit()
{
    if(m_state==nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. "
                           "The object is then deactivated. "
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }


    /// Check that the triangles and quads datafield contain something, otherwise get context
    /// topology
    if(d_triangles.getValue().size() == 0 && d_quads.getValue().size() == 0)
    {
        msg_info() <<"No triangles and quads given. Get context topology.";
        BaseMeshTopology* topology = getContext()->getMeshTopology();

        if(topology==nullptr)
        {
            msg_error() << "There is no topology state associated with this node. "
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
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    int numTris = d_triangles.getValue().size() ;
    auto triangles = d_triangles.getValue() ;
    for(int i=0;i<numTris;i++){
        for(int j=0;j<3;j++){
            if( triangles[i][j] >= positions.size() )
                msg_error() << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                                   <<". is too large regarding mechanicalState size of(" << positions.size() << ")" ;
        }
    }

    /// Check that the quads datafield does not contains indices that would crash the
    /// component.
    int numQuads = d_quads.getValue().size() ;
    auto quads = d_quads.getValue() ;
    for(int i=0;i<numQuads;i++){
        for(int j=0;j<4;j++){
            if( quads[i][j] >= positions.size() )
                msg_error() << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                                   << " is too large regarding mechanicalState size of("
                                   << positions.size() << ")" ;
        }
    }
}


template<class DataTypes>
SReal SurfacePressureModel<DataTypes>::getCavityVolume(const VecCoord& positions)
{
    // NB : The computation  of the cavity volume is not relevant
    // in the case of an open mesh.

    ReadAccessor<Data<vector<Triangle>>>  triangles = d_triangles;
    ReadAccessor<Data<vector<Quad>>>      quads     = d_quads;

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
void SurfacePressureModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                            DataMatrixDeriv &cMatrix,
                                                            unsigned int &cIndex,
                                                            const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    m_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    ReadAccessor<Data<vector<Quad>>>     quadList = d_quads;
    ReadAccessor<Data<vector<Triangle>>> triList  = d_triangles;

    MatrixDeriv& matrix = *cMatrix.beginEdit();
    matrix.begin();
    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);

    cIndex++;

    VecCoord positions = x.getValue();
    for (Quad quad :  quadList)
    {
        Deriv triangle1Normal = cross(positions[quad[1]] - positions[quad[0]], positions[quad[3]] - positions[quad[0]])/2.0;
        Deriv triangle2Normal = cross(positions[quad[3]] - positions[quad[2]], positions[quad[1]] - positions[quad[2]])/2.0;
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
        Deriv triangleNormal = cross(positions[triangle[1]]- positions[triangle[0]], positions[triangle[2]] -positions[triangle[0]])/2.0;
        if(d_flipNormal.getValue())
            triangleNormal = -triangleNormal;

        for (unsigned i=0; i<3; i++)
        {
            rowIterator.addCol(triangle[i], triangleNormal*(1.0/3.0));
        }
    }

    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;
}


template<class DataTypes>
void SurfacePressureModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                             BaseVector *resV,
                                                             const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    d_cavityVolume.setValue(getCavityVolume(m_state->readPositions().ref()));
    Real dfree = Jdx->element(0) + d_cavityVolume.getValue() - d_initialCavityVolume.getValue();
    resV->set(m_constraintIndex.getValue(), dfree);
}


template<class DataTypes>
void SurfacePressureModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                  sofa::core::MultiVecDerivId res,
                                                  const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
            return ;
    
    d_pressure.setValue(lambda->element(m_constraintIndex.getValue()));

    // Compute actual cavity volume and volume growth from updated positions of mechanical
    // Eulalie.C: For now the position of the mechanical state is not up to date when storeLambda() is called
    //            so the value of delta is one step behind...
    //ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    //d_cavityVolume.setValue(getCavityVolume(positions.ref()));
    d_volumeGrowth.setValue(d_cavityVolume.getValue()-d_initialCavityVolume.getValue());
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if (d_drawPressure.getValue()) drawValue(vparams);
    if (!vparams->displayFlags().getShowInteractionForceFields()) return;

    float red, green, blue;

    if (d_pressure.getValue() > 0)
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
void SurfacePressureModel<DataTypes>::drawValue(const sofa::core::visual::VisualParams* vparams)
{
    float scale = (float)( ( vparams->sceneBBox().maxBBox() - vparams->sceneBBox().minBBox() ).norm() * d_drawScale.getValue() );
    string value = getValueString(d_pressure.getValue());
    vparams->drawTool()->draw3DText(vparams->sceneBBox().maxBBox(),scale,RGBAColor(1.,1.,1.,1.),value.c_str());
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
    ReadAccessor<Data<VecCoord>> x = m_state->readPositions();
    ReadAccessor<Data<vector<Quad>>>   quadList = d_quads;

    vector<Vec3> points(quadList.size()*4);
    for (unsigned int i =0; i<quadList.size(); i++)
    {
        Quad quad = quadList[i];
        points[i*4]  =x[quad[0]];
        points[i*4+1]=x[quad[1]];
        points[i*4+2]=x[quad[2]];
        points[i*4+3]=x[quad[3]];
    }

    vparams->drawTool()->drawQuads(points, RGBAColor(red,green,blue,1));
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::drawTriangles(const VisualParams* vparams, float red, float green, float blue)
{
    ReadAccessor<Data<VecCoord>> x = m_state->readPositions();
    ReadAccessor<Data<vector<Triangle>>> triList  = d_triangles;

    vector<Vec3> points(triList.size()*3);
    for (unsigned int i =0; i<triList.size(); i++)
    {
        Triangle tri = triList[i];
        points[i*3]  =x[tri[0]];
        points[i*3+1]=x[tri[1]];
        points[i*3+2]=x[tri[2]];
    }

    vparams->drawTool()->drawTriangles(points, RGBAColor(red,green,blue,1));
}

template<class DataTypes>
void SurfacePressureModel<DataTypes>::drawLines(const VisualParams* vparams, float red, float green, float blue)
{
    ReadAccessor<Data<VecCoord>> x = m_state->readPositions();

    for (Edge e : m_edges)
    {
        vparams->drawTool()->drawLine(x[e[0]], x[e[1]], RGBAColor(red,green,blue,1));
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

} // namespace

