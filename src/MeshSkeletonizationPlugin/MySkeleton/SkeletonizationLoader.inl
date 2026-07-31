#pragma once

#include <MeshSkeletonizationPlugin/MySkeleton/SkeletonizationLoader.h>

using namespace sofa::core::objectmodel;

namespace meshskeletonizationplugin
{

template <class DataTypes>
SkeletonizationLoader<DataTypes>::SkeletonizationLoader()
    : d_inVertices(initData (&d_inVertices, "inputVertices", "List of input mesh vertices"))
    , d_inTriangles(initData(&d_inTriangles, "inputTriangles", "List of input mesh triangles"))
    , d_outSkeletonFilename(initData(&d_outSkeletonFilename, "outputSkeleton", "File path to output skeleton file"))
    , d_inMeshFilename(initData (&d_inMeshFilename, "filename", "Input mesh in .off format")) 
    , d_inEntryPoint(initData(&d_inEntryPoint, Vec3(0,0,0), "entryPoint", "Approx. entry point; closest skeleton vertex becomes the tree root"))
    , d_outTreeFilename(initData(&d_outTreeFilename, "outputTree", "File path to export the rooted tree (SWC)"))
{
    addInput(&d_inVertices);
    addInput(&d_inTriangles);
    addInput(&d_inMeshFilename);
    addInput(&d_inEntryPoint);

    addOutput(&d_outSkeletonFilename);
    addOutput(&d_outTreeFilename);
}


template <class DataTypes>
void SkeletonizationLoader<DataTypes>::init() 
{
    //Input
    if(d_outSkeletonFilename.getValue().empty())
    {
        msg_error() << "No input File to store the skeleton data, please set a inputFile path.";
        return;
    }
}


template <class DataTypes>
void SkeletonizationLoader<DataTypes>::geometryToPolyhedron(Polyhedron &s)
{
    VecCoord inVertices = d_inVertices.getValue();
    SeqTriangles inTriangles = d_inTriangles.getValue();

    geometryToPolyhedronOp<HalfedgeDS> gen(inVertices, inTriangles);
    s.delegate(gen);
}


template <class DataTypes>
void SkeletonizationLoader<DataTypes>::doUpdate() 
{
    Polyhedron tmesh;

    if(d_inMeshFilename.getFullPath() != "") 
    {
        const char* filename = d_inMeshFilename.getFullPath().c_str();
        std::ifstream input(filename);
        input >> tmesh;
        msg_info() << "Loading Polyhedron from file.";
    } 
    else 
    {
        geometryToPolyhedron(tmesh);
        msg_info() << "Number of vertices of the input mesh: " << boost::num_vertices(tmesh);
    }

    if (!CGAL::is_triangle_mesh(tmesh))
    {
        msg_error() << "Input geometry is not triangulated.";
        return;	
    }
        
    CGAL::extract_mean_curvature_flow_skeleton(tmesh, m_skeleton);
    msg_info() << "Number of vertices of the output skeleton: " << boost::num_vertices(m_skeleton);
    msg_info() << "Number of edges of the output skeleton: " << boost::num_edges(m_skeleton);

    if (d_outSkeletonFilename.isSet())
    {
        std::ofstream OutputP(d_outSkeletonFilename.getFullPath(), std::ofstream::out | std::ofstream::trunc);
        Export_polylines extractor(m_skeleton, OutputP);
        CGAL::split_graph_into_polylines(m_skeleton, extractor);
        OutputP.close();
    }

    auto root = m_vesselTree.findClosestVertex(m_skeleton, d_inEntryPoint.getValue());
    m_vesselTree.build(m_skeleton, root);
    msg_info() << "Vessel tree built: " << m_vesselTree.parents().size() + 1
               << " node(s) reached, " << m_vesselTree.loopEdges().size() / 2
               << " loop edge(s) detected.";

    if (d_outTreeFilename.isSet())
        m_vesselTree.exportToSWC(m_skeleton, d_outTreeFilename.getFullPath());
}


template <class DataTypes>
void SkeletonizationLoader<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams) 
{
    using Color = sofa::type::RGBAColor;
    std::vector< type::Vec3 > dvec;

    for(const Skeleton_edge& e : CGAL::make_range(edges(m_skeleton))) 
    {
        const Point& s = m_skeleton[source(e, m_skeleton)].point;
        const Point& t = m_skeleton[target(e, m_skeleton)].point;

                
        dvec.emplace_back(Coord(s[0], s[1], s[2]));
        dvec.emplace_back(Coord(t[0], t[1], t[2]));

        vparams->drawTool()->drawLines( dvec, 40, Color::red());
        dvec.clear();
    } 
}

} // namespace meshskeletonizationplugin