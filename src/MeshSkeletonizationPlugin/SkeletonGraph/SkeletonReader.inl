#pragma once
#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonReader.h>
using namespace sofa::core::objectmodel;


namespace meshskeletonizationplugin
{


template <class DataTypes>
SkeletonReader<DataTypes>::SkeletonReader()
    : d_inSkeletonFilename(initData(&d_inSkeletonFilename, "filename", "Skeleton polyline file to read (e.g. skeleton.txt)"))
    , d_inVertices(initData(&d_inVertices, "inputVertices", "Optional input mesh vertices, to link skeleton nodes to the mesh"))
    , d_inEntryPoint(initData(&d_inEntryPoint, Vec3(0, 0, 0), "entryPoint", "Approx. entry point; closest node becomes the tree root"))
    , d_outVTKFilename(initData(&d_outVTKFilename, "outputVTK", "File path to export the rooted tree (.vtk)"))
    , d_outReportFilename(initData(&d_outReportFilename, "outputReport", "File path to export a per-node CSV report (id, x, y, z, parentId, childrenIds, pathFromRoot)"))
    , d_outNodeCount(initData(&d_outNodeCount, 0, "nodeCount", "Number of skeleton nodes read"))
{
    addInput(&d_inSkeletonFilename);
    addInput(&d_inVertices);
    addInput(&d_inEntryPoint);


    addOutput(&d_outVTKFilename);
    addOutput(&d_outReportFilename);
    addOutput(&d_outNodeCount);
}


template <class DataTypes>
void SkeletonReader<DataTypes>::init()
{
    if (d_inSkeletonFilename.getValue().empty())
    {
        msg_error() << "No input skeleton file set, please set the 'filename' data.";
        d_componentState.setValue(ComponentState::Invalid);
        return;
    }
    update();
}


template <class DataTypes>
void SkeletonReader<DataTypes>::doUpdate()
{
    if (d_inSkeletonFilename.getFullPath().empty()){
        d_componentState.setValue(ComponentState::Invalid);
        return;
    }

    if (!m_graph.loadFromFile(d_inSkeletonFilename.getFullPath()))
    {
       
        msg_error() << "Could not open skeleton file: " << d_inSkeletonFilename.getFullPath();
        d_componentState.setValue(ComponentState::Invalid);
        return;
    }


    msg_info() << "Skeleton loaded: " << m_graph.nodes().size() << " node(s).";
    d_outNodeCount.setValue(static_cast<int>(m_graph.nodes().size()));


    const Vec3& entry = d_inEntryPoint.getValue();
    m_graph.buildTree({ double(entry[0]), double(entry[1]), double(entry[2]) });


    if (!m_graph.hasRoot())
    {


        msg_error() << "No root could be established from: " << d_inSkeletonFilename.getFullPath();
        d_componentState.setValue(ComponentState::Invalid);
        return;
    }
    if (m_graph.hasRoot())
        msg_info() << "Tree built, root id " << m_graph.rootId()
                    << ", " << m_graph.loopEdges().size() << " loop edge(s) detected.";


    if (!d_inVertices.getValue().empty())
    {
        std::vector<std::array<double, 3>> meshVerts;
        meshVerts.reserve(d_inVertices.getValue().size());
        for (const auto& v : d_inVertices.getValue())
            meshVerts.push_back({ double(v[0]), double(v[1]), double(v[2]) });
        m_graph.computeMeshCorrespondence(meshVerts);
    }


    if (d_outVTKFilename.isSet())
        m_graph.exportToVTK(d_outVTKFilename.getFullPath());
   
    if (d_outReportFilename.isSet())
        m_graph.exportReportCSV(d_outReportFilename.getFullPath());
}




template <class DataTypes>
void SkeletonReader<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
    using Color = sofa::type::RGBAColor;
    std::vector< type::Vec3 > dvec;
   
   
    for (const SkeletonNode& node : m_graph.nodes())
    {
        for (int childId : node.childrenIds())
        {
            const SkeletonNode& child = m_graph.nodes()[childId];
            const auto& p0 = node.position();
            const auto& p1 = child.position();


            dvec.emplace_back(Coord(p0[0], p0[1], p0[2]));
            dvec.emplace_back(Coord(p1[0], p1[1], p1[2]));


            vparams->drawTool()->drawLines(dvec, 2, Color::blue());
            dvec.clear();
        }
    }
}


} // namespace meshskeletonizationplugin