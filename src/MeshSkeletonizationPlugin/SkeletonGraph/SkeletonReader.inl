#pragma once

#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonReader.h>

using namespace sofa::core::objectmodel;

namespace meshskeletonizationplugin
{
template <class DataTypes>
SkeletonReader<DataTypes>::SkeletonReader()
    : d_inSkeletonFilename(initData(&d_inSkeletonFilename, "filename", "Skeleton polyline file to read (e.g. skeleton.txt)"))
    , d_inVertices(initData(&d_inVertices, "inputVertices", "Optional input mesh vertices, to link skeleton nodes to the mesh"))
    , d_inEntryPoint(initData(&d_inEntryPoint, Vec3(0, 0, 0), "entryPoint", "Approx. entry point; closest node becomes the tree root. If left unset, a root is instead picked automatically from the largest connected component of the skeleton."))
    , d_outVTKFilename(initData(&d_outVTKFilename, "outputVTK", "File path to export the rooted tree (.vtk)"))
    , d_outReportFilename(initData(&d_outReportFilename, "outputReport", "File path to export a per-node CSV report (id, x, y, z, parentId, childrenIds, pathFromRoot)"))
    , d_outNodeCount(initData(&d_outNodeCount, 0, "nodeCount", "Number of skeleton nodes read"))
    , d_outPositions(initData(&d_outPositions, "positions", "Flat vertex positions, one per skeleton node, indexed like graph().nodes()'"))
    , d_outEdges(initData(&d_outEdges, "edges", "Vector of edges representing the Skeleton Graph"))
{
    addInput(&d_inSkeletonFilename);
    addInput(&d_inVertices);
    addInput(&d_inEntryPoint);
    addOutput(&d_outVTKFilename);
    addOutput(&d_outReportFilename);
    addOutput(&d_outNodeCount);
    addOutput(&d_outPositions);
    addOutput(&d_outEdges);
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
    if (d_inSkeletonFilename.getFullPath().empty())
    {
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

    if (d_inEntryPoint.isSet())
    {
        const Vec3& entry = d_inEntryPoint.getValue();
        m_graph.buildTree({ double(entry[0]), double(entry[1]), double(entry[2]) });
        msg_info() << "Rooted using explicit entryPoint " << entry << ".";
    }
    else
    {
        // No explicit entry point given - don't silently root the tree at
        // whatever's nearest the origin (which can land in a small,
        // disconnected fragment far from the real vessel trunk, since real
        // coordinates are almost never near (0,0,0)). Pick a root from the
        // largest connected component instead.
        m_graph.buildTreeAutoRoot();
        msg_info() << "No entryPoint set; auto-rooted from the largest connected component.";
    }

    if (!m_graph.hasRoot())
    {
        // The file parsed but produced no usable rooted tree, so every
        // downstream output would be empty.
        msg_error() << "No root could be established from: " << d_inSkeletonFilename.getFullPath();
        d_componentState.setValue(ComponentState::Invalid);
        return;
    }

    msg_info() << "Tree built, root id " << m_graph.rootId()
               << ", " << m_graph.loopEdges().size() << " loop edge(s) detected.";

    // Flat vertices/edges for a MechanicalObject + EdgeSetTopologyContainer
    // to consume directly (e.g. to barycentric-map the skeleton onto the
    // liver so it deforms with it) - same convention as MeshOBJLoader's
    // d_positions/d_edges. Node id == vector index throughout, so these
    // stay index-aligned with graph().nodes() and every other per-node
    // output (affectedNodeIds, nodeSegments, ...).
    {
        const auto& nodes = m_graph.nodes();

        VecCoord positions;
        positions.resize(nodes.size());
        for (const SkeletonNode& n : nodes)
        {
            const auto& p = n.position();
            positions[n.id()] = Coord(p[0], p[1], p[2]);
        }
        d_outPositions.setValue(positions);

        SeqEdges edges;
        for (const SkeletonNode& n : nodes)
            for (int parentId : n.parentIds())
                edges.push_back(Edge(parentId, n.id()));
        d_outEdges.setValue(edges);

        msg_info() << "Exported " << positions.size() << " position(s) and "
                   << edges.size() << " edge(s) for topology/mechanical binding.";
    }

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

    d_componentState.setValue(ComponentState::Valid);
}


template <class DataTypes>
void SkeletonReader<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (d_componentState.getValue() != ComponentState::Valid)
        return;

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
