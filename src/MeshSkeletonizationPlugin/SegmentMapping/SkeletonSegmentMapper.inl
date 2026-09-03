#pragma once
#include <MeshSkeletonizationPlugin/SegmentMapping/SkeletonSegmentMapper.h>

// Reuses the Kernel/Polyhedron/Point typedefs already declared (at global
// scope) in MeshSkeletonization.h, so segment meshes are built the same way
// MeshSkeletonization builds its own polyhedron - and the shared closed-mesh
// query helper (also used by TumorCutPointSelector) so this doesn't keep its
// own private copy of the same CGAL polyhedron/AABB-tree plumbing.
#include <MeshSkeletonizationPlugin/MeshSkeletonization.h>
#include <MeshSkeletonizationPlugin/CGALMeshUtils.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/accessor.h>
#include <sofa/type/RGBAColor.h>
#include <limits>
#include <memory>
#include <set>

namespace meshskeletonizationplugin
{

template <class DataTypes>
SkeletonSegmentMapper<DataTypes>::SkeletonSegmentMapper()
    : l_skeletonReader(initLink("skeletonReader",
        "SkeletonReader whose graph should be segmented; if empty, the first "
        "SkeletonReader found in the scene is used"))
    , l_segmentMeshes(initLink("segmentMeshes",
        "One closed mesh loader per liver segment (e.g. Couinaud collision meshes), "
        "in the same order as segmentNames"))
    , d_inSegmentNames(initData(&d_inSegmentNames, "segmentNames",
        "Human-readable name per entry of segmentMeshes, e.g. 'II', 'IVa', 'VIII'"))
    , d_outSegmentReportFilename(initData(&d_outSegmentReportFilename, "outSegmentReportFilename",
        "Optional path to export a CSV report: id, x, y, z, segment name"))
    , d_outNodeSegments(initData(&d_outNodeSegments, "nodeSegments",
        "Segment index per skeleton node (-1 if unknown), indexed like the graph's nodes"))
    , d_outNodeSegmentNames(initData(&d_outNodeSegmentNames, "nodeSegmentNames",
        "Same as nodeSegments, resolved to names ('unknown' if -1)"))
    , d_outSegmentCount(initData(&d_outSegmentCount, 0, "segmentCount",
        "Number of distinct segments found (excluding -1/unknown)"))
{
    addOutput(&d_outNodeSegments);
    addOutput(&d_outNodeSegmentNames);
    addOutput(&d_outSegmentCount);
}

template <class DataTypes>
void SkeletonSegmentMapper<DataTypes>::init()
{
    if (!l_skeletonReader)
    {
        l_skeletonReader.set(this->getContext()->template get<SkeletonReader<DataTypes>>());
    }
    if (!l_skeletonReader)
    {
        msg_error() << "No SkeletonReader<DataTypes> found. Set 'skeletonReader' to a valid "
                        "component path, or add one earlier in the scene.";
    }

    if (l_segmentMeshes.empty())
    {
        msg_error() << "No segment meshes linked: set 'segmentMeshes' to a list of mesh "
                        "loader paths (one per liver segment).";
    }
    else if (l_segmentMeshes.size() != d_inSegmentNames.getValue().size())
    {
        msg_warning() << "segmentMeshes (" << l_segmentMeshes.size() << ") and segmentNames ("
                       << d_inSegmentNames.getValue().size() << ") have different sizes; "
                          "unnamed segments will be reported by index only.";
    }

    setDirtyValue();
    update(); // compute eagerly, don't wait for something to read an output Data
}

template <class DataTypes>
void SkeletonSegmentMapper<DataTypes>::doUpdate()
{
    if (!l_skeletonReader || l_segmentMeshes.empty())
        return;

    // Work on our own copy of the reader's graph so we don't need a
    // non-const accessor on SkeletonReader just for this.
    m_graph = l_skeletonReader->graph();

    // Build one point-in-mesh query per linked segment mesh.
    std::vector<cgalutils::ClosedMeshQuery> segmentQueries(l_segmentMeshes.size());
    for (std::size_t i = 0; i < l_segmentMeshes.size(); ++i)
    {
        sofa::core::loader::MeshLoader* loader = l_segmentMeshes.get(i);
        if (!loader)
            continue;

        segmentQueries[i].buildFrom(loader->d_positions.getValue(), loader->d_triangles.getValue());
    }

    // Raw per-node label: which segment mesh contains this node, or (if none
    // does) whichever segment mesh's surface is closest.
    const auto& nodes = m_graph.nodes();
    std::vector<int> raw(nodes.size(), -1);

    for (const SkeletonNode& node : nodes)
    {
        const auto& p = node.position();
        Point query(p[0], p[1], p[2]);

        int inside = -1;
        for (std::size_t i = 0; i < segmentQueries.size() && inside == -1; ++i)
        {
            if (segmentQueries[i].insideTest &&
                (*segmentQueries[i].insideTest)(query) == CGAL::ON_BOUNDED_SIDE)
            {
                inside = static_cast<int>(i);
            }
        }

        if (inside != -1)
        {
            raw[node.id()] = inside;
            continue;
        }

        // Fallback: closest segment surface.
        double bestDist2 = std::numeric_limits<double>::max();
        int bestSeg = -1;
        for (std::size_t i = 0; i < segmentQueries.size(); ++i)
        {
            if (!segmentQueries[i].tree)
                continue;
            double d2 = CGAL::to_double(segmentQueries[i].tree->squared_distance(query));
            if (d2 < bestDist2)
            {
                bestDist2 = d2;
                bestSeg = static_cast<int>(i);
            }
        }
        raw[node.id()] = bestSeg;
    }

    m_graph.assignSegmentLabels(raw);

    const auto& names = d_inSegmentNames.getValue();

    sofa::helper::WriteAccessor<sofa::core::objectmodel::Data<sofa::type::vector<int>>> outSegments(d_outNodeSegments);
    sofa::helper::WriteAccessor<sofa::core::objectmodel::Data<sofa::type::vector<std::string>>> outSegmentNames(d_outNodeSegmentNames);
    outSegments.resize(nodes.size());
    outSegmentNames.resize(nodes.size());

    std::set<int> distinct;
    for (const SkeletonNode& n : nodes)
    {
        int seg = m_graph.segmentOf(n.id());
        outSegments[n.id()] = seg;
        outSegmentNames[n.id()] = (seg >= 0 && seg < static_cast<int>(names.size())) ? names[seg] : "unknown";
        if (seg != -1)
            distinct.insert(seg);
    }

    d_outSegmentCount.setValue(static_cast<int>(distinct.size()));

    if (d_outSegmentReportFilename.isSet() && !d_outSegmentReportFilename.getFullPath().empty())
    {
        m_graph.exportSegmentReportCSV(d_outSegmentReportFilename.getFullPath(), names);
    }
}

template <class DataTypes>
void SkeletonSegmentMapper<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    static const std::vector<sofa::type::RGBAColor> palette = {
        sofa::type::RGBAColor(0.90f, 0.10f, 0.10f, 1.0f),
        sofa::type::RGBAColor(0.10f, 0.60f, 0.90f, 1.0f),
        sofa::type::RGBAColor(0.20f, 0.80f, 0.20f, 1.0f),
        sofa::type::RGBAColor(0.95f, 0.75f, 0.10f, 1.0f),
        sofa::type::RGBAColor(0.60f, 0.20f, 0.80f, 1.0f),
        sofa::type::RGBAColor(0.90f, 0.45f, 0.10f, 1.0f),
        sofa::type::RGBAColor(0.10f, 0.80f, 0.70f, 1.0f),
        sofa::type::RGBAColor(0.50f, 0.50f, 0.50f, 1.0f),
    };
    static const sofa::type::RGBAColor unknownColor(0.7f, 0.7f, 0.7f, 1.0f);

    std::vector<sofa::type::Vec3> points;
    std::vector<sofa::type::RGBAColor> colors;
    points.reserve(m_graph.nodes().size());
    colors.reserve(m_graph.nodes().size());

    for (const SkeletonNode& n : m_graph.nodes())
    {
        const auto& p = n.position();
        points.emplace_back(p[0], p[1], p[2]);

        int seg = m_graph.segmentOf(n.id());
        colors.push_back(seg >= 0 ? palette[seg % palette.size()] : unknownColor);
    }

    vparams->drawTool()->drawPoints(points, 6.0f, colors);
}

} // namespace meshskeletonizationplugin
