#pragma once
#include <MeshSkeletonizationPlugin/TumorMapping/TumorCutPointSelector.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/accessor.h>
#include <sofa/type/RGBAColor.h>

#include <limits>
#include <set>

namespace meshskeletonizationplugin
{

template <class DataTypes>
TumorCutPointSelector<DataTypes>::TumorCutPointSelector()
    : l_tumorMeshes(initLink("tumorMeshes",
        "One mesh loader per tumor, already separate"))
    , l_segmentMapper(initLink("segmentMapper",
        "Segmented skeleton to test the tumor(s) against; if empty, the first "
        "SkeletonSegmentMapper found in the scene is used"))
    , d_inTumorNames(initData(&d_inTumorNames, "tumorNames",
        "Optional, same order as tumorMeshes, e.g. 'tumor 1', 'tumor 2' - for readable output/reports"))
    , d_outTumorCount(initData(&d_outTumorCount, 0, "tumorCount",
        "Number of linked tumors"))
    , d_outCutNodeIds(initData(&d_outCutNodeIds, "cutNodeIds",
        "Union of every touching node across all tumors - link this into SkeletonResectionSimulator.cutNodeIds"))
    , d_outTouchingTumorIndex(initData(&d_outTouchingTumorIndex, "touchingTumorIndex",
        "Same length/order as cutNodeIds: which tumor (0-based, into tumorMeshes) each entry belongs to"))
    , d_outUntouchedTumorIds(initData(&d_outUntouchedTumorIds, "untouchedTumorIds",
        "0-based indices of tumors touching no skeleton node at all"))
{
    addOutput(&d_outTumorCount);
    addOutput(&d_outCutNodeIds);
    addOutput(&d_outTouchingTumorIndex);
    addOutput(&d_outUntouchedTumorIds);
}

template <class DataTypes>
void TumorCutPointSelector<DataTypes>::init()
{
    if (!l_segmentMapper)
        l_segmentMapper.set(this->getContext()->template get<SkeletonSegmentMapper<DataTypes>>());

    if (l_tumorMeshes.empty())
        msg_error() << "No tumor meshes linked. Set 'tumorMeshes' to a list of MeshLoader component paths.";
    if (!l_segmentMapper)
        msg_error() << "No SkeletonSegmentMapper<DataTypes> found. Set 'segmentMapper' to a "
                        "valid component path, or add one earlier in the scene.";
    else if (!l_tumorMeshes.empty() && d_inTumorNames.getValue().size() != l_tumorMeshes.size())
        msg_warning() << "tumorMeshes (" << l_tumorMeshes.size() << ") and tumorNames ("
                       << d_inTumorNames.getValue().size() << ") have different sizes; "
                          "unnamed tumors will be reported by index only.";

    setDirtyValue();
    update(); // compute eagerly, don't wait for something to read an output Data
}

template <class DataTypes>
void TumorCutPointSelector<DataTypes>::rebuildTumors()
{
    m_tumors.clear();
    m_tumors.resize(l_tumorMeshes.size());

    for (std::size_t i = 0; i < l_tumorMeshes.size(); ++i)
    {
        sofa::core::loader::MeshLoader* loader = l_tumorMeshes.get(i);
        if (!loader)
            continue;

        const auto& positions = loader->d_positions.getValue();
        const auto& triangles = loader->d_triangles.getValue();

        TumorQuery& tumor = m_tumors[i];
        tumor.bboxMin = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
        tumor.bboxMax = { -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max() };
        for (const auto& v : positions)
        {
            for (int d = 0; d < 3; ++d)
            {
                tumor.bboxMin[d] = std::min(tumor.bboxMin[d], static_cast<double>(v[d]));
                tumor.bboxMax[d] = std::max(tumor.bboxMax[d], static_cast<double>(v[d]));
            }
        }

        tumor.query.buildFrom(positions, triangles);
    }
}

template <class DataTypes>
void TumorCutPointSelector<DataTypes>::doUpdate()
{
    if (l_tumorMeshes.empty() || !l_segmentMapper)
        return;

    rebuildTumors();
    m_graph = l_segmentMapper->graph();

    std::vector<int> cutNodeIds;
    std::vector<int> touchingTumorIndex;
    std::vector<bool> tumorTouched(m_tumors.size(), false);

    for (const SkeletonNode& node : m_graph.nodes())
    {
        const auto& p = node.position();

        for (std::size_t t = 0; t < m_tumors.size(); ++t)
        {
            TumorQuery& tumor = m_tumors[t];

            // Broad phase: cheap bounding-box rejection.
            bool insideBBox = true;
            for (int d = 0; d < 3; ++d)
            {
                if (p[d] < tumor.bboxMin[d] || p[d] > tumor.bboxMax[d])
                {
                    insideBBox = false;
                    break;
                }
            }
            if (!insideBBox)
                continue;

            // Narrow phase: real point-in-mesh test against the tumor surface.
            if (!tumor.query.insideTest)
                continue;

            Point query(p[0], p[1], p[2]);
            if ((*tumor.query.insideTest)(query) == CGAL::ON_BOUNDED_SIDE)
            {
                cutNodeIds.push_back(node.id());
                touchingTumorIndex.push_back(static_cast<int>(t));
                tumorTouched[t] = true;
                tumor.touchingNodeIds.push_back(node.id());
            }
        }
    }

    std::vector<int> untouchedTumorIds;
    for (std::size_t t = 0; t < m_tumors.size(); ++t)
        if (!tumorTouched[t])
            untouchedTumorIds.push_back(static_cast<int>(t));

    d_outTumorCount.setValue(static_cast<int>(m_tumors.size()));
    d_outCutNodeIds.setValue(cutNodeIds);
    d_outTouchingTumorIndex.setValue(touchingTumorIndex);
    d_outUntouchedTumorIds.setValue(untouchedTumorIds);

    if (!untouchedTumorIds.empty())
    {
        msg_warning() << untouchedTumorIds.size() << " of " << m_tumors.size()
                      << " tumor(s) touch no skeleton node directly (Case 2 - nearest node "
                         "in the tumor's segment - is not yet implemented).";
    }
}

template <class DataTypes>
void TumorCutPointSelector<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    static const sofa::type::RGBAColor tumorBoxColor(1.0f, 1.0f, 1.0f, 1.0f);
    static const sofa::type::RGBAColor touchingNodeColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Tumor bounding boxes, wireframe - a visual sanity check that the
    // bbox broad-phase looks right.
    for (const TumorQuery& tumor : m_tumors)
    {
        const auto& lo = tumor.bboxMin;
        const auto& hi = tumor.bboxMax;
        std::vector<sofa::type::Vec3> corners = {
            {lo[0], lo[1], lo[2]}, {hi[0], lo[1], lo[2]}, {hi[0], hi[1], lo[2]}, {lo[0], hi[1], lo[2]},
            {lo[0], lo[1], hi[2]}, {hi[0], lo[1], hi[2]}, {hi[0], hi[1], hi[2]}, {lo[0], hi[1], hi[2]},
        };
        static const int edges[12][2] = {
            {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {0,4},{1,5},{2,6},{3,7}
        };
        std::vector<sofa::type::Vec3> lines;
        lines.reserve(24);
        for (auto& e : edges)
        {
            lines.push_back(corners[e[0]]);
            lines.push_back(corners[e[1]]);
        }
        vparams->drawTool()->drawLines(lines, 1.5f, tumorBoxColor);
    }

    // Touching skeleton nodes, highlighted.
    std::vector<sofa::type::Vec3> touchingPoints;
    for (const TumorQuery& tumor : m_tumors)
    {
        for (int id : tumor.touchingNodeIds)
        {
            const SkeletonNode* n = m_graph.node(id);
            if (!n)
                continue;
            const auto& p = n->position();
            touchingPoints.emplace_back(p[0], p[1], p[2]);
        }
    }
    if (!touchingPoints.empty())
        vparams->drawTool()->drawPoints(touchingPoints, 9.0f, touchingNodeColor);
}

} // namespace meshskeletonizationplugin
