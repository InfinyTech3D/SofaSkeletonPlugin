#pragma once
#include <MeshSkeletonizationPlugin/SkeletonResection/SkeletonResectionSimulator.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/accessor.h>
#include <sofa/type/RGBAColor.h>

#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <algorithm>
#include <iterator>

namespace meshskeletonizationplugin
{

template <class DataTypes>
SkeletonResectionSimulator<DataTypes>::SkeletonResectionSimulator()
    : l_segmentMapper(initLink("segmentMapper",
        "Segmented skeleton to run resection scenarios against; if empty, the first "
        "SkeletonSegmentMapper found in the scene is used"))
    , l_segmentVisualModels(initLink("segmentVisualModels",
        "Optional: the visual object (e.g. OglModel) for each segment, same order as "
        "segmentNames. Its 'color' Data is pushed directly every frame - more reliable "
        "than linking a color attribute to segmentColors[i] in the scene."))
    , d_inCutNodeIds(initData(&d_inCutNodeIds, "cutNodeIds",
        "Node id(s) where the vessel is (candidate) severed"))
    , d_activationDelay(initData(&d_activationDelay, 5.0, "activationDelay",
        "Seconds after simulation start before affected nodes/segments actually switch "
        "color to red/black; everything renders as perfused (green) until then"))
    , d_outReportFilename(initData(&d_outReportFilename, "outReportFilename",
        "Optional path to export a CSV report (id, x, y, z, segment) of the affected nodes"))
    , d_outAffectedNodeIds(initData(&d_outAffectedNodeIds, "affectedNodeIds",
        "Node ids that would lose blood supply (includes the cut nodes themselves)"))
    , d_outAffectedNodeCount(initData(&d_outAffectedNodeCount, 0, "affectedNodeCount",
        "Convenience: size of affectedNodeIds"))
    , d_outAffectedSegmentIds(initData(&d_outAffectedSegmentIds, "affectedSegmentIds",
        "Distinct segment indices touched by the affected nodes"))
    , d_outAffectedSegmentNames(initData(&d_outAffectedSegmentNames, "affectedSegmentNames",
        "Same as affectedSegmentIds, resolved to names"))
    , d_outSegmentColors(initData(&d_outSegmentColors, "segmentColors",
        "One color per segment (same order as the linked mapper's segmentNames): green "
        "until activationDelay elapses, then red for affected segments. Link an OglModel's "
        "'color' Data to e.g. '@resectionSim.segmentColors[0]' for the first segment."))
{
    addInput(&d_inCutNodeIds);
    addOutput(&d_outAffectedNodeIds);
    addOutput(&d_outAffectedNodeCount);
    addOutput(&d_outAffectedSegmentIds);
    addOutput(&d_outAffectedSegmentNames);
    addOutput(&d_outSegmentColors);
}

template <class DataTypes>
void SkeletonResectionSimulator<DataTypes>::init()
{
    if (!l_segmentMapper)
    {
        l_segmentMapper.set(this->getContext()->template get<SkeletonSegmentMapper<DataTypes>>());
    }
    if (!l_segmentMapper)
    {
        msg_error() << "No SkeletonSegmentMapper<DataTypes> found. Set 'segmentMapper' to a "
                        "valid component path, or add one earlier in the scene.";
    }

    setDirtyValue();
    update(); // computes each time without waiting for something to read an output Data
}

template <class DataTypes>
void SkeletonResectionSimulator<DataTypes>::doUpdate()
{
    if (!l_segmentMapper)
        return;
    // Working on copy of the already segmented graph so the candidate is re-run per candidate without touching the mapper
    m_graph = l_segmentMapper->graph();

    const auto& cutIds = d_inCutNodeIds.getValue();
    std::vector<int> affected = m_graph.simulateResection(cutIds);
    std::vector<int> segments = m_graph.affectedSegments(affected);

    const auto& segmentNames = l_segmentMapper->d_inSegmentNames.getValue();
    std::vector<std::string> segmentNameStrs;
    segmentNameStrs.reserve(segments.size());
    for (int seg : segments)
        segmentNameStrs.push_back(seg >= 0 && seg < static_cast<int>(segmentNames.size())
            ? segmentNames[seg] : "unknown");

    d_outAffectedNodeIds.setValue(affected);
    d_outAffectedNodeCount.setValue(static_cast<int>(affected.size()));
    d_outAffectedSegmentIds.setValue(segments);
    d_outAffectedSegmentNames.setValue(segmentNameStrs);

    updateSegmentColors(); // initial colors; draw() keeps these fresh every frame after this

    if (d_outReportFilename.isSet() && !d_outReportFilename.getFullPath().empty())
    {
        std::ofstream out(d_outReportFilename.getFullPath(), std::ofstream::out | std::ofstream::trunc);
        out << "# cutNodeIds=" << cutIds.size()
            << ",affectedNodeCount=" << affected.size()
            << ",affectedSegmentCount=" << segments.size()
            << ",affectedSegments=";
        for (std::size_t i = 0; i < segmentNameStrs.size(); ++i)
            out << (i ? ";" : "") << segmentNameStrs[i];
        out << "\n";
        out << "id,x,y,z,segment\n";
        out << std::fixed << std::setprecision(6);
        for (int id : affected)
        {
            const SkeletonNode* n = m_graph.node(id);
            if (!n)
                continue;
            const auto& p = n->position();
            int seg = m_graph.segmentOf(id);
            std::string segName = (seg >= 0 && seg < static_cast<int>(segmentNames.size())) ? segmentNames[seg] : "unknown";
            out << id << "," << p[0] << "," << p[1] << "," << p[2] << "," << segName << "\n";
        }
    }
}

template <class DataTypes>
void SkeletonResectionSimulator<DataTypes>::updateSegmentColors()
{
    // colors of the segments before and after (green --> red)
    static const sofa::type::RGBAColor safeColor(0.20f, 0.80f, 0.20f, 1.0f);
    static const sofa::type::RGBAColor affectedColor(0.90f, 0.10f, 0.10f, 1.0f);

    m_colorsActive = (this->getContext()->getTime() >= d_activationDelay.getValue());

    if (!l_segmentMapper)
        return;

    // --- Diagnostics: run once, so we can see exactly what's wrong instead of silently doing nothing.
    if (!m_loggedVisualModelLinkStatus)
    {
        m_loggedVisualModelLinkStatus = true;
        if (l_segmentVisualModels.empty())
        {
            msg_warning() << "segmentVisualModels is empty: no visual objects linked, so no "
                              "mesh color will ever change. Check the 'segmentVisualModels' "
                              "attribute/paths in the scene.";
        }
        else
        {
            m_loggedMissingColorData.assign(l_segmentVisualModels.size(), false);
            for (std::size_t i = 0; i < l_segmentVisualModels.size(); ++i)
            {
                sofa::core::objectmodel::BaseObject* obj = l_segmentVisualModels.get(i);
                if (!obj)
                {
                    msg_warning() << "segmentVisualModels[" << i << "] did not resolve to any object "
                                     "(bad path?).";
                    continue;
                }
                msg_info() << "segmentVisualModels[" << i << "] resolved to '" << obj->getName()
                           << "' (" << obj->getClassName() << ").";
            }
        }
    }

    const auto& segmentNames = l_segmentMapper->d_inSegmentNames.getValue();
    const auto& segments = d_outAffectedSegmentIds.getValue();
    std::set<int> affectedSegSet(segments.begin(), segments.end());

    sofa::helper::WriteAccessor<sofa::core::objectmodel::Data<sofa::type::vector<sofa::type::RGBAColor>>> colors(d_outSegmentColors);
    colors.resize(segmentNames.size());
    for (std::size_t i = 0; i < segmentNames.size(); ++i)
        colors[i] = (m_colorsActive && affectedSegSet.count(static_cast<int>(i))) ? affectedColor : safeColor;

    // Push directly into each linked visual model's own "color" Data, if
    // provided - this is what actually makes the mesh repaint, since a
    // passive scene-level Data link to segmentColors[i] isn't guaranteed to
    // be re-read every frame by the visual model itself.
    for (std::size_t i = 0; i < l_segmentVisualModels.size() && i < colors.size(); ++i)
    {
        sofa::core::objectmodel::BaseObject* obj = l_segmentVisualModels.get(i);
        if (!obj)
            continue;

        // OglModel doesn't expose a plain "color" Data that is why this method was used
        sofa::core::objectmodel::BaseData* materialData = obj->findData("material");
        if (!materialData)
        {
            if (i < m_loggedMissingColorData.size() && !m_loggedMissingColorData[i])
            {
                m_loggedMissingColorData[i] = true;
                std::ostringstream fields;
                for (sofa::core::objectmodel::BaseData* d : obj->getDataFields())
                    fields << d->getName() << " ";
                msg_warning() << "'" << obj->getName() << "' (" << obj->getClassName()
                              << ") has no Data named 'material' either. Its actual Data fields are: "
                              << fields.str();
            }
            continue;
        }

        std::istringstream iss(materialData->getValueString());
        std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

        auto diffuseIt = std::find(tokens.begin(), tokens.end(), "Diffuse");
        if (diffuseIt == tokens.end() || std::distance(diffuseIt, tokens.end()) < 6)
        {
            if (i < m_loggedMissingColorData.size() && !m_loggedMissingColorData[i])
            {
                m_loggedMissingColorData[i] = true;
                msg_warning() << "Could not find a 'Diffuse <flag> r g b a' pattern in '"
                              << obj->getName() << "'s material string: " << materialData->getValueString();
            }
            continue;
        }

        const auto& c = colors[i];
        *(diffuseIt + 1) = "1"; // force useDiffuse on, so our color is actually applied
        std::ostringstream rs, gs, bs, as;
        rs << c[0]; gs << c[1]; bs << c[2]; as << c[3];
        *(diffuseIt + 2) = rs.str();
        *(diffuseIt + 3) = gs.str();
        *(diffuseIt + 4) = bs.str();
        *(diffuseIt + 5) = as.str();

        std::ostringstream newStr;
        for (std::size_t k = 0; k < tokens.size(); ++k)
            newStr << (k ? " " : "") << tokens[k];
        materialData->read(newStr.str());
    }
}

template <class DataTypes>
void SkeletonResectionSimulator<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
    updateSegmentColors(); // keeps segmentColors (and m_colorsActive) current every frame

    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    static const sofa::type::RGBAColor perfusedColor(0.20f, 0.80f, 0.20f, 1.0f);
    static const sofa::type::RGBAColor affectedColor(0.0f, 0.0f, 0.0f, 1.0f);
    static const sofa::type::RGBAColor cutColor(1.0f, 1.0f, 1.0f, 1.0f);

    const auto& affected = d_outAffectedNodeIds.getValue();
    std::set<int> affectedSet(affected.begin(), affected.end());
    std::set<int> cutSet(d_inCutNodeIds.getValue().begin(), d_inCutNodeIds.getValue().end());

    std::vector<sofa::type::Vec3> points;
    std::vector<sofa::type::RGBAColor> colors;
    points.reserve(m_graph.nodes().size());
    colors.reserve(m_graph.nodes().size());

    for (const SkeletonNode& n : m_graph.nodes())
    {
        const auto& p = n.position();
        points.emplace_back(p[0], p[1], p[2]);

        if (!m_colorsActive)
            colors.push_back(perfusedColor); // before activationDelay: everything shows as perfused
        else if (cutSet.count(n.id()))
            colors.push_back(cutColor);
        else if (affectedSet.count(n.id()))
            colors.push_back(affectedColor);
        else
            colors.push_back(perfusedColor);
    }

    vparams->drawTool()->drawPoints(points, 7.0f, colors);
}

} // namespace meshskeletonizationplugin
