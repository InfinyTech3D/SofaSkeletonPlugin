#pragma once

#include <MeshSkeletonizationPlugin/config.h>

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/RGBAColor.h>

#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonGraph.h>
#include <MeshSkeletonizationPlugin/SegmentMapping/SkeletonSegmentMapper.h>

using namespace sofa;
using namespace sofa::defaulttype;

namespace meshskeletonizationplugin
{

/// Simulates a resection (vessel transection) at one or more skeleton node
/// ids and reports what it would devascularize: which skeleton nodes can no
/// longer reach the root (blood source) once the cut node(s) are removed
/// from the vessel graph, and which liver segments those nodes belong to.
///
/// This is deliberately NOT a plain "everything below the cut" (subtree)
/// computation: a node distal to a cut can still be perfused if a
/// collateral/anastomosis connects it back to the root via a path that
/// avoids every cut node. See SkeletonGraph::simulateResection() for the
/// underlying reachability analysis.
///
/// Typical use: link to an existing SkeletonSegmentMapper (so both the tree
/// and the segment labels are available), set d_inCutNodeIds to the node(s)
/// a candidate resection would sever, and read back d_outAffectedNodeIds /
/// d_outAffectedSegmentNames to see the consequence before committing to a
/// surgical plan.
template <class DataTypes>
class SkeletonResectionSimulator : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SkeletonResectionSimulator, DataTypes), sofa::core::DataEngine);

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename Coord::value_type Real;
    typedef type::Vec<3, Real> Vec3;

    /// The segmented skeleton to run resection scenarios against. If left
    /// unset, the first SkeletonSegmentMapper<DataTypes> found in the
    /// context is used.
    sofa::core::objectmodel::SingleLink<
        SkeletonResectionSimulator<DataTypes>,
        SkeletonSegmentMapper<DataTypes>,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK>
        l_segmentMapper;

    /// Optional: the actual visual object (e.g. OglModel) for each segment,
    /// in the same order as the linked mapper's segmentNames. If set, their
    /// "color" Data is pushed directly every draw() call - more reliable
    /// than a scene-level Data link, since most visual models don't re-read
    /// a linked color every frame on their own.
    sofa::core::objectmodel::MultiLink<
        SkeletonResectionSimulator<DataTypes>,
        sofa::core::objectmodel::BaseObject,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH>
        l_segmentVisualModels;

    // Inputs
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_inCutNodeIds; ///< Node id(s) where the vessel is (candidate) severed
    sofa::core::objectmodel::Data<double> d_activationDelay;              ///< Seconds after simulation start before affected nodes/segments actually switch color (default 5.0); everything shows as perfused (green) until then
    sofa::core::objectmodel::DataFileName d_outReportFilename;            ///< Optional CSV report: id, x, y, z, segment, for affected nodes only

    // Outputs
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_outAffectedNodeIds;             ///< Node ids that would lose blood supply (includes the cut nodes themselves)
    sofa::core::objectmodel::Data<int> d_outAffectedNodeCount;                               ///< Convenience: size of d_outAffectedNodeIds
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_outAffectedSegmentIds;           ///< Distinct segment indices touched by the affected nodes
    sofa::core::objectmodel::Data<sofa::type::vector<std::string>> d_outAffectedSegmentNames; ///< Same, resolved to names via the linked SkeletonSegmentMapper's segmentNames
    sofa::core::objectmodel::Data<sofa::type::vector<sofa::type::RGBAColor>> d_outSegmentColors; ///< One color per segment, same order as the linked mapper's segmentNames - green until activationDelay elapses, then red for affected segments. Link an OglModel's "color" to e.g. "@resectionSim.segmentColors[0]" per segment (index syntax support depends on SOFA version).

    void init() override;
    void doUpdate() override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;

private:
    SkeletonResectionSimulator();
    virtual ~SkeletonResectionSimulator() = default;

    /// Recomputes m_colorsActive from the current simulation time and
    /// refreshes d_outSegmentColors accordingly. Called every draw() so the
    /// activationDelay threshold takes effect without needing an event
    /// listener (and the Sofa.Simulation.Core dependency that would add).
    void updateSegmentColors();

    /// Local working copy of the linked mapper's (already segmented) graph.
    SkeletonGraph m_graph;

    /// Whether affected nodes/segments should currently render as
    /// affected (red/black) rather than perfused (green) - false until
    /// d_activationDelay seconds have elapsed since simulation start.
    bool m_colorsActive{ false };

    /// Guards so diagnostic messages below are logged once, not every frame.
    bool m_loggedVisualModelLinkStatus{ false };
    std::vector<bool> m_loggedMissingColorData;
};

#if !defined(SKELETONRESECTIONSIMULATOR_CPP)
extern template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonResectionSimulator<defaulttype::Vec3Types>;
#endif

} // namespace meshskeletonizationplugin
