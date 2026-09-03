#pragma once

#include <MeshSkeletonizationPlugin/config.h>

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/loader/MeshLoader.h>
#include <sofa/core/visual/VisualParams.h>

#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonGraph.h>
#include <MeshSkeletonizationPlugin/SegmentMapping/SkeletonSegmentMapper.h>
#include <MeshSkeletonizationPlugin/CGALMeshUtils.h>

#include <array>

using namespace sofa;
using namespace sofa::defaulttype;

namespace meshskeletonizationplugin
{

/// Case 1 of tumor-driven cut point selection: for each tumor that directly
/// touches the vessel skeleton, find which skeleton node(s) it touches, so
/// those can be fed straight into SkeletonResectionSimulator.cutNodeIds.
///
/// Tumors are provided as a list of already-separate mesh loaders (one per
/// tumor) - same MultiLink pattern SkeletonSegmentMapper uses for its 8
/// segment meshes. If your tumors currently live merged in a single file,
/// split them into individual meshes upstream of this component (e.g. with
/// TumorSet::splitConnectedComponents(), kept available separately) before
/// loading each one with its own MeshOBJLoader.
///
/// For each tumor / skeleton node pair: a cheap bounding-box check first
/// discards nodes nowhere near that tumor (broad phase), then a real
/// point-in-mesh test (CGAL::Side_of_triangle_mesh, same approach as
/// SkeletonSegmentMapper) decides whether the node is actually inside the
/// tumor volume (narrow phase). The bounding box alone is not used as the
/// decision - a non-convex or elongated tumor's box can contain a lot of
/// space the tumor doesn't actually occupy, which would false-positive.
///
/// Tumors that touch no skeleton node at all are reported separately
/// (outUntouchedTumorIds) - those are candidates for Case 2 (nearest node
/// in the tumor's segment), not yet implemented here.
template <class DataTypes>
class TumorCutPointSelector : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TumorCutPointSelector, DataTypes), sofa::core::DataEngine);

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename Coord::value_type Real;
    typedef type::Vec<3, Real> Vec3;

    /// One mesh loader per tumor, already separate (no merged-file splitting
    /// is done by this component).
    sofa::core::objectmodel::MultiLink<
        TumorCutPointSelector<DataTypes>,
        sofa::core::loader::MeshLoader,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH>
        l_tumorMeshes;

    /// The segmented skeleton to test tumor(s) against. If left unset, the
    /// first SkeletonSegmentMapper<DataTypes> found in the context is used.
    sofa::core::objectmodel::SingleLink<
        TumorCutPointSelector<DataTypes>,
        SkeletonSegmentMapper<DataTypes>,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK>
        l_segmentMapper;

    // Inputs
    sofa::core::objectmodel::Data<sofa::type::vector<std::string>> d_inTumorNames; ///< Optional, same order as l_tumorMeshes, e.g. "tumor 1", "tumor 2" - purely for readable output/reports

    // Outputs
    sofa::core::objectmodel::Data<int> d_outTumorCount; ///< Number of linked tumors
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_outCutNodeIds; ///< Union of every touching node across all tumors - link this straight into SkeletonResectionSimulator.cutNodeIds
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_outTouchingTumorIndex; ///< Same length/order as outCutNodeIds: which tumor (0-based, into l_tumorMeshes) each entry belongs to
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_outUntouchedTumorIds; ///< 0-based indices of tumors touching no skeleton node at all

    void init() override;
    void doUpdate() override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;

private:
    TumorCutPointSelector();
    virtual ~TumorCutPointSelector() = default;

    /// One linked tumor's geometry (bbox + CGAL point-in-mesh query) plus
    /// which skeleton nodes it was found to touch (filled by doUpdate(),
    /// used by draw()).
    struct TumorQuery
    {
        std::array<double, 3> bboxMin{};
        std::array<double, 3> bboxMax{};
        cgalutils::ClosedMeshQuery query;
        std::vector<int> touchingNodeIds;
    };

    /// Rebuilds bbox + ClosedMeshQuery for every linked tumor mesh. Pure
    /// geometry prep, no skeleton involved yet.
    void rebuildTumors();

    std::vector<TumorQuery> m_tumors;
    SkeletonGraph m_graph; ///< local copy of the linked mapper's graph, for node positions
};

#if !defined(TUMORCUTPOINTSELECTOR_CPP)
extern template class SOFA_MESHSKELETONIZATIONPLUGIN_API TumorCutPointSelector<defaulttype::Vec3Types>;
#endif

} // namespace meshskeletonizationplugin
