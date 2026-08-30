#pragma once

#include <MeshSkeletonizationPlugin/config.h>

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/loader/MeshLoader.h>

#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonGraph.h>
#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonReader.h>

using namespace sofa;
using namespace sofa::defaulttype;

namespace meshskeletonizationplugin
{

template <class DataTypes>
class SkeletonSegmentMapper : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SkeletonSegmentMapper, DataTypes), sofa::core::DataEngine);

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename Coord::value_type Real;
    typedef type::Vec<3, Real> Vec3;

    /// The SkeletonReader whose graph should be segmented. If left unset, the
    /// first SkeletonReader<DataTypes> found in the context is used.
    sofa::core::objectmodel::SingleLink<
        SkeletonSegmentMapper<DataTypes>,
        SkeletonReader<DataTypes>,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK>
        l_skeletonReader;

    /// One closed mesh per liver segment (the Couinaud collision meshes), in the same order as d_inSegmentNames.
    sofa::core::objectmodel::MultiLink<
        SkeletonSegmentMapper<DataTypes>,
        sofa::core::loader::MeshLoader,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH>
        l_segmentMeshes;

    // Inputs
    sofa::core::objectmodel::Data<sofa::type::vector<std::string>> d_inSegmentNames; ///< Human-readable name per entry of l_segmentMeshes, e.g. "II", "IVa", "VIII"
    sofa::core::objectmodel::DataFileName d_outSegmentReportFilename;                ///< Optional CSV report: id, x, y, z, segment name

    // Outputs
    sofa::core::objectmodel::Data<sofa::type::vector<int>> d_outNodeSegments;         ///< Segment index per skeleton node (into l_segmentMeshes/d_inSegmentNames, -1 = unknown), indexed like graph().nodes()
    sofa::core::objectmodel::Data<sofa::type::vector<std::string>> d_outNodeSegmentNames; ///< Same as d_outNodeSegments, resolved to names ("unknown" if -1)
    sofa::core::objectmodel::Data<int> d_outSegmentCount;                             ///< Number of distinct segments found (excluding unknown)

    /// Direct access to the segmented graph, e.g. for another component to
    /// query via getContext()->get<SkeletonSegmentMapper<DataTypes>>()->graph().
    const SkeletonGraph& graph() const { return m_graph; }

    void init() override;
    void doUpdate() override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;

private:
    SkeletonSegmentMapper();
    virtual ~SkeletonSegmentMapper() = default;
    /// Local working copy of the linked reader's graph, augmented with segment labels 
    SkeletonGraph m_graph;
};

#if !defined(SKELETONSEGMENTMAPPER_CPP)
extern template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonSegmentMapper<defaulttype::Vec3Types>;
#endif

} // namespace meshskeletonizationplugin
