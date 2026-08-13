#pragma once


#include <MeshSkeletonizationPlugin/config.h>


#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/visual/VisualParams.h>


#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonGraph.h>


using namespace sofa;
using namespace sofa::defaulttype;


namespace meshskeletonizationplugin
{


template <class DataTypes>
class SkeletonReader : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SkeletonReader, DataTypes), sofa::core::DataEngine);


    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename Coord::value_type Real;
    typedef type::Vec<3, Real> Vec3;


    // Inputs
    sofa::core::objectmodel::DataFileName d_inSkeletonFilename; ///< Path to the skeleton polyline file to read (e.g. skeleton.txt)
    sofa::core::objectmodel::Data<VecCoord> d_inVertices;       ///< Optional input mesh vertices, to link skeleton nodes to the mesh
    sofa::core::objectmodel::Data<Vec3> d_inEntryPoint;         ///< Approx. entry point; closest node becomes the tree root


    // Outputs
    sofa::core::objectmodel::DataFileName d_outVTKFilename;     ///< File path to (re-)export the rooted tree as VTK
    sofa::core::objectmodel::DataFileName d_outReportFilename;  ///< File path to (re-)export a per-node CSV report (id, coord, parentId, childrenIds, pathFromRoot)
    sofa::core::objectmodel::Data<int> d_outNodeCount;          ///< Number of skeleton nodes read


    const SkeletonGraph& graph() const { return m_graph; }


    const std::vector<int>& parentsOf(int nodeId) const { return m_graph.parentsOf(nodeId); }
    const std::vector<int>& childrenOf(int nodeId) const { return m_graph.childrenOf(nodeId); }
    std::vector<int> pathFromRoot(int nodeId) const { return m_graph.pathFromRoot(nodeId); }
    std::vector<int> subtree(int nodeId) const { return m_graph.subtree(nodeId); }


    // Overrides of the base class must stay public.
    void init() override;
    void doUpdate() override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;


private:
    SkeletonReader();
    virtual ~SkeletonReader() = default;


    SkeletonGraph m_graph;
};


#if !defined(SKELETONREADER_CPP)
extern template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonReader<defaulttype::Vec3Types>;
#endif


} // namespace meshskeletonizationplugin
