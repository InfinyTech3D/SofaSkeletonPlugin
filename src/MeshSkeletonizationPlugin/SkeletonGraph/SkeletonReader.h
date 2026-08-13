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
    sofa::core::objectmodel::DataFileName d_outReportFilename; /// CSV report that gives out the ids, 3D coordinates, parent vertex and child vertex

    // Outputs
    sofa::core::objectmodel::DataFileName d_outVTKFilename;     ///< File path to (re-)export the rooted tree as VTK
    sofa::core::objectmodel::Data<int> d_outNodeCount;          ///< Number of skeleton nodes read

    /// Direct access to the loaded graph, e.g. for another component to query
    /// via getContext()->get<SkeletonReader<DataTypes>>()->graph().
    const SkeletonGraph& graph() const { return m_graph; }

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
