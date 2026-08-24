#define SKELETONSEGMENTMAPPER_CPP
#include <MeshSkeletonizationPlugin/SegmentMapping/SkeletonSegmentMapper.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace meshskeletonizationplugin
{
using namespace sofa::defaulttype;

void registerSkeletonSegmentMapper(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Maps a previously-read skeleton (SkeletonReader) onto liver segments (e.g. "
        "Couinaud I-VIII), using a per-mesh-vertex segment label and smoothing "
        "per-branch by majority vote")
        .add< SkeletonSegmentMapper<Vec3Types> >());
}

template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonSegmentMapper<Vec3Types>;

} // namespace meshskeletonizationplugin
