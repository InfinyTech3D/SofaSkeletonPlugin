#define TUMORCUTPOINTSELECTOR_CPP
#include <MeshSkeletonizationPlugin/TumorMapping/TumorCutPointSelector.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace meshskeletonizationplugin
{
using namespace sofa::defaulttype;

void registerTumorCutPointSelector(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Case 1 tumor-driven cut point selection: splits a (possibly multi-tumor) mesh "
        "into its separate tumors and finds which skeleton node(s) each one directly "
        "touches, via bounding-box pre-filtering plus a real point-in-mesh test")
        .add< TumorCutPointSelector<Vec3Types> >());
}

template class SOFA_MESHSKELETONIZATIONPLUGIN_API TumorCutPointSelector<Vec3Types>;

} // namespace meshskeletonizationplugin
