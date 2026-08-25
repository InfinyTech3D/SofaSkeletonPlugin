#define SKELETONRESECTIONSIMULATOR_CPP
#include <MeshSkeletonizationPlugin/SkeletonResection/SkeletonResectionSimulator.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace meshskeletonizationplugin
{
using namespace sofa::defaulttype;

void registerSkeletonResectionSimulator(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Simulates a vessel resection at one or more skeleton nodes and reports which "
        "nodes and liver segments would lose blood supply, accounting for collateral "
        "(loop/anastomosis) paths that bypass the cut")
        .add< SkeletonResectionSimulator<Vec3Types> >());
}

template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonResectionSimulator<Vec3Types>;

} // namespace meshskeletonizationplugin
