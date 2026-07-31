#define SKELETONIZATIONLOADER_CPP
#include <MeshSkeletonizationPlugin/MySkeleton/SkeletonizationLoader.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace meshskeletonizationplugin
{
using namespace sofa::defaulttype;

void registerSkeletonizationLoader(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Load a mesh skeleton and build a rooted father/children tree, exported as SWC")
        .add< SkeletonizationLoader<Vec3Types> >());
}

template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonizationLoader<Vec3Types>;
}