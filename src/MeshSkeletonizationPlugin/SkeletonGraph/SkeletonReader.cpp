#define SKELETONREADER_CPP
#include <MeshSkeletonizationPlugin/SkeletonGraph/SkeletonReader.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace meshskeletonizationplugin
{
using namespace sofa::defaulttype;

void registerSkeletonReader(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Read a skeleton polyline file (skeleton.txt) and store it as a graph of "
        "nodes, each with its id, parent ids, children ids, and (if an input mesh "
        "is given) its correspondence to the closest mesh vertex")
        .add< SkeletonReader<Vec3Types> >());
}

template class SOFA_MESHSKELETONIZATIONPLUGIN_API SkeletonReader<Vec3Types>;
}