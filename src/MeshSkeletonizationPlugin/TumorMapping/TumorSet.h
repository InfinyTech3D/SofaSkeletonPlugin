#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace meshskeletonizationplugin
{

/// One connected tumor extracted from a (possibly merged) tumor mesh.
/// Vertex/triangle indices are LOCAL to this tumor (remapped from the
/// original merged mesh), so each Tumor is a fully self-contained mesh.
struct Tumor
{
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<int, 3>> triangles;

    std::array<double, 3> bboxMin{ 0.0, 0.0, 0.0 };
    std::array<double, 3> bboxMax{ 0.0, 0.0, 0.0 };
    std::array<double, 3> centroid{ 0.0, 0.0, 0.0 };

    /// Index of this tumor's original connected component in the source
    /// mesh (0, 1, 2, ...), stable/deterministic for a given input so it
    /// can be used as a display label ("tumor 0", "tumor 1", ...).
    int id{ -1 };
};

/// Splits a single mesh that may contain several disjoint tumors merged
/// into one file (common when tumors are exported together from a
/// segmentation tool) into one Tumor per connected component, each with
/// its own bounding box and centroid precomputed.
///
/// Two vertices are considered connected if some triangle has an edge
/// between them; a tumor is a maximal set of vertices connected this way.
/// No SOFA/CGAL dependency, so it's usable and testable independently of
/// the plugin's SOFA components.
class TumorSet
{
public:
    static std::vector<Tumor> splitConnectedComponents(
        const std::vector<std::array<double, 3>>& vertices,
        const std::vector<std::array<int, 3>>& triangles);

private:
    static int find(std::vector<int>& parent, int x);
    static void unite(std::vector<int>& parent, int a, int b);
};

} // namespace meshskeletonizationplugin