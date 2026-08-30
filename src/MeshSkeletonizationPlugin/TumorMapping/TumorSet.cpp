#include "TumorSet.h"

#include <limits>
#include <map>

namespace meshskeletonizationplugin
{

int TumorSet::find(std::vector<int>& parent, int x)
{
    while (parent[x] != x)
    {
        parent[x] = parent[parent[x]]; // path halving
        x = parent[x];
    }
    return x;
}

void TumorSet::unite(std::vector<int>& parent, int a, int b)
{
    int ra = find(parent, a);
    int rb = find(parent, b);
    if (ra != rb)
        parent[ra] = rb;
}

std::vector<Tumor> TumorSet::splitConnectedComponents(
    const std::vector<std::array<double, 3>>& vertices,
    const std::vector<std::array<int, 3>>& triangles)
{
    std::vector<Tumor> result;
    if (vertices.empty())
        return result;

    // Union-find over vertices, connected by triangle edges.
    std::vector<int> parent(vertices.size());
    for (std::size_t i = 0; i < parent.size(); ++i)
        parent[i] = static_cast<int>(i);

    for (const auto& tri : triangles)
    {
        unite(parent, tri[0], tri[1]);
        unite(parent, tri[1], tri[2]);
        unite(parent, tri[2], tri[0]);
    }

    // Group vertices by root, assigning each distinct root a stable,
    // deterministic tumor id in first-seen order.
    std::map<int, int> rootToTumorId; // union-find root -> tumor index
    std::vector<std::vector<int>> tumorVertexIds; // tumor index -> original vertex ids

    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        int root = find(parent, static_cast<int>(i));
        auto it = rootToTumorId.find(root);
        int tumorId;
        if (it == rootToTumorId.end())
        {
            tumorId = static_cast<int>(tumorVertexIds.size());
            rootToTumorId[root] = tumorId;
            tumorVertexIds.emplace_back();
        }
        else
        {
            tumorId = it->second;
        }
        tumorVertexIds[tumorId].push_back(static_cast<int>(i));
    }

    // Build each Tumor: remap global vertex ids to local (0..n-1) ids,
    // carry over only the triangles fully inside this component, and
    // compute bbox/centroid while we're at it.
    result.resize(tumorVertexIds.size());
    std::vector<int> globalToLocal(vertices.size(), -1);

    for (std::size_t t = 0; t < tumorVertexIds.size(); ++t)
    {
        Tumor& tumor = result[t];
        tumor.id = static_cast<int>(t);

        const auto& ids = tumorVertexIds[t];
        tumor.vertices.reserve(ids.size());

        std::array<double, 3> bmin{ std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
        std::array<double, 3> bmax{ std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() };
        std::array<double, 3> sum{ 0.0, 0.0, 0.0 };

        for (int localIdx = 0; localIdx < static_cast<int>(ids.size()); ++localIdx)
        {
            int globalIdx = ids[localIdx];
            globalToLocal[globalIdx] = localIdx;

            const auto& p = vertices[globalIdx];
            tumor.vertices.push_back(p);

            for (int k = 0; k < 3; ++k)
            {
                bmin[k] = std::min(bmin[k], p[k]);
                bmax[k] = std::max(bmax[k], p[k]);
                sum[k] += p[k];
            }
        }

        tumor.bboxMin = bmin;
        tumor.bboxMax = bmax;
        for (int k = 0; k < 3; ++k)
            tumor.centroid[k] = sum[k] / static_cast<double>(ids.size());
    }

    for (const auto& tri : triangles)
    {
        int root = find(parent, tri[0]);
        int tumorId = rootToTumorId.at(root);
        result[tumorId].triangles.push_back({
            globalToLocal[tri[0]],
            globalToLocal[tri[1]],
            globalToLocal[tri[2]]
        });
    }

    return result;
}

} // namespace meshskeletonizationplugin