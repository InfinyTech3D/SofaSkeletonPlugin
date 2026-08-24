#pragma once

#include <array>
#include <vector>

namespace meshskeletonizationplugin
{

/// A single node of a skeletonized structure (e.g. a vessel centerline point).
/// Deliberately has no SOFA/CGAL dependency so it can be reused, tested,
/// and serialized independently of the plugin's simulation types.
class SkeletonNode
{
public:
    SkeletonNode() = default;
    SkeletonNode(int id, double x, double y, double z)
        : m_id(id), m_position{ x, y, z }
    {
    }

    int id() const { return m_id; }
    void setId(int id) { m_id = id; }

    const std::array<double, 3>& position() const { return m_position; }
    void setPosition(double x, double y, double z) { m_position = { x, y, z }; }

    /// Ids of the node(s) this node is connected to towards the root.
    /// Usually a single parent; more than one means this node closes a
    /// loop/anastomosis in the raw skeleton graph.
    const std::vector<int>& parentIds() const { return m_parentIds; }
    void addParentId(int id) { m_parentIds.push_back(id); }

    /// Ids of the node(s) reached by moving away from the root.
    const std::vector<int>& childrenIds() const { return m_childrenIds; }
    void addChildId(int id) { m_childrenIds.push_back(id); }

    bool isRoot() const { return m_parentIds.empty(); }
    bool isLeaf() const { return m_childrenIds.empty(); }
    bool isBranchPoint() const { return m_childrenIds.size() > 1; }

    /// Index of the closest vertex in the input mesh, -1 if not computed.
    int meshVertexId() const { return m_meshVertexId; }
    void setMeshVertexId(int vId) { m_meshVertexId = vId; }

    /// Distance from this node to that mesh vertex, <0 if not computed.
    double distanceToMesh() const { return m_distanceToMesh; }
    void setDistanceToMesh(double d) { m_distanceToMesh = d; }

    /// Estimated local vessel radius at this node, <0 if not available.
    double radius() const { return m_radius; }
    void setRadius(double r) { m_radius = r; }

private:
    int m_id{ -1 };
    std::array<double, 3> m_position{ 0.0, 0.0, 0.0 };

    std::vector<int> m_parentIds;
    std::vector<int> m_childrenIds;

    int m_meshVertexId{ -1 };
    double m_distanceToMesh{ -1.0 };
    double m_radius{ -1.0 };
};

} // namespace meshskeletonizationplugin