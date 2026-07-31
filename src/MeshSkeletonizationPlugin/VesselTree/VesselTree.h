#pragma once

#include <MeshSkeletonizationPlugin/config.h>

#include <boost/graph/graph_traits.hpp>
#include <map>
#include <vector>
#include <string>

namespace meshskeletonizationplugin
{
template <class Graph>
class VesselTree
{
public:
    using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
    using Edge   = typename boost::graph_traits<Graph>::edge_descriptor;

    template <class Point3>
    Vertex findClosestVertex(const Graph& g, const Point3& p) const;
    void build(const Graph& g, Vertex root);


    void exportToSWC(const Graph& g, const std::string& filename) const;
    void exportToVTK(const Graph& g, const std::string& filename) const;

    const std::map<Vertex, Vertex>& parents() const { return m_parent; }
    const std::map<Vertex, std::vector<Vertex>>& children() const { return m_children; }
    const std::vector<std::pair<Vertex, Vertex>>& loopEdges() const { return m_loopEdges; }
    Vertex root() const { return m_root; }
    bool hasRoot() const { return m_hasRoot; }
    std::vector<Vertex> subtree(Vertex node) const;

private:
    /// Stable sequential id per reached vertex, root always id 0.
    std::map<Vertex, int> assignVertexIds() const;

    std::map<Vertex, Vertex> m_parent;
    std::map<Vertex, std::vector<Vertex>> m_children;
    std::vector<std::pair<Vertex, Vertex>> m_loopEdges;
    Vertex m_root{};
    bool m_hasRoot = false;
};

} // namespace meshskeletonizationplugin

#include <MeshSkeletonizationPlugin/VesselTree/VesselTree.inl>