
#pragma once

#include <MeshSkeletonizationPlugin/VesselTree/VesselTree.h>

#include <boost/graph/adjacency_list.hpp>
#include <queue>
#include <limits>
#include <fstream>
#include <iomanip>

namespace meshskeletonizationplugin
{

template <class Graph>
template <class Point3>
typename VesselTree<Graph>::Vertex
VesselTree<Graph>::findClosestVertex(const Graph& g, const Point3& p) const
{
    Vertex best{};
    double bestDist = std::numeric_limits<double>::max();

    for (const Vertex& v : boost::make_iterator_range(boost::vertices(g)))
    {
        const auto& s = g[v].point;
        double dx = s[0] - p[0];
        double dy = s[1] - p[1];
        double dz = s[2] - p[2];
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < bestDist)
        {
            bestDist = d2;
            best = v;
        }
    }
    return best;
}


template <class Graph>
void VesselTree<Graph>::build(const Graph& g, Vertex root)
{
    m_parent.clear();
    m_children.clear();
    m_loopEdges.clear();

    std::map<Vertex, bool> visited;
    std::queue<Vertex> q;

    m_root = root;
    m_hasRoot = true;
    visited[root] = true;
    q.push(root);

    while (!q.empty())
    {
        Vertex u = q.front();
        q.pop();

        for (const Edge& e : boost::make_iterator_range(boost::out_edges(u, g)))
        {
            Vertex v = boost::target(e, g);

            if (!visited[v])
            {
                visited[v] = true;
                m_parent[v] = u;
                m_children[u].push_back(v);
                q.push(v);
            }
            else
            {
                // v already reached. Skip the edge we just arrived from;
                // anything else is a real loop / anastomosis, not a tree edge.
                auto itParentU = m_parent.find(u);
                bool cameFromV = (itParentU != m_parent.end() && itParentU->second == v);
                if (!cameFromV)
                    m_loopEdges.emplace_back(u, v);
            }
        }
    }
}


template <class Graph>
std::map<typename VesselTree<Graph>::Vertex, int>
VesselTree<Graph>::assignVertexIds() const
{
    std::map<Vertex, int> vertexId;
    if (!m_hasRoot)
        return vertexId;
    vertexId[m_root] = 0;
    int nextId = 1;
    for (const auto& kv : m_parent)
        if (!vertexId.count(kv.first)) vertexId[kv.first] = nextId++;
    return vertexId;
}


template <class Graph>
std::vector<typename VesselTree<Graph>::Vertex>
VesselTree<Graph>::subtree(Vertex node) const
{
    std::vector<Vertex> out{node};
    std::vector<Vertex> stack{node};
    while (!stack.empty())
    {
        Vertex n = stack.back();
        stack.pop_back();
        auto it = m_children.find(n);
        if (it != m_children.end())
        {
            for (Vertex c : it->second)
            {
                out.push_back(c);
                stack.push_back(c);
            }
        }
    }
    return out;
}


template <class Graph>
void VesselTree<Graph>::exportToSWC(const Graph& g, const std::string& filename) const
{
    if (!m_hasRoot)
        return;

    std::map<Vertex, int> vertexId = assignVertexIds();

    std::ofstream out(filename, std::ofstream::out | std::ofstream::trunc);
    out << "# id type x y z radius parent\n";
    for (const auto& loop : m_loopEdges)
    {
        int a = vertexId.at(loop.first);
        int b = vertexId.at(loop.second);
        if (a < b) // each undirected loop is recorded from both ends; write once
            out << "# loop " << a << " " << b << "\n";
    }

    out << std::fixed << std::setprecision(6);
    for (const auto& kv : vertexId)
    {
        Vertex v = kv.first;
        int id = kv.second;
        const auto& p = g[v].point;

        int deg = static_cast<int>(boost::degree(v, g));
        int type = (v == m_root) ? 1 : (deg == 1 ? 6 : 5); // 1=root, 6=terminal, 5=branch/regular

        int parentId = -1;
        auto itParent = m_parent.find(v);
        if (itParent != m_parent.end())
            parentId = vertexId.at(itParent->second);

        out << id << " " << type << " "
            << p[0] << " " << p[1] << " " << p[2] << " "
            << 0.0 << " " << parentId << "\n";
    }
}

template <class Graph>
void VesselTree<Graph>::exportToVTK(const Graph& g, const std::string& filename) const
{
    if (!m_hasRoot)
        return;

    std::map<Vertex, int> vertexId = assignVertexIds();

    // depth(v) = number of edges from root, computed by walking each vertex's
    // parent chain up to a vertex whose depth is already known (memoized).
    std::map<Vertex, int> depth;
    depth[m_root] = 0;
    for (const auto& kv : vertexId)
    {
        Vertex v = kv.first;
        if (depth.count(v)) continue;
        std::vector<Vertex> path;
        Vertex cur = v;
        while (!depth.count(cur))
        {
            path.push_back(cur);
            auto it = m_parent.find(cur);
            if (it == m_parent.end()) break; // only the root has no parent
            cur = it->second;
        }
        int d = depth.count(cur) ? depth[cur] : 0;
        for (auto it = path.rbegin(); it != path.rend(); ++it)
            depth[*it] = ++d;
    }

    // POINTS, in id order
    std::vector<Vertex> orderedVerts(vertexId.size());
    for (const auto& kv : vertexId)
        orderedVerts[kv.second] = kv.first;

    // LINES: tree edges (child -> parent) + deduplicated loop edges
    std::vector<std::pair<int, int>> lines;
    for (const auto& kv : m_parent)
        lines.emplace_back(vertexId.at(kv.second), vertexId.at(kv.first));

    std::vector<int> isLoop(lines.size(), 0);
    for (const auto& loop : m_loopEdges)
    {
        int a = vertexId.at(loop.first);
        int b = vertexId.at(loop.second);
        if (a < b)
        {
            lines.emplace_back(a, b);
            isLoop.push_back(1);
        }
    }

    std::ofstream out(filename, std::ofstream::out | std::ofstream::trunc);
    out << "# vtk DataFile Version 3.0\n";
    out << "Vessel tree (VesselTree module)\n";
    out << "ASCII\n";
    out << "DATASET POLYDATA\n";

    out << "POINTS " << orderedVerts.size() << " float\n";
    out << std::fixed << std::setprecision(6);
    for (Vertex v : orderedVerts)
    {
        const auto& p = g[v].point;
        out << p[0] << " " << p[1] << " " << p[2] << "\n";
    }

    out << "LINES " << lines.size() << " " << (lines.size() * 3) << "\n";
    for (const auto& l : lines)
        out << "2 " << l.first << " " << l.second << "\n";

    out << "POINT_DATA " << orderedVerts.size() << "\n";
    out << "SCALARS type int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (Vertex v : orderedVerts)
    {
        int deg = static_cast<int>(boost::degree(v, g));
        int type = (v == m_root) ? 1 : (deg == 1 ? 6 : 5); // 1=root, 5=branch, 6=terminal
        out << type << "\n";
    }
    out << "SCALARS depth int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (Vertex v : orderedVerts)
        out << depth[v] << "\n";

    out << "CELL_DATA " << lines.size() << "\n";
    out << "SCALARS is_loop int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (int flag : isLoop)
        out << flag << "\n";
}

} // namespace meshskeletonizationplugin
