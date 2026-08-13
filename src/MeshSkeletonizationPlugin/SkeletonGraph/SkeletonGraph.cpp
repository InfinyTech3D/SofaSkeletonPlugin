#include "SkeletonGraph.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>

namespace meshskeletonizationplugin
{

void SkeletonGraph::clear()
{
    m_nodes.clear();
    m_adjacency.clear();
    m_loopEdges.clear();
    m_rootId = -1;
}

int SkeletonGraph::findOrCreateNode(const std::array<double, 3>& p, double tol)
{
    const double tol2 = tol * tol;
    for (const SkeletonNode& n : m_nodes)
    {
        const auto& q = n.position();
        double dx = q[0] - p[0];
        double dy = q[1] - p[1];
        double dz = q[2] - p[2];
        if (dx * dx + dy * dy + dz * dz <= tol2)
            return n.id();
    }

    int newId = static_cast<int>(m_nodes.size());
    m_nodes.emplace_back(newId, p[0], p[1], p[2]);
    return newId;
}

void SkeletonGraph::connect(int a, int b)
{
    if (a == b)
        return;

    auto& neighborsA = m_adjacency[a];
    if (std::find(neighborsA.begin(), neighborsA.end(), b) == neighborsA.end())
        neighborsA.push_back(b);

    auto& neighborsB = m_adjacency[b];
    if (std::find(neighborsB.begin(), neighborsB.end(), a) == neighborsB.end())
        neighborsB.push_back(a);
}

bool SkeletonGraph::loadFromFile(const std::string& filename, double mergeTolerance)
{
    std::ifstream in(filename);
    if (!in.is_open())
        return false;

    clear();

    std::string line;
    int previousId = -1;

    while (std::getline(in, line))
    {
        // Trim trailing whitespace/CR so blank-line detection works on
        // files written or edited on Windows too.
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
            line.pop_back();

        if (line.empty())
        {
            previousId = -1; // end of current polyline
            continue;
        }

        std::istringstream iss(line);
        std::array<double, 3> p{};
        if (!(iss >> p[0] >> p[1] >> p[2]))
            continue; // skip malformed/comment lines

        int id = findOrCreateNode(p, mergeTolerance);

        if (previousId >= 0)
            connect(previousId, id);

        previousId = id;
    }

    return true;
}

int SkeletonGraph::closestNodeId(const std::array<double, 3>& p) const
{
    int best = -1;
    double bestDist = std::numeric_limits<double>::max();
    for (const SkeletonNode& n : m_nodes)
    {
        const auto& q = n.position();
        double dx = q[0] - p[0];
        double dy = q[1] - p[1];
        double dz = q[2] - p[2];
        double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < bestDist)
        {
            bestDist = d2;
            best = n.id();
        }
    }
    return best;
}

void SkeletonGraph::buildTree(const std::array<double, 3>& entryPoint)
{
    int root = closestNodeId(entryPoint);
    if (root >= 0)
        buildTree(root);
}

void SkeletonGraph::buildTree(int rootId)
{
    m_loopEdges.clear();

    for (SkeletonNode& n : m_nodes)
    {
        n = SkeletonNode(n.id(), n.position()[0], n.position()[1], n.position()[2]);
    }

    if (rootId < 0 || rootId >= static_cast<int>(m_nodes.size()))
    {
        m_rootId = -1;
        return;
    }

    m_rootId = rootId;

    std::vector<bool> visited(m_nodes.size(), false);
    std::vector<int> parentOf(m_nodes.size(), -1);
    std::queue<int> q;

    visited[rootId] = true;
    q.push(rootId);

    std::set<std::pair<int, int>> seenEdges; // to dedupe loop-edge reporting

    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        auto it = m_adjacency.find(u);
        if (it == m_adjacency.end())
            continue;

        for (int v : it->second)
        {
            if (!visited[v])
            {
                visited[v] = true;
                parentOf[v] = u;
                m_nodes[u].addChildId(v);
                m_nodes[v].addParentId(u);
                q.push(v);
            }
            else if (parentOf[u] != v) // don't re-report the edge we arrived from
            {
                std::pair<int, int> key = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
                if (seenEdges.insert(key).second)
                {
                    // Real loop/anastomosis: record extra parent/child links
                    // without duplicating the primary tree edge.
                    m_nodes[u].addChildId(v);
                    m_nodes[v].addParentId(u);
                    m_loopEdges.emplace_back(u, v);
                }
            }
        }
    }
}

void SkeletonGraph::computeMeshCorrespondence(const std::vector<std::array<double, 3>>& meshVertices)
{
    if (meshVertices.empty())
        return;

    for (SkeletonNode& n : m_nodes)
    {
        const auto& p = n.position();
        int bestVid = -1;
        double bestDist2 = std::numeric_limits<double>::max();

        for (int i = 0; i < static_cast<int>(meshVertices.size()); ++i)
        {
            const auto& v = meshVertices[i];
            double dx = v[0] - p[0];
            double dy = v[1] - p[1];
            double dz = v[2] - p[2];
            double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < bestDist2)
            {
                bestDist2 = d2;
                bestVid = i;
            }
        }

        n.setMeshVertexId(bestVid);
        n.setDistanceToMesh(std::sqrt(bestDist2));
    }
}

void SkeletonGraph::exportToVTK(const std::string& filename) const
{
    if (!hasRoot())
        return;

    const int n = static_cast<int>(m_nodes.size());

    // Node ids are assigned sequentially at creation time (0..n-1), so id ==
    // vector index and no separate id-remapping table is needed here.

    // depth(v) = number of edges from root, via each node's primary parent
    // (parentIds().front()), memoized as we go.
    std::vector<int> depth(n, -1);
    depth[m_rootId] = 0;
    for (int v = 0; v < n; ++v)
    {
        if (depth[v] >= 0)
            continue;

        std::vector<int> path;
        int cur = v;
        while (depth[cur] < 0)
        {
            path.push_back(cur);
            const auto& parents = m_nodes[cur].parentIds();
            if (parents.empty())
                break; // only the root should have no parent
            cur = parents.front();
        }
        int d = depth[cur] >= 0 ? depth[cur] : 0;
        for (auto it = path.rbegin(); it != path.rend(); ++it)
            depth[*it] = ++d;
    }

    // LINES: primary tree edge (parentIds()[0] -> id) for every non-root
    // node, plus any extra parent entries (index > 0) recorded as loop edges.
    std::vector<std::pair<int, int>> lines;
    std::vector<int> isLoop;
    for (const SkeletonNode& node : m_nodes)
    {
        const auto& parents = node.parentIds();
        for (std::size_t j = 0; j < parents.size(); ++j)
        {
            lines.emplace_back(parents[j], node.id());
            isLoop.push_back(j == 0 ? 0 : 1);
        }
    }

    std::ofstream out(filename, std::ofstream::out | std::ofstream::trunc);
    out << "# vtk DataFile Version 3.0\n";
    out << "Skeleton graph (SkeletonReader module)\n";
    out << "ASCII\n";
    out << "DATASET POLYDATA\n";

    out << "POINTS " << n << " float\n";
    out << std::fixed << std::setprecision(6);
    for (const SkeletonNode& node : m_nodes)
    {
        const auto& p = node.position();
        out << p[0] << " " << p[1] << " " << p[2] << "\n";
    }

    out << "LINES " << lines.size() << " " << (lines.size() * 3) << "\n";
    for (const auto& l : lines)
        out << "2 " << l.first << " " << l.second << "\n";

    out << "POINT_DATA " << n << "\n";
    out << "SCALARS type int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (const SkeletonNode& node : m_nodes)
    {
        int type = (node.id() == m_rootId) ? 1 : (node.isLeaf() ? 6 : 5); // 1=root, 5=branch, 6=terminal
        out << type << "\n";
    }
    out << "SCALARS depth int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (int d : depth)
        out << d << "\n";

    out << "CELL_DATA " << lines.size() << "\n";
    out << "SCALARS is_loop int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (int flag : isLoop)
        out << flag << "\n";
}

} // namespace meshskeletonizationplugin
