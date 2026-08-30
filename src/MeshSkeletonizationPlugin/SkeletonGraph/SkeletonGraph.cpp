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
    m_nodeSegment.clear();
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
        // Trim trailing whitespace/CR so blank-line detection works on files written or edited on Windows too.
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

bool SkeletonGraph::loadFromVTK(const std::string& filename)
{
    std::ifstream in(filename);
    if (!in.is_open())
        return false;

    clear();

    // Skip the 4-line ASCII VTK header: version, title, "ASCII", "DATASET POLYDATA".
    std::string discard;
    for (int i = 0; i < 4; ++i)
        std::getline(in, discard);

    std::string tag;
    int currentBlockCount = 0; // set by POINT_DATA/CELL_DATA, used to size SCALARS reads

    while (in >> tag)
    {
        if (tag == "POINTS")
        {
            int numPoints = 0;
            std::string dataType;
            in >> numPoints >> dataType;

            m_nodes.clear();
            m_nodes.reserve(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                std::array<double, 3> p{};
                in >> p[0] >> p[1] >> p[2];
                m_nodes.emplace_back(i, p[0], p[1], p[2]);
            }
        }
        else if (tag == "LINES")
        {
            int numLines = 0, listSize = 0;
            in >> numLines >> listSize;
            for (int i = 0; i < numLines; ++i)
            {
                int count = 0, a = 0, b = 0;
                in >> count >> a >> b;
                if (count == 2 && a >= 0 && b >= 0 &&
                    a < static_cast<int>(m_nodes.size()) && b < static_cast<int>(m_nodes.size()))
                {
                    // exportToVTK() writes "parentId childId" per line.
                    m_nodes[a].addChildId(b);
                    m_nodes[b].addParentId(a);

                    // Also record the raw undirected connectivity - this is
                    // what simulateResection()'s reachability BFS walks, so
                    // without it a cut would never propagate to children.
                    connect(a, b);
                }
            }
        }
        else if (tag == "POINT_DATA" || tag == "CELL_DATA")
        {
            in >> currentBlockCount;
        }
        else if (tag == "SCALARS")
        {
            std::string name, dataType, lookupTag, lookupName;
            in >> name >> dataType >> lookupTag >> lookupName; // "<name> <type>\nLOOKUP_TABLE default"

            if (name == "type" && currentBlockCount == static_cast<int>(m_nodes.size()))
            {
                for (int i = 0; i < currentBlockCount; ++i)
                {
                    int t = 0;
                    in >> t;
                    if (t == 1)
                        m_rootId = i;
                }
            }
            else
            {
                // Not needed to rebuild the tree (depth, is_loop, ...): consume and discard.
                double dummy;
                for (int i = 0; i < currentBlockCount; ++i)
                    in >> dummy;
            }
        }
    }

    return hasRoot();
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

void SkeletonGraph::buildTreeAutoRoot()
{
    if (m_nodes.empty())
        return;

    std::vector<bool> visited(m_nodes.size(), false);
    int bestRoot = -1;
    std::size_t bestSize = 0;

    for (const SkeletonNode& start : m_nodes)
    {
        if (visited[start.id()])
            continue;

        // BFS this component, just to measure it and grab a representative node.
        std::vector<int> component;
        std::queue<int> q;
        visited[start.id()] = true;
        q.push(start.id());

        while (!q.empty())
        {
            int u = q.front();
            q.pop();
            component.push_back(u);

            auto it = m_adjacency.find(u);
            if (it == m_adjacency.end())
                continue;

            for (int v : it->second)
            {
                if (!visited[v])
                {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }

        if (component.size() > bestSize)
        {
            bestSize = component.size();
            bestRoot = component.front();
        }
    }

    if (bestRoot >= 0)
        buildTree(bestRoot);
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

const SkeletonNode* SkeletonGraph::node(int nodeId) const
{
    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size()))
        return nullptr;
    return &m_nodes[nodeId];
}


bool SkeletonGraph::parentsOf(int nodeId, std::vector<int>& parents) const
{
    parents.clear();
 
    const SkeletonNode* n = node(nodeId);
    if (n == nullptr || n->parentIds().empty())
        return false;
 
    parents = n->parentIds();
    return true;
}
 
bool SkeletonGraph::childrenOf(int nodeId, std::vector<int>& children) const
{
    children.clear();
 
    const SkeletonNode* n = node(nodeId);
    if (n == nullptr || n->childrenIds().empty())
        return false;
 
    children = n->childrenIds();
    return true;
}

std::vector<int> SkeletonGraph::pathFromRoot(int nodeId) const
{
    std::vector<int> path;
    if (!hasRoot() || node(nodeId) == nullptr) return path;

    std::vector<int> parents;
    std::set<int> seen;
    int cur = nodeId;
    while (true)
    {
        if (!seen.insert(cur).second) { path.clear(); return path; }
        path.push_back(cur);
        if (cur == m_rootId) break;
        if (!parentsOf(cur, parents)) { path.clear(); return path; }
        cur = parents.front();
    }
    std::reverse(path.begin(), path.end());
    return path;
}
std::vector<int> SkeletonGraph::subtree(int nodeId) const
{
    std::vector<int> result;
    if (node(nodeId) == nullptr) return result;
 
    std::vector<int> children;
    std::set<int> visited;
    std::queue<int> q;
    q.push(nodeId); visited.insert(nodeId);
 
    while (!q.empty())
    {
        int cur = q.front(); q.pop();
        result.push_back(cur);
        if (childrenOf(cur, children))
        {
            for (int child : children)
                if (visited.insert(child).second) q.push(child);
        }
    }
    return result;
}
static std::string joinIds(const std::vector<int>& ids)
{
    std::string s;
    for (size_t i = 0; i < ids.size(); ++i)
    {
        if (i > 0) s += ";";
        s += std::to_string(ids[i]);
    }
    return s;
}

void SkeletonGraph::exportReportCSV(const std::string& filename) const
{
    if (!hasRoot())
        return;

    std::ofstream out(filename, std::ofstream::out | std::ofstream::trunc);
    out << "id,x,y,z,parentId,childrenIds,pathFromRoot,loopParentIds\n";
    out << std::fixed << std::setprecision(6);

    for (const SkeletonNode& n : m_nodes)
    {
        const auto& p = n.position();
        const auto& parents = n.parentIds();

        int primaryParent = parents.empty() ? -1 : parents.front();

        std::vector<int> loopParents;
        if (parents.size() > 1)
            loopParents.assign(parents.begin() + 1, parents.end());

        std::vector<int> path = pathFromRoot(n.id());

        out << n.id() << ","
            << p[0] << "," << p[1] << "," << p[2] << ","
            << primaryParent << ","
            << "\"" << joinIds(n.childrenIds()) << "\","
            << "\"" << joinIds(path) << "\","
            << "\"" << joinIds(loopParents) << "\"\n";
    }
}

// --- Liver-segment mapping ---------------------------------------------------

void SkeletonGraph::assignSegmentLabels(const std::vector<int>& rawNodeLabels)
{
    const int n = static_cast<int>(m_nodes.size());
    m_nodeSegment.assign(n, -1);

    if (!hasRoot() || rawNodeLabels.empty())
        return;

    const std::vector<int>& raw = rawNodeLabels;

    // Walk the tree from the root, splitting it into maximal branches: a
    // branch runs from the root (or right after a branch point) down to the
    // next branch point or leaf. Every node in a branch gets that branch's
    // majority raw label, so a few mislabeled nodes near a segment boundary
    // don't fragment an otherwise-clear branch.
    std::vector<int> stack;
    stack.push_back(m_rootId);

    while (!stack.empty())
    {
        int start = stack.back();
        stack.pop_back();

        std::vector<int> chain;
        int cur = start;
        while (true)
        {
            chain.push_back(cur);
            const SkeletonNode* curNode = node(cur);
            const auto& children = curNode->childrenIds();

            if (children.size() == 1)
            {
                cur = children.front();
                continue;
            }

            // Reached a leaf (0 children) or a branch point (>1 children):
            // end this chain here, and start a fresh chain at each child.
            for (int c : children)
                stack.push_back(c);
            break;
        }

        std::map<int, int> counts;
        for (int id : chain)
            if (id < static_cast<int>(raw.size()) && raw[id] != -1)
                ++counts[raw[id]];

        int majority = -1;
        int best = 0;
        for (const auto& kv : counts)
        {
            if (kv.second > best)
            {
                best = kv.second;
                majority = kv.first;
            }
        }

        for (int id : chain)
            m_nodeSegment[id] = majority;
    }
}

int SkeletonGraph::segmentOf(int nodeId) const
{
    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodeSegment.size()))
        return -1;
    return m_nodeSegment[nodeId];
}

std::vector<int> SkeletonGraph::nodesInSegment(int segment) const
{
    std::vector<int> result;
    for (const SkeletonNode& n : m_nodes)
        if (segmentOf(n.id()) == segment)
            result.push_back(n.id());
    return result;
}

void SkeletonGraph::exportSegmentReportCSV(const std::string& filename) const
{
    if (!hasRoot())
        return;

    std::ofstream out(filename, std::ofstream::out | std::ofstream::trunc);
    out << "id,x,y,z,segment\n";
    out << std::fixed << std::setprecision(6);

    for (const SkeletonNode& n : m_nodes)
    {
        const auto& p = n.position();
        out << n.id() << ","
            << p[0] << "," << p[1] << "," << p[2] << ","
            << segmentOf(n.id()) << "\n";
    }
}

void SkeletonGraph::exportSegmentReportCSV(const std::string& filename, const std::vector<std::string>& segmentNames) const
{
    if (!hasRoot())
        return;

    std::set<int> distinctSegments;
    for (const SkeletonNode& n : m_nodes)
    {
        int seg = segmentOf(n.id());
        if (seg != -1)
            distinctSegments.insert(seg);
    }

    std::ofstream out(filename, std::ofstream::out | std::ofstream::trunc);
    out << "# totalNodes=" << m_nodes.size() << ",distinctSegments=" << distinctSegments.size() << "\n";
    out << "id,x,y,z,segment\n";
    out << std::fixed << std::setprecision(6);

    for (const SkeletonNode& n : m_nodes)
    {
        const auto& p = n.position();
        int seg = segmentOf(n.id());

        std::string name = "unknown";
        if (seg >= 0 && seg < static_cast<int>(segmentNames.size()))
            name = segmentNames[seg];

        out << n.id() << ","
            << p[0] << "," << p[1] << "," << p[2] << ","
            << name << "\n";
    }
}

std::vector<int> SkeletonGraph::simulateResection(const std::vector<int>& cutNodeIds) const
{
    std::vector<int> affected;
    if (!hasRoot())
        return affected;

    std::set<int> cutSet(cutNodeIds.begin(), cutNodeIds.end());

    // If the root itself is cut, nothing downstream can be perfused any more.
    if (cutSet.count(m_rootId))
    {
        affected.reserve(m_nodes.size());
        for (const SkeletonNode& n : m_nodes)
            affected.push_back(n.id());
        return affected;
    }

    // Reachability BFS over the raw connectivity graph (which already
    // includes any collateral/loop edges): a cut node is simply never
    // pushed/expanded, which is equivalent to deleting it and all of its
    // edges from the graph. Anything still reachable from the root through
    // some other path (e.g. an anastomosis bypassing the cut) stays
    // perfused; everything else is affected.
    std::vector<bool> reachable(m_nodes.size(), false);
    std::queue<int> q;
    reachable[m_rootId] = true;
    q.push(m_rootId);

    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        auto it = m_adjacency.find(u);
        if (it == m_adjacency.end())
            continue;

        for (int v : it->second)
        {
            if (cutSet.count(v) || reachable[v])
                continue;
            reachable[v] = true;
            q.push(v);
        }
    }

    for (const SkeletonNode& n : m_nodes)
        if (cutSet.count(n.id()) || !reachable[n.id()])
            affected.push_back(n.id());

    return affected;
}

std::vector<int> SkeletonGraph::affectedSegments(const std::vector<int>& affectedNodeIds) const
{
    std::set<int> segments;
    for (int id : affectedNodeIds)
    {
        int seg = segmentOf(id);
        if (seg != -1)
            segments.insert(seg);
    }
    return std::vector<int>(segments.begin(), segments.end());
}

} // namespace meshskeletonizationplugin