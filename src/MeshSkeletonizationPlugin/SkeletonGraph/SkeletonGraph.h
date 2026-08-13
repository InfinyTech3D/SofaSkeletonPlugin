#pragma once

#include "SkeletonNode.h"

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace meshskeletonizationplugin
{

/// Loads a polyline skeleton file (the format written by
/// MeshSkeletonization/SkeletonizationLoader's Export_polylines: blocks of
/// "x y z" lines separated by a blank line) and stores it as a graph of
/// SkeletonNode, then turns it into a rooted parent/children tree.
class SkeletonGraph
{
public:
    /// Parses the file, merging points that are within `mergeTolerance` of an
    /// already-seen point into a single node. Returns false if the file could
    /// not be opened. Only fills raw connectivity; call buildTree() afterwards
    /// to populate parent/children ids on the nodes.
    bool loadFromFile(const std::string& filename, double mergeTolerance = 1e-6);

    /// Picks the node closest to entryPoint as root and (re)computes every
    /// node's parentIds/childrenIds via BFS over the raw connectivity.
    void buildTree(const std::array<double, 3>& entryPoint);
    void buildTree(int rootId);

    /// Best-effort correspondence between each skeleton node and the closest
    /// vertex of the input (vessel) mesh; fills meshVertexId/distanceToMesh.
    void computeMeshCorrespondence(const std::vector<std::array<double, 3>>& meshVertices);

    const std::vector<SkeletonNode>& nodes() const { return m_nodes; }
    std::vector<SkeletonNode>& nodes() { return m_nodes; }

    int rootId() const { return m_rootId; }
    bool hasRoot() const { return m_rootId >= 0; }

    /// Ids of nodes connected by a loop/anastomosis (extra edge beyond the tree).
    const std::vector<std::pair<int, int>>& loopEdges() const { return m_loopEdges; }
    //newzest additions 
    const SkeletonNode* node(int nodeId) const; 
    bool parentsOf(int nodeId, std::vector<int>& parents) const;
    bool childrenOf(int nodeId, std::vector<int>& children) const;     
    std::vector<int> pathFromRoot(int nodeId) const; 
    std::vector<int> subtree(int nodeId) const;
    void exportReportCSV(const std::string& filename) const; //< export CSV report using @sa d_outReportFilename that will give out the ids, 3D coordinates, parent vertex and child vertex
    void exportToVTK(const std::string& filename) const;
    void clear();

private:
    int findOrCreateNode(const std::array<double, 3>& p, double tol);
    int closestNodeId(const std::array<double, 3>& p) const;
    void connect(int a, int b);

    std::vector<SkeletonNode> m_nodes;
    std::map<int, std::vector<int>> m_adjacency; ///< raw undirected connectivity from the file
    std::vector<std::pair<int, int>> m_loopEdges;
    int m_rootId{ -1 };
};

} // namespace meshskeletonizationplugin
