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

    /// Loads a skeleton already exported as VTK POLYDATA by exportToVTK() (points +
    /// "parent child" LINES + a "type" POINT_DATA scalar marking the root with value
    /// 1). Unlike loadFromFile(), the tree structure (parent/children ids, root) is
    /// read directly from the file instead of recomputed by buildTree(); other scalar
    /// fields (depth, is_loop) are read past and ignored. Returns false if the file
    /// could not be opened or no root ("type"==1) was found.
    bool loadFromVTK(const std::string& filename);

    /// Picks the node closest to entryPoint as root and (re)computes every
    /// node's parentIds/childrenIds via BFS over the raw connectivity.
    void buildTree(const std::array<double, 3>& entryPoint);
    void buildTree(int rootId);

    /// Picks a root automatically, with no entry point needed: the node
    /// belonging to the LARGEST connected component of the raw connectivity
    /// graph. Real vessel skeletonization output is typically one dominant
    /// tree plus a handful of small disconnected fragments (noise); this
    /// guarantees the tree is rooted in the dominant structure instead of
    /// risking a tiny isolated fragment, which is easy to hit by accident
    /// with a naive nearest-point-to-some-coordinate pick (e.g. a caller
    /// falling back to the origin when no real entry point is known).
    void buildTreeAutoRoot();

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

    // --- Liver-segment mapping -------------------------------------------------

    /// Assigns each node a liver-segment id (e.g. an index into a list of
    /// Couinaud segments) from `rawNodeLabels`, indexed the same way as
    /// nodes() (rawNodeLabels[nodeId], -1 = unknown/undetermined for that
    /// node, e.g. because it fell outside every segment mesh). Typically
    /// produced externally per node, e.g. via a point-in-mesh test against a
    /// set of closed segment meshes (see SkeletonSegmentMapper).
    ///
    /// Raw per-node labels can be noisy near segment boundaries, so the tree
    /// is split into maximal branches (runs of nodes between the root,
    /// branch points, and leaves) and every node in a branch is set to that
    /// branch's majority raw label. Requires the tree to have a root
    /// (buildTree() or loadFromVTK()). Branches with no labeled node at all
    /// get -1.
    void assignSegmentLabels(const std::vector<int>& rawNodeLabels);

    /// Segment id assigned to a node by the last assignSegmentLabels() call,
    /// or -1 if it hasn't been run (or the node has no label).
    int segmentOf(int nodeId) const;

    /// Ids of all nodes currently assigned to the given segment.
    std::vector<int> nodesInSegment(int segment) const;

    /// Writes a CSV report (id, x, y, z, segment) reflecting the last
    /// assignSegmentLabels() call.
    void exportSegmentReportCSV(const std::string& filename) const;

    /// Same as above, but resolves each segment id to a human-readable name
    /// via `segmentNames` (segmentNames[segmentId]); ids without a matching
    /// entry, and -1 (unknown), are written as "unknown".
    void exportSegmentReportCSV(const std::string& filename, const std::vector<std::string>& segmentNames) const;

    // --- Resection (devascularization) simulation -------------------------------

    /// Simulates a resection (vessel transection) at each of `cutNodeIds`:
    /// every cut node is fully removed from the connectivity graph (all of
    /// its edges, both primary tree and any loop/anastomosis edges), then
    /// reachability from the root is recomputed over what remains. This is
    /// deliberately NOT the same as subtree(): a node below a cut can still
    /// be reachable (and so still perfused) if a collateral/anastomosis
    /// connects it back to the root through a path that avoids every cut
    /// node - subtree() would incorrectly flag it as affected.
    ///
    /// Returns every node that can no longer reach the root (i.e. would lose
    /// blood supply), plus the cut nodes themselves. If the root itself is
    /// cut, every node is returned. Requires hasRoot().
    std::vector<int> simulateResection(const std::vector<int>& cutNodeIds) const;

    /// Convenience: the distinct liver segments touched by `affectedNodeIds`
    /// (typically the result of simulateResection()) - i.e. the segments
    /// that would lose blood supply. Requires assignSegmentLabels() to have
    /// been called already; nodes with no segment (-1) are ignored.
    std::vector<int> affectedSegments(const std::vector<int>& affectedNodeIds) const;

private:
    int findOrCreateNode(const std::array<double, 3>& p, double tol);
    int closestNodeId(const std::array<double, 3>& p) const;
    void connect(int a, int b);

    std::vector<SkeletonNode> m_nodes;
    std::map<int, std::vector<int>> m_adjacency; ///< raw undirected connectivity from the file
    std::vector<std::pair<int, int>> m_loopEdges;
    int m_rootId{ -1 };

    /// Segment id per node (parallel to m_nodes, indexed by node id), filled by
    /// assignSegmentLabels(). Empty until that has been called at least once.
    std::vector<int> m_nodeSegment;
};

} // namespace meshskeletonizationplugin