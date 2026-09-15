#pragma once
// Reuses the Kernel/Polyhedron/Point/HalfedgeDS typedefs already declared (at global scope) in MeshSkeletonization.h.
#include <MeshSkeletonizationPlugin/MeshSkeletonization.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <memory>

namespace meshskeletonizationplugin
{
namespace cgalutils
{
    /// Builds a CGAL Polyhedron_3 from a flat vertex/triangle mesh, mirroring
    /// MeshSkeletonization::geometryToPolyhedronOp but as a free/shared helper
    /// so multiple components can build closed meshes without duplicating it.
    template <class VecCoord, class SeqTriangles>
    class MeshToPolyhedronOp : public CGAL::Modifier_base<HalfedgeDS>
    {
    public:
        MeshToPolyhedronOp(const VecCoord& vertices, const SeqTriangles& triangles)
            : m_vertices(vertices), m_triangles(triangles)
        {
        }

        void operator()(HalfedgeDS& hds) override
        {
            CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder(hds, true);
            builder.begin_surface(m_vertices.size(), m_triangles.size());

            for (const auto& v : m_vertices)
                builder.add_vertex(Point(v[0], v[1], v[2]));

            for (const auto& tri : m_triangles)
            {
                builder.begin_facet();
                for (int j = 0; j < 3; ++j)
                    builder.add_vertex_to_facet(tri[j]);
                builder.end_facet();
            }

            if (builder.check_unconnected_vertices())
                builder.remove_unconnected_vertices();

            builder.end_surface();
        }

    private:
        const VecCoord& m_vertices;
        const SeqTriangles& m_triangles;
    };

    using AABBTraits = CGAL::AABB_traits<Kernel, CGAL::AABB_face_graph_triangle_primitive<Polyhedron>>;
    using AABBTree = CGAL::AABB_tree<AABBTraits>;
    using PointInsideTest = CGAL::Side_of_triangle_mesh<Polyhedron, Kernel>;

    /// A closed mesh (segment, tumor, ...) ready for point-in-mesh and
    /// closest-point queries: build a Polyhedron_3 from raw vertices/
    /// triangles, then an AABB tree + inside/outside test backed by it.
    struct ClosedMeshQuery
    {
        Polyhedron polyhedron;
        std::unique_ptr<AABBTree> tree;
        std::unique_ptr<PointInsideTest> insideTest;

        void build()
        {
            tree = std::make_unique<AABBTree>(CGAL::faces(polyhedron).first, CGAL::faces(polyhedron).second, polyhedron);
            tree->accelerate_distance_queries();
            insideTest = std::make_unique<PointInsideTest>(*tree);
        }

        /// Convenience: build the polyhedron from flat vertices/triangles and
        /// immediately build the tree/inside-test on top of it. No-ops (and
        /// leaves tree/insideTest null) if the resulting polyhedron is empty.
        template <class VecCoord, class SeqTriangles>
        void buildFrom(const VecCoord& vertices, const SeqTriangles& triangles)
        {
            MeshToPolyhedronOp<VecCoord, SeqTriangles> op(vertices, triangles);
            polyhedron.delegate(op);
            if (!polyhedron.is_empty())
                build();
        }
    };

} // namespace cgalutils
} // namespace meshskeletonizationplugin
