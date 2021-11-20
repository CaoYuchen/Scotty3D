
#pragma once

#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "trace.h"

namespace PT {

template<typename Primitive> class BVH {
public:
    BVH() = default;
    BVH(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);
    void build(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);

    BVH(BVH&& src) = default;
    BVH& operator=(BVH&& src) = default;

    BVH(const BVH& src) = delete;
    BVH& operator=(const BVH& src) = delete;

    BBox bbox() const;
    Trace hit(const Ray& ray) const;

    BVH copy() const;
    size_t visualize(GL::Lines& lines, GL::Lines& active, size_t level, const Mat4& trans) const;

    std::vector<Primitive> destructure();
    void clear();

    void construct(size_t root_idx, size_t max_leaf_size);
    std::pair<float, size_t> eval_cost(size_t root_idx, size_t xyz);
    // bool sort_x(Primitive a, Primitive b) {
    //     return (a.bbox().center().x < b.bbox().center().x);
    // }
    // bool sort_y(Primitive a, Primitive b) {
    //     return (a.bbox().center().y < b.bbox().center().y);
    // }
    // bool sort_z(Primitive a, Primitive b) {
    //     return (a.bbox().center().z < b.bbox().center().z);
    // }

private:
    class Node {

        BBox bbox;
        size_t start, size, l, r;

        // A node is a leaf if l == r, since all interior nodes must have distinct children
        bool is_leaf() const;
        friend class BVH<Primitive>;
    };
    size_t new_node(BBox box = {}, size_t start = 0, size_t size = 0, size_t l = 0, size_t r = 0);

    std::vector<Node> nodes;
    std::vector<Primitive> primitives;
    size_t root_idx = 0;
};

} // namespace PT

#ifdef SCOTTY3D_BUILD_REF
#include "../reference/bvh.inl"
#else
#include "../student/bvh.inl"
#endif
