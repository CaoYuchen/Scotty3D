
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these
    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());

    new_node(box, 0, primitives.size(), 0, 0);
    root_idx = 0;

    construct(max_leaf_size);
    return;
}

template<typename Primitive> void BVH<Primitive>::construct(size_t max_leaf_size) {
    float cost_x, cost_y, cost_z, cost_min;
    std::vector<Primitive> left_prims, right_prims;
    BBox left_box, right_box;
    // size_t root_left, root_right;
    size_t i_min, i_x, i_y, i_z;
    size_t root_temp = root_idx;

    if(nodes[root_idx].size <= max_leaf_size) {
        return;
    }
    // cost along x axis
    std::sort(primitives.begin() + nodes[root_idx].start,
              primitives.begin() + nodes[root_idx].start + nodes[root_idx].size,
              [](const Primitive& a, const Primitive& b) {
                  return (a.bbox().center().x < b.bbox().center().x);
              });
    std::tie(cost_x, i_x) = eval_cost();
    // cost along y axis
    std::sort(primitives.begin() + nodes[root_idx].start,
              primitives.begin() + nodes[root_idx].start + nodes[root_idx].size,
              [](const Primitive& a, const Primitive& b) {
                  return (a.bbox().center().y < b.bbox().center().y);
              });
    std::tie(cost_y, i_y) = eval_cost();
    // cost along z axis
    std::sort(primitives.begin() + nodes[root_idx].start,
              primitives.begin() + nodes[root_idx].start + nodes[root_idx].size,
              [](const Primitive& a, const Primitive& b) {
                  return (a.bbox().center().z < b.bbox().center().z);
              });
    std::tie(cost_z, i_z) = eval_cost();

    // find the minimum of x,y,z
    cost_min = std::min(cost_x, std::min(cost_y, cost_z));
    if(cost_min == cost_x) {
        i_min = i_x;
        std::sort(primitives.begin() + nodes[root_idx].start,
                  primitives.begin() + nodes[root_idx].start + nodes[root_idx].size,
                  [](const Primitive& a, const Primitive& b) {
                      return (a.bbox().center().x < b.bbox().center().x);
                  });

    } else if(cost_min == cost_y) {
        i_min = i_y;
        std::sort(primitives.begin() + nodes[root_idx].start,
                  primitives.begin() + nodes[root_idx].start + nodes[root_idx].size,
                  [](const Primitive& a, const Primitive& b) {
                      return (a.bbox().center().y < b.bbox().center().y);
                  });
    } else {
        i_min = i_z;
        std::sort(primitives.begin() + nodes[root_idx].start,
                  primitives.begin() + nodes[root_idx].start + nodes[root_idx].size,
                  [](const Primitive& a, const Primitive& b) {
                      return (a.bbox().center().z < b.bbox().center().z);
                  });
    }

    for(size_t l = nodes[root_idx].start; l < i_min + 1; l++) {
        left_box.enclose(primitives[l].bbox());
    }
    for(size_t r = i_min + 1; r < nodes[root_idx].start + nodes[root_idx].size; r++) {
        right_box.enclose(primitives[r].bbox());
    }

    nodes[root_idx].l = nodes[root_idx].start;
    nodes[root_idx].r = i_min + 1;

    root_idx = new_node(left_box, nodes[root_temp].start, i_min + 1 - nodes[root_temp].start, 0, 0);

    construct(max_leaf_size);
    root_idx = new_node(right_box, i_min + 1, nodes[root_temp].size - (i_min + 1), 0, 0);
    construct(max_leaf_size);
    return;
}

template<typename Primitive> std::pair<float, size_t> BVH<Primitive>::eval_cost() {
    float cost, cost_min = FLT_MAX;
    size_t i_min;
    BBox box, left_box, right_box;
    std::vector<Primitive> left_prims, right_prims;

    for(size_t i = nodes[root_idx].start; i < nodes[root_idx].start + nodes[root_idx].size; i++) {
        for(size_t j = nodes[root_idx].start; j < i + 1; j++) {
            left_box.enclose(primitives[j].bbox());
            // overall box
            box.enclose(primitives[j].bbox());
        }
        for(size_t k = i + 1; k < nodes[root_idx].start + nodes[root_idx].size; k++) {
            right_box.enclose(primitives[k].bbox());
            box.enclose(primitives[k].bbox());
        }
        // for(const Primitive& prim : left_prims) left_box.enclose(prim.bbox());
        // for(const Primitive& prim : right_prims) right_box.enclose(prim.bbox());
        cost = 1.0f + left_box.surface_area() / box.surface_area() * left_prims.size() +
               right_box.surface_area() / box.surface_area() * right_prims.size();
        if(cost < cost_min) {
            cost_min = cost;
            i_min = i;
        }
    }
    return std::make_pair(cost_min, i_min);
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive
    // in the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.
    static size_t root = 0;
    Trace ret, ret_temp;

    if(nodes[root].is_leaf()) {
        size_t start = nodes[root].start;
        size_t end = nodes[root].start + nodes[root].size;
        Vec2 t;
        for(size_t i = start; i < end; i++) {
            Trace hit = primitives[i].hit(ray);
            ret = Trace::min(ret, hit);
        }
    } else {
        Vec2 t1, t2, t_second;
        size_t first, second;
        size_t left = nodes[root].l;
        size_t right = nodes[root].r;
        primitives[left].bbox().hit(ray, t1);
        primitives[right].bbox().hit(ray, t2);

        if(t1.x >= t2.x) {
            first = right;
            second = left;
            t_second = t1;
        } else {
            first = left;
            second = right;
            t_second = t2;
        }
        root = first;
        ret = hit(ray);
        if(t_second.x < ret.distance) {
            root = second;
            ret_temp = hit(ray);
        }
        if(ret_temp.hit) {
            ret = ret_temp;
        }
    }

    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    // A node is a leaf if l == r, since all interior nodes must
    // have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {
    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
