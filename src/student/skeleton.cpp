
#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    if(start == end) return end; // not a line segment

    Vec3 sp = point - start; // a
    Vec3 se = end - start;   // b

    // Component of sp along se (acos(theta))
    float t = dot(sp, se) / se.norm_squared(); // a * cos(theta) = a.b / |b| * b / |b|

    if(t <= 0)
        return start; // far left
    else if(t >= 1)
        return end; // far right
    else
        return t * se + start; // in between
    // return Vec3{};
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    Mat4 T = Mat4::I;
    const Joint* j = this;
    while(!j->is_root()) {
        j = j->parent;
        T = Mat4::translate(j->extent) * T;
    }

    return T;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    Mat4 T = Mat4::euler(pose);
    const Joint* j = this;
    while(!j->is_root()) {
        j = j->parent;
        T = Mat4::euler(j->pose) * Mat4::translate(j->extent) * T;
    }
    return T;
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return j->joint_to_bind() * j->extent + base_pos;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return j->joint_to_posed() * j->extent + base_pos;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.

    return Mat4::translate(base_pos) * j->joint_to_bind();
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.
    return Mat4::translate(base_pos) * j->joint_to_posed();
}

void Skeleton::find_joints(const GL::Mesh& mesh, std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping: vertex index -> list of joints that should effect the vertex.
    // A joint should effect a vertex if it is within Joint::radius distance of the
    // bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    map.resize(verts.size());

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for_joints([&](Joint* j) {
        // What vertices does joint j effect?
        Mat4 j2b = joint_to_bind(j); // account for root node's position in mesh space
        Mat4 b2j = j2b.inverse();
        Vec3 start;
        Vec3 end = j->extent;

        unsigned int i = 0;
        for(auto v : verts) {
            Vec3 pos = b2j * v.pos;
            Vec3 closest = closest_on_line_segment(start, end, pos);
            if((closest - pos).norm() <= j->radius) {
                map[i].push_back(j); // creates if not found.
            }
            i += 1;
        }
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();

    for(size_t i = 0; i < verts.size(); i++) {

        // Skin vertex i. Note that its position is given in object bind space.
        unsigned int ui = input.indices()[i];

        GL::Mesh::Vert v = verts[i]; // mesh vertex
        float w = 0.f;               // total weights
        Vec3 newPos;                 // new position
        Vec3 newNorm;                // new normal

        if(!(map.at(ui).size() > 0)) {
            continue;
        }

        for(auto j : map.at(ui)) {
            Mat4 j2p = joint_to_posed(j); // account for root node's position in mesh space
            Mat4 b2p = j2p * joint_to_bind(j).inverse(); // bind to posed

            Vec3 start = j2p * Vec3();
            Vec3 end = j2p * j->extent;
            Vec3 point = b2p * v.pos;
            Vec3 norm = b2p * v.norm - point;

            Vec3 closest = closest_on_line_segment(start, end, point);

            float wij = 1.f / ((closest - point).norm());
            w += wij;
            newPos += (wij * point);
            newNorm += (wij * norm);
        }

        newPos /= w;
        newNorm /= w;
        verts[i].pos = newPos;
        verts[i].norm = newNorm;
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
    const Mat4 j2p = joint_to_posed();

    Vec3 c = current;
    Vec3 t = target;
    Vec3 o = j2p * Vec3();
    Vec3 p = c - o;

    Vec3 x_axis = Vec3(1.f, 0.f, 0.f);
    Vec3 y_axis = Vec3(0.f, 1.f, 0.f);
    Vec3 z_axis = Vec3(0.f, 0.f, 1.f);

    x_axis = (j2p * x_axis - o).unit();
    y_axis = (j2p * y_axis - o).unit();
    z_axis = (j2p * z_axis - o).unit();

    angle_gradient.x += dot(cross(x_axis, p), (c - t));
    angle_gradient.y += dot(cross(y_axis, p), (c - t));
    angle_gradient.z += dot(cross(z_axis, p), (c - t));

    if(parent != nullptr) parent->compute_gradient(target, current);
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    const float time_step = 0.01f;
    for(int i = 0; i < 100; i++) {
        for(auto h = active_handles.begin(); h != active_handles.end(); h++) {
            Vec3 current = posed_end_of((*h)->joint) - base_pos;
            (*h)->joint->compute_gradient((*h)->target, current);

            for(Joint* j = (*h)->joint; j != nullptr; j = j->parent) {
                j->pose -= time_step * (j->angle_gradient);
                j->angle_gradient = Vec3();
            }
        }
    }
}
