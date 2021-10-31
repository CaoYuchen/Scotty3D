
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    if(v->on_boundary()) {
        (void)v;
        return std::nullopt;
    }

    std::vector<Halfedge_Mesh::HalfedgeRef> hf;
    std::vector<Halfedge_Mesh::HalfedgeRef> hd;
    std::vector<Halfedge_Mesh::EdgeRef> ve;
    std::vector<Halfedge_Mesh::FaceRef> vf;
    Halfedge_Mesh::HalfedgeRef temp = v->halfedge();
    // cache all the edge, face, halfedge that contains v
    do {
        hd.push_back(temp);
        hd.push_back(temp->twin());
        vf.push_back(temp->face());
        ve.push_back(temp->edge());
        temp = temp->twin()->next();
    } while(temp != v->halfedge());
    // go through every halfedge and cache them
    Halfedge_Mesh::HalfedgeRef ht = v->halfedge()->next();
    hf.push_back(ht);
    do {
        if(ht->next()->next()->vertex() != v) {
            hf.push_back(ht->next());
            ht = ht->next();
        } else {
            ht = ht->next()->twin();
        }
    } while(ht != v->halfedge()->next());

    Halfedge_Mesh::FaceRef f = new_face();
    f->halfedge() = hf[0];
    for(size_t i = 0; i < hf.size(); i++) {
        hf[i]->next() = hf[(i - 1 + hf.size()) % hf.size()];

        hf[i]->face() = f;
        hf[i]->vertex()->halfedge() = hf[i];
    }
    // erase all edges, haledges, faces that connect the vertex
    for(size_t i = 0; i < hd.size(); i++) {
        erase(hd[i]);
    }

    for(size_t i = 0; i < ve.size(); i++) {
        erase(ve[i]);
    }

    for(size_t i = 0; i < vf.size(); i++) {
        erase(vf[i]);
    }

    erase(v);

    return f;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    if(e->on_boundary()) {
        (void)e;
        return std::nullopt;
    }

    Halfedge_Mesh::FaceRef f0 = e->halfedge()->face();
    Halfedge_Mesh::HalfedgeRef h0 = e->halfedge();
    Halfedge_Mesh::HalfedgeRef h1 = h0->twin();

    // modify the halfedge of two vertices of deleted edge
    h0->vertex()->halfedge() = h1->next();
    h1->vertex()->halfedge() = h0->next();
    // modify face0's halfedge to something not deleted.
    f0->halfedge() = h0->next();

    // to get h1->prev and assign it's next to h0->next
    Halfedge_Mesh::HalfedgeRef h = h1->next();
    h->face() = f0;
    do {
        h = h->next();
        h->face() = f0;
    } while(h->next() != h1);
    h->next() = h0->next();

    //  to get h0->prev and assign it's next to h1->next
    h = h0->next();
    do {
        h = h->next();
    } while(h->next() != h0);
    h->next() = h1->next();

    // Delete edge, face 1, and h0 & h1
    erase(e);
    erase(h1->face());
    erase(h0);
    erase(h1);
    return f0;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    if(e->on_boundary()) {
        return e->halfedge()->vertex();
    }

    Halfedge_Mesh::HalfedgeRef h = e->halfedge();
    Halfedge_Mesh::HalfedgeRef old_h = h;
    Halfedge_Mesh::HalfedgeRef h0 = h->twin();
    Halfedge_Mesh::FaceRef f0 = h->face();
    Halfedge_Mesh::FaceRef f1 = h0->face();
    // check if edge is in triangle
    bool right = false;
    bool left = false;
    if(f0->degree() == 3) {
        right = true;
    }
    if(f1->degree() == 3) {
        left = true;
    }

    // set the center vertex
    Halfedge_Mesh::VertexRef v = h->vertex();
    Halfedge_Mesh::VertexRef v0 = h0->vertex();
    v->pos = e->center();
    v->halfedge() = h0->next()->twin()->next();
    v0->halfedge() = h->next()->twin()->next();

    if(right) {
        if(h->next()->edge()->on_boundary() || h->next()->next()->edge()->on_boundary()) {
            return e->halfedge()->vertex();
        }
        // get related halfedges
        Halfedge_Mesh::HalfedgeRef r1 = h->next();
        Halfedge_Mesh::HalfedgeRef r2 = r1->next();
        Halfedge_Mesh::HalfedgeRef r3 = r2->twin();
        Halfedge_Mesh::HalfedgeRef r4 = r1->twin();
        // get related edge
        Halfedge_Mesh::EdgeRef er1 = r2->edge();
        Halfedge_Mesh::EdgeRef er2 = r1->edge();
        // get related vertex
        Halfedge_Mesh::VertexRef v1 = r2->vertex();

        // reassign value
        v1->halfedge() = r3->next();
        r3->twin() = r4;
        r4->twin() = r3;
        r4->edge() = er1;
        er1->halfedge() = r3;

        h = r4->next();

        erase(r1);
        erase(r2);
        erase(er2);
    } else {
        // get related halfedges
        Halfedge_Mesh::HalfedgeRef r1 = h->next();
        Halfedge_Mesh::HalfedgeRef r2 = h;
        while(r2->next() != h) {
            r2 = r2->next();
        }
        // reassign value
        r2->next() = r1;
        r1->vertex() = v;
        r1->face()->halfedge() = r1;

        h = r1->twin()->next();
    }

    // update the bottom vertex to the returned vertex
    // Halfedge_Mesh::HalfedgeRef temp1 = h->next()->twin()->next();
    Halfedge_Mesh::HalfedgeRef temp1 = h;
    while(temp1 != h0) {
        temp1->vertex() = v;
        temp1 = temp1->twin()->next();
    }
    if(left) {
        if(h0->next()->edge()->on_boundary() || h0->next()->next()->edge()->on_boundary()) {
            return e->halfedge()->vertex();
        }
        // get related halfedges
        Halfedge_Mesh::HalfedgeRef l1 = h0->next();
        Halfedge_Mesh::HalfedgeRef l2 = l1->next();
        Halfedge_Mesh::HalfedgeRef l3 = l2->twin();
        Halfedge_Mesh::HalfedgeRef l4 = l1->twin();
        // get related edge
        Halfedge_Mesh::EdgeRef el1 = l1->edge();
        Halfedge_Mesh::EdgeRef el2 = l2->edge();
        // get related vertex
        Halfedge_Mesh::VertexRef v2 = l2->vertex();

        // reassign value
        v2->halfedge() = l3->next();
        l3->twin() = l4;
        l4->twin() = l3;
        l3->edge() = el1;
        el1->halfedge() = l4;
        // l3->vertex() = v;
        erase(l1);
        erase(l2);
        erase(el2);
    } else {
        // get related halfedges
        Halfedge_Mesh::HalfedgeRef l1 = h0->next();
        Halfedge_Mesh::HalfedgeRef l2 = h0;
        while(l2->next() != h0) {
            l2 = l2->next();
        }
        // reassign value
        l2->next() = l1;
        l1->face()->halfedge() = l1;
    }
    // deleteHalfedge(h);
    erase(old_h);
    erase(h0);
    erase(e);
    erase(v0);
    if(right) {
        erase(f0);
    }
    if(left) {
        erase(f1);
    }
    return v;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    if(e->on_boundary()) {
        (void)e;
        return std::nullopt;
    }

    Halfedge_Mesh::HalfedgeRef h0 = e->halfedge();
    Halfedge_Mesh::HalfedgeRef h1 = h0->next();
    Halfedge_Mesh::HalfedgeRef h2 = h1->next();
    Halfedge_Mesh::HalfedgeRef h3 = h0->twin();
    Halfedge_Mesh::HalfedgeRef h4 = h3->next();
    Halfedge_Mesh::HalfedgeRef h5 = h4->next();
    Halfedge_Mesh::HalfedgeRef h6 = h1->twin();
    Halfedge_Mesh::HalfedgeRef h7 = h4->twin();
    Halfedge_Mesh::HalfedgeRef h8 = h3;
    while(h8->next() != h3) {
        h8 = h8->next();
    }
    Halfedge_Mesh::HalfedgeRef h9 = h0;
    while(h9->next() != h0) {
        h9 = h9->next();
    }

    Halfedge_Mesh::VertexRef v0 = h0->vertex();
    Halfedge_Mesh::VertexRef v1 = h3->vertex();
    Halfedge_Mesh::VertexRef v2 = h6->vertex();
    Halfedge_Mesh::VertexRef v3 = h7->vertex();

    Halfedge_Mesh::FaceRef f0 = h0->face();
    Halfedge_Mesh::FaceRef f1 = h3->face();

    v0->halfedge() = h4;
    v1->halfedge() = h1;
    v2->halfedge() = h2;
    v3->halfedge() = h5;
    f0->halfedge() = h2;
    f1->halfedge() = h5;

    h0->next() = h2;
    h0->twin() = h3;
    h0->vertex() = v3;
    h0->face() = f0;
    h1->next() = h3;

    h1->face() = f1;
    if(h2->next() == h0) {
        h2->next() = h4;
    }

    h3->next() = h5;
    h3->twin() = h0;
    h3->vertex() = v2;
    h3->face() = f1;
    h4->next() = h0;

    h4->face() = f0;
    h8->next() = h1;

    h9->next() = h4;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    if(e->on_boundary()) {
        if(e->halfedge()->face()->degree() == 3) {
            Halfedge_Mesh::VertexRef m = new_vertex();
            std::vector<Halfedge_Mesh::EdgeRef> ve;
            Halfedge_Mesh::FaceRef nf = new_face();
            std::vector<Halfedge_Mesh::HalfedgeRef> vh;
            ve.push_back(new_edge());
            ve.push_back(new_edge());
            vh.push_back(new_halfedge());
            vh.push_back(new_halfedge());
            vh.push_back(new_halfedge());
            vh.push_back(new_halfedge());

            // position for the new vertex
            m->pos = e->center();
            Halfedge_Mesh::HalfedgeRef h0 = e->halfedge()->twin();
            Halfedge_Mesh::HalfedgeRef h1 = e->halfedge()->next();
            Halfedge_Mesh::HalfedgeRef h2 = h1->next();
            Halfedge_Mesh::HalfedgeRef h3 = h0->next();

            Halfedge_Mesh::FaceRef of = e->halfedge()->face();

            of->halfedge() = h2;
            nf->halfedge() = h1;
            Halfedge_Mesh::FaceRef face_boundary = h0->face();

            e->halfedge()->next() = vh[2];
            e->halfedge()->twin() = vh[1];
            e->halfedge()->edge() = ve[0];

            h1->next() = vh[3];
            h1->face() = nf;

            h0->next() = vh[1];
            h0->twin() = vh[0];

            ve[0]->halfedge() = h0;
            ve[1]->halfedge() = vh[2];
            m->halfedge() = vh[0];

            vh[0]->Halfedge_Mesh::Halfedge::set_neighbors(h1, h0, m, ve[0], nf);
            vh[1]->Halfedge_Mesh::Halfedge::set_neighbors(h3, e->halfedge(), m, e, face_boundary);
            vh[2]->Halfedge_Mesh::Halfedge::set_neighbors(h2, vh[3], m, ve[1], of);
            vh[3]->Halfedge_Mesh::Halfedge::set_neighbors(vh[0], vh[2], h2->vertex(), ve[1], nf);

            return m;
        }
        (void)e;
        return std::nullopt;
    }

    if(e->halfedge()->face()->degree() != 3 || e->halfedge()->twin()->face()->degree() != 3) {
        (void)e;
        return std::nullopt;
    }
    // allocate space for new things
    std::vector<Halfedge_Mesh::EdgeRef> ve;
    std::vector<Halfedge_Mesh::HalfedgeRef> vh;
    std::vector<Halfedge_Mesh::FaceRef> vf;
    Halfedge_Mesh::VertexRef m = new_vertex();

    ve.push_back(new_edge());
    ve.push_back(new_edge());
    ve.push_back(new_edge());
    vf.push_back(new_face());
    vf.push_back(new_face());
    vh.push_back(new_halfedge());
    vh.push_back(new_halfedge());
    vh.push_back(new_halfedge());
    vh.push_back(new_halfedge());
    vh.push_back(new_halfedge());
    vh.push_back(new_halfedge());

    // position for the new vertex
    m->pos = e->center();

    // old halfedges
    Halfedge_Mesh::HalfedgeRef h0 = e->halfedge();
    Halfedge_Mesh::HalfedgeRef h1 = h0->next();
    Halfedge_Mesh::HalfedgeRef h2 = h1->next();
    Halfedge_Mesh::HalfedgeRef h3 = h0->twin()->next();
    Halfedge_Mesh::HalfedgeRef h4 = h3->next();

    // assign halfedges for the new vertex and old vertex
    m->halfedge() = h0;
    h0->vertex()->halfedge() = h3;

    // assign halfedges for new faces and old faces
    vf[0]->halfedge() = h3;
    vf[1]->halfedge() = h2;
    h0->face()->halfedge() = h0;
    h0->twin()->face()->halfedge() = h0->twin();
    h1->face() = h0->face();
    h2->face() = vf[1];
    h3->face() = vf[0];
    h4->face() = h0->twin()->face();

    // assign halfedges for new edges
    ve[0]->halfedge() = vh[0];
    ve[1]->halfedge() = vh[2];
    ve[2]->halfedge() = vh[4];

    // reassign halfedges
    h0->Halfedge_Mesh::Halfedge::set_neighbors(h1, h0->twin(), m, e, h0->face());
    h0->twin()->next() = vh[0];
    h1->next() = vh[5];
    h2->next() = vh[3];
    h4->next() = h0->twin();
    h3->next() = vh[1];
    vh[0]->Halfedge_Mesh::Halfedge::set_neighbors(h4, vh[1], m, ve[0], h0->twin()->face());
    vh[1]->Halfedge_Mesh::Halfedge::set_neighbors(vh[2], vh[0], h4->vertex(), ve[0], vf[0]);
    vh[2]->Halfedge_Mesh::Halfedge::set_neighbors(h3, vh[3], m, ve[1], vf[0]);
    vh[3]->Halfedge_Mesh::Halfedge::set_neighbors(vh[4], vh[2], h3->vertex(), ve[1], vf[1]);
    vh[4]->Halfedge_Mesh::Halfedge::set_neighbors(h2, vh[5], m, ve[2], vf[1]);
    vh[5]->Halfedge_Mesh::Halfedge::set_neighbors(h0, vh[4], h2->vertex(), ve[2], h0->face());

    return m;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these
    positions, as well as the normal and tangent offset fields to assign positions to
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    if(f->is_boundary()) {
        (void)f;
        return std::nullopt;
    }

    // the degree of the face to bevel
    int n = f->degree();

    // h is the face's haleedge
    Halfedge_Mesh::HalfedgeRef h = f->halfedge();
    Halfedge_Mesh::HalfedgeRef t = h;

    // adding new elements to corresponding vectors
    std::vector<Halfedge_Mesh::VertexRef> vv;
    std::vector<Halfedge_Mesh::EdgeRef> ve;
    std::vector<Halfedge_Mesh::FaceRef> vf;
    std::vector<Halfedge_Mesh::HalfedgeRef> vh;

    for(int i = 0; i < n; i++) {
        vv.push_back(new_vertex());
        ve.push_back(new_edge());
        ve.push_back(new_edge());
        vf.push_back(new_face());
        vh.push_back(new_halfedge());
        vh.push_back(new_halfedge());
        vh.push_back(new_halfedge());
        vh.push_back(new_halfedge());
    }

    // assign pointers to those value
    for(int i = 0; i < n; i++) {
        h = t;

        // for h: new face and new next
        h->face() = vf[i];
        vv[i]->halfedge() = vh[4 * i + 1];
        vv[i]->pos = h->next()->vertex()->pos;
        ve[2 * i]->halfedge() = vh[4 * i];

        ve[2 * i + 1]->halfedge() = vh[4 * i + 1];
        vf[i]->halfedge() = vh[4 * i + 1];

        vh[4 * i]->Halfedge_Mesh::Halfedge::set_neighbors(vh[4 * i + 1], vh[(4 * i + 7) % (4 * n)],
                                                          h->next()->vertex(), ve[2 * i], vf[i]);
        vh[4 * i + 1]->Halfedge_Mesh::Halfedge::set_neighbors(vh[4 * i + 3], vh[4 * i + 2], vv[i],
                                                              ve[2 * i + 1], vf[i]);
        vh[4 * i + 2]->Halfedge_Mesh::Halfedge::set_neighbors(
            vh[(4 * i + 6) % (4 * n)], vh[4 * i + 1], vv[(n + i - 1) % n], ve[2 * i + 1], f);
        vh[4 * i + 3]->Halfedge_Mesh::Halfedge::set_neighbors(
            h, vh[(4 * n + 4 * i - 4) % (4 * n)], vv[(n + i - 1) % n],
            ve[(2 * n + 2 * i - 2) % (2 * n)], vf[i]);

        t = t->next();
        h->next() = vh[4 * i];
    }

    f->halfedge() = vh[2];

    return f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;

    int N = (int)new_halfedges.size();
    Vec3 n = face->normal();

    for(int i = 0; i < N; i++) {
        int a = (i + N - 1) % N;
        int b = i;
        int c = (i + 1) % N;

        // Get the actual 3D vertex coordinates at these vertices
        Vec3 pa = start_positions[a];
        Vec3 pb = start_positions[b];
        Vec3 pc = start_positions[c];

        // mid-point as inset/offset point
        Vec3 m = (pc + pa) / 2.f;
        Vec3 t = (pb - m).unit();

        // Shrink or Expand. Elevate or Depress
        new_halfedges[i]->vertex()->pos = pb + t * tangent_offset - n * normal_offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(FaceRef f = faces_begin(); f < faces_end(); f++) {
        if(f->degree() == 3 || f->is_boundary()) {
            return;
        }
        size_t n = f->degree() - 3;
        std::vector<EdgeRef> new_e;
        std::vector<FaceRef> new_f;
        std::vector<HalfedgeRef> new_h;
        // alloct new edge, face and halfedge
        for(size_t i = 0; i < n; i++) {
            new_e.push_back(new_edge());
            new_f.push_back(new_face());
            new_h.push_back(new_halfedge());
            new_h.push_back(new_halfedge());
        }

        VertexRef v = f->halfedge()->vertex();
        HalfedgeRef h0 = f->halfedge();
        HalfedgeRef h1 = h0->next();

        // deal with the first
        HalfedgeRef h2 = h1->next();
        new_e[0]->halfedge() = new_h[0];
        h1->next() = new_h[0];
        new_h[0]->set_neighbors(h0, new_h[1], h2->vertex(), new_e[0], f);

        // deal with the middle
        for(size_t j = 0; j < n - 1; j++) {
            HalfedgeRef temp2 = h2;
            h2 = h2->next();
            new_h[2 * j + 1]->set_neighbors(temp2, new_h[2 * j], v, new_e[j], new_f[j]);
            new_h[2 * j + 2]->set_neighbors(new_h[2 * j + 1], new_h[2 * j + 3], h2->vertex(),
                                            new_e[j + 1], new_f[j]);
            temp2->next() = new_h[2 * j + 2];
            new_f[j]->halfedge() = temp2;
            new_e[j + 1]->halfedge() = new_h[2 * j + 2];
        }

        // deal with the last
        // the last halfedge is new_h[n*2-1]
        h2->face() = new_f[n - 1];
        h2->next()->face() = new_f[n - 1];
        h2->next()->next() = new_h[n * 2 - 1];
        new_h[n * 2 - 1]->set_neighbors(h2, new_h[n * 2 - 2], v, new_e[n - 1], new_f[n - 1]);
        new_f[n - 1]->halfedge() = h2;
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->new_pos = e->center();
    }
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }
    // Edges
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        if(e->on_boundary()) {
            e->new_pos = e->center();
        } else {
            Vec3 f1 = e->halfedge()->face()->center();
            Vec3 f2 = e->halfedge()->twin()->face()->center();
            Vec3 end1 = e->halfedge()->vertex()->pos;
            Vec3 end2 = e->halfedge()->next()->vertex()->pos;
            e->new_pos = (f1 + f2 + end1 + end2) / 4;
        }
    }
    // Vertices
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        if(v->on_boundary()) {
            v->new_pos = v->center();
        } else {
            // calculate Q & R
            HalfedgeRef hf = v->halfedge();
            HalfedgeRef edge_begin = hf;
            Vec3 f_sum, e_sum, Q, R, S;
            size_t n = v->degree();
            do {
                f_sum += hf->face()->new_pos;
                e_sum += hf->edge()->center();
                hf = hf->twin()->next();
            } while(hf != edge_begin);
            Q = f_sum / n;
            R = e_sum / n;
            // calculate S
            S = v->pos;
            // average of average
            n = v->degree();
            v->new_pos = (Q + 2 * R + (n - 3) * S) / n;
        }
    }
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->is_new = false;
        unsigned int n = v->degree();
        double u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * double(n)));
        v->new_pos = (1.0 - n * u) * v->pos + n * u * v->neighborhood_center();
    }
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->is_new = false;
        Vec3 C = e->halfedge()->next()->next()->vertex()->pos;
        Vec3 D = e->halfedge()->twin()->next()->next()->vertex()->pos;
        e->new_pos = 3.0 / 8.0 * 2 * e->center() + 1.0 / 8.0 * (C + D);
    }
    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new.
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    EdgeRef edge_cur = edges_begin();
    size_t n = n_edges();
    std::vector<EdgeRef> edges;
    std::vector<VertexRef> vertices;
    for(size_t i = 0; i < n; i++) {
        EdgeRef edge_next = edge_cur;
        edge_next++;
        if(!edge_cur->is_new) {
            // if(split_edge(edge_cur)) {
            edges.push_back(edge_cur);
            VertexRef m = split_edge(edge_cur).value();
            // m->new_pos = edge_cur->new_pos;
            vertices.push_back(m);
            m->is_new = true;
            m->halfedge()->next()->next()->edge()->is_new = true;
            m->halfedge()->twin()->next()->edge()->is_new = true;
            // }
        }
        edge_cur = edge_next;
    }

    // Now flip any new edge that connects an old and new vertex.
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        if(e->is_new &&
           (e->halfedge()->vertex()->is_new ^ e->halfedge()->twin()->vertex()->is_new)) {
            flip_edge(e);
        }
    }
    // Finally, copy new vertex positions into the Vertex::pos.
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        if(!v->is_new) {
            v->pos = v->new_pos;
        }
    }
    for(size_t i = 0; i < vertices.size(); i++) {
        vertices[i]->pos = edges[i]->new_pos;
    }
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    // std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    // std::unordered_map<FaceRef, Mat4> face_quadrics;
    // std::unordered_map<EdgeRef, Edge_Record> edge_records;
    // PQueue<Edge_Record> edge_queue;

    // // Compute initial quadrics for each face by simply writing the plane equation
    // // for the face in homogeneous coordinates. These quadrics should be stored
    // // in face_quadrics
    // for(FaceRef f = faces_begin(); f != faces_end(); f++) {
    //     Vec3 N = f->normal();
    //     Vec3 p = f->center();
    //     float dist = -dot(N, p);
    //     Vec4 v = (N[0], N[1], N[2], dist);
    //     f->normal() = outer(v, v);
    // }
    // // -> Compute an initial quadric for each vertex as the sum of the quadrics
    // //    associated with the incident faces, storing it in vertex_quadrics
    // for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
    //     HalfedgeRef h = v->halfedge();
    //     Mat4 quad = h->face()->quadric;
    //     while(h->next() != v->halfedge()) {
    //         h = h->next();
    //         quad += h->face()->quadric;
    //     }
    // }
    // // -> Build a priority queue of edges according to their quadric error cost,
    // //    i.e., by building an Edge_Record for each edge and sticking it in the
    // //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // PQueue<Edge_Record> queue;
    // for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
    //     e->record = edge_records(e);
    //     queue.insert(e->record);
    // }
    // // -> Until we reach the target edge budget, collapse the best edge. Remember
    // //    to remove from the queue any edge that touches the collapsing edge
    // //    BEFORE it gets collapsed, and add back into the queue any edge touching
    // //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    // //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    // //    top of the queue.
    // Size budget = 3 * n_edges() / 4;
    // // cout << "current edge number" << n_edges() << endl;
    // // cout << "should have: " << n_edges() - budget << endl;
    // int i = 0;
    // while(n_edges() > budget) {
    //     // cout << "round " << i << endl;
    //     i++;

    //     Edge_Record best = queue.top();
    //     EdgeRef e = best.edge;
    //     queue.pop();

    //     VertexRef v0 = e->halfedge()->vertex();
    //     // VertexRef v1 = e->halfedge()->twin()->vertex();

    //     // remove all related edges
    //     HalfedgeRef temp1 = e->halfedge()->twin()->next();
    //     while(temp1 != e->halfedge()) {
    //         queue.remove(temp1->edge()->record);
    //         temp1 = temp1->twin()->next();
    //     }

    //     HalfedgeRef temp2 = e->halfedge()->next();
    //     while(temp2 != e->halfedge()->twin()) {
    //         queue.remove(temp2->edge()->record);
    //         temp2 = temp2->twin()->next();
    //     }

    //     VertexRef v = collapse_edge(e);
    //     // cout << "v degree: " << v->degree() << endl;
    //     // update quadric of related vertices, faces and edges
    //     HalfedgeRef hv = v->halfedge();
    //     Mat4 mv;
    //     mv.Zero;
    //     while(hv->twin()->next() != v->halfedge()) {
    //         mv += hv->face()->quadric;
    //         hv->edge()->record = EdgeRecord(hv->edge());
    //         queue.insert(hv->edge()->record);
    //         hv = hv->twin()->next();
    //     }
    //     v->quadric = mv;
    // }

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
