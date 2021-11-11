
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    BBox box;
    Vec3 c = box.center();
    float r = radius;
    float b = dot(ray.point - c, ray.dir);
    float b2_ac = b * b - dot(ray.point - c, ray.point - c) + r * r;
    float t1, t2, t;
    Trace ret;

    if(b2_ac >= 0) {
        t1 = -1 * b + sqrt(b2_ac);
        t2 = -1 * b - sqrt(b2_ac);
        if(t1 < ray.dist_bounds.x) {
            if(t2 > ray.dist_bounds.y || t2 < ray.dist_bounds.x)
                return ret;
            else
                t = t2;
        } else if(t1 > ray.dist_bounds.y) {
            return ret;
        } else {
            t = t1;
        }
    } else
        return ret;

    ret.origin = ray.point;
    ret.hit = true;                      // was there an intersection?
    ret.distance = t;                    // at what distance did the intersection occur?
    ret.position = ray.at(t);            // where was the intersection?
    ret.normal = (ray.at(t) - c).unit(); // what was the surface normal at the intersection?
    return ret;
}

} // namespace PT
