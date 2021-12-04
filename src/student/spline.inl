
#include "../geometry/spline.h"
#include "debug.h"

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    float t1 = time;
    float t2 = time * time;
    float t3 = time * time * time;
    T p = (2 * t3 - 3 * t2 + 1) * position0 + (t3 - 2 * t2 + t1) * tangent0 +
          (-2 * t3 + 3 * t2) * position1 + (t3 - t2) * tangent1;

    return p;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    if(control_points.size() < 1) {
        return T();
    } else if(control_points.size() == 1) {
        return control_points.begin()->second;
    } else if(time <= control_points.begin()->first) {
        return control_points.begin()->second;
    } else if(time >= prev(control_points.end())->first) {
        return prev(control_points.end())->second;
    } else {
        auto t2_iter = control_points.upper_bound(time);
        auto t1_iter = t2_iter;
        t1_iter--;
        float t1 = t1_iter->first;
        float t2 = t2_iter->first;
        T p1 = t1_iter->second;
        T p2 = t2_iter->second;
        float t0, t3;
        T p0, p3;
        T m1, m2;
        float norm_time;
        if(t1_iter == control_points.begin()) {
            t0 = t1 - (t2 - t1);
            p0 = p1 - (p2 - p1);
        } else {
            t0 = (--t1_iter)->first;
            p0 = t1_iter->second;
        }

        if(t2_iter == prev(control_points.end())) {
            t3 = t2 + (t2 - t1);
            p3 = p2 + (p2 - p1);
        } else {
            t3 = (++t2_iter)->first;
            p3 = t2_iter->second;
        }

        norm_time = (time - t1) / (t2 - t1);

        float norm_t0 = (t0 - t1) / (t2 - t1);
        float norm_t1 = 0;
        float norm_t2 = 1;
        float norm_t3 = (t3 - t1) / (t2 - t1);

        m1 = (p2 - p0) / (norm_t2 - norm_t0);
        m2 = (p3 - p1) / (norm_t3 - norm_t1);

        return cubic_unit_spline(norm_time, p1, p2, m1, m2);
    }
}
