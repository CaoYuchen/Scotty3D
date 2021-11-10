
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    float v_tan = tan(get_fov() / 180.0f * M_PI * 0.5f);
    float h_tan = tan(get_h_fov() / 180.0f * M_PI * 0.5f);
    Vec3 cam =
        Vec3((screen_coord.x - 0.5f) * h_tan * 2.0f, (screen_coord.y - 0.5f) * v_tan * 2.0f, -1.0f);
    Vec3 d = iview * cam - iview * Vec3{};
    Ray ray(position, d);

    return ray;
}
