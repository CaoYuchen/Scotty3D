
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Scene_Particles::Particle::update(const PT::Object& scene, float dt, float radius) {

    // TODO(Animation): Task 4

    // Compute the trajectory of this particle for the next dt seconds.

    // (1) Build a ray representing the particle's path if it travelled at constant velocity.

    // (2) Intersect the ray with the scene and account for collisions. Be careful when placing
    // collision points using the particle radius. Move the particle to its next position.

    // (3) Account for acceleration due to gravity.

    // (4) Repeat until the entire time step has been consumed.

    // (5) Decrease the particle's age and return whether it should die.
    // dt is always clamped from 0 to 0.05f;
    float tLeft = dt; // time left for collision looping
    float eps = 1e-3; // minimum time to continue loop

    while((tLeft - eps) > 0) {
        const Ray v(pos, velocity);             // ray from particle origin; velocity is always unit
        v.dist_bounds.y = velocity.norm() * dt; // how far the particle will travel

        PT::Trace hit = scene.hit(v); // hit something?

        if(hit.hit) {
            float ct = hit.distance / velocity.norm(); // collision time

            // Reflect vector about normal: r = d - 2(d.n)n
            pos = hit.position -
                  radius * v.dir; // don't put the particle on the surface. It will stick
            velocity = velocity - 2 * dot(velocity, hit.normal) * hit.normal;

            tLeft -= ct; // update time left
        } else {
            // Forward Euler. does not conserve energy in the system
            pos += velocity * tLeft;
            velocity += acceleration * tLeft; // accel is just gravity in Scotty3D
            break;
        }
    }

    age -= dt;
    return age > 0; // dead particle?
}
