/*
 * Integrator.cpp
 */

#include "Integrator.h"

namespace BulletPhysic {
namespace math {

void EulerIntegrator::step(dynamics::RigidBody& rb, float dt)
{
    Vec3 a = config::gravity;

    if (rb.mass() > 0.0f)
    {
        a += rb.forceAccum() / rb.mass();
    }

    Vec3 v = rb.velocity() + a * dt;
    Vec3 x = rb.position() + v * dt;

    if (x.y < config::groundY)
    {
        x.y = config::groundY;
        v = {0.0f, 0.0f, 0.0f};
        rb.setState(x, v);
        rb.clearForces();
        return;
    }


    rb.setState(x, v);
    rb.clearForces();
}

void RK4Integrator::step(dynamics::RigidBody& rb, float dt)
{
    Vec3 a0 = config::gravity;
    if (rb.mass() > 0.0f)
    {
        a0 += rb.forceAccum() / rb.mass();
    }

    const Vec3 v0 = rb.velocity();
    const Vec3 x0 = rb.position();

    const Vec3 k1_v = a0 * dt;
    const Vec3 k1_x = v0 * dt;

    const Vec3 k2_v = a0 * dt;
    const Vec3 k2_x = (v0 + 0.5f * k1_v) * dt;

    const Vec3 k3_v = a0 * dt;
    const Vec3 k3_x = (v0 + 0.5f * k2_v) * dt;

    const Vec3 k4_v = a0 * dt;
    const Vec3 k4_x = (v0 + k3_v) * dt;

    Vec3 v = v0 + (k1_v + k2_v * 2.0f + k3_v * 2.0f + k4_v) * (1.0f / 6.0f);
    Vec3 x = x0 + (k1_x + k2_x * 2.0f + k3_x * 2.0f + k4_x) * (1.0f / 6.0f);

    if (x.y < config::groundY)
    {
        x.y = config::groundY;
        v = {0.0f, 0.0f, 0.0f};
        rb.setState(x, v);
        rb.clearForces();
        return;
    }


    rb.setState(x, v);
    rb.clearForces();
}

} // namespace math
} // namespace BulletPhysic
