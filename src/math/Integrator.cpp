/*
 * Integrator.cpp
 */

#include "Integrator.h"

namespace BulletPhysic {
namespace math {

void EulerIntegrator::step(dynamics::RigidBody& rb, dynamics::forces::ForceRegistry* forceRegistry, float dt)
{
    // clear previous forces
    rb.clearForces();

    // apply all registered forces
    if (forceRegistry)
    {
        forceRegistry->applyForces(rb, dt);
    }

    // calculate acceleration
    Vec3 a = {0, 0, 0};
    if (rb.mass() > 0.0f)
    {
        a = rb.forceAccum() / rb.mass();
    }

    // integrate
    Vec3 v = rb.velocity() + a * dt;
    Vec3 x = rb.position() + v * dt;

    // ground collision check
    if (x.y < config::ground)
    {
        x.y = config::ground;
        v = {0.0f, 0.0f, 0.0f};
        rb.setState(x, v);
        rb.clearForces();
        return;
    }

    rb.setState(x, v);
    rb.clearForces();
}

void RK4Integrator::step(dynamics::RigidBody& rb, dynamics::forces::ForceRegistry* forceRegistry, float dt)
{
    // clear previous forces
    rb.clearForces();

    const Vec3 x0 = rb.position();
    const Vec3 v0 = rb.velocity();

    // helper lambda to calculate acceleration at given state
    auto calcAccel = [&](const Vec3& pos, const Vec3& vel) -> Vec3
    {
        // temporarily set state
        dynamics::RigidBody tempRb = rb;
        tempRb.setState(pos, vel);
        tempRb.clearForces();

        if (forceRegistry)
        {
            forceRegistry->applyForces(tempRb, dt);
        }

        Vec3 a = {0, 0, 0};
        if (tempRb.mass() > 0.0f)
        {
            a = tempRb.forceAccum() / tempRb.mass();
        }

        return a;
    };

    // RK4 steps
    Vec3 a0 = calcAccel(x0, v0);

    const Vec3 k1_v = a0 * dt;
    const Vec3 k1_x = v0 * dt;

    Vec3 a1 = calcAccel(x0 + k1_x * 0.5f, v0 + k1_v * 0.5f);
    const Vec3 k2_v = a1 * dt;
    const Vec3 k2_x = (v0 + k1_v * 0.5f) * dt;

    Vec3 a2 = calcAccel(x0 + k2_x * 0.5f, v0 + k2_v * 0.5f);
    const Vec3 k3_v = a2 * dt;
    const Vec3 k3_x = (v0 + k2_v * 0.5f) * dt;

    Vec3 a3 = calcAccel(x0 + k3_x, v0 + k3_v);
    const Vec3 k4_v = a3 * dt;
    const Vec3 k4_x = (v0 + k3_v) * dt;

    // combine steps
    Vec3 v = v0 + (k1_v + k2_v * 2.0f + k3_v * 2.0f + k4_v) * (1.0f / 6.0f);
    Vec3 x = x0 + (k1_x + k2_x * 2.0f + k3_x * 2.0f + k4_x) * (1.0f / 6.0f);

    // ground collision check
    if (x.y < config::ground)
    {
        x.y = config::ground;
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
