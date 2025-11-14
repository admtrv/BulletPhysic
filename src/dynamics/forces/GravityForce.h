/*
 * GravityForce.h
 */

#pragma once

#include "Force.h"
#include "Config.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {

class GravityForce : public IForce {
public:
    explicit GravityForce(const math::Vec3& gravity = config::gravityVec) : m_gravity(gravity) {}

    // gravity: F = m * g
    void apply(RigidBody& rb, float /*dt*/) override
    {
        if (rb.mass() > 0.0f)
        {
            rb.addForce(m_gravity * rb.mass());
        }
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Gravity";
    math::Vec3 m_gravity;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic