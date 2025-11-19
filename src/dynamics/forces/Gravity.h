/*
 * Gravity.h
 */

#pragma once

#include "Force.h"
#include "Constants.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {

class Gravity : public IForce {
public:
    explicit Gravity(const math::Vec3& gravity = constants::GRAVITY) : m_gravity(gravity) {}

    // gravity: F = m * g
    void apply(RigidBody& rb, const PhysicsContext& context, float /*dt*/) override
    {
        if (rb.mass() > 0.0f)
        {
            // if Geographic corrected gravity magnitude
            if (context.gravity.has_value())
            {
                rb.addForce(math::Vec3{0.0f, -*context.gravity, 0.0f} * rb.mass());
            }
            else
            {
                rb.addForce(m_gravity * rb.mass());
            }
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
