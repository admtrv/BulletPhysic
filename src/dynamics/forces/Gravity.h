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
    // gravity: F = m * g
    void apply(RigidBody& rb, PhysicsContext& context, float /*dt*/) override
    {
        if (rb.mass() > 0.0f)
        {
            // get corrected gravity acceleration from context or use classic constant
            if (context.gravity.has_value())
            {
                math::Vec3 g = math::Vec3{0.0f, -*context.gravity, 0.0f};
                math::Vec3 force = rb.mass() * g;

                rb.addForce(force);
            }
            else
            {
                math::Vec3 g = constants::GRAVITY;
                math::Vec3 force = rb.mass() * g;

                rb.addForce(force);
            }
        }
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Gravity";
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
