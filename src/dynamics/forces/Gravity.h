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
            math::Vec3 force;

            // get corrected gravity acceleration from context or use classic constant
            if (context.gravity.has_value())
            {
                math::Vec3 g = math::Vec3{0.0f, -*context.gravity, 0.0f};
                force = rb.mass() * g;
            }
            else
            {
                math::Vec3 g = constants::GRAVITY;
                force = rb.mass() * g;
            }

            m_force = force;
            rb.addForce(force);
        }
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Gravitational";
    std::string m_symbol = "Fg";
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
