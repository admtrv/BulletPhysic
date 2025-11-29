/*
 * SpinDrift.h
 */

#pragma once

#include "Force.h"
#include "dynamics/RigidBody.h"
#include "Constants.h"
#include "math/Vec3.h"

#include <cmath>
#include <memory>

namespace BulletPhysic {
namespace dynamics {
namespace forces {

// yaw of repose: alpha_e = 2 * Ix * p * (g x V) / (rho * S * d * V^4 * C_M_alpha)
static math::Vec3 calculateYawOfRepose(const ProjectileRigidBody& projectile, const PhysicsContext & context, const math::Vec3& velocity)
{
    const auto& specs = projectile.getSpecs();

    if (!specs.overtuningCoefficient
        || !specs.diameter.has_value()
        || !specs.momentOfInertiaX.has_value()
        || !specs.area.has_value()
        || !specs.spinRate.has_value())
    {
        return {0.0f, 0.0f, 0.0f};
    }

    float velocityMagnitude = velocity.length();

    if (velocityMagnitude < 1e-3f)
    {
        return {0.0f, 0.0f, 0.0f};
    }

    // denominator
    float rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);
    float S = specs.area.value();
    float d = specs.diameter.value();
    float velocityMagnitudePow4 = velocityMagnitude * velocityMagnitude * velocityMagnitude * velocityMagnitude;
    float C_M_alpha = specs.overtuningCoefficient;

    float denominator = rho * S * d * velocityMagnitudePow4 * C_M_alpha;

    // numerator
    float Ix = specs.momentOfInertiaX.value();
    float p = specs.spinRate.value();
    math::Vec3 g = constants::GRAVITY;
    math::Vec3 gCrossV = g.cross(velocity);

    math::Vec3 numerator = 2.0f * Ix * p * gCrossV;

    int spinSign = specs.rifling->direction == ProjectileRigidBody::ProjectileSpecs::Rifling::Direction::RIGHT ? 1 : -1;

    return spinSign * numerator / denominator;
}

class Lift : public IForce {
public:

    // F_L = 1/2 * rho * S * C_L_alpha * V^2 * alpha_e
    void apply(RigidBody& rb, PhysicsContext& context, float /*dt*/) override
    {
        if (auto* projectile = dynamic_cast<const ProjectileRigidBody*>(&rb))
        {
            const auto& specs = projectile->getSpecs();
            if (!specs.liftCoefficient
                || !specs.area.has_value()
                || !specs.diameter.has_value()
                || !specs.rifling.has_value())
            {
                return;
            }

            math::Vec3 velocity = rb.velocity();
            float velocityMagnitude = velocity.length();
            if (velocityMagnitude < 1e-3f)
            {
                return;
            }
            float velocityMagnitudePow2 = velocityMagnitude * velocityMagnitude;

            float rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);
            float S = specs.area.value();
            float C_L_alpha = specs.liftCoefficient;

            // yaw of repose
            math::Vec3 alpha_e = calculateYawOfRepose(*projectile, context, velocity);

            math::Vec3 liftForce = 0.5f * rho * S * C_L_alpha * velocityMagnitudePow2 * alpha_e;

            rb.addForce(liftForce);
        }
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Lift";
};

class Magnus : public IForce {
public:

    // F_M = -1/2 * rho * S * d * p * C_mag_f * (alpha_e x V)
    void apply(RigidBody& rb, PhysicsContext& context, float /*dt*/) override
    {
        if (auto* projectile = dynamic_cast<const ProjectileRigidBody*>(&rb))
        {
            const auto& specs = projectile->getSpecs();
            if (!specs.MagnusCoefficient
                || !specs.diameter.has_value()
                || !specs.area.has_value()
                || !specs.spinRate.has_value())
            {
                return;
            }

            math::Vec3 velocity = rb.velocity();
            float velocityMagnitude = velocity.length();
            if (velocityMagnitude < 1e-3f)
            {
                return;
            }

            float rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);
            float d = specs.diameter.value();
            float S = specs.area.value();
            float p = specs.spinRate.value();
            float C_mag_f = specs.MagnusCoefficient;

            // yaw of repose
            math::Vec3 alpha_e = calculateYawOfRepose(*projectile, context, velocity);

            math::Vec3 alphaCrossV = alpha_e.cross(velocity);

            math::Vec3 magnusForce = -0.5f * rho * S * d * p * C_mag_f * alphaCrossV;

            rb.addForce(magnusForce);
        }
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Magnus";
};

// combined spin drift related forces: Lift + Magnus
class SpinDrift : public IForce {
public:
    SpinDrift()
        : m_lift(std::make_unique<Lift>())
        , m_magnus(std::make_unique<Magnus>())
    {}

    void apply(RigidBody& rb, PhysicsContext& context, float dt) override
    {
        m_lift->apply(rb, context, dt);
        m_magnus->apply(rb, context, dt);
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Spin Drift Influence";

    std::unique_ptr<Lift> m_lift;
    std::unique_ptr<Magnus> m_magnus;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
