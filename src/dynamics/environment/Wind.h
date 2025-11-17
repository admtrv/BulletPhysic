/*
 * Wind.h
 */

#pragma once

#include "Environment.h"

namespace BulletPhysic {
namespace dynamics {
namespace environment {

// provides wind velocity
class Wind : public IEnvironment {
public:
    explicit Wind(const math::Vec3& windVelocity = math::Vec3{0.0f, 0.0f, 0.0f}) : m_velocity(windVelocity) {}

    void update(PhysicsContext& context, const RigidBody& /*rb*/) override
    {
        context.wind = m_velocity;
    }

    void setWind(const math::Vec3& windVel) { m_velocity = windVel; }
    const math::Vec3& getWind() const { return m_velocity; }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Wind";
    math::Vec3 m_velocity;
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
