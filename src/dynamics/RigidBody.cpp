/*
 * RigidBody.cpp
 */

#include "RigidBody.h"

namespace BulletPhysic {
namespace dynamics {

// RigidBody

void RigidBody::setMass(float mass)
{
    m_mass = (mass > 0.0f ? mass : 1.0f);
}

void RigidBody::setPosition(const math::Vec3& pos)
{
    m_position = pos;
}

void RigidBody::setVelocity(const math::Vec3& v)
{
    m_velocity = v;
}

void RigidBody::setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg)
{
    const float elev = math::deg2rad(elevationDeg);
    const float azim = math::deg2rad(azimuthDeg);

    const float ce = std::cos(elev);
    const float se = std::sin(elev);
    const float sa = std::sin(azim);
    const float ca = std::cos(azim);

    m_velocity = {ce * sa * speed, se * speed, ce * ca * speed};
}

void RigidBody::addForce(const math::Vec3& f)
{
    m_forces += f;
}

void RigidBody::clearForces()
{
    m_forces = math::Vec3{};
}

void RigidBody::setState(const math::Vec3& pos, const math::Vec3& vel)
{
    m_position = pos;
    m_velocity = vel;
}

// ProjectileRigidBody

ProjectileRigidBody::ProjectileRigidBody(const ProjectileSpecs& specs) : RigidBody(), m_specs(specs)
{
    setMass(specs.mass);

    if (!m_specs.area.has_value() && m_specs.diameter.has_value())
    {
        m_specs.area = calculateArea(m_specs.diameter.value());
    }

    if (!m_specs.momentOfInertiaX.has_value() && m_specs.diameter.has_value())
    {
        m_specs.momentOfInertiaX = calculateMomentOfInertiaX(specs.mass, m_specs.diameter.value());
    }
}

std::unique_ptr<RigidBody> ProjectileRigidBody::clone() const
{
    return std::make_unique<ProjectileRigidBody>(*this);
}

float ProjectileRigidBody::calculateArea(float diameter)
{
    // cross-sectional area: S = pi * d ^ 2 / 4
    return math::constants::PI * diameter * diameter * 0.25f;
}

float ProjectileRigidBody::calculateMomentOfInertiaX(float mass, float diameter)
{
    // uniform cylinder approximation: Ix = 1/8 * m * d^2
    return 0.125f * mass * diameter * diameter;
}

float ProjectileRigidBody::calculateSpinRate(float velocity, float twistRate, float diameter)
{
    // spin rate: p = 2 * pi * V / (n * d)
    return 2.0f * math::constants::PI * velocity / (twistRate * diameter);
}

void ProjectileRigidBody::setInitialSpinRate(float velocity)
{
    // auto-calculate spin rate on first velocity set if not already set
    if (!m_specs.spinRate.has_value() &&
        m_specs.rifling.has_value() &&
        m_specs.diameter.has_value() &&
        velocity > 1e-3f)
    {
        m_specs.spinRate = calculateSpinRate(velocity, m_specs.rifling->twistRate, m_specs.diameter.value());
    }
}

} // namespace dynamics
} // namespace BulletPhysic
