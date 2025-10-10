/*
 * RigidBody.cpp
 */

#include "RigidBody.h"

namespace luchphysic {
namespace dynamics {

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

} // namespace dynamics
} // namespace luchphysic
