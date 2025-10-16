/*
 * RigidBody.h
 */

#pragma once

#include "math/Vec3.h"
#include "math/Angles.h"

#include <cmath>

namespace BulletPhysic {
namespace dynamics {

class RigidBody {
public:
    void setMass(float mass);
    void setPosition(const math::Vec3& pos);
    void setVelocity(const math::Vec3& vel);
    void setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg);

    void addForce(const math::Vec3& f);
    void clearForces();

    float mass() const { return m_mass; }
    const math::Vec3& position() const { return m_position; }
    const math::Vec3& velocity() const { return m_velocity; }
    const math::Vec3& forceAccum() const { return m_forces; }

    void setState(const math::Vec3& pos, const math::Vec3& vel);

private:
    float m_mass = 1.0f;
    math::Vec3 m_position{};
    math::Vec3 m_velocity{};
    math::Vec3 m_forces{};
};

} // namespace dynamics
} // namespace BulletPhysic
