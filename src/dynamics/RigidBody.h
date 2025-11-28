/*
 * RigidBody.h
 */

#pragma once

#include "math/Vec3.h"
#include "math/Angles.h"
#include "math/Constants.h"
#include "forces/drag/DragModel.h"

#include <cmath>
#include <optional>
#include <memory>

namespace BulletPhysic {
namespace dynamics {

class RigidBody {
public:
    virtual ~RigidBody() = default;

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

    void setGrounded(bool grounded) { m_isGrounded = grounded; }
    bool isGrounded() const { return m_isGrounded; }

    virtual std::unique_ptr<RigidBody> clone() const { return std::make_unique<RigidBody>(*this); }

private:
    float m_mass = 1.0f;
    math::Vec3 m_position{};
    math::Vec3 m_velocity{};
    math::Vec3 m_forces{};
    bool m_isGrounded = false;

};

// Projectile overload of RigidBody
class ProjectileRigidBody : public RigidBody {
public:

    struct ProjectileSpecs {
        float mass;  // kg

        std::optional<float> area;      // m^2 (cross-sectional area for drag)
        std::optional<float> diameter;  // m (caliber)

        std::optional<forces::drag::DragCurveModel> dragModel;

        struct Rifling {
            enum class Direction {
                RIGHT,
                LEFT
            };

            float twistRate;    // n in calibers per turn
            Direction direction = Direction::RIGHT;
        };

        std::optional<Rifling> rifling;
    };

    // constructors
    ProjectileRigidBody() : RigidBody(), m_specs{1.0f} {}
    explicit ProjectileRigidBody(const ProjectileSpecs& specs);

    // getters
    const ProjectileSpecs& getSpecs() const { return m_specs; }
    ProjectileSpecs& getSpecs() { return m_specs; }

    // helpers
    static float calculateArea(float diameter);

    std::unique_ptr<RigidBody> clone() const override
    {
        return std::make_unique<ProjectileRigidBody>(*this);
    }

private:
    ProjectileSpecs m_specs;

};

} // namespace dynamics
} // namespace BulletPhysic
