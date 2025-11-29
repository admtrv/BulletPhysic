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

    virtual std::unique_ptr<RigidBody> clone() const { return std::make_unique<RigidBody>(*this); }

    void setMass(float mass);
    void setPosition(const math::Vec3& pos);
    virtual void setVelocity(const math::Vec3& vel);
    virtual void setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg);

    void addForce(const math::Vec3& f);
    void clearForces();

    float mass() const { return m_mass; }
    const math::Vec3& position() const { return m_position; }
    const math::Vec3& velocity() const { return m_velocity; }
    const math::Vec3& accumulatedForces() const { return m_forces; }

    void setState(const math::Vec3& pos, const math::Vec3& vel);

    void setGrounded(bool grounded) { m_isGrounded = grounded; }
    bool isGrounded() const { return m_isGrounded; }

private:
    float m_mass = 1.0f;
    math::Vec3 m_position{};
    math::Vec3 m_velocity{};
    math::Vec3 m_forces{};
    bool m_isGrounded = false;

};

// Projectile overload of RigidBody

struct ProjectileSpecs {
    // basic parameters
    float mass;  // kg

    // dimensional parameters
    std::optional<float> area;              // m^2 (cross-sectional area)
    std::optional<float> diameter;          // m (caliber)

    // aerodynamic coefficients
    std::optional<forces::drag::DragCurveModel> dragModel;          // C_d

    float overtuningCoefficient = constants::DEFAULT_C_M_ALPHA;     // C_M_alpha
    float liftCoefficient = constants::DEFAULT_C_L_ALPHA;           // C_L_alpha
    float MagnusCoefficient = constants::DEFAULT_C_MAG_F;           // C_mag_f

    // rotation-dependent parameters
    struct Rifling {
        enum class Direction {
            RIGHT,  // clockwise
            LEFT    // counterclockwise
        };

        Direction direction;    // rifling direction
        float twistRate;        // n (calibers per turn)
    };

    std::optional<Rifling> rifling;         // muzzle twist parameters
    std::optional<float> spinRate;          // initial spin rate (rad/s)
    std::optional<float> momentOfInertiaX;  // kg * m^2
};

class ProjectileRigidBody : public RigidBody {
public:
    ProjectileRigidBody() : RigidBody(), m_specs{1.0f} {}
    explicit ProjectileRigidBody(const ProjectileSpecs& specs);

    std::unique_ptr<RigidBody> clone() const override;

    // override to calculate spin rate on first velocity set
    void setVelocity(const math::Vec3& vel) override
    {
        RigidBody::setVelocity(vel);
        setInitialSpinRate(vel.length());
    }
    void setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg) override
    {
        RigidBody::setVelocityFromAngles(speed, elevationDeg, azimuthDeg);
        setInitialSpinRate(speed);
    }
    void setInitialSpinRate(float velocity);

    const ProjectileSpecs& getSpecs() const { return m_specs; }
    ProjectileSpecs& getSpecs() { return m_specs; }

private:
    ProjectileSpecs m_specs;

    // helpers
    static float calculateArea(float diameter);
    static float calculateMomentOfInertiaX(float mass, float diameter);
    static float calculateSpinRate(float velocity, float twistRate, float diameter);

};

} // namespace dynamics
} // namespace BulletPhysic
