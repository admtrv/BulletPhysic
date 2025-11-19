/*
 * Drag.h
 */

#pragma once

#include "Force.h"
#include "Constants.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {

// quadratic aerodynamic drag force: F = -0.5 * rho * Cd * A * |v| * v
class Drag : public IForce {
public:
    Drag(float dragCoefficient = constants::DEFAULT_SPHERE_CD, float area = constants::DEFAULT_SPHERE_AREA)
        : m_dragCoefficient(dragCoefficient)
        , m_area(area)
    {}

    void apply(RigidBody& rb, const PhysicsContext& context, float /*dt*/) override
    {
        math::Vec3 velocity = rb.velocity();

        // apply wind if available
        if (context.wind.has_value())
        {
            velocity = velocity - *context.wind;
        }

        float velMag = velocity.length();
        if (velMag < 1e-6f)
        {
            return; // no drag if not moving
        }

        // get air density from context or use default
        float rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);

        // F = -0.5 * rho * Cd * A * |v| * v
        float coeff = -0.5f * rho * m_dragCoefficient * m_area * velMag;
        math::Vec3 dragForce = velocity * coeff;

        rb.addForce(dragForce);
    }

    const std::string& getName() const override { return m_name; }

    void setDragCoefficient(float cd) { m_dragCoefficient = cd; }
    void setArea(float area) { m_area = area; }

private:
    std::string m_name = "Air Drag";
    float m_dragCoefficient;
    float m_area;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
