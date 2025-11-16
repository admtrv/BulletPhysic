/*
 * WindDragForce.h
 */

#pragma once

#include "Force.h"
#include "DragForce.h"

#include <memory>

namespace BulletPhysic {
namespace dynamics {
namespace forces {

// wind effect using drag model relative to wind velocity
class WindDragForce : public IForce {
public:
    WindDragForce(
        const math::Vec3& windVelocity = math::Vec3{0.0f, 0.0f, 0.0f},
        std::unique_ptr<IDragModel> dragModel = nullptr,
        float area = constants::DEFAULT_SPHERE_AREA,
        float density = constants::SEA_LEVEL_DENSITY,
        std::unique_ptr<IAtmosphereModel> atmosphere = nullptr,
        float groundY = 0.0f)
            : m_windVelocity(windVelocity)
            , m_dragModel(std::move(dragModel))
            , m_area(area)
            , m_density(density)
            , m_atmosphere(std::move(atmosphere))
            , m_groundY(groundY)
    {
        if (!m_dragModel)
        {
            m_dragModel = std::make_unique<QuadraticDrag>();
        }
    }

    // drag relative to wind: F = -0.5 * Cd * A * rho * v_rel^2, where v_rel = v_object - v_wind
    void apply(RigidBody& rb, float /*dt*/) override
    {
        if (!m_dragModel)
        {
            return;
        }

        // don't apply wind when object is grounded
        if (rb.isGrounded())
        {
            return;
        }

        // determine air density: use atmosphere model if available, otherwise use constant
        float airDensity = m_density;

        if (m_atmosphere)
        {
            float altitude = rb.position().y - m_groundY;
            airDensity = m_atmosphere->getDensity(altitude);
        }

        // relative velocity
        math::Vec3 relVelocity = rb.velocity() - m_windVelocity;

        // use drag model to calculate force relative to wind
        math::Vec3 windDrag = m_dragModel->calculate(relVelocity, m_area, airDensity);

        rb.addForce(windDrag);
    }

    void setWindVelocity(const math::Vec3& windVel) { m_windVelocity = windVel; }
    const math::Vec3& getWindVelocity() const { return m_windVelocity; }

    void setAtmosphereModel(std::unique_ptr<IAtmosphereModel> atmosphere) { m_atmosphere = std::move(atmosphere); }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Wind Drag";
    math::Vec3 m_windVelocity;
    std::unique_ptr<IDragModel> m_dragModel;
    float m_area;
    float m_density;
    std::unique_ptr<IAtmosphereModel> m_atmosphere;
    float m_groundY;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
