/*
 * Drag.h
 */

#pragma once

#include "dynamics/forces/Force.h"
#include "Constants.h"
#include "DragModel.h"

#include <memory>

namespace BulletPhysic {
namespace dynamics {
namespace forces {
namespace drag {

// quadratic aerodynamic drag force: F = -0.5 * rho * Cd * A * |v| * v
class Drag : public IForce {
public:
    // constructor with constant Cd
    Drag(float dragCoefficient = constants::DEFAULT_SPHERE_CD, float area = constants::DEFAULT_SPHERE_AREA)
        : m_dragModel(std::make_unique<drag::CustomDragModel>(dragCoefficient))
        , m_area(area)
    {}

    // constructor with standard drag model (G1-G7, GL)
    explicit Drag(drag::DragCurveModel model, float area = constants::DEFAULT_SPHERE_AREA)
        : m_dragModel(std::make_unique<drag::StandardDragModel>(model))
        , m_area(area)
    {}

    // constructor with custom drag model
    explicit Drag(std::unique_ptr<drag::IDragModel> model, float area = constants::DEFAULT_SPHERE_AREA)
        : m_dragModel(std::move(model))
        , m_area(area)
    {}

    void apply(RigidBody& rb, PhysicsContext& context, float /*dt*/) override
    {
        math::Vec3 velocity = rb.velocity();

        // apply wind if available
        if (context.wind.has_value())
        {
            velocity = velocity - *context.wind;
        }

        float velMag = velocity.length();

        // get air density from context or use default
        float rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);

        // Mach = u / c
        float mach = velMag / constants::BASE_SPEED_OF_SOUND;

        // get Cd based on velocity (Mach number)
        float cd = m_dragModel->getCd(mach);

        // F = -0.5 * rho * Cd * A * |v| * v
        float coeff = -0.5f * rho * cd * m_area * velMag;
        math::Vec3 dragForce = velocity * coeff;

        rb.addForce(dragForce);
    }

    const std::string& getName() const override { return m_name; }

    void setDragModel(std::unique_ptr<drag::IDragModel> model) { m_dragModel = std::move(model); }
    void setArea(float area) { m_area = area; }

private:
    std::string m_name = "Air Drag";
    std::unique_ptr<drag::IDragModel> m_dragModel;
    float m_area;
};

} // namespace drag
} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
