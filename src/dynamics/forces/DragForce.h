/*
 * DragForce.h
 */

#pragma once

#include "ForceGenerator.h"
#include "Constants.h"
#include "Config.h"

#include <cmath>
#include <memory>

namespace BulletPhysic {
namespace dynamics {
namespace forces {

// base drag interface
class IDragModel {
public:
    virtual ~IDragModel() = default;
    virtual math::Vec3 calculate(const math::Vec3& velocity, float area, float density) const = 0;
};

// simple linear drag: F = -b * v
class LinearDrag : public IDragModel {
public:
    explicit LinearDrag(float coefficient = constants::DEFAULT_SPHERE_LINEAR_B) : m_coefficient(coefficient) {}

    math::Vec3 calculate(const math::Vec3& velocity, float /*area*/, float /*density*/) const override
    {
        return velocity * (-m_coefficient);
    }

private:
    float m_coefficient;

};

// quadratic drag: F = - 1/2 * Cd * A * rho * v^2
class QuadraticDrag : public IDragModel {
public:
    explicit QuadraticDrag(float dragCoefficient = constants::DEFAULT_SPHERE_CD) : m_dragCoefficient(dragCoefficient) {}

    math::Vec3 calculate(const math::Vec3& velocity, float area, float density) const override
    {
        float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);

        if (speed < 0.001f)
        {
            return {0, 0, 0};
        }

        float dragMagnitude = 0.5f * m_dragCoefficient * area * density * speed * speed;

        // direction opposite to velocity
        return velocity * (-dragMagnitude / speed);
    }

    void setDragCoefficient(float cd) { m_dragCoefficient = cd; }
    float getDragCoefficient() const { return m_dragCoefficient; }

private:
    float m_dragCoefficient;

};

// main drag force generator
class DragForce : public IForceGenerator {
public:
    DragForce(std::unique_ptr<IDragModel> model, float area = constants::DEFAULT_SPHERE_AREA, float density = constants::SEA_LEVEL_DENSITY)
        : m_model(std::move(model))
        , m_area(area)
        , m_density(density) {}

    void apply(RigidBody& rb, float /*dt*/) override
    {
        if (!m_model)
        {
            return;
        }

        math::Vec3 dragForce = m_model->calculate(rb.velocity(), m_area, m_density);
        rb.addForce(dragForce);
    }

    void setModel(std::unique_ptr<IDragModel> model) { m_model = std::move(model); }
    void setCrossSectionArea(float area) { m_area = area; }
    void setAirDensity(float density) { m_density = density; }

    float getCrossSectionArea() const { return m_area; }
    float getAirDensity() const { return m_density; }

private:
    std::unique_ptr<IDragModel> m_model;
    float m_area;
    float m_density;

};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic